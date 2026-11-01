//=====================================================================================
//                                    motor_core[电机框架层]
//
//      模仿 Linux 设备驱动模型，把"电机"抽象成三层：
//
//        motor_driver   —— 驱动：一种协议（大疆/达妙/凌空），提供函数表 +
//        上下限 +
//                          probe/remove（匹配阶段初始化设备）+ 注册/提交钩子
//        motor_device   —— 设备：挂在某条 FDCAN
//        上的一个具体电机（名字+类型+ID） motor_group    ——
//        分组：按控制域分组（底盘/云台/发射），控制任务按组遍历
//
//      data flow:
//        控制环:  motor_set_value(dev, v)     —— 限幅后交给驱动
//                                                · 单机驱动：立刻发本机报文
//                                                ·
//                                                一帧多机驱动：只写入提交管理器的缓存
//                 motor_commit_all(group)     ——
//                 遍历组内设备，调用驱动的提交钩子，
//                                                把 0x200/0x1FF
//                                                这类共享帧发出去
//        反馈环:  HAL_FDCAN_RxFifo0Callback
//                 └─ motor_receive(hfdcan, id, data) —— 查表找到设备并解包
//
//      上层算法只需要：注册设备 -> motor_init() -> 控制环末尾
//      motor_commit_all(group) 具体是"一帧 4
//      机"还是"一机一帧"，由驱动自己的提交管理器决定，上层不感知。
//
//      线程安全: 暂存值与反馈数据由"单个控制任务写 + 中断读"访问，约定 32
//      位以内的
//                字段以字访问（float/int16_t），避免撕裂。
//=====================================================================================

#ifndef __MOTOR_CORE_H__
#define __MOTOR_CORE_H__

#include "fdcan.h"
#include "main.h"
#include <stdint.h>

/*===| 帧刷新兜底：值没变时，最多隔多少毫秒重发一次本帧（0 = 不刷新）
 *    时间基准是 DWT_GetTimeline_ms()（DWT_Init 之后才有意义）|===*/
#ifndef MOTOR_FRAME_REFRESH_MS
#define MOTOR_FRAME_REFRESH_MS 10U
#endif

/*===| 设备识别号（motor_type_id_t）|===|
 *
 *  约定：识别号是一个 uint8，由各驱动文件在自己的头文件里用 enum 定义，
 *        同一块板子上全局唯一即可。建议分段：
 *
 *          0x0X  大疆（DJI）    见 Motor_DJI_Driver.h
 *          0x1X  达妙（DM）     见 Motor_DAMIAO_Driver.h
 *          0x2X  凌空（LK）     见 Motor_LINGKONG_Driver.h
 *          0x3X~ 其它厂商
 *
 *        框架只比较数值是否相等，不认识任何具体取值 —— 新增厂商/型号
 *        不需要改本文件，只需要在自己的头文件里占一个号。
 */
typedef uint8_t motor_type_id_t;

/*===| 一、设备的反馈数据 |===*/

/**
 * @brief 电机反馈数据（每个设备独占一份，由 motor_device_t 内嵌持有）
 * @note  字段名与旧 Motor_Data_StructTypeDef 一致（首字母大写写法），
 *        每个量只有一个名字，调试器里不会出现同一地址的两份变量
 */
typedef struct {
  int16_t Encoder;                  // 编码器值
  int16_t Encoder_Last;             // 上一个编码器值
  int32_t Round;                    // 圈数
  float Angle;                      // 绝对角度[°]
  float Angle_Last;                 // 上一个绝对角度[°]
  float Total_Angle;                // 总角度[°]
  float Total_Angle_Last;           // 上一次总角度[°]
  float Speed_RPM;                  // 转速[RPM]
  float Total_Angle_Speed_RPM;      // 由总角度差分得到的转速[RPM]
  float Total_Angle_Speed_RPM_Last; // 上一次差分转速[RPM]
  uint32_t Total_Angle_DWT_Count;   // 差分转速用的 DWT 计数
  float Torque;                     // 力矩
  int8_t Temperature;               // 温度[°C]
  uint8_t Error;                    // 驱动状态码（含义见各驱动头文件）
  volatile uint8_t If_Online;       // 在线标志：驱动收到有效反馈置 1
  float Angle_Offset_Camera;
  float Angle_Camera;
  float Total_Angle_4340;
} motor_data_t;

/*===| 二、函数操作表（驱动实现，填 NULL 表示不支持）|===*/

struct motor_device_t; // 前置声明

/**
 * @brief 电机操作表
 * @note  所有回调第一个参数统一为 motor_device_t*，与 Linux 的
 *        "ops 只收 struct device*" 一致：总线、ID、通道都从设备里取，
 *        驱动内部不出现 per-device 的 static 状态。
 *
 *        控制值只有 set_value 一个入口：它只负责"限幅换算 + 暂存"，
 *        是否立刻发包由驱动自己决定（单机驱动直发，一帧多机驱动交给
 *        自己的提交管理器），调用方一律由 motor_commit_all() 驱动。
 */
typedef struct {
  /*--- 控制类 ---*/
  int (*enable)(struct motor_device_t *dev);   // 使能
  int (*disable)(struct motor_device_t *dev);  // 失能
  int (*clear)(struct motor_device_t *dev);    // 清除故障
  int (*set_zero)(struct motor_device_t *dev); // 设置当前机械零点

  /**
   * @brief 恢复：失联/故障后的自愈流程（error_core 调用，见 error_core.h）
   * @note  必须是非阻塞的：像"清故障 -> 延时 1ms -> 使能"这种多步流程，
   *        第一次调用只走第一步就返回 0，由错误处理任务过一会儿再调用，
   *        直到整条流程走完。用 osDelay()/DWT_Delay() 阻塞实现会把错误
   *        处理任务卡在这里，期间别的设备的通知进不来。
   *        流程和"什么算恢复成功"跟协议走，由各驱动在自己的 ctx 里实现，
   *        框架不提供通用序列。dev->enabled == 0 时不应把电机拉起来。
   * @return 1 流程已走完；0 未走完（下次继续）；-1 本型号没有恢复流程
   */
  int (*recover)(struct motor_device_t *dev);

  /*--- 反馈类 ---*/
  /**
   * @brief 解析反馈帧
   * @note  驱动把协议原始值解码成物理量；解码区间（力矩上限等）
   *        从驱动私有 ctx 读取，不再走 motor_device_t。
   *        在线标志 data.If_Online 由驱动按协议决定是否置 1（例如达妙要求
   *        状态码和 dev->enabled 一致：使能回 1、失能回 0），框架不替驱动置位
   */
  void (*storage_data)(struct motor_device_t *dev, const uint8_t *data);

  /*--- 控制值：换算 + 暂存（单机驱动在此直接发包）---*/
  int (*set_value)(struct motor_device_t *dev, float value);
} motor_ops_t;

/*===| 三、设备 |===*/

/**
 * @brief 电机设备
 * @note  设备只描述"它是什么、挂在哪"，不描述"怎么控制"；
 *        怎么控制由 probe 挂钩的 ops 与驱动上下文决定。
 */
typedef struct motor_device_t {
  const char *name; // 设备名，如 "chassis_m1"（调试用，热路径不比较字符串）
  motor_type_id_t type_id; // 设备识别号：决定用哪个驱动，取值见各驱动头文件

  /*--- 应用声明：驱动 probe 时读取 ---*/
  /**
   * @brief 所属控制域（motor_group_id_t，定义见 motor_init.h）
   * @note  在设备表里直接填，motor_init 据此自动入组，不需要再维护映射表
   */
  int8_t group;
  FDCAN_HandleTypeDef *hfdcan; // 挂载的 FDCAN
  uint16_t tx_id; // 发送报文 ID（多通道设备为广播 ID）
  uint16_t rx_id; // 反馈报文 ID

  /*--- 框架填写 ---*/
  const motor_ops_t *ops;        // 由 motor_driver_match 填充
  struct motor_driver_t *driver; // 匹配到的驱动
  struct motor_device_t *next;   // 设备链表
  uint8_t enabled; // 应用层使能标志：上电为 0，motor_enable/disable 维护
  uint8_t index;                 // 设备注册序号
  int8_t channel; // 驱动 probe 填写的通道/槽位（-1 表示未分配）

  /**
   * @brief 驱动私有上下文（opaque）
   * @note  驱动定义自己的结构体、在 probe 里从驱动内的槽位池取一块挂到这里，
   *        上层永远不需要认识它的类型，框架也不解释它。
   */
  struct motor_ctx_t *ctx;

  /*--- 控制值：实际下发值（驱动限幅后回写）---*/
  float output; // 上层最后写入的控制值（驱动限幅后回写实际下发值）

  motor_data_t data; // 本设备的反馈数据
} motor_device_t;

/*===| 四、驱动 |===*/

/**
 * @brief 驱动私有上下文基类：驱动把自定义上下文结构体的首成员放成它
 */
typedef struct motor_ctx_t {
  motor_device_t *dev; // 反向指针，便于钩子里回读设备
  struct motor_driver_t *drv;
} motor_ctx_t;

typedef struct motor_driver_t {
  const char *name; // 驱动名，如 "dji_3508"
  motor_type_id_t type_id; // 本驱动负责的设备识别号（取值见驱动头文件）

  const motor_ops_t *ops; // 本驱动的操作表实例（驱动文件内 static 定义）

  /**
   * @brief probe：匹配阶段用来初始化设备（对应 Linux 的 probe）
   * @note  在这里挂驱动上下文、按 rx_id 分配通道/槽位、校验配置。
   *        型号相关的输出限幅不在这里做，由 motor_init.c 的型号表统一填。
   *        返回 0 表示认领该设备，非 0 表示拒绝（设备保持未匹配）。
   */
  int (*probe)(motor_device_t *dev);

  /**
   * @brief remove：反注册设备/热切换驱动时清理（对应 Linux 的 remove）
   */
  int (*remove)(motor_device_t *dev);

  struct motor_driver_t *next; // 驱动链表
} motor_driver_t;

/*===| 五、分组：把设备按控制域绑在一起 |===*/

typedef struct motor_group_t {
  const char *name;         // 组名，如 "chassis"
  motor_device_t **devices; // 设备指针数组（外部提供存储）
  uint16_t count;           // 当前设备数
  uint16_t capacity;        // 数组容量
} motor_group_t;

/*===| 六、框架 API |===*/

/**
 * @brief  注册一个驱动（同一驱动文件的所有型号在各自的 Xxx_Register
 * 里依次调用）
 * @return 0 成功，-1 失败（参数非法）
 */
int motor_driver_register(motor_driver_t *drv);

/**
 * @brief  注册设备：挂到设备链表并填充框架默认值
 * @note   设备数组必须常驻（static 或全局），框架只存指针
 * @return 0 成功，-1 失败
 */
int motor_device_register(motor_device_t *dev, uint16_t count);

/**
 * @brief  匹配驱动与设备：按 type 绑定 ops，并调用驱动的 probe 初始化设备
 * @note   必须在 motor_init() 中、CAN 中断打开之前调用
 * @return 成功匹配（probe 通过）的设备数
 */
int motor_driver_match(void);

/**
 * @brief  反馈分发：供 FDCAN 接收中断调用，按 (hfdcan, rx_id) 找到设备并解包
 * @return 0 找到设备，-1 无对应设备 / 无解析函数
 */
int motor_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                  const uint8_t *data);

/**
 * @brief  按名查找设备（调试/初始化用，不在控制环里调用）
 */
motor_device_t *motor_find_by_name(const char *name);

/*===| 七、内联热路径：写值与提交 |===*/

/**
 * @brief 写控制值：原样交给驱动（限幅与换算在驱动里按型号完成）
 * @note  单机驱动内部直接发包；一帧多机驱动只更新自己的帧缓存，
 *        等 motor_commit_all() 统一下发
 * @return 0 成功，-1 失败（设备/操作表/set_value 为空）
 */
static inline int motor_set_value(motor_device_t *dev, float value) {
  if (dev == NULL || dev->ops == NULL || dev->ops->set_value == NULL)
    return -1;

  dev->output = value;
  return dev->ops->set_value(dev, value);
}

/**
 * @brief 使能 / 失能 / 清故障 / 设零（驱动没实现就返回 -1，不崩）
 */
static inline int motor_enable(motor_device_t *dev) {
  if (dev == NULL || dev->ops == NULL || dev->ops->enable == NULL)
    return -1;
  int ret = dev->ops->enable(dev);
  if (ret == 0)
    dev->enabled = 1;
  return ret;
}

static inline int motor_disable(motor_device_t *dev) {
  if (dev == NULL || dev->ops == NULL || dev->ops->disable == NULL)
    return -1;
  int ret = dev->ops->disable(dev);
  if (ret == 0)
    dev->enabled = 0;
  return ret;
}

static inline int motor_clear(motor_device_t *dev) {
  if (dev == NULL || dev->ops == NULL || dev->ops->clear == NULL)
    return -1;
  return dev->ops->clear(dev);
}

static inline int motor_set_zero(motor_device_t *dev) {
  if (dev == NULL || dev->ops == NULL || dev->ops->set_zero == NULL)
    return -1;
  return dev->ops->set_zero(dev);
}

/**
 * @brief 恢复（错误处理用）：返回 -1 表示该型号没实现恢复流程
 */
static inline int motor_recover(motor_device_t *dev) {
  if (dev == NULL || dev->ops == NULL || dev->ops->recover == NULL)
    return -1;
  return dev->ops->recover(dev);
}

/**
 * @brief 已注册的驱动 / 设备数量（上电自检或调试用）
 */
uint16_t motor_driver_count(void);
uint16_t motor_device_count(void);

/*===| 八、旧接口兼容别名 |===*/
typedef motor_data_t Motor_Data_StructTypeDef;

/**
 * @brief 由总角度差分计算转速（带一阶低通）
 * @param data 电机数据
 * @param K    滤波系数 [0,1]，越大越跟随原始值
 */
void Get_TotalAngle_Speed(motor_data_t *data, float K);

#endif /* __MOTOR_CORE_H__ */
