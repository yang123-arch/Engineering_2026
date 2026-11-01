//=====================================================================================
//                                    motor_init[电机初始化]
//
//      这里是使用人员唯一需要动的文件，三件事：
//        1. 驱动注册（Motor_Driver_Register_All，内联，只在内部调用）
//        2. 设备声明 g_motor_devices[]：名字 + 种类 + 总线 + 收发 ID + 所属控制域
//        3. 型号参数表：按具体型号给出输出上下限（力矩 N·m / 电流 A）
//
//      motor_init() 把它们串起来：
//        register drivers -> register devices -> match(probe) -> 型号参数 -> 分组
//
//      控制环（上层算法只认识这三样东西）：
//        motor_set_value(&g_motor_devices[MOTOR_IDX_XXX], 输出值);
//        motor_commit_all(MOTOR_GROUP_CHASSIS);
//=====================================================================================

#ifndef __MOTOR_INIT_H__
#define __MOTOR_INIT_H__

#include "motor_core.h"

/*===| 控制域（电机组）：新增控制域在这里加一项，再补一行 motor_init.c 的组名表 |===*/
typedef enum {
  MOTOR_GROUP_NONE = 0, // 不入组（只挂设备，暂不参与控制）

  MOTOR_GROUP_CHASSIS, // 底盘
  MOTOR_GROUP_GIMBAL,  // 云台
  MOTOR_GROUP_SHOOT,   // 发射

  MOTOR_GROUP_MAX // 必须在最后
} motor_group_id_t;

/*===| 设备表：motor_init.c 定义，外部只读 |===*/
extern motor_device_t g_motor_devices[];
extern const uint16_t g_motor_device_count;

/*===| 设备下标：应用层用这些宏索引 g_motor_devices[]，不用记名字 |===*/
enum {
  MOTOR_IDX_CHASSIS_1 = 0,
  MOTOR_IDX_CHASSIS_2,
  MOTOR_IDX_CHASSIS_3,
  MOTOR_IDX_CHASSIS_4,
  MOTOR_IDX_CHASSIS_LEG1,
  MOTOR_IDX_CHASSIS_LEG2,
  MOTOR_IDX_GIMBAL_YAW,
  MOTOR_IDX_GIMBAL_PITCH1,
  MOTOR_IDX_GIMBAL_PITCH2,
  MOTOR_IDX_CAMERA_Y,
  MOTOR_IDX_CAMERA_X,
  MOTOR_IDX_GIMBAL_J4,
  MOTOR_IDX_GIMBAL_J5,
  MOTOR_IDX_GIMBAL_J6,
  MOTOR_IDX_GIMBAL_J7,
  MOTOR_IDX_GIMBAL_CLAP,

  MOTOR_IDX_MAX // 必须在最后
};

/**
 * @brief 电机子系统初始化：注册驱动 -> 注册设备 -> 匹配(probe) -> 型号参数 -> 分组
 * @note  必须在 CAN_Init() 之前调用，保证中断来时 ops 已就绪
 * @return 0 成功，-1 有设备没匹配到驱动
 */
int motor_init(void);

/**
 * @brief 取分组（组名表在 motor_init.c 里维护）
 */
motor_group_t *motor_group_get(motor_group_id_t id);

/**
 * @brief 该组里是否含有某种设备识别号（motor_commit_all 用来跳过无关驱动）
 */
int motor_group_has_type(const motor_group_t *group, motor_type_id_t type_id);

/**
 * @brief 提交某个控制域：把本周期写过的值下发到总线
 * @note  控制环写法（顺序固定）：
 *          1. 算法算完输出，逐个 motor_set_value(dev, value)
 *             —— 单机驱动（达妙单机/凌空）在这里就把报文发了
 *          2. 本组控制任务末尾调用 motor_commit_all(MOTOR_GROUP_XXX)
 *             —— 里面直接调用各驱动的 commit（大疆 Motor_DJI_CommitAll 等），
 *                只对本组涉及的驱动发帧，值没变不会重复占总线
 */
void motor_commit_all(motor_group_id_t id);

/**
 * @brief 使能 / 失能一个控制域
 * @note  只处理状态需要改变的设备（已使能的不再发使能帧，已失能的不再发
 *        失能帧），可以放在会被反复调用的入口里；想强制重发单台用
 *        motor_enable()/motor_disable()
 */
void motor_enable_all(motor_group_id_t id);
void motor_disable_all(motor_group_id_t id);

#endif /* __MOTOR_INIT_H__ */
