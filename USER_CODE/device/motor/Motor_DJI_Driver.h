//=====================================================================================
//                                Motor_DJI_Driver[大疆电机驱动]
//
//      协议特征：一条报文同时控制 4 个电机（电流环）
//        0x200 -> 电调 ID 1~4，0x1FF -> 电调 ID 5~8
//        反馈 ID = 0x200 + 电调 ID（0x201 ~ 0x208）
//        电流量程 -16384 ~ 16384 对应 -20 ~ 20 A（大端）
//
//      控制方式：驱动自带"槽位 + dirty 提交管理器"。
//        motor_set_value(dev, 电流[A])  按型号限幅后换算成原始值写入缓存
//        Motor_DJI_CommitAll()          由 motor_commit_all 调用，发
//        0x200/0x1FF
//=====================================================================================

#ifndef __Motor_DJI_Driver__
#define __Motor_DJI_Driver__

#include "main.h"
#include "motor_core.h"

/* 电调电流指令量程（协议原始单位）*/
#define MOTOR_DJI_CURRENT_MAX 16384
#define MOTOR_DJI_CURRENT_MIN (-16384)

/* 每安培对应的电调原始值：16384 / 20 A */
#define MOTOR_DJI_RAW_PER_AMP (MOTOR_DJI_CURRENT_MAX / 20.0f)

/*===| 各型号额定力矩 [N·m]：set_value 统一收力矩，驱动内部放缩到 ±16384 |===*/
#define MOTOR_DJI_3508_TORQUE_MAX 4.5f
#define MOTOR_DJI_6020_TORQUE_MAX 1.2f
/* 2006 真实力矩参数待测：暂用旧电流量程 20 做占位，
   数值上与旧的"按电流 A 下发"完全一致，保证旧逻辑不变 */
#define MOTOR_DJI_2006_TORQUE_MAX 16384.0f
/* 兼容型号：沿用旧工程 20A 满量程的语义。set_value 收"等效电流"，
   ±20 对应 raw ±16384，与旧版 MOTOR_DJI_OUTPUT_A_MAX/MIN 行为完全一致，
   旧设备表项换 type_id 即恢复原参数 */
#define MOTOR_DJI_LEGACY_TORQUE_MAX 16384.0f

/* 反馈 ID 基址：电调 ID = rx_id - 0x200 */
#define MOTOR_DJI_RX_ID_BASE 0x200U

/*===| 设备识别号：本厂商占用 0x00~0x0F |===*/
typedef enum {
  MOTOR_TYPE_DJI_NONE = 0x00,
  MOTOR_TYPE_DJI_3508 = 0x01,   // 3508 减速电机（电流环）
  MOTOR_TYPE_DJI_2006 = 0x02,   // 2006 减速电机（电流环）
  MOTOR_TYPE_DJI_6020 = 0x03,   // 6020 云台电机（电流环）
  MOTOR_TYPE_DJI_LEGACY = 0x04, // 兼容型号：±20 等效电流 -> ±16384 raw
} motor_type_dji_t;             //新增记得修改Motor_DJI_Probe

/*===| 驱动实例（每种型号一个）|===*/
extern motor_driver_t Motor_DJI_3508_Driver;
extern motor_driver_t Motor_DJI_2006_Driver;
extern motor_driver_t Motor_DJI_6020_Driver;
extern motor_driver_t Motor_DJI_Legacy_Driver;

/**
 * @brief 注册本文件支持的全部大疆电机驱动
 * @note  motor_init.c 只需要调用这一次
 */
int Motor_DJI_Register(void);

/**
 * @brief 大疆所有帧的提交入口
 * @note  由 motor_init.c 的 motor_commit_all() 调用；
 *        组里没有大疆设备时 motor_init 会跳过（见 motor_group_has_type()）
 */
void Motor_DJI_CommitAll(void);

/*===| 底层协议函数：需手动指定 CAN 总线与 ID，调试或旁路框架时使用 |===*/
void Motor_DJI_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                           int16_t ID1_Currnet, int16_t ID2_Currnet,
                           int16_t ID3_Currnet, int16_t ID4_Currnet);

void Motor_DJI_Storage_Data(uint8_t *Data,
                            Motor_Data_StructTypeDef *Motor_Data_Struct);

#endif /* __Motor_DJI_Driver__ */
