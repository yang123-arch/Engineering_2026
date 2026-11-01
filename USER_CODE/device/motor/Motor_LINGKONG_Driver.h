//=====================================================================================
//                                Motor_LINGKONG_Driver[凌空电机驱动]
//
//      协议特征：一条报文只控制一个电机
//        转矩电流 : 0xA1 + 电流值（4、5 字节，小端）
//        使能     : 0x88      失能 : 0x80
//        清故障   : 0x9B      设零 : 0x19
//        读状态2  : 0x9C      反馈帧头 : 0xA1
//
//      控制方式（单通道）：set_value 直接发包，不参与提交管理器
//=====================================================================================

#ifndef __Motor_LINGKONG_Driver__
#define __Motor_LINGKONG_Driver__

#include "main.h"
#include "motor_core.h"

/* 转矩电流协议量程（int16 原始值）*/
#define MOTOR_LK_CURRENT_MAX 32767
#define MOTOR_LK_CURRENT_MIN (-32768)

/*===| MG4010E：set_value 收 N·m |===
 *      手册没写清楚力矩和转矩电流原始值的对应关系，暂按"原始值 ±2048 对应
 *      输出轴 ±2.5 N·m"线性换算（MG 系列转矩电流指令量程是 ±2048）。
 *      反馈的 Torque 按同一比例换成 N·m。确认手册后改这两个宏 |===*/
#define MOTOR_LK_MG4010E_RAW_MAX 2048.0f // 转矩电流原始值满量程
#define MOTOR_LK_MG4010E_T_MAX 2.5f      // 满量程对应的输出轴力矩[N·m]

/*===| 设备识别号：本厂商占用 0x20~0x2F |===*/
typedef enum {
  MOTOR_TYPE_LK_NONE = 0x00,
  MOTOR_TYPE_LK_MG6010 = 0x20,  // MG6010 系列：set_value 收原始值
  MOTOR_TYPE_LK_MG8010 = 0x21,  // MG8010 系列：set_value 收原始值
  MOTOR_TYPE_LK_MG4010E = 0x22, // MG4010E：set_value 收 N·m
} motor_type_lk_t;

/*===| 驱动实例（每种型号一个）|===*/
extern motor_driver_t Motor_LK_MG6010_Driver;
extern motor_driver_t Motor_LK_MG8010_Driver;
extern motor_driver_t Motor_LK_MG4010E_Driver;

/**
 * @brief 注册本文件支持的全部凌空电机驱动
 * @note  motor_init.c 只需要调用这一次
 */
int Motor_LINGKONG_Register(void);

/*===| 底层协议函数：需手动指定 CAN 总线与 ID |===*/
void Motor_LK_CMD_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_LK_CMD_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_LK_CMD_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_LK_CMD_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_LK_Send_Storage_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);

void Motor_LK_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                          int16_t Current);

/*===| 反馈解包（同时兼容旧的直接调用形式）|===*/
void Motor_LK_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *data);

#endif /* __Motor_LINGKONG_Driver__ */
