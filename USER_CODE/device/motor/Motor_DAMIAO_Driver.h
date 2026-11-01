//=====================================================================================
//                                Motor_DAMIAO_Driver[达妙电机驱动]
//
//      本文件提供两种达妙驱动，分别对应两类设备：
//
//        Motor_DM_4005/3507/8009/8009P/8006/4340/4310_Driver   单机：一条报文一个电机
//        Motor_DAMIAO_1TO4_Driver         一拖四：一条报文 4 个电机
//                                  暂存到本驱动的提交管理器，commit_hook 发整帧
//
//      MIT 只有力矩是变量，位置/速度/Kp/Kd 固定为 0（纯力矩模式）；
//      q/v/t 量程是每设备参数：驱动 probe 时按型号写进 ctx，
//      数值见 Motor_DMAIAO_Type.h 的 MOTOR_DM_XXX_Q/V/T_LIMIT。
//      反馈力矩用同一个区间解码，所以 3507 的反馈直接就是它自己的量纲。
//=====================================================================================

#ifndef __Motor_DAMIAO_Driver__
#define __Motor_DAMIAO_Driver__

#include "Motor_DMAIAO_Type.h"
#include "main.h"
#include "motor_core.h"

/*===| 达妙 MIT 协议规定的 Kp/Kd 量程（与型号无关，编码时用）|===*/
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/*===| 达妙单机反馈状态码（data.Error，反馈帧 Data[0] 高 4 位）|===
 *      0 = 失能、1 = 使能，其余为故障码（过压/欠压/过流/过温/通信丢失等）。
 *      在线判据：应用使能时回 1、应用失能时回 0；只有 1 才会下发应用力矩。
 *      上层想知道"电机刚恢复"，看 data.Error 从非 1 变成 1 的那一拍即可 */
#define MOTOR_DM_STATE_DISABLED 0U
#define MOTOR_DM_STATE_ENABLED 1U

/* 兜底解码量程：仅当设备没给出额定力矩时使用（正常路径见各型号驱动） */
#define T_MIN -20.0f
#define T_MAX 20.0f


/* 一拖四默认输出限幅 [A]（probe 时写进共享 ctx） */
#define MOTOR_DM_1TO4_OUTPUT_A_MAX 20.0f
#define MOTOR_DM_1TO4_OUTPUT_A_MIN (-20.0f)

/*===| 一拖四电流指令量程（协议原始值）与换算 |===*/
#define MOTOR_DM_1TO4_CURRENT_MAX 16384
#define MOTOR_DM_1TO4_CURRENT_MIN (-16384)
#define MOTOR_DM_1TO4_RAW_PER_AMP (MOTOR_DM_1TO4_CURRENT_MAX / 20.0f)

/* 一拖四反馈 ID 基址：通道号 = rx_id - 0x110 */
#define MOTOR_DM_1TO4_RX_ID_BASE 0x110U

/*===| 设备识别号：本厂商占用 0x10~0x1F |===
 *      单机必须小于 0x18：SetTorqueRange / SetMITParams 按 type_id 是否小于
 *      一拖四判断是不是单机 */
typedef enum {
  MOTOR_TYPE_DM_NONE = 0x00,
  MOTOR_TYPE_DM_4005 = 0x10,    // 4005 单机
  MOTOR_TYPE_DM_3507 = 0x11,    // 3507 单机
  MOTOR_TYPE_DM_8009 = 0x12,    // 8009 单机
  MOTOR_TYPE_DM_8009P = 0x13,   // 8009P 单机
  MOTOR_TYPE_DM_8006 = 0x14,    // 8006 单机
  MOTOR_TYPE_DM_4340 = 0x15,    // 4340 单机
  MOTOR_TYPE_DM_4310 = 0x16,    // 4310 单机
  MOTOR_TYPE_DM_1TO4 = 0x18,    // 一拖四（4 路电流环）
  MOTOR_TYPE_DM_DJI = 0x19,     // 兼容型号：沿用旧工程 16384 原始值区间
} motor_type_dm_t;

/*===| 驱动实例（每种型号一个）|===*/
extern motor_driver_t Motor_DM_4005_Driver;
extern motor_driver_t Motor_DM_3507_Driver;
extern motor_driver_t Motor_DM_8009_Driver;
extern motor_driver_t Motor_DM_8009P_Driver;
extern motor_driver_t Motor_DM_8006_Driver;
extern motor_driver_t Motor_DM_4340_Driver;
extern motor_driver_t Motor_DM_4310_Driver;
extern motor_driver_t Motor_DM_DJI_Driver;
extern motor_driver_t Motor_DAMIAO_1TO4_Driver;

/**
 * @brief 注册本文件支持的全部达妙电机驱动
 * @note  motor_init.c 只需要调用这一次
 */
int Motor_DAMIAO_Register(void);

/**
 * @brief 达妙一拖四所有帧的提交入口（由 motor_commit_all 调用）
 */
void Motor_DM1to4_CommitAll(void);

/**
 * @brief 指定某台达妙电机的力矩量程[N·m]（额定值），同步更新输出限幅 + MIT 编码 + 反馈解码
 * @note  区间越窄，MIT 12 位编码的单位力矩分辨率越高；只接受对称区间 ±t_limit
 */
int Motor_DM_SetTorqueRange(motor_device_t *dev, float t_limit);

/**
 * @brief 动态设置某台达妙电机的 MIT 位置/速度/Kp/Kd（默认全 0 = 纯力矩）
 * @note  写入 dm_ctx_t，下次 set_value 时随力矩一起下发；想恢复纯力矩就再调用一次传全 0
 */
int Motor_DM_SetMITParams(motor_device_t *dev, float p, float v, float kp,
                          float kd);

/*===| 底层协议函数：需手动指定 CAN 总线与 ID |===*/
void Motor_DM_CMD_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_DM_CMD_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_DM_CMD_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);
void Motor_DM_CMD_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);

void Motor_DM_CMD_MIT(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, float _torq,
                      float t_limit, float q_limit, float v_limit, float pos,
                      float vel, float kp, float kd);
void Motor_DM_CMD_Position(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                           float Position, float Speed);
void Motor_DM_CMD_Speed(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                        float Speed);

void Motor_DM1to4_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                              int16_t ID1_Currnet, int16_t ID2_Currnet,
                              int16_t ID3_Currnet, int16_t ID4_Currnet);
void Motor_DM1to4_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID);

/*===| 反馈解包：力矩字段必须按型号额定值解码，故需要传区间 |===*/
void Motor_DM_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *data,
                           float q_limit, float v_limit, float t_limit);
void Motor_DM1to4_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *data);

float DM_uint_to_float(int x_int, float x_min, float x_max, int bits);
int DM_float_to_uint(float x, float x_min, float x_max, int bits);

#endif /* __Motor_DAMIAO_Driver__ */
