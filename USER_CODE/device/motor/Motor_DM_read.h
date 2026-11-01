//=====================================================================================
//                                Motor_DM_read[达妙寄存器读取（临时）]
//
//      临时用的"寄存器读取驱动"：读达妙单机内部的映射范围（PMAX/VMAX/TMAX）
//      和加减速度，核对达妙驱动解码用的 q/v/t_limit。和别的驱动一样在
//      Motor_Driver_Register_All 里注册，设备表里 type_id 填 MOTOR_TYPE_DM_READ
//      的设备挂到本驱动，不经过实际运行的 Motor_DAMIAO_Driver。
//
//        读命令   ID 0x7FF：[ID_L, ID_H, 0x33, RID, 0, 0, 0, 0]
//                 ID_L/ID_H 是电机的 CAN ID（设备表 tx_id）
//        应答     ID = 电机 MST_ID（设备表 rx_id）：
//                 [ID_L, ID_H, 0x33, RID, D4~D7 小端 float]
//
//      读一个寄存器：motor_set_value(dev, DM_REG_PMAX)，应答存进 g_dm_range。
//      ops 只实现 set_value / storage_data / disable，其余为 NULL，
//      所以挂在本驱动上的电机没法被使能。
//
//      读取模式（MOTOR_DM_READ_MODE = 1）：
//        motor_init.c   设备表里要读的达妙行手动改成 MOTOR_TYPE_DM_READ
//        Gimbal.c       原云台任务停用，改为上电调用一次 Motor_DM_Read_All
//        Chassis.c      原底盘任务停用（app_freertos.c 的弱定义空任务顶上）
//        RoboControl.c  Robo_Restart 不拉起电机，整机保持失能
//=====================================================================================

#ifndef __MOTOR_DM_READ_H__
#define __MOTOR_DM_READ_H__

#include "motor_core.h"
#include "motor_init.h"
#include <stdint.h>

/*===| 读取模式开关：1 = 整机切到寄存器读取，0 = 正常运行 |===*/
#ifndef MOTOR_DM_READ_MODE
#define MOTOR_DM_READ_MODE 0
#endif

/* 本驱动的设备识别号：占达妙段（0x10~0x1F）的最后一个号 */
#define MOTOR_TYPE_DM_READ 0x1FU

/* float 最大值，约 3.402823466e+38f */
#define DM_FMAX 3.402823466e+38f

/* 读取参数命令字 */
#define DM_READ_PARAM_CMD 0x33U
#define DM_READ_PARAM_CAN_ID 0x7FFU

/* 寄存器地址 RID */
#define DM_REG_ACC 0x04U  /* 加速度，float，范围 (0.0, fmax) */
#define DM_REG_DEC 0x05U  /* 减速度，float，范围 [-fmax, 0.0) */
#define DM_REG_PMAX 0x15U /* 位置映射范围，float，范围 (0.0, fmax] */
#define DM_REG_VMAX 0x16U /* 速度映射范围，float，范围 (0.0, fmax] */
#define DM_REG_TMAX 0x17U /* 扭矩映射范围，float，范围 (0.0, fmax] */

/* ok_mask 的位：哪个寄存器已经读到 */
#define DM_READ_OK_PMAX (1U << 0)
#define DM_READ_OK_VMAX (1U << 1)
#define DM_READ_OK_TMAX (1U << 2)
#define DM_READ_OK_ACC (1U << 3)
#define DM_READ_OK_DEC (1U << 4)
#define DM_READ_OK_ALL 0x1FU

/* 每台电机的读取状态 */
typedef enum {
  DM_READ_IDLE = 0, // 未读（没挂本驱动的设备一直是它）
  DM_READ_BUSY,     // 读取中
  DM_READ_DONE,     // 全部寄存器都读到了
  DM_READ_FAIL, // 有寄存器重试后仍没应答，看 ok_mask 哪几位是 0
} dm_read_state_t;

/* 电机参数缓存：没读到的字段保持 0（这几个寄存器的合法值都不为 0） */
typedef struct {
  float acc;   /* 加速度 */
  float dec;   /* 减速度，负值 */
  float p_max; /* 位置映射范围[rad] */
  float v_max; /* 速度映射范围[rad/s] */
  float t_max; /* 扭矩映射范围[N·m] */
  volatile uint8_t
      ok_mask; /* 已读到的寄存器（DM_READ_OK_XXX），接收中断里置位 */
  uint8_t state; /* dm_read_state_t */
} DM_MotorRangeParam_t;

/* 读取结果，下标 = g_motor_devices[] 下标（MOTOR_IDX_XXX） */
extern DM_MotorRangeParam_t g_dm_range[MOTOR_IDX_MAX];

/* 驱动实例 */
extern motor_driver_t Motor_DM_Read_Driver;

/**
 * @brief 注册本驱动（motor_init.c 的 Motor_Driver_Register_All 里调用）
 */
int Motor_DM_Read_Register(void);

/**
 * @brief 读取全部挂在本驱动上的电机（阻塞，只能在任务里调）
 * @note  先把整机电机强制失能，再逐台、逐个寄存器 motor_set_value(dev, RID)
 *        等应答；电机都在线时约 100ms，每个不在线的电机再多 150ms
 * @return 全部寄存器都读到的电机台数
 */
int Motor_DM_Read_All(void);

#endif /* __MOTOR_DM_READ_H__ */
