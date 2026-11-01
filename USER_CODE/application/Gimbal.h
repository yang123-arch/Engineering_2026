#ifndef __Gimbal__
#define __Gimbal__

#include "Motor_DMAIAO_Type.h"
#include "joint_calc.h"
#include "main.h"
#include "motor_core.h"

/*===| J5（4340）沿用旧读数 |===
 *      旧驱动按 ±π 解码，读数是现在（按 ±12.5 rad 解码）的 π/12.5 倍。
 *      角度环（Total_Angle_4340 = 旧读数 × 4）、上电目标、重力补偿都用旧读数，
 *      保证输出和换型号前一致 |===*/
#define GIMBAL_J5_LEGACY_ANGLE(angle)                                          \
  ((angle) / MOTOR_DM_DJI_K_Q(MOTOR_DM_4340_Q_LIMIT))

/*===| J6 关节测试开关（srv/chirp_pid_test.h）|===
 *      1 = J6 交给 Motor_Test（默认做匀速往返，Motor_Test.Test 可改成扫频），
 *          其余关节照常由云台任务控制、撑住姿态；遥控拨 S 档开始测试
 *      0 = 正常运行 |===*/
#ifndef GIMBAL_MOTOR_TEST
#define GIMBAL_MOTOR_TEST 1
#endif

/*===| 云台状态参数枚举 |===*/
typedef enum {
  Gimbal_State_Normal = 0,
  Gimbal_State_Slow = 1,
  Gimbal_State_Aim = 2,
} Gimbal_State_EnumTypedef;

/*===| 云台数据结构体定义 |===*/
typedef struct {
  Gimbal_State_EnumTypedef Gimbal_State; //云台状态
  uint8_t If_Setting_Zero;

  /*===| Yaw单位为度 |===*/
  float Yaw_Target;   //目标Yaw值
  float Yaw_Feedback; //当前Yaw值

  int16_t Yaw_Encoder_Err; //云台和底盘的编码器偏移角
  float Yaw_Err;           //云台和底盘的偏移角
  float Vision_Yaw_Err;

  /*===| pitch单位为度 |===*/
  float Pitch1_Target;   //目标Pitch值
  float Pitch1_Feedback; //当前Pitch值

  float Pitch2_Target;
  float J4_Target;
  float J5_Target;
  float J6_Target;
  float J7_Target;
  float Clap_Target;

  /*===| 各个关节：设备 + PID（joint_calc.h），全部串级 |===*/
  joint_t Yaw;
  joint_t Pitch1;
  joint_t Pitch2;
  joint_t J4;
  joint_t J5; // 角度反馈用 Total_Angle_4340
  joint_t J6;
  joint_t J7;
  joint_t Clap;

  /*===| 发射机构电机数据（未接入设备表，见 Gimbal.c） |===*/
  Motor_Data_StructTypeDef *Fric1;
  Motor_Data_StructTypeDef *Fric2;
  Motor_Data_StructTypeDef *Trigger;
} Gimbal_Control_StructTypeDef;

/*===| 云台控制数据结构体 |===*/
extern Gimbal_Control_StructTypeDef Gimbal_Control_Struct;

#endif
