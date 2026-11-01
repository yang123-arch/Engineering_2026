#ifndef __Chassis__
#define __Chassis__

#include "joint_calc.h"
#include "main.h"
#include "motor_core.h"

/*===| 腿 1 扫频测试开关 |===
 *      1 = 停用原底盘任务和云台任务，Chassis_Task 换成腿 1 扫频测试（Chassis.c
 * 末尾） 0 = 正常运行 |===*/
#ifndef LEG_CHIRP_TEST
#define LEG_CHIRP_TEST 0
#endif

/*===| 底盘运动状态枚举定义 |===*/
typedef enum {
  Chassis_OFF = 0,
  Chassis_STATIC = 1,
  Chassis_FOLLOW = 2,
  Chassis_F_SIDE = 3,
  Chassis_SPIN = 4,
  Chassis_DASH = 5
} Chassis_State_EnumTypedef;

/*===| 底盘数据结构体定义 |===*/
typedef struct {
  /*===| 底盘运动状态 |===*/
  Chassis_State_EnumTypedef State;

  float Power_MAX;

  /*===| 底盘设置速度 |===*/
  float Vx_Target;
  float Vy_Target;
  float Wz_Target;
  float Vx_Set; //底盘实际设定左右速度 m/s
  float Vy_Set; //底盘实际设定前后速度 m/s
  float Wz_Set; //底盘目标设定旋转角速度   rad/s

  float Leg_Angle_Target;

  float Camera_Y_Target;
  float Camera_X_Target;

  /*===| 里程计 |===*/
  float Odometer_Chassis_Vx;
  float Odometer_Chassis_Vy;
  float Odometer_Chassis_Wz;
  float Odometer_Gimbal_Vx;
  float Odometer_Gimbal_Vy;
  float Odometer_Gimbal_Wz;
  float Odometer_Chassis_X;
  float Odometer_Chassis_Y;
  float Odometer_Chassis_Angle;
  float Odometer_Gimbal_X;
  float Odometer_Gimbal_Y;
  float Odometer_Gimbal_Angle;

  uint32_t Odometer_DWT_Count;

  /*===| Yaw与机械中值偏差角度 |===*/
  float Yaw_ErrAngle;

  /*===| 各个轮解算后的速度 |===*/
  float MotorID_SpeedSet[4];

  /*===| 各个电机：设备 + PID（joint_calc.h） |===*/
  joint_t Wheel[4]; // 麦轮，单速度环
  joint_t Leg[2];   // 腿（8009P），串级
  joint_t Camera_Y; // 串级，角度反馈用 Angle_Camera
  joint_t Camera_X; // 串级，角度反馈用 Angle_Camera

  /*===| PID结构体 |===*/
  PID_Struct_TypeDef Gimbal_Vx_PID;
  PID_Struct_TypeDef Gimbal_Vy_PID;
  PID_Struct_TypeDef Gimbal_Wz_PID;
  PID_Struct_TypeDef Energy_Buffer_PID;
  /*===| 缓冲区能量百分比 |===*/
  float PowerBuffer_K;
} Chassis_StructTypeDef;

/*===| 底盘控制数据结构体 |===*/
extern Chassis_StructTypeDef Chassis;

void Chassis_Speed_Addto(float Set, float *Chassis_Speed);

#endif
