//======================================================================
//                      Gimbal[云台控制]
//      驱动相应电机使云台陀螺仪达到RoboControl的Pitch，Yaw目标角度
//======================================================================

#include "Gimbal.h"
#include "Aim.h"
#include "Compensation.h"
#include "Function.h"
#include "INS.h"
#include "Motor_DM_read.h"
#include "Motor_LINGKONG_Driver.h"
#include "Remote_Control.h"
#include "RoboControl.h"
#include "chirp_pid_test.h"
#include "motor_init.h"

#if GIMBAL_MOTOR_TEST && LEG_CHIRP_TEST
#error "GIMBAL_MOTOR_TEST 要云台任务在跑：先把 Chassis.h 的 LEG_CHIRP_TEST 改成 0"
#endif

#include "FreeRTOS.h"
#include "task.h"

/*===| 云台控制数据结构体 |===*/
/* 发射机构电机（Fric1/Fric2/Trigger）暂未接入新电机框架的设备表，
   但 Shoot.c 的活动函数（Shoot_Single 等）仍会解引用这三个指针。
   旧工程它们指向真实电机数据；现在指向静态哑数据，保证解引用安全、
   读数恒 0。后续接入发射机构时在 g_motor_devices[] 补设备并改回指针。 */
static Motor_Data_StructTypeDef s_shoot_dummy_data[3];
Gimbal_Control_StructTypeDef Gimbal_Control_Struct = {
    .Yaw = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_YAW],
            .mode = JOINT_MODE_CASCADE},
    .Pitch1 = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_PITCH1],
               .mode = JOINT_MODE_CASCADE},
    .Pitch2 = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_PITCH2],
               .mode = JOINT_MODE_CASCADE},
    .J4 = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_J4],
           .mode = JOINT_MODE_CASCADE},
    .J5 = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_J5],
           .mode = JOINT_MODE_CASCADE,
           .position_fb =
               &g_motor_devices[MOTOR_IDX_GIMBAL_J5].data.Total_Angle_4340},
    .J6 = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_J6],
           .mode = JOINT_MODE_CASCADE},
    .J7 = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_J7],
           .mode = JOINT_MODE_CASCADE},
    .Clap = {.dev = &g_motor_devices[MOTOR_IDX_GIMBAL_CLAP],
             .mode = JOINT_MODE_CASCADE},
    .Fric1 = &s_shoot_dummy_data[0],
    .Fric2 = &s_shoot_dummy_data[1],
    .Trigger = &s_shoot_dummy_data[2],
};
/*===| 重力补偿数据结构体 |===*/
Link_StructTypeDef Gimbal_gravity_Struct[Joint_NUM + 1];

/*===| 旧 PID 参数换算到具体型号（Motor_DMAIAO_Type.h 的 MOTOR_DM_DJI_K_X）|===
 *      PID_Init 里仍写旧工程整定的数，乘上换算系数后输出和用 DM_DJI 时一致：
 *        速度环输出从 ±16384 原始值改成 N·m：全部参数 × KT
 *        速度读数比例变了的关节（Pitch：VMAX 45，J5：VMAX 10）：
 *          角度环输出是速度目标，全部参数 × KV；速度环 Kp、Ki、Kd、Kf 再 ÷ KV
 *      KT 用整定旧参数时电机里的 TMAX：J4 当时是 5，寄存器改成 10 后
 *      按 5 换算，物理力矩不变 |===*/
#define YAW_KT MOTOR_DM_DJI_K_T(20.0f)
#define PITCH_KT MOTOR_DM_DJI_K_T(54.0f)
#define J4_KT MOTOR_DM_DJI_K_T(5.0f)
#define J5_KT MOTOR_DM_DJI_K_T(28.0f)
#define J6_KT MOTOR_DM_DJI_K_T(10.0f)
#define CLAP_KT MOTOR_DM_DJI_K_T(5.0f)
#define PITCH_KV MOTOR_DM_DJI_K_V(MOTOR_DM_8009_V_LIMIT)
#define J5_KV MOTOR_DM_DJI_K_V(MOTOR_DM_4340_V_LIMIT)
/* J7（MG4010E）：旧参数输出的是转矩电流原始值，现在 set_value 收 N·m，
   全部 × 每个原始值对应的 N·m，下发的原始值不变 */
#define J7_KT (MOTOR_LK_MG4010E_T_MAX / MOTOR_LK_MG4010E_RAW_MAX)

/*===| J6 关节测试（GIMBAL_MOTOR_TEST）的安全区间 |===
 *      机械限位 ±80°：目标只在 ±70° 里走，实际超出 ±75° 立刻不出力 |===*/
#define J6_TEST_SAFE 70.0f
#define J6_TEST_LIMIT 75.0f

static inline float Angle_Limit(float angle, float min, float max) {
  if (angle > max)
    angle = max;
  else if (angle < min)
    angle = min;
  return angle;
}

/* 达妙寄存器读取模式（Motor_DM_read.h 的 MOTOR_DM_READ_MODE）下停用云台控制，
   本任务改为上电读一次寄存器 */
/* 腿 1 扫频测试（Chassis.h 的 LEG_CHIRP_TEST）期间停用，由 app_freertos.c 的
   弱定义空任务顶上 */
#if !LEG_CHIRP_TEST
//#if !MOTOR_DM_READ_MODE
void Gimbal_Task(void *argument) {
  osDelay(300);

  /*===| 上电瞬间用当前实际位置初始化 PID 目标，避免电机猛转 |===*/
  RoboControl_Struct.Robo_Target_Yaw =
      Gimbal_Control_Struct.Yaw.dev->data.Total_Angle;
  RoboControl_Struct.Robo_Target_Pitch1 =
      Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle;
  RoboControl_Struct.Robo_Target_Pitch2 =
      Gimbal_Control_Struct.Pitch2.dev->data.Total_Angle;
  RoboControl_Struct.Robo_Target_J4 =
      Gimbal_Control_Struct.J4.dev->data.Total_Angle;
  RoboControl_Struct.Robo_Target_J5 =
      GIMBAL_J5_LEGACY_ANGLE(Gimbal_Control_Struct.J5.dev->data.Total_Angle);
  RoboControl_Struct.Robo_Target_J6 =
      Gimbal_Control_Struct.J6.dev->data.Total_Angle;
  RoboControl_Struct.Robo_Target_J7 =
      Gimbal_Control_Struct.J7.dev->data.Total_Angle;
  RoboControl_Struct.Robo_Target_Clap =
      Gimbal_Control_Struct.Clap.dev->data.Total_Angle;

  /*===| 云台PID参数初始化 |===*/
  Gimbal_Control_Struct.J5.speed_pid.I_Output = 0;
  Gimbal_Control_Struct.J5.speed_pid.Output = 0;
  //                                                P     I      D F
  //                                                I_MAX   O_MAX
  joint_init_position(&Gimbal_Control_Struct.Yaw, 4, 0, 10, 0, 0, 80);
  joint_init_speed(&Gimbal_Control_Struct.Yaw, 230 * YAW_KT, 0, 0, 0,
                   8000 * YAW_KT, 16000 * YAW_KT);
  joint_init_position(&Gimbal_Control_Struct.Pitch1, 3 * PITCH_KV, 0,
                      2 * PITCH_KV, 0, 0, 40 * PITCH_KV);
  joint_init_speed(&Gimbal_Control_Struct.Pitch1, 200 * PITCH_KT / PITCH_KV, 0,
                   0, 100 * PITCH_KT / PITCH_KV, 2000 * PITCH_KT,
                   12000 * PITCH_KT);
  joint_init_position(&Gimbal_Control_Struct.Pitch2, 3 * PITCH_KV, 0,
                      2 * PITCH_KV, 0, 0, 40 * PITCH_KV);
  joint_init_speed(&Gimbal_Control_Struct.Pitch2, 200 * PITCH_KT / PITCH_KV, 0,
                   0, 100 * PITCH_KT / PITCH_KV, 2000 * PITCH_KT,
                   16000 * PITCH_KT);
  joint_init_position(&Gimbal_Control_Struct.J4, 6, 0, 1, 0, 0, 40);
  joint_init_speed(&Gimbal_Control_Struct.J4, 200 * J4_KT, 0, 0, 80 * J4_KT,
                   2000 * J4_KT, 14000 * J4_KT);
  joint_init_position(&Gimbal_Control_Struct.J5, 10 * J5_KV, 0, 5 * J5_KV, 0, 0,
                      40 * J5_KV);
  joint_init_speed(&Gimbal_Control_Struct.J5, 150 * J5_KT / J5_KV, 0, 0,
                   100 * J5_KT / J5_KV, 2000 * J5_KT, 16000 * J5_KT);
  joint_init_position(&Gimbal_Control_Struct.J6, 3, 0, 0, 0, 0, 20);
  joint_init_speed(&Gimbal_Control_Struct.J6, 50 * J6_KT, 0, 0, 50 * J6_KT,
                   8000 * J6_KT, 14000 * J6_KT);
  joint_init_position(&Gimbal_Control_Struct.J7, 7, 0, 0, 0, 0, 40);
  joint_init_speed(&Gimbal_Control_Struct.J7, 20 * J7_KT, 0, 0, 10 * J7_KT,
                   500 * J7_KT, 1400 * J7_KT);
  joint_init_position(&Gimbal_Control_Struct.Clap, 5, 0, 50, 0, 0, 40);
  joint_init_speed(&Gimbal_Control_Struct.Clap, 20 * CLAP_KT, 0.3f * CLAP_KT, 0,
                   30 * CLAP_KT, 8000 * CLAP_KT, 12000 * CLAP_KT);

  /*===| 重力补偿参数初始化 |===*/
  link_init(Gimbal_gravity_Struct);

#if GIMBAL_MOTOR_TEST
  /*===| J6 交给 Motor_Test：PID 用上面填好的 J6 参数，每 2ms 调一次 |===*/
  Motor_Test_Init(&Motor_Test, &Gimbal_Control_Struct.J6, 0.002f);
  Motor_Test.Test = MOTOR_TEST_TYPE_RAMP;
  Motor_Test.Safe_Min = -J6_TEST_SAFE;
  Motor_Test.Safe_Max = J6_TEST_SAFE;
  Motor_Test.Limit_Min = -J6_TEST_LIMIT;
  Motor_Test.Limit_Max = J6_TEST_LIMIT;
#endif

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); /* 1kHz 定点周期 */

  for (;;) {
    Gimbal_Control_Struct.J5.dev->data.Total_Angle_4340 =
        GIMBAL_J5_LEGACY_ANGLE(Gimbal_Control_Struct.J5.dev->data.Total_Angle) *
        4.0f;

    /*===| 得到云台控制参数 ===*/
    Gimbal_Control_Struct.Gimbal_State = RoboControl_Struct.Gimbal_State;

    //      Acc_Slow(RoboControl_Struct.Robo_Target_Yaw,
    //      &Gimbal_Control_Struct.Yaw_Target,      30.0f,0.001f);
    //      Acc_Slow(RoboControl_Struct.Robo_Target_Pitch1,
    //      &Gimbal_Control_Struct.Pitch1_Target,   100.0f,0.001f);
    //      Acc_Slow(RoboControl_Struct.Robo_Target_Pitch2,
    //      &Gimbal_Control_Struct.Pitch2_Target,   100.0f,0.001f);
    //		Acc_Slow(RoboControl_Struct.Robo_Target_J4,
    //&Gimbal_Control_Struct.J4_Target,       100.0f,0.001f);
    //		Acc_Slow(RoboControl_Struct.Robo_Target_J5,
    //&Gimbal_Control_Struct.J5_Target,       100.0f,0.001f);
    //		Acc_Slow(RoboControl_Struct.Robo_Target_J6,
    //&Gimbal_Control_Struct.J6_Target,       100.0f,0.001f);
    //		Acc_Slow(RoboControl_Struct.Robo_Target_J7,
    //&Gimbal_Control_Struct.J7_Target,       100.0f,0.001f);
    //		Acc_Slow(RoboControl_Struct.Robo_Target_Clap,
    //&Gimbal_Control_Struct.Clap_Target,       200.0f,0.001f);

    Gimbal_Control_Struct.Yaw_Target = RoboControl_Struct.Robo_Target_Yaw;
    Gimbal_Control_Struct.Pitch1_Target = RoboControl_Struct.Robo_Target_Pitch1;
    Gimbal_Control_Struct.Pitch2_Target = RoboControl_Struct.Robo_Target_Pitch2;
    Gimbal_Control_Struct.J4_Target = RoboControl_Struct.Robo_Target_J4;
    Gimbal_Control_Struct.J5_Target = RoboControl_Struct.Robo_Target_J5;
    Gimbal_Control_Struct.J6_Target = RoboControl_Struct.Robo_Target_J6;
    Gimbal_Control_Struct.J7_Target = RoboControl_Struct.Robo_Target_J7;
    Gimbal_Control_Struct.Clap_Target = RoboControl_Struct.Robo_Target_Clap;

    /*===| 得到Yaw与底盘正方向的偏移角度[-180 ~ +180度] |===*/
    Gimbal_Control_Struct.Yaw_Err =
        Gimbal_Control_Struct.Yaw.dev->data.Angle - Gimbal_Median_Yaw;
    if (Gimbal_Control_Struct.Yaw_Err >= 180)
      Gimbal_Control_Struct.Yaw_Err = 360 - Gimbal_Control_Struct.Yaw_Err;
    if (Gimbal_Control_Struct.Yaw_Err <= -180)
      Gimbal_Control_Struct.Yaw_Err = 360 + Gimbal_Control_Struct.Yaw_Err;
    RoboControl_Struct.Yaw_Err = Gimbal_Control_Struct.Yaw_Err;

    /*===| 串级PID（设备失能时 joint_calc 自己清积分）|===*/
    joint_calc(&Gimbal_Control_Struct.Yaw, Gimbal_Control_Struct.Yaw_Target);
    joint_calc(&Gimbal_Control_Struct.Pitch1,
               Gimbal_Control_Struct.Pitch1_Target);
    joint_calc(&Gimbal_Control_Struct.Pitch2,
               Gimbal_Control_Struct.Pitch2_Target);
    joint_calc(&Gimbal_Control_Struct.J4, Gimbal_Control_Struct.J4_Target);
    joint_calc(&Gimbal_Control_Struct.J5, Gimbal_Control_Struct.J5_Target);
#if !GIMBAL_MOTOR_TEST /* 测试时 J6 的 PID 由 Motor_Test_Step 算，这里不能再算一遍 */
    joint_calc(&Gimbal_Control_Struct.J6, Gimbal_Control_Struct.J6_Target);
#endif
    joint_calc(&Gimbal_Control_Struct.J7, Gimbal_Control_Struct.J7_Target);
    joint_calc(&Gimbal_Control_Struct.Clap, Gimbal_Control_Struct.Clap_Target);

    /*===| 重力补偿各关节电机力矩计算 |===*/
    gravity_calc(Gimbal_gravity_Struct);
    Torque_Conversion(Gimbal_gravity_Struct);

    if (Gimbal_Control_Struct.Yaw.dev->data.If_Online == 0) {
      Gimbal_Control_Struct.Yaw.speed_pid.Output_Max = 0;
    } else {
      Gimbal_Control_Struct.Yaw.speed_pid.Output_Max = 16000 * YAW_KT;
    }

    /*===| 发送电流参数 |===*/
    static uint8_t Tick = 0;
    Tick = !Tick;

    if (RoboControl_Struct.IF_Seting_Zero) {
      // FDCAN2
      motor_set_zero(Gimbal_Control_Struct.Yaw.dev);
      //            Motor_DM_CMD_SetZero(&Gimbal_Pitch1_CAN,Gimbal_Pitch1_Send_CAN_ID);
      //            //FDCAN3
      //            Motor_DM_CMD_SetZero(&Gimbal_J5_CAN,Gimbal_J5_Send_CAN_ID);
      //            Motor_DM_CMD_SetZero(&Gimbal_J6_CAN,Gimbal_J6_Send_CAN_ID);
      //            osDelay(10);
      //            //FDCAN2
      //            Motor_DM_CMD_SetZero(&Gimbal_Pitch2_CAN,Gimbal_Pitch2_Send_CAN_ID);
      //            Motor_DM_CMD_SetZero(&Gimbal_J4_CAN,Gimbal_J4_Send_CAN_ID);
      //            //FDCAN3
      //            Motor_LK_CMD_SetZero(&Gimbal_J7_CAN,Gimbal_J7_Send_CAN_ID);
      //            Motor_DM_CMD_SetZero(&Gimbal_Clap_CAN,Gimbal_Clap_Send_CAN_ID);

      osDelay(100);
      RoboControl_Struct.IF_Seting_Zero = 0;
    } else {
      if (RoboControl_Struct.Robo_Enable) {

        if (Tick) {
          //                    FDCAN2
          motor_set_value(Gimbal_Control_Struct.Yaw.dev,
                          RoboControl_Struct.Smooth_Start_K *
                                  Gimbal_Control_Struct.Yaw.output +
                              Gimbal_gravity_Struct[0].Joint_torque_output);
          motor_set_value(Gimbal_Control_Struct.Pitch1.dev,
                          RoboControl_Struct.Smooth_Start_K *
                                  Gimbal_Control_Struct.Pitch1.output +
                              Gimbal_gravity_Struct[1].Joint_torque_output);

          //					Motor_DM_CMD_MIT(&Gimbal_Yaw_CAN,Gimbal_Yaw_Send_CAN_ID,0,0,0,0,Gimbal_gravity_Struct[0].Joint_torque_output);
          //					Motor_DM_CMD_MIT(&Gimbal_Pitch1_CAN,Gimbal_Pitch1_Send_CAN_ID,0,0,0,0,Gimbal_gravity_Struct[1].Joint_torque_output);
          // FDCAN3
          motor_set_value(Gimbal_Control_Struct.J5.dev,
                          RoboControl_Struct.Smooth_Start_K *
                                  Gimbal_Control_Struct.J5.output +
                              Gimbal_gravity_Struct[4].Joint_torque_output);
#if GIMBAL_MOTOR_TEST
          /* J6 测试：重力补偿当前馈，拨 S 档开始；不乘 Smooth_Start_K
             （Motor_Test 使能时目标从当前位置起步） */
          Motor_Test.Feedforward = Gimbal_gravity_Struct[5].Joint_torque_output;
          Motor_Test_Step(&Motor_Test, Remote.Mode == Remote_Mode_S);
#else
          motor_set_value(Gimbal_Control_Struct.J6.dev,
                          RoboControl_Struct.Smooth_Start_K *
                                  Gimbal_Control_Struct.J6.output +
                              Gimbal_gravity_Struct[5].Joint_torque_output);
#endif

          //					Motor_DM_CMD_MIT(&Gimbal_J5_CAN,Gimbal_J5_Send_CAN_ID,0,0,0,0,Gimbal_gravity_Struct[4].Joint_torque_output);
          //					Motor_DM_CMD_MIT(&Gimbal_J6_CAN,Gimbal_J6_Send_CAN_ID,0,0,0,0,Gimbal_gravity_Struct[5].Joint_torque_output);

        } else {
          // FDCAN2
          motor_set_value(Gimbal_Control_Struct.Pitch2.dev,
                          RoboControl_Struct.Smooth_Start_K *
                                  Gimbal_Control_Struct.Pitch2.output +
                              Gimbal_gravity_Struct[2].Joint_torque_output);

          //					Motor_DM_CMD_MIT(&Gimbal_Pitch2_CAN,Gimbal_Pitch2_Send_CAN_ID,0,0,0,0,Gimbal_gravity_Struct[2].Joint_torque_output);
          // FDCAN3
          /* J4 旧寄存器 TMAX = 5：旧版超过 ±16384 原始值（±5 N·m）被驱动截住。
             寄存器改成 10 后驱动截在 ±10 N·m，这里补截旧满量程保持一致 */
          float j4_torque = RoboControl_Struct.Smooth_Start_K *
                                Gimbal_Control_Struct.J4.output +
                            Gimbal_gravity_Struct[3].Joint_torque_output;
          Limit_float(&j4_torque, MOTOR_DM_DJI_T_LIMIT * J4_KT,
                      -MOTOR_DM_DJI_T_LIMIT * J4_KT);
          motor_set_value(Gimbal_Control_Struct.J4.dev, j4_torque);
          motor_set_value(Gimbal_Control_Struct.J7.dev,
                          RoboControl_Struct.Smooth_Start_K *
                                  Gimbal_Control_Struct.J7.output +
                              Gimbal_gravity_Struct[6].Joint_torque_output);
          motor_set_value(Gimbal_Control_Struct.Clap.dev,
                          RoboControl_Struct.Smooth_Start_K *
                              Gimbal_Control_Struct.Clap.output);

          //					Motor_DM_CMD_MIT(&Gimbal_J4_CAN,Gimbal_J4_Send_CAN_ID,0,0,0,0,Gimbal_gravity_Struct[3].Joint_torque_output);
          //					Motor_LK_SendCurrent(&Gimbal_J7_CAN,Gimbal_J7_Send_CAN_ID,Gimbal_gravity_Struct[6].Joint_torque_output);
        }
      } else {
        if (Tick) {
          // FDCAN2/
          motor_set_value(Gimbal_Control_Struct.Yaw.dev, 0);
          motor_set_value(Gimbal_Control_Struct.Pitch1.dev, 0);
          // FDCAN3
          motor_set_value(Gimbal_Control_Struct.J5.dev, 0);
#if GIMBAL_MOTOR_TEST
          /* 失能时也要调：Motor_Test 每拍 2ms，驱动对失能设备只发 0 力矩 */
          Motor_Test_Step(&Motor_Test, 0U);
#else
          motor_set_value(Gimbal_Control_Struct.J6.dev, 0);
#endif
        } else { // FDCAN2
          motor_set_value(Gimbal_Control_Struct.Pitch2.dev, 0);
          // FDCAN3
          motor_set_value(Gimbal_Control_Struct.J7.dev, 0);
          motor_set_value(Gimbal_Control_Struct.Clap.dev, 0);
          motor_set_value(Gimbal_Control_Struct.J4.dev, 0);
        }
      }
    }

    motor_commit_all(MOTOR_GROUP_GIMBAL);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif /* !LEG_CHIRP_TEST */
//#else
/**
 * @brief 读取模式下的云台任务：等电机上电后读一次全部达妙单机的寄存器，之后空转
 * @note  结果在 g_dm_range[MOTOR_IDX_XXX]，读完的台数在 g_dm_read_done
 */
// volatile int g_dm_read_done = -1; // -1 = 还没读完

// void Gimbal_Task(void *argument) {
//(void)argument;
// osDelay(300); /* 等电机上电，和原云台任务一致 */

// g_dm_read_done = Motor_DM_Read_All();

// for (;;)
// osDelay(1000);
//}
//#endif /* !MOTOR_DM_READ_MODE */
