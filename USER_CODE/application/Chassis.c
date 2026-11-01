//===========================================================================
//                      Chassis[底盘控制]
//      根据云台发下来的Vx,Vy,Wz,YawErr进行运动解算，实现底盘的各种运动状态
//      以及根据裁判系统反馈数据和[超电状态]实现功率控制
//===========================================================================
#include "Chassis.h"
#include "Function.h"
#include "INS.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DM_read.h"
#include "Power_Limit.h"
#include "Referee_Unpack.h"
#include "Remote_Control.h"
#include "RoboControl.h"
#include "USB_Driver.h"
#include "chirp.h"
#include "motor_init.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

/*===| 底盘控制数据结构体 |===*/
Chassis_StructTypeDef Chassis = {
    .Wheel =
        {
            {.dev = &g_motor_devices[MOTOR_IDX_CHASSIS_1],
             .mode = JOINT_MODE_SPEED},
            {.dev = &g_motor_devices[MOTOR_IDX_CHASSIS_2],
             .mode = JOINT_MODE_SPEED},
            {.dev = &g_motor_devices[MOTOR_IDX_CHASSIS_3],
             .mode = JOINT_MODE_SPEED},
            {.dev = &g_motor_devices[MOTOR_IDX_CHASSIS_4],
             .mode = JOINT_MODE_SPEED},
        },
    .Leg =
        {
            {.dev = &g_motor_devices[MOTOR_IDX_CHASSIS_LEG1],
             .mode = JOINT_MODE_CASCADE},
            {.dev = &g_motor_devices[MOTOR_IDX_CHASSIS_LEG2],
             .mode = JOINT_MODE_CASCADE},
        },
    .Camera_Y = {.dev = &g_motor_devices[MOTOR_IDX_CAMERA_Y],
                 .mode = JOINT_MODE_CASCADE,
                 .position_fb =
                     &g_motor_devices[MOTOR_IDX_CAMERA_Y].data.Angle_Camera},
    .Camera_X = {.dev = &g_motor_devices[MOTOR_IDX_CAMERA_X],
                 .mode = JOINT_MODE_CASCADE,
                 .position_fb =
                     &g_motor_devices[MOTOR_IDX_CAMERA_X].data.Angle_Camera},
};

/* 腿电机旧 PID 参数换算系数：整定时寄存器 TMAX = 54（Motor_DMAIAO_Type.h 的
   MOTOR_DM_DJI_K_T）；8009P 的 VMAX 仍是 30，速度读数不变 */
#define LEG_KT MOTOR_DM_DJI_K_T(54.0f)

/* Chassis_Task 一轮执行耗时统计（不含 vTaskDelayUntil 阻塞时间） */
/* volatile uint32_t task_cycles_total = 0;
volatile uint32_t task_cycles_count = 0;
volatile float task_time_avg_us = 0.0f;
volatile float task_time_max_us = 0.0f; */

/* 达妙寄存器读取模式（Motor_DM_read.h 的 MOTOR_DM_READ_MODE）下停用底盘控制，
   由 app_freertos.c 的弱定义空任务顶上 */
/* 腿 1 扫频测试（Chassis.h 的 LEG_CHIRP_TEST）时换成文件末尾的测试任务 */
#if !LEG_CHIRP_TEST
//#if !MOTOR_DM_READ_MODE
/**
 * @brief 底盘任务
 */
void Chassis_Task(void *argument) {
  osDelay(1000);

  /* 反馈数据简写：设备在 g_motor_devices[] 里，地址不会变 */
  Motor_Data_StructTypeDef *wheel[4];
  for (uint8_t i = 0; i < 4; i++)
    wheel[i] = &Chassis.Wheel[i].dev->data;
  Motor_Data_StructTypeDef *cam_x = &Chassis.Camera_X.dev->data;
  Motor_Data_StructTypeDef *cam_y = &Chassis.Camera_Y.dev->data;

  /*===| 底盘PID参数初始化 |===*/
  //轮电机
  for (uint8_t i = 0; i < 4; i++) {
    joint_init_speed(&Chassis.Wheel[i], 5, 1, 0, 0, 1000, 16000);
  }
  //腿电机（8009P）：速度环输出从 ±16384 原始值改成 N·m，旧参数 × LEG_KT
  for (uint8_t i = 0; i < 2; i++) {
    joint_init_position(&Chassis.Leg[i], 2, 0, 0, 0, 0, 200);
    joint_init_speed(&Chassis.Leg[i], 50 * LEG_KT, 0, 0, 150 * LEG_KT,
                     8000 * LEG_KT, 16000 * LEG_KT);
  }

  joint_init_position(&Chassis.Camera_Y, 6, 0, 500, 0, 0, 400);
  joint_init_speed(&Chassis.Camera_Y, 12, 0, 0, 0, 500, 3000);

  joint_init_position(&Chassis.Camera_X, 7, 0, 0, 0, 0, 40);
  joint_init_speed(&Chassis.Camera_X, 60, 0.1f, 0, 0, 500, 16000);

  //缓冲区能量
  PID_Init(&Chassis.Energy_Buffer_PID, 5, 0, 0, 0, 0, 225);

  cam_x->Angle_Offset_Camera = cam_x->Total_Angle / 1.0f;
  cam_y->Angle_Offset_Camera = cam_y->Total_Angle / 36.0f;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2); /* 500Hz 定点周期 */

  for (;;) {
    uint32_t task_t0 = DWT->CYCCNT;

    /*===| 得到Yaw偏移弧度，用于相对云台全向移动 |===*/
    Chassis.Yaw_ErrAngle = -RoboControl_Struct.Yaw_Err * PI / 180.0f;

    Chassis.Leg_Angle_Target = RoboControl_Struct.Robo_Target_Leg_Angle;

    Chassis.Camera_X_Target = RoboControl_Struct.Robo_Target_Camera_X;
    Chassis.Camera_Y_Target = RoboControl_Struct.Robo_Target_Camera_Y;

    cam_x->Angle_Camera =
        cam_x->Total_Angle / 1.0f - cam_x->Angle_Offset_Camera;
    cam_y->Angle_Camera =
        cam_y->Total_Angle / 36.0f - cam_y->Angle_Offset_Camera;

    /*===| 冲刺模式关闭横向运动，前进速度增加 |===*/
    if (RoboControl_Struct.Chassis_State == Chassis_DASH) {
      Chassis.Vx_Set = 0;
      Chassis.Vy_Set = Chassis_Dash_Speed_Multiple * Chassis.Vy_Set *
                           cosf(Chassis.Yaw_ErrAngle) +
                       Chassis.Vx_Set * sinf(Chassis.Yaw_ErrAngle);
      Chassis.Wz_Set = Chassis.Wz_Target;
    } else {
      Chassis.Vx_Target =
          RoboControl_Struct.Robo_Target_Vx * cosf(Chassis.Yaw_ErrAngle) +
          RoboControl_Struct.Robo_Target_Vy * sinf(Chassis.Yaw_ErrAngle);
      Chassis.Vy_Target =
          RoboControl_Struct.Robo_Target_Vy * cosf(Chassis.Yaw_ErrAngle) -
          RoboControl_Struct.Robo_Target_Vx * sinf(Chassis.Yaw_ErrAngle);
      //					  Chassis.Chassis_Vx_Set =
      // RoboControl_Struct.Robo_Target_Vx * cosf(Chassis.Yaw_ErrAngle);
      //            Chassis.Vy_Set = RoboControl_Struct.Robo_Target_Vy *
      //            cosf(Chassis.Yaw_ErrAngle);

      Chassis.Wz_Set = Chassis.Wz_Target;
    }
    /*===| 里程计积分 |===*/
    float Dt = DWT_GetDeltaT(&Chassis.Odometer_DWT_Count);

    Chassis.Odometer_Chassis_Vx = (Chassis_RPM_to_m_s * 0.25f) *
                                  (-wheel[0]->Speed_RPM - wheel[1]->Speed_RPM +
                                   wheel[2]->Speed_RPM + wheel[3]->Speed_RPM);
    Chassis.Odometer_Chassis_Vy = (Chassis_RPM_to_m_s * 0.25f) *
                                  (-wheel[0]->Speed_RPM + wheel[1]->Speed_RPM -
                                   wheel[2]->Speed_RPM + wheel[3]->Speed_RPM);
    Chassis.Odometer_Chassis_Wz = (Chassis_RPM_to_m_s * 0.25f) *
                                  (-wheel[0]->Speed_RPM - wheel[1]->Speed_RPM -
                                   wheel[2]->Speed_RPM - wheel[3]->Speed_RPM) /
                                  Chassis_R_Distance;
    Chassis.Odometer_Chassis_X += Chassis.Odometer_Chassis_Vx * Dt;
    Chassis.Odometer_Chassis_Y += Chassis.Odometer_Chassis_Vy * Dt;
    Chassis.Odometer_Chassis_Angle +=
        Chassis.Odometer_Chassis_Wz * Dt * (180.0f / PI);

    Chassis.Odometer_Gimbal_Vy =
        Chassis.Odometer_Chassis_Vx * cosf(Chassis.Yaw_ErrAngle) +
        Chassis.Odometer_Chassis_Vy * sinf(Chassis.Yaw_ErrAngle);
    Chassis.Odometer_Gimbal_Vx =
        Chassis.Odometer_Chassis_Vx * sinf(Chassis.Yaw_ErrAngle) -
        Chassis.Odometer_Chassis_Vy * cosf(Chassis.Yaw_ErrAngle);
    Chassis.Odometer_Gimbal_Wz = Chassis.Odometer_Chassis_Wz;
    Chassis.Odometer_Gimbal_X += Chassis.Odometer_Gimbal_Vx * Dt;
    Chassis.Odometer_Gimbal_Y += Chassis.Odometer_Gimbal_Vy * Dt;
    Chassis.Odometer_Gimbal_Angle +=
        Chassis.Odometer_Gimbal_Wz * Dt * (180.0f / PI);

    /*===| 缓启动 |===*/
    if (Chassis.Vx_Target > Chassis.Vx_Set &&
        Chassis.Vx_Target >= +Dt * Chassis_Acc)
      Chassis.Vx_Set += Dt * Chassis_Acc;
    else if (Chassis.Vx_Target < Chassis.Vx_Set &&
             Chassis.Vx_Target <= -Dt * Chassis_Acc)
      Chassis.Vx_Set -= Dt * Chassis_Acc;
    else
      Chassis.Vx_Set = Chassis.Vx_Target;

    if (Chassis.Vy_Target > Chassis.Vy_Set &&
        Chassis.Vy_Target >= +Dt * Chassis_Acc)
      Chassis.Vy_Set += Dt * Chassis_Acc;
    else if (Chassis.Vy_Target < Chassis.Vy_Set &&
             Chassis.Vy_Target <= -Dt * Chassis_Acc)
      Chassis.Vy_Set -= Dt * Chassis_Acc;
    else
      Chassis.Vy_Set = Chassis.Vy_Target;

    if (RoboControl_Struct.Robo_Target_Wz > Chassis.Wz_Target &&
        RoboControl_Struct.Robo_Target_Wz >=
            +Dt * Chassis_Acc / Chassis_R_Distance)
      Chassis.Wz_Target += Dt * Chassis_Acc / Chassis_R_Distance;
    else if (RoboControl_Struct.Robo_Target_Wz < Chassis.Wz_Target &&
             RoboControl_Struct.Robo_Target_Wz <=
                 -Dt * Chassis_Acc / Chassis_R_Distance)
      Chassis.Wz_Target -= Dt * Chassis_Acc / Chassis_R_Distance;
    else
      Chassis.Wz_Target = RoboControl_Struct.Robo_Target_Wz;

    //        Chassis.Gimbal_Vx_Set = Chassis.Chassis_Vx_Target;
    //        Chassis.Gimbal_Vy_Set = Chassis.Chassis_Vy_Target;
    //        Chassis.Gimbal_Wz_Set = Chassis.Chassis_Wz_Target;

    /*===| 运动学解算[麦轮] |===*/
    Chassis.MotorID_SpeedSet[0] =
        (+Chassis.Vy_Set + Chassis.Vx_Set) +
        (Chassis_A_Distance + Chassis_B_Distance) * Chassis.Wz_Set;
    Chassis.MotorID_SpeedSet[1] =
        (-Chassis.Vy_Set + Chassis.Vx_Set) +
        (Chassis_A_Distance + Chassis_B_Distance) * Chassis.Wz_Set;
    Chassis.MotorID_SpeedSet[2] =
        (+Chassis.Vy_Set - Chassis.Vx_Set) +
        (Chassis_A_Distance + Chassis_B_Distance) * Chassis.Wz_Set;
    Chassis.MotorID_SpeedSet[3] =
        (-Chassis.Vy_Set - Chassis.Vx_Set) +
        (Chassis_A_Distance + Chassis_B_Distance) * Chassis.Wz_Set;

    /*===| PID计算得到每个电机的输出（设备失能时 joint_calc 自己清积分）|===*/
    for (uint8_t i = 0; i < 4; i++)
      joint_calc(&Chassis.Wheel[i],
                 Chassis.MotorID_SpeedSet[i] / Chassis_RPM_to_m_s);

    joint_calc(&Chassis.Leg[0], Chassis.Leg_Angle_Target);
    joint_calc(&Chassis.Leg[1], -Chassis.Leg_Angle_Target);
    /* 腿未开启时下面发 0，dev->enabled 看不到这个开关，这里单独清积分 */
    if (!RoboControl_Struct.If_Enable_Leg) {
      joint_reset(&Chassis.Leg[0]);
      joint_reset(&Chassis.Leg[1]);
    }

    joint_calc(&Chassis.Camera_Y, Chassis.Camera_Y_Target);
    joint_calc(&Chassis.Camera_X, Chassis.Camera_X_Target);

    //        //底盘目标在最低时关闭输出
    //        if(Chassis.Leg_Angle_Target < 5)
    //        {
    //            Chassis.Leg_Angle_PID[0].Output = 0;
    //            Chassis.Leg_Angle_PID[0].I_Output = 0;
    //            Chassis.Leg_Angle_PID[1].Output = 0;
    //            Chassis.Leg_Angle_PID[1].I_Output = 0;
    //            Chassis.Leg_Speed_PID[0].Output = 0;
    //            Chassis.Leg_Speed_PID[0].I_Output = 0;
    //            Chassis.Leg_Speed_PID[1].Output = 0;
    //            Chassis.Leg_Speed_PID[1].I_Output = 0;
    //        }
    /*===| 冲刺模式减少前轮输出 |===*/
    if (RoboControl_Struct.Chassis_State == Chassis_DASH) {
      Chassis.Wheel[0].output *= 0.3f;
      Chassis.Wheel[1].output *= 0.3f;
    }

    /*===| 功率控制 |===*/
    //        if(RoboControl_Struct.SuperCap_State == 1)
    //        {
    //            Chassis.Power_MAX = Robo_State.chassis_power_limit + 50.0f +
    //            30 * RoboControl_Struct.Chassis_Speed_Level;
    //             if(RoboControl_Struct.SuperCap_V >= 8 &&
    //             RoboControl_Struct.SuperCap_V <= 12)Chassis.Power_MAX *= (
    //             RoboControl_Struct.SuperCap_V-8) / 4.0f;
    //        }
    //        else
    //        {
    //            Chassis.Power_MAX = Robo_State.chassis_power_limit - 5.0f *
    //            (40.0f - Robo_PowerHeatData.buffer_energy);
    //        }
    Chassis.Power_MAX = Robo_State.chassis_power_limit -
                        5.0f * (40.0f - Robo_PowerHeatData.buffer_energy);
    Limit_float(&Chassis.Power_MAX, 120, 0);

    /*  Power_Limit(&Chassis, Chassis.Chassis1, Chassis.Power_MAX);
    Power_Limit(&Chassis, Chassis.Chassis2, Chassis.Power_MAX);
    Power_Limit(&Chassis, Chassis.Chassis3, Chassis.Power_MAX);
    Power_Limit(&Chassis, Chassis.Chassis4, Chassis.Power_MAX); */

    /*===| 发送电流参数 |===*/
    static uint8_t Tick = 0;
    Tick = !Tick;

    if (RoboControl_Struct.IF_Seting_Zero) {

      motor_set_zero(Chassis.Leg[0].dev);
      motor_set_zero(Chassis.Leg[1].dev);

      cam_x->Angle_Offset_Camera += cam_x->Angle_Camera;
      cam_y->Angle_Offset_Camera += cam_y->Angle_Camera;

      osDelay(100);
      RoboControl_Struct.IF_Seting_Zero = 0;
    } else {
      if (RoboControl_Struct.Robo_Enable) {
        if (Tick) {
          /* 设备表用 DJI_LEGACY 兼容型号：set_value 直接收电调原始值
             （±16384 满量程），与旧版 set_channel_currents 行为一致 */
          for (uint8_t i = 0; i < 4; i++)
            motor_set_value(Chassis.Wheel[i].dev, Chassis.Wheel[i].output);

          Motor_DJI_SendCurrent(&Camera_Y_CAN, Gimbal_Camera_Y_Send_CAN_ID,
                                Chassis.Camera_Y.output, 0, 0, 0);
          motor_set_value(Chassis.Camera_X.dev, Chassis.Camera_X.output);

        } else {
          if (RoboControl_Struct.If_Enable_Leg) {
            motor_set_value(Chassis.Leg[0].dev, Chassis.Leg[0].output);
            motor_set_value(Chassis.Leg[1].dev, Chassis.Leg[1].output);

          } else {
            motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_LEG1], 0);
            motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_LEG2], 0);
          }
        }
      } else {
        if (Tick) {
          Chassis.Gimbal_Vx_PID.I_Output = 0;
          Chassis.Gimbal_Vy_PID.I_Output = 0;
          Chassis.Gimbal_Wz_PID.I_Output = 0;

          motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_1], 0);
          motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_2], 0);
          motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_3], 0);
          motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_4], 0);
          Motor_DJI_SendCurrent(&Camera_Y_CAN, Gimbal_Camera_Y_Send_CAN_ID, 0,
                                0, 0, 0);
          motor_set_value(&g_motor_devices[MOTOR_IDX_CAMERA_X], 0);

        } else {
          motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_LEG1], 0);
          motor_set_value(&g_motor_devices[MOTOR_IDX_CHASSIS_LEG2], 0);
        }
      }
    }

    motor_commit_all(MOTOR_GROUP_CHASSIS);

    /* 一轮任务耗时统计：DWT->CYCCNT 是 170MHz 计数器（见 main.c
       DWT_Init(170)）， 差值即本周期实际执行的时钟周期数（不含下面的
       vTaskDelayUntil 阻塞时间）。 170 cycles = 1us。每 1000
       轮更新一次平均值/最大值，清 0 重计。 */
    /* {
      uint32_t cycles = DWT->CYCCNT - task_t0;
      float time_us = (float)cycles / 170.0f;

      if (time_us > task_time_max_us)
        task_time_max_us = time_us;

      task_cycles_total += cycles;
      if (++task_cycles_count >= 1000U) {
        task_time_avg_us = (float)task_cycles_total / 170.0f / 1000.0f;
        task_cycles_total = 0U;
        task_cycles_count = 0U;
        task_time_max_us = 0.0f;
      }
    } */

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
//#endif /* !MOTOR_DM_READ_MODE */
#else /* LEG_CHIRP_TEST */
/*===| 腿 1 扫频测试（LEG_CHIRP_TEST）|===
 * 500 Hz，只控制腿 1，其余电机保持失能。
 * 使能 / 失能仍由 Remote_Rx_Task 管（C 档、遥控断连 -> Robo_Stop），
 * 这里只看 Robo_Enable 和 Mode：
 *   N 档：位置保持。目标从使能时的位置斜坡走到 Target
 *   S 档：先位置保持；到位并静止 Settle_Time 后换成弱位置环
 *         （带宽 Sweep_Pos_BW，目标是扫频起始角），加上到位期间测到的
 *         保持力矩，叠加扫频。
 *         扫完、或离起始角超过 Guard，回到位置保持（DONE），
 *         再扫一次要切回 N 再切 S
 * 每拍经 USB 按 JustFloat 发 6 个小端 float + 帧尾 00 00 80 7F：
 *   注入信号[N·m]、控制器输出[N·m]、实际下发总力矩[N·m]、
 *   速度反馈[RPM]、角度反馈[°]、帧序号（每拍加 1，× 0.002 s 即时间）
 * 离线处理见 scripts/speed_loop_id.py
 * |===*/

#define LEG_TEST_DT 0.004f
#define LEG_TEST_CH 6U                               // 每帧 float 个数
#define LEG_TEST_FRAME_BYTES (LEG_TEST_CH * 4U + 4U) // 一帧 28 字节
/* 每块能放的帧数：USB_Send 长度是 uint8_t，一块不能超过 255 字节 */
#define LEG_TEST_BUF_FRAMES (255U / LEG_TEST_FRAME_BYTES)

/**
 * @brief 测试阶段
 */
typedef enum {
  LEG_TEST_HOLD = 0, // 位置保持；S 档下同时做到位判定
  LEG_TEST_SWEEP,    // 弱位置环 + 保持力矩 + 扫频
  LEG_TEST_DONE,     // 扫完或中止，位置保持，等切回 N 档
} Leg_Test_Phase_EnumTypeDef;

/**
 * @brief 腿 1 扫频测试数据（调试器里看 / 改）
 */
typedef struct {
  /* 参数：可在调试器里改；扫频参数在开始扫频那一拍生效 */
  float Target;       // 保持角[°]（Total_Angle）
  float Ramp_Speed;   // 目标斜坡速度[°/s]
  float Settle_Tol;   // 到位判定：离 Target 不超过[°]
  float Settle_Speed; // 到位判定：速度不超过[RPM]
  float Settle_Time;  // 到位判定：连续满足的时长[s]
  float Guard;        // 扫频中离起始角超过[°]就中止
  float Sweep_Pos_BW; // 扫频时弱位置环带宽[Hz]：拉住漂移，要远低于 F0
  float F0;           // 扫频起始频率[Hz]
  float F1;           // 扫频终止频率[Hz]
  float Duration; // 扫频时长[s]
  float Amp;      // 扫频幅值[N·m]
  Chirp_Type_EnumTypeDef Type;

  /* 状态 */
  Leg_Test_Phase_EnumTypeDef Phase;
  float Hold_Torque; // 到位期间控制器输出的平均值[N·m]，扫频时当常值前馈
  float Sweep_Center; // 开始扫频时的角度[°]
  uint32_t Abort;     // 越过 Guard 中止的次数
  float Angle_Set;    // 斜坡后的角度目标[°]
  float Chirp_Out;    // 注入信号[N·m]
  float Ctrl_Out; // 控制器输出[N·m]（速度环输出，扫频时再加 Hold_Torque）
  float
      Total_Out; // 实际下发总力矩[N·m]（驱动限幅、失能置 0 之后的 dev->output）
  float Speed;   // 本拍 PID 用的速度反馈[RPM]
  uint32_t Frame; // 帧序号
  uint32_t Drop;  // USB 一直没空、缓冲满了丢掉的帧数
  Chirp_Struct_TypeDef Chirp;
} Leg_Test_StructTypeDef;

Leg_Test_StructTypeDef Leg_Test = {
    .Target = -9.0f,
    .Ramp_Speed = 20.0f,
    .Settle_Tol = 10.0f,
    .Settle_Speed = 2.0f,
    .Settle_Time = 0.5f,
    .Guard = 6.0f,
    .Sweep_Pos_BW = 1.0f,
    .F0 = 5.0f,
    .F1 = 50.0f,
    .Duration = 20.0f,
    .Amp = 2.0f,
    .Type = CHIRP_LOG,
};

/* USB 发送用两块缓冲轮流写：一块交给 USB 发送时，往另一块里攒帧 */
static uint8_t s_leg_usb_buf[2][LEG_TEST_BUF_FRAMES * LEG_TEST_FRAME_BYTES];
static uint8_t s_leg_usb_fill; // 正在写的那一块
static uint16_t s_leg_usb_len; // 这一块已写的字节数

/**
 * @brief 按小端写一个 float
 */
static void Leg_Test_Put_Float(uint8_t *p, float f) {
  uint32_t u;
  memcpy(&u, &f, sizeof(u));
  p[0] = (uint8_t)u;
  p[1] = (uint8_t)(u >> 8);
  p[2] = (uint8_t)(u >> 16);
  p[3] = (uint8_t)(u >> 24);
}

/**
 * @brief 追加一帧 JustFloat，USB 空闲时把攒下的帧发出去
 * @note  USB_Send 不复制数据：交出去的那块在下一次 USB_Send 返回 OK
 * 之前不能动。 返回 OK 说明上一块已经发完，这时换回去写才安全
 */
static void Leg_Test_Send(const float ch[LEG_TEST_CH]) {
  if (s_leg_usb_len + LEG_TEST_FRAME_BYTES <= sizeof(s_leg_usb_buf[0])) {
    uint8_t *p = &s_leg_usb_buf[s_leg_usb_fill][s_leg_usb_len];
    for (uint8_t i = 0; i < LEG_TEST_CH; i++)
      Leg_Test_Put_Float(p + 4U * i, ch[i]);
    p[4U * LEG_TEST_CH + 0U] = 0x00U; /* JustFloat 帧尾 */
    p[4U * LEG_TEST_CH + 1U] = 0x00U;
    p[4U * LEG_TEST_CH + 2U] = 0x80U;
    p[4U * LEG_TEST_CH + 3U] = 0x7FU;
    s_leg_usb_len += LEG_TEST_FRAME_BYTES;
  } else {
    Leg_Test.Drop++;
  }

  if (s_leg_usb_len > 0U &&
      USB_Send(s_leg_usb_buf[s_leg_usb_fill], (uint8_t)s_leg_usb_len) == 0U) {
    s_leg_usb_fill ^= 1U;
    s_leg_usb_len = 0U;
  }
}

/**
 * @brief 腿 1 扫频测试任务（顶替 Chassis_Task）
 */
void Chassis_Task(void *argument) {
  /*   (void)argument;
    osDelay(1000);

    Motor_Data_StructTypeDef *leg = &Chassis.Leg[0].dev->data;
    motor_device_t *leg_dev = Chassis.Leg[0].dev;
    motor_device_t *leg2_dev = Chassis.Leg[1].dev;

    //===| 腿 1 PID：同原 Chassis_Task |===
    PID_Init(&Chassis.Leg[0].position_pid, 2, 0, 0, 0, 0, 200);
    PID_Init(&Chassis.Leg[0].speed_pid, 50 * LEG_KT, 0, 0, 150 * LEG_KT,
             8000 * LEG_KT, 16000 * LEG_KT);

    uint8_t run_last = 0U;
    uint32_t settle_cnt = 0U; // 到位条件连续满足的拍数
    float settle_sum = 0.0f; // 到位期间控制器输出的累加，求保持力矩用

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 500Hz 定点周期

    for (;;) {
      // 腿 1 以外的电机保持失能（Robo_Restart 会把所有组都使能）
      for (uint16_t i = 0; i < g_motor_device_count; i++) {
        if (i != MOTOR_IDX_CHASSIS_LEG1 && g_motor_devices[i].enabled != 0U)
          motor_disable(&g_motor_devices[i]);
      }

      //===| 反馈快照：PID 计算和发送用同一份
      float angle = leg->Total_Angle;
      Leg_Test.Speed = leg->Speed_RPM;

      // 应用已使能、电机也回了"已使能"才算在控制。刚开始控制的那一拍，
         //目标从当前实际位置起步（反馈和状态码在同一帧里更新，不会是旧位置）
      uint8_t run = (RoboControl_Struct.Robo_Enable != 0U &&
                     leg->Error == MOTOR_DM_STATE_ENABLED)
                        ? 1U
                        : 0U;
      if (run == 0U || run_last == 0U) {
        Leg_Test.Angle_Set = angle;
        Chassis.Leg[0].position_pid.I_Output = 0;
        Chassis.Leg[0].speed_pid.I_Output = 0;
      } else {
        Acc_Slow(Leg_Test.Target, &Leg_Test.Angle_Set, Leg_Test.Ramp_Speed,
                 LEG_TEST_DT);
      }

      //===| 阶段切换
      if (run == 0U || run_last == 0U || Remote.Mode != Remote_Mode_S) {
        // 没在控制、刚开始控制、或不在 S 档：位置保持，到位计时重来
        Chirp_Stop(&Leg_Test.Chirp);
        Leg_Test.Phase = LEG_TEST_HOLD;
        settle_cnt = 0U;
        settle_sum = 0.0f;
      } else if (Leg_Test.Phase == LEG_TEST_HOLD) {
        // 到位：斜坡走完、离 Target 够近、基本静止，连续 Settle_Time。
           // 环带载会有静差，离得远进不了扫频就调大 Settle_Tol
        if (Leg_Test.Angle_Set == Leg_Test.Target &&
            fabsf(angle - Leg_Test.Target) <= Leg_Test.Settle_Tol &&
            fabsf(Leg_Test.Speed) <= Leg_Test.Settle_Speed) {
          settle_cnt++;
          settle_sum += Leg_Test.Ctrl_Out; // 上一拍的输出
        } else {
          settle_cnt = 0U;
          settle_sum = 0.0f;
        }

        // 换成弱位置环后，它的刚度扛不住负载，所以把到位期间的平均输出
           // 当常值力矩加上，代替原来位置环扛着的那部分；弱位置环只负责
           // 拉住保持力矩估偏（静摩擦）带来的漂移
        if (settle_cnt > 0U &&
            (float)settle_cnt * LEG_TEST_DT >= Leg_Test.Settle_Time &&
            Chirp_Init(&Leg_Test.Chirp, Leg_Test.F0, Leg_Test.F1,
                       Leg_Test.Duration, Leg_Test.Amp, Leg_Test.Type) == 0) {
          Leg_Test.Hold_Torque = settle_sum / (float)settle_cnt;
          Leg_Test.Sweep_Center = angle;
          Chassis.Leg[0].speed_pid.I_Output = 0; // 积分已算进保持力矩
          Chirp_Start(&Leg_Test.Chirp);
          Leg_Test.Phase = LEG_TEST_SWEEP;
        }
      } else if (Leg_Test.Phase == LEG_TEST_SWEEP) {
        if (fabsf(angle - Leg_Test.Sweep_Center) > Leg_Test.Guard) {
          Chirp_Stop(&Leg_Test.Chirp);
          Leg_Test.Abort++;
          Leg_Test.Phase = LEG_TEST_DONE;
        } else if (Leg_Test.Chirp.Running == 0U) {
          Leg_Test.Phase = LEG_TEST_DONE;
        }
      }
      run_last = run;

      //===| PID
       // 角度环每拍都算，扫频时只是不用它的输出：恢复位置保持那一拍
       // Error_Last 是新的，D 项不会因为误差跳变冲一下。
       // 扫频时速度目标来自弱位置环：只有 P，目标是扫频起始角（不是 Target，
       // 否则带载静差会变成一个速度目标，把腿往 Target 拽）。
       // 2π·BW 是 (rad/s)/rad，换成 RPM/° 要 ÷6。
       // 扫频前后控制器总输出是连续的：腿停在扫频起点附近，位置环恢复后
       // 输出的就是原来那份保持力矩，正好接替 Hold_Torque
      PID_Position_Calculate(&Chassis.Leg[0].position_pid, Leg_Test.Angle_Set,
                             angle);
      float speed_set = Chassis.Leg[0].position_pid.Output;
      if (Leg_Test.Phase == LEG_TEST_SWEEP)
        speed_set = 2.0f * PI * Leg_Test.Sweep_Pos_BW / 6.0f *
                    (Leg_Test.Sweep_Center - angle);
      PID_Position_Calculate(&Chassis.Leg[0].speed_pid, speed_set,
                             Leg_Test.Speed);

      Leg_Test.Ctrl_Out = 0.0f;
      if (run != 0U) {
        Leg_Test.Ctrl_Out = Chassis.Leg[0].speed_pid.Output;
        if (Leg_Test.Phase == LEG_TEST_SWEEP)
          Leg_Test.Ctrl_Out += Leg_Test.Hold_Torque;
      }
      Leg_Test.Chirp_Out = Chirp_Calculate(&Leg_Test.Chirp, LEG_TEST_DT);

      //===| 发送：失能时驱动自己改发 0 力矩帧，照常调用以保持有反馈
      motor_set_value(leg_dev, Leg_Test.Ctrl_Out + Leg_Test.Chirp_Out);
      Leg_Test.Total_Out = leg_dev->output;
      // 腿 2 已失能，这一帧只是"问一下"：达妙一问一答，不发帧就没有反馈。
         // 驱动看到设备失能只会发 0 力矩帧（电机还回"已使能"就改发失能帧），
         // 电机本身失能也不出力。腿 2 因此保持在线，error_core 不会反复恢复它
      motor_set_value(leg2_dev, 0.0f);

      //===| USB 记录 |===
      float ch[LEG_TEST_CH] = {Leg_Test.Chirp_Out,
                               Leg_Test.Ctrl_Out,
                               Leg_Test.Total_Out,
                               Leg_Test.Speed,
                               angle,
                               (float)Leg_Test.Frame};
      Leg_Test_Send(ch);
      Leg_Test.Frame++;

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    } */
}
#endif /* LEG_CHIRP_TEST */
