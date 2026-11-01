//========================================================
//                      Commnuicate[双板通信]
//      双板通信的发送和解包
//      当前有；云台Vx,Vy,Wz,YawErr，以及一些状态标志位
//
//
//
//     该任务目前已被删除，但相关代码保留以备后续使用
//
//
//     该任务目前已被删除，但相关代码保留以备后续使用
//========================================================

#include "Communicate.h"
#include "INS.h"
#include "RoboControl.h"
#include "Shoot.h"
#include "SuperCap_Driver.h"
#include "Vofa.h"

#define CCMRAM __attribute__((section("ccmram")))

/*===| 通信错误看门狗---如果10ms内没有收到双板通信数据，则重置底盘状态 |===*/
uint8_t Board_Commnuicate_Error_Ticker;

/* uint16_t Learn1_Tick = 0;
uint16_t Learn1_Tick_end = 0;
float Play1_Tick = 0;
float Learn1_Data[600][8];

uint16_t Learn2_Tick = 0;
uint16_t Learn2_Tick_end = 0;
float Play2_Tick = 0;
float Learn2_Data[600][8];

uint16_t Learn3_Tick = 0;
uint16_t Learn3_Tick_end = 0;
float Play3_Tick = 0;
float Learn3_Data[600][8]; */

float Learn_DWT_Tick;
uint32_t Play_DWT_Count;
/**
 * @brief 通信任务
 */
/* void Communicate_Task(void *argument) {
 osDelay(50);
 Learn_DWT_Tick = DWT_GetTimeline_ms();
 for (;;) {
     if (DWT_GetTimeline_ms() >= Learn_DWT_Tick + 100) {
      Learn_DWT_Tick = DWT_GetTimeline_ms();

      if (RoboControl_Struct.Learn_State == 0) {
        if (Learn1_Tick != 0) {
          Learn1_Tick_end = Learn1_Tick - 1;

          RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw.Total_Angle;
          RoboControl_Struct.Robo_Target_Pitch1 = Gimbal_Control_Struct.Pitch1.Total_Angle;
          RoboControl_Struct.Robo_Target_Pitch2 = Gimbal_Control_Struct.Pitch2.Total_Angle;
          RoboControl_Struct.Robo_Target_J4 = Gimbal_Control_Struct.J4.Total_Angle;
          RoboControl_Struct.Robo_Target_J5 = Gimbal_Control_Struct.J5.Total_Angle;
          RoboControl_Struct.Robo_Target_J6 = Gimbal_Control_Struct.J6.Total_Angle;
          RoboControl_Struct.Robo_Target_J7 = Gimbal_Control_Struct.J7.Total_Angle;
          RoboControl_Struct.Robo_Target_Clap = Gimbal_Control_Struct.Clap.Total_Angle;
        } else if (Learn2_Tick != 0) {
          Learn2_Tick_end = Learn2_Tick - 1;

          RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw.Total_Angle;
          RoboControl_Struct.Robo_Target_Pitch1 = Gimbal_Control_Struct.Pitch1.Total_Angle;
          RoboControl_Struct.Robo_Target_Pitch2 = Gimbal_Control_Struct.Pitch2.Total_Angle;
          RoboControl_Struct.Robo_Target_J4 = Gimbal_Control_Struct.J4.Total_Angle;
          RoboControl_Struct.Robo_Target_J5 = Gimbal_Control_Struct.J5.Total_Angle;
          RoboControl_Struct.Robo_Target_J6 = Gimbal_Control_Struct.J6.Total_Angle;
          RoboControl_Struct.Robo_Target_J7 = Gimbal_Control_Struct.J7.Total_Angle;
          RoboControl_Struct.Robo_Target_Clap = Gimbal_Control_Struct.Clap.Total_Angle;
        } else if (Learn3_Tick != 0) {
          Learn3_Tick_end = Learn3_Tick - 1;

          RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw.Total_Angle;
          RoboControl_Struct.Robo_Target_Pitch1 = Gimbal_Control_Struct.Pitch1.Total_Angle;
          RoboControl_Struct.Robo_Target_Pitch2 = Gimbal_Control_Struct.Pitch2.Total_Angle;
          RoboControl_Struct.Robo_Target_J4 = Gimbal_Control_Struct.J4.Total_Angle;
          RoboControl_Struct.Robo_Target_J5 = Gimbal_Control_Struct.J5.Total_Angle;
          RoboControl_Struct.Robo_Target_J6 = Gimbal_Control_Struct.J6.Total_Angle;
          RoboControl_Struct.Robo_Target_J7 = Gimbal_Control_Struct.J7.Total_Angle;
          RoboControl_Struct.Robo_Target_Clap = Gimbal_Control_Struct.Clap.Total_Angle;
        }
        Learn1_Tick = 0;
        Learn2_Tick = 0;
        Learn3_Tick = 0;
      } else if (RoboControl_Struct.Learn_State == 1) {
        Learn1_Data[Learn1_Tick][0] = Gimbal_Control_Struct.Yaw.Total_Angle;
        Learn1_Data[Learn1_Tick][1] = Gimbal_Control_Struct.Pitch1.Total_Angle;
        Learn1_Data[Learn1_Tick][2] = Gimbal_Control_Struct.Pitch2.Total_Angle;
        Learn1_Data[Learn1_Tick][3] = Gimbal_Control_Struct.J4.Total_Angle;
        Learn1_Data[Learn1_Tick][4] = Gimbal_Control_Struct.J5.Total_Angle;
        Learn1_Data[Learn1_Tick][5] = Gimbal_Control_Struct.J6.Total_Angle;
        Learn1_Data[Learn1_Tick][6] = Gimbal_Control_Struct.J7.Total_Angle;
        Learn1_Data[Learn1_Tick][7] = Gimbal_Control_Struct.Clap.Total_Angle;
        Learn1_Tick++;
        if (Learn1_Tick >= 600) {
          RoboControl_Struct.Learn_State = 0;
        }
      } else if (RoboControl_Struct.Learn_State == 2) {
        Learn2_Data[Learn2_Tick][0] = Gimbal_Control_Struct.Yaw.Total_Angle;
        Learn2_Data[Learn2_Tick][1] = Gimbal_Control_Struct.Pitch1.Total_Angle;
        Learn2_Data[Learn2_Tick][2] = Gimbal_Control_Struct.Pitch2.Total_Angle;
        Learn2_Data[Learn2_Tick][3] = Gimbal_Control_Struct.J4.Total_Angle;
        Learn2_Data[Learn2_Tick][4] = Gimbal_Control_Struct.J5.Total_Angle;
        Learn2_Data[Learn2_Tick][5] = Gimbal_Control_Struct.J6.Total_Angle;
        Learn2_Data[Learn2_Tick][6] = Gimbal_Control_Struct.J7.Total_Angle;
        Learn2_Data[Learn2_Tick][7] = Gimbal_Control_Struct.Clap.Total_Angle;
        Learn2_Tick++;
        if (Learn2_Tick >= 600) {
          RoboControl_Struct.Learn_State = 0;
        }
      } else if (RoboControl_Struct.Learn_State == 3) {
        Learn3_Data[Learn3_Tick][0] = Gimbal_Control_Struct.Yaw.Total_Angle;
        Learn3_Data[Learn3_Tick][1] = Gimbal_Control_Struct.Pitch1.Total_Angle;
        Learn3_Data[Learn3_Tick][2] = Gimbal_Control_Struct.Pitch2.Total_Angle;
        Learn3_Data[Learn3_Tick][3] = Gimbal_Control_Struct.J4.Total_Angle;
        Learn3_Data[Learn3_Tick][4] = Gimbal_Control_Struct.J5.Total_Angle;
        Learn3_Data[Learn3_Tick][5] = Gimbal_Control_Struct.J6.Total_Angle;
        Learn3_Data[Learn3_Tick][6] = Gimbal_Control_Struct.J7.Total_Angle;
        Learn3_Data[Learn3_Tick][7] = Gimbal_Control_Struct.Clap.Total_Angle;
        Learn3_Tick++;
        if (Learn3_Tick >= 600) {
          RoboControl_Struct.Learn_State = 0;
        }
      }
    }

    float PlayDt = DWT_GetDeltaT(&Play_DWT_Count);
    if (RoboControl_Struct.Play_State == 0) {
      Play1_Tick = 0;
      Play2_Tick = 0;
      Play3_Tick = 0;
    } else if (RoboControl_Struct.Play_State == 1) {
      RoboControl_Struct.Robo_Target_Yaw =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][0] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][0];
      RoboControl_Struct.Robo_Target_Pitch1 =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][1] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][1];
      RoboControl_Struct.Robo_Target_Pitch2 =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][2] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][2];
      RoboControl_Struct.Robo_Target_J4 =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][3] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][3];
      RoboControl_Struct.Robo_Target_J5 =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][4] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][4];
      RoboControl_Struct.Robo_Target_J6 =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][5] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][5];
      RoboControl_Struct.Robo_Target_J7 =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][6] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][6];
      RoboControl_Struct.Robo_Target_Clap =
          (1.0f - (Play1_Tick - (uint16_t)Play1_Tick)) *
              Learn1_Data[(uint16_t)Play1_Tick][7] +
          (Play1_Tick - (uint16_t)Play1_Tick) *
              Learn1_Data[(uint16_t)Play1_Tick + 1][7];
      Play1_Tick += 10.0f * PlayDt;
      if (Play1_Tick > Learn1_Tick_end - 1) {
        RoboControl_Struct.Play_State = 0;
      }
    } else if (RoboControl_Struct.Play_State == 2) {
      RoboControl_Struct.Robo_Target_Yaw =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][0] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][0];
      RoboControl_Struct.Robo_Target_Pitch1 =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][1] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][1];
      RoboControl_Struct.Robo_Target_Pitch2 =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][2] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][2];
      RoboControl_Struct.Robo_Target_J4 =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][3] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][3];
      RoboControl_Struct.Robo_Target_J5 =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][4] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][4];
      RoboControl_Struct.Robo_Target_J6 =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][5] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][5];
      RoboControl_Struct.Robo_Target_J7 =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][6] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][6];
      RoboControl_Struct.Robo_Target_Clap =
          (1.0f - (Play2_Tick - (uint16_t)Play2_Tick)) *
              Learn2_Data[(uint16_t)Play2_Tick][7] +
          (Play2_Tick - (uint16_t)Play2_Tick) *
              Learn2_Data[(uint16_t)Play2_Tick + 1][7];
      Play2_Tick += 10.0f * PlayDt;
      if (Play2_Tick > Learn2_Tick_end - 1) {
        RoboControl_Struct.Play_State = 0;
      }
    } else if (RoboControl_Struct.Play_State == 3) {
      RoboControl_Struct.Robo_Target_Yaw =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][0] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][0];
      RoboControl_Struct.Robo_Target_Pitch1 =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][1] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][1];
      RoboControl_Struct.Robo_Target_Pitch2 =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][2] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][2];
      RoboControl_Struct.Robo_Target_J4 =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][3] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][3];
      RoboControl_Struct.Robo_Target_J5 =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][4] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][4];
      RoboControl_Struct.Robo_Target_J6 =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][5] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][5];
      RoboControl_Struct.Robo_Target_J7 =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][6] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][6];
      RoboControl_Struct.Robo_Target_Clap =
          (1.0f - (Play3_Tick - (uint16_t)Play3_Tick)) *
              Learn3_Data[(uint16_t)Play3_Tick][7] +
          (Play3_Tick - (uint16_t)Play3_Tick) *
              Learn3_Data[(uint16_t)Play3_Tick + 1][7];
      Play3_Tick += 10.0f * PlayDt;
      if (Play3_Tick > Learn3_Tick_end - 1) {
        RoboControl_Struct.Play_State = 0;
      }
    }

   osDelay(1);
 }
}
 */