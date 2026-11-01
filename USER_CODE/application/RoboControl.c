//====================================================================================
//                           RoboControl[机器状态控制]
//      根据操控意愿把遥控器数据处理成机器人
//      整体[RoboEnable,Vx,Vy,Wz,Yaw,Pitch等],各个模块[Gimbal,Shoot,Chassis等]的状态,
//      各个模块再根据状态执行相应的功能
//      其中包括状态设置，冲突处理，错误检测与处理等
//
//      机器人状态结构体：RoboControl_Struct
//====================================================================================

#include "RoboControl.h"
#include "Aim.h"
#include "Buzzer_init.h"
#include "INS.h"
#include "Remote_Control.h"
#include "SuperCap_Driver.h"
#include "Motor_DM_read.h"
#include "math.h"
#include "motor_init.h"

uint16_t robo_stop = 0;

/*===| 机器人整体状态数据结构体 |===*/
RoboControl_StructTypeDef RoboControl_Struct = {0};

void RoboControl_UpdateSuperCapFeedback(uint8_t *data) {
  SuperCap_Unpack_Data(data);
  RoboControl_Struct.SuperCap_V =
      SuperCap_FeedBack_Data.SuperCap_V_x100 / 100.0f;
  RoboControl_Struct.Referee_P = SuperCap_FeedBack_Data.Referee_P_x10 / 10.0f;
  RoboControl_Struct.Chassis_P = SuperCap_FeedBack_Data.Chassis_P_x10 / 10.0f;
}

/*===| 计算Wz的PID结构体 |===*/
PID_Struct_TypeDef RoboGimbal_Wz_PID_Struct = {0};

/*===| 自定义控制器数据 |===*/
typedef __PACKED_STRUCT {
  int32_t Rotary_Position;
  float Joy_X;
  float Joy_Y;
  int16_t Angle[7];
  uint8_t K1;
  uint8_t K2;
  uint8_t Encoder_Button;
  uint8_t Joy_Button;
}
CustomerData_StrDef;
CustomerData_StrDef CustomerData;
CustomerData_StrDef CustomerData_Last;

uint32_t Robo_Task_DWT_Count;
float Robo_Task_Dt;

/* Remote is updated directly by the legacy UART idle receive callback. */
void Remote_Rx_Task(void *argument) {
  /*===| 默认模块状态 |===*/
  RoboControl_Struct.Chassis_State = Chassis_STATIC;
  RoboControl_Struct.Shoot_State = Shoot_State_Off;
  RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
  RoboControl_Struct.Chassis_Speed_Level = 3;
  RoboControl_Struct.SuperCap_State = 0;
  RoboControl_Struct.En_MODE_Flag_2 = 0;
  RoboControl_Struct.En_MODE_Flag_3 = 0;
  RoboControl_Struct.number = 50;
  RoboControl_Struct.Robo_Target_Camera_X = 0;
  //	RoboControl_Struct.Robo_Target_Camera_Y = 24;
  Referee_Data_Init();
  for (;;) {
    Robo_Task_Dt = DWT_GetDeltaT(&Robo_Task_DWT_Count);
    Gimbal_Control_Struct.J5.dev->data.Total_Angle_4340 =
        GIMBAL_J5_LEGACY_ANGLE(Gimbal_Control_Struct.J5.dev->data.Total_Angle) *
        4.0f;

    /*===| 如果遥控器连接 |===*/
    if (Remote.If_Remote_Connect) {
      /*===| 选择控制模式：摇杆|键鼠|自定义控制器 |===*/
      if (Remote.Mode != Remote_Mode_S &&
          RoboControl_Struct.Controler != Customer)
        RoboControl_Struct.Controler = Joystick;
      else if (Remote.Mode == Remote_Mode_S &&
               RoboControl_Struct.Controler != Customer)
        RoboControl_Struct.Controler = KeyboardMouse;

      /*===| 判断是否停机 |===*/
      if (Remote.Mode == Remote_Mode_C && RoboControl_Struct.Robo_Enable == 1)
        Robo_Stop();
      else if (Remote.Mode != Remote_Mode_C &&
               RoboControl_Struct.Robo_Enable == 0)
        Robo_Restart();
    } else {
      Robo_Stop();
    }

    /*===| 遥控方式 |===*/
    /*===| 遥控器摇杆控制 |===*/
    if (RoboControl_Struct.Controler == Joystick) {
      RemoteControl_Float();
      if (Remote.If_Remote_Data_New == 1 &&
          Remote_Last.If_Remote_Data_New == 1) {
        if (Remote_ReleaseSingle_Pause)
          RoboControl_Struct.Controler = Customer;
        RemoteControl_Bool();
        Remote.If_Remote_Data_New = 0;
        Remote_Last.If_Remote_Data_New = 0;
        RoboControl_Struct.En_MODE_Flag_2 = 0;
        RoboControl_Struct.En_MODE_Flag_3 = 0;
      }
    }
    /*===| 遥控器键鼠控制 |===*/
    else if (RoboControl_Struct.Controler == KeyboardMouse) {
      KeyControl_Float();
      if (Remote.If_Remote_Data_New == 1 &&
          Remote_Last.If_Remote_Data_New == 1) {
        if (Remote_ReleaseSingle_B)
          RoboControl_Struct.Controler = Customer;
        KeyControl_Bool();
        Remote.If_Remote_Data_New = 0;
        Remote_Last.If_Remote_Data_New = 0;
        RoboControl_Struct.En_MODE_Flag_2 = 0;
        RoboControl_Struct.En_MODE_Flag_3 = 0;
      }
    }
    /*===| 自定义控制器控制 |===*/
    else if (RoboControl_Struct.Controler == Customer) {
      if (Remote.Mode == Remote_Mode_N)
        RemoteControl_Float();
      if (Remote.Mode == Remote_Mode_S)
        KeyControl_Float();
      if (Remote.If_Remote_Data_New == 1 &&
          Remote_Last.If_Remote_Data_New == 1) {
        if (Remote_ReleaseSingle_Pause)
          RoboControl_Struct.Controler = Joystick;
        if (Remote_ReleaseSingle_B)
          RoboControl_Struct.Controler = KeyboardMouse;
        Remote.If_Remote_Data_New = 0;
        Remote_Last.If_Remote_Data_New = 0;
      }

      memcpy(&CustomerData, Customer_Controller_Struct.Customer_Controller_Data,
             30);

      RoboControl_Struct.Robo_Target_Yaw =
          0.01f * (float)CustomerData.Angle[0] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[0];
      RoboControl_Struct.Robo_Target_Pitch1 =
          0.01f * (float)CustomerData.Angle[1] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[1];
      RoboControl_Struct.Robo_Target_Pitch2 =
          0.01f * (float)CustomerData.Angle[2] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[2];
      RoboControl_Struct.Robo_Target_J4 =
          0.01f * (float)CustomerData.Angle[3] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[3];
      RoboControl_Struct.Robo_Target_J5 =
          0.01f * (float)CustomerData.Angle[4] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[4];
      RoboControl_Struct.Robo_Target_J6 =
          0.01f * (float)CustomerData.Angle[5] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[5];
      RoboControl_Struct.Robo_Target_J7 =
          0.01f * (float)CustomerData.Angle[6] / 100.0f +
          (1 - 0.01f) * RoboControl_Struct.CustomerData_Last_Angle[6];

      RoboControl_Struct.CustomerData_Last_Angle[0] =
          RoboControl_Struct.Robo_Target_Yaw;
      RoboControl_Struct.CustomerData_Last_Angle[1] =
          RoboControl_Struct.Robo_Target_Pitch1;
      RoboControl_Struct.CustomerData_Last_Angle[2] =
          RoboControl_Struct.Robo_Target_Pitch2;
      RoboControl_Struct.CustomerData_Last_Angle[3] =
          RoboControl_Struct.Robo_Target_J4;
      RoboControl_Struct.CustomerData_Last_Angle[4] =
          RoboControl_Struct.Robo_Target_J5;
      RoboControl_Struct.CustomerData_Last_Angle[5] =
          RoboControl_Struct.Robo_Target_J6;
      RoboControl_Struct.CustomerData_Last_Angle[6] =
          RoboControl_Struct.Robo_Target_J7;

      if (CustomerData.Encoder_Button == 0) //旋转编码器控制底盘左右
      {
        RoboControl_Struct.En_MODE_Flag = 0;
        RoboControl_Struct.En_MODE_Flag_3 = 0;

        //				if(CustomerData_Last.Rotary_Position -
        // CustomerData.Rotary_Position>=4 && RoboControl_Struct.En_MODE_Flag_2
        //==0)
        //				{
        //					CustomerData_Last.Rotary_Position
        //= CustomerData.Rotary_Position;
        // RoboControl_Struct.En_MODE_Flag_2 = 1;
        //				}
        //				else
        // if(CustomerData_Last.Rotary_Position
        //- CustomerData.Rotary_Position<=-4 &&
        // RoboControl_Struct.En_MODE_Flag_2
        //==0)
        //				{
        //					CustomerData_Last.Rotary_Position
        //= CustomerData.Rotary_Position;
        // RoboControl_Struct.En_MODE_Flag_2 = -1;
        //				}
        //				else
        // if(CustomerData_Last.Rotary_Position
        //- CustomerData.Rotary_Position<=-4 &&
        // RoboControl_Struct.En_MODE_Flag_2
        //==1)
        //				{
        //					CustomerData_Last.Rotary_Position
        //= CustomerData.Rotary_Position;
        // RoboControl_Struct.En_MODE_Flag_2 = 0;
        //				}
        //				else
        // if(CustomerData_Last.Rotary_Position
        //- CustomerData.Rotary_Position>=4 && RoboControl_Struct.En_MODE_Flag_2
        //==-1)
        //				{
        //					CustomerData_Last.Rotary_Position
        //= CustomerData.Rotary_Position;
        // RoboControl_Struct.En_MODE_Flag_2 = 0;
        //				}
        //				RoboControl_Struct.Robo_Target_Camera_Y
        //+= CustomerData.Rotary_Position;
      } else if (CustomerData.Encoder_Button == 1) {
        RoboControl_Struct.En_MODE_Flag = 1;
        //				RoboControl_Struct.En_MODE_Flag_2 = 0;

        if (CustomerData_Last.Rotary_Position - CustomerData.Rotary_Position >=
                5 &&
            RoboControl_Struct.En_MODE_Flag_3 == 0) {
          CustomerData_Last.Rotary_Position = CustomerData.Rotary_Position;
          RoboControl_Struct.En_MODE_Flag_3 = 1;
        } else if (CustomerData_Last.Rotary_Position -
                           CustomerData.Rotary_Position <=
                       -5 &&
                   RoboControl_Struct.En_MODE_Flag_3 == 0) {
          CustomerData_Last.Rotary_Position = CustomerData.Rotary_Position;
          RoboControl_Struct.En_MODE_Flag_3 = -1;
        } else if (CustomerData_Last.Rotary_Position -
                           CustomerData.Rotary_Position <=
                       -5 &&
                   RoboControl_Struct.En_MODE_Flag_3 == 1) {
          CustomerData_Last.Rotary_Position = CustomerData.Rotary_Position;
          RoboControl_Struct.En_MODE_Flag_3 = 0;
        } else if (CustomerData_Last.Rotary_Position -
                           CustomerData.Rotary_Position >=
                       5 &&
                   RoboControl_Struct.En_MODE_Flag_3 == -1) {
          CustomerData_Last.Rotary_Position = CustomerData.Rotary_Position;
          RoboControl_Struct.En_MODE_Flag_3 = 0;
        }
        //				RoboControl_Struct.Robo_Target_Camera_X
        //+= CustomerData.Rotary_Position;
      }

      if (CustomerData.Joy_Button == 1) {
        if (RoboControl_Struct.Joy_MODE_Flag == 1) {
          RoboControl_Struct.Robo_Target_Camera_X = 0;
          //					RoboControl_Struct.Robo_Target_Camera_Y
          //= 24;
          RoboControl_Struct.Joy_MODE_Flag = 0;
        } else if (RoboControl_Struct.Joy_MODE_Flag == 0) {
          RoboControl_Struct.Robo_Target_Camera_X -=
              Robo_Task_Dt * 100.0f * CustomerData.Joy_X;
          //					RoboControl_Struct.Robo_Target_Camera_Y
          //+= Robo_Task_Dt * 100.0f * CustomerData.Joy_Y;
        }
      } else if (CustomerData.Joy_Button == 0) {
        RoboControl_Struct.Joy_MODE_Flag = 1;
      }
      RoboControl_Struct.Robo_Target_Pitch1 *= 2;
      if (CustomerData.K1 == 0)
        RoboControl_Struct.Robo_Target_Clap = 65;
      if (CustomerData.K2 == 0)
        RoboControl_Struct.Robo_Target_Clap = 0;
    }

    /*===| 控制参数限幅 |===*/
    Limit_float(&RoboControl_Struct.Robo_Target_Yaw, 250, -250);
    Limit_float(&RoboControl_Struct.Robo_Target_Pitch1, -10, -190);
    Limit_float(&RoboControl_Struct.Robo_Target_Pitch2, 0, -92);
    Limit_float(&RoboControl_Struct.Robo_Target_J4, 130, -130);
    Limit_float(&RoboControl_Struct.Robo_Target_J5, 90, -90);
    Limit_float(&RoboControl_Struct.Robo_Target_J6, 75, -75);
    Limit_float(&RoboControl_Struct.Robo_Target_J7, 80, -80);
    Limit_float(&RoboControl_Struct.Robo_Target_Clap, 65, -4);
    Limit_float(&RoboControl_Struct.Robo_Target_Camera_Y, 100, 3);
    Limit_float(&RoboControl_Struct.Robo_Target_Camera_X, 180, -180);

    Limit_float(&RoboControl_Struct.Robo_Target_Leg_Angle, -1, -41);
    /*===| 根据底盘运动状态得到Wz |===*/
    Get_Chassis_Wz();

    /*===| 控制参数低通滤波 |===*/
    Control_Filter();

    /*===| 平滑启停 |===*/
    if (RoboControl_Struct.Robo_Enable)
      RoboControl_Struct.Smooth_Start_K += 0.00033333f;
    else
      RoboControl_Struct.Smooth_Start_K -= 0.001f;
    Limit_float(&RoboControl_Struct.Smooth_Start_K, 1, 0);

    osDelay(1);
  }
}

/**
 * @brief 根据底盘运动状态得到Wz
 */
void Get_Chassis_Wz(void) {
  /*===| 底盘关闭 |===*/
  //    if(RoboControl_Struct.Chassis_State == Chassis_OFF)
  //    {
  //        RoboControl_Struct.Robo_Target_Wz = 0;
  //    }
  //    /*===| 底盘静止 |===*/
  //    else if(RoboControl_Struct.Chassis_State == Chassis_STATIC)
  //    {
  //        RoboControl_Struct.Robo_Target_Wz = 0;
  //    }
  //    /*===| 底盘跟随模式 |===*/
  //    else if(RoboControl_Struct.Chassis_State == Chassis_FOLLOW)
  //    {
  //        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0,
  //        RoboControl_Struct.Yaw_Err); RoboControl_Struct.Robo_Target_Wz =
  //        RoboGimbal_Wz_PID_Struct.Output;
  //    }
  //    /*===| 底盘侧向跟随模式 |===*/
  //    else if(RoboControl_Struct.Chassis_State == Chassis_F_SIDE)
  //    {
  //        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 2.0f *
  //        (RoboControl_Struct.Chassis_Follow_45_Direction_Flag - 0.5f) * 45,
  //        RoboControl_Struct.Yaw_Err); RoboControl_Struct.Robo_Target_Wz =
  //        RoboGimbal_Wz_PID_Struct.Output;
  //    }
  //    /*===| 底盘小陀螺 |===*/
  //    else if(RoboControl_Struct.Chassis_State == Chassis_SPIN)
  //    {
  //        /*===| 小陀螺得到Wz,每次切换正反转 |===*/
  //        if(RoboControl_Struct.SPIN_Direction_Flag)
  //        {
  //            RoboControl_Struct.Robo_Target_Wz = Robo_Wz_MaxSpeed;
  //        }
  //        else
  //        {
  //            RoboControl_Struct.Robo_Target_Wz = -Robo_Wz_MaxSpeed;
  //        }
  //        /*===| 根据档位限制旋转速度，遥控器控制状态不限制 |===*/
  //        RoboControl_Struct.Robo_Target_Wz =
  //        RoboControl_Struct.Robo_Target_Wz * (1.0f +
  //        RoboControl_Struct.Chassis_Speed_Level) / 6.0f;
  //
  //        Gimbal_Control_Struct.Yaw.dev->data.Round = 0;
  //    }
  //    /*===| 冲刺模式 |===*/
  //    else if(RoboControl_Struct.Chassis_State == Chassis_DASH)
  //    {
  //        /*===| 如果底盘没有回正，则等待底盘回到正方向 |===*/
  //        if(RoboControl_Struct.Yaw_Err > 5.0f || RoboControl_Struct.Yaw_Err <
  //        -5.0f)
  //        {
  //            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
  //            PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0,
  //            RoboControl_Struct.Yaw_Err); RoboControl_Struct.Robo_Target_Wz =
  //            RoboGimbal_Wz_PID_Struct.Output;
  //        }
  //        else
  //        {
  //            RoboControl_Struct.Chassis_State = Chassis_DASH;
  //            RoboControl_Struct.Robo_Target_Wz = 0;
  //        }
  //    }
}

/**
 * @brief 输入参数低通滤波
 */
void Control_Filter(void) {
  RoboControl_Struct.Robo_Target_Yaw =
      0.3f * RoboControl_Struct.Robo_Target_Yaw +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_Yaw_Last;
  RoboControl_Struct.Robo_Target_Pitch1 =
      0.3f * RoboControl_Struct.Robo_Target_Pitch1 +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_Pitch1_Last;
  RoboControl_Struct.Robo_Target_Pitch2 =
      0.3f * RoboControl_Struct.Robo_Target_Pitch2 +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_Pitch2_Last;
  RoboControl_Struct.Robo_Target_J4 =
      0.3f * RoboControl_Struct.Robo_Target_J4 +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_J4_Last;
  RoboControl_Struct.Robo_Target_J5 =
      0.3f * RoboControl_Struct.Robo_Target_J5 +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_J5_Last;
  RoboControl_Struct.Robo_Target_J6 =
      0.3f * RoboControl_Struct.Robo_Target_J6 +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_J6_Last;
  RoboControl_Struct.Robo_Target_J7 =
      0.3f * RoboControl_Struct.Robo_Target_J7 +
      (1 - 0.3f) * RoboControl_Struct.Robo_Target_J7_Last;

  RoboControl_Struct.Robo_Target_Yaw_Last = RoboControl_Struct.Robo_Target_Yaw;
  RoboControl_Struct.Robo_Target_Pitch1_Last =
      RoboControl_Struct.Robo_Target_Pitch1;
  RoboControl_Struct.Robo_Target_Pitch2_Last =
      RoboControl_Struct.Robo_Target_Pitch2;
  RoboControl_Struct.Robo_Target_J4_Last = RoboControl_Struct.Robo_Target_J4;
  RoboControl_Struct.Robo_Target_J5_Last = RoboControl_Struct.Robo_Target_J5;
  RoboControl_Struct.Robo_Target_J6_Last = RoboControl_Struct.Robo_Target_J6;
  RoboControl_Struct.Robo_Target_J7_Last = RoboControl_Struct.Robo_Target_J7;
}

/**
 * @brief 遥控器控制
 */
void RemoteControl_Float(void) {
  if (Remote_Release_Pause) {
    if (Remote_Press_Custom_L && Remote.Wheel > -0.6f && Remote.Wheel < 0.6f) {
      Acc_Slow(0, &RoboControl_Struct.Robo_Target_Yaw, 200.0f, Robo_Task_Dt);
      Acc_Slow(-20, &RoboControl_Struct.Robo_Target_Pitch1, 350.0f,
               Robo_Task_Dt);
      Acc_Slow(-10, &RoboControl_Struct.Robo_Target_Pitch2, 200.0f,
               Robo_Task_Dt);
      Acc_Slow(0, &RoboControl_Struct.Robo_Target_J4, 200.0f, Robo_Task_Dt);
      Acc_Slow(0, &RoboControl_Struct.Robo_Target_J5, 200.0f, Robo_Task_Dt);
      Acc_Slow(0, &RoboControl_Struct.Robo_Target_J6, 200.0f, Robo_Task_Dt);
      Acc_Slow(0, &RoboControl_Struct.Robo_Target_J7, 200.0f, Robo_Task_Dt);
      Acc_Slow(1, &RoboControl_Struct.Robo_Target_Leg_Angle, 35.0f,
               Robo_Task_Dt);
      RoboControl_Struct.Robo_Target_Camera_X = 0;
      RoboControl_Struct.Robo_Target_Camera_Y = 24;
    }
    if (Remote.Wheel > -0.6f && Remote.Wheel < 0.6f) {
      RoboControl_Struct.If_Enable_Leg = 1;
      RoboControl_Struct.Robo_Target_Vx =
          Robo_Vx_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f *
          Remote.Left_X; //+ 0.3*RoboControl_Struct.En_MODE_Flag_2;
      RoboControl_Struct.Robo_Target_Vy =
          Robo_Vy_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f *
          Remote.Left_Y;
      RoboControl_Struct.Robo_Target_Wz =
          Robo_Wz_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f *
              Remote.Right_X +
          0.5 * RoboControl_Struct.En_MODE_Flag_3;
      //			RoboControl_Struct.Robo_Target_Yaw -=
      // Robo_Task_Dt * 200.0f * Remote.Right_X;
      RoboControl_Struct.Robo_Target_Leg_Angle +=
          Robo_Task_Dt * 35 * Remote.Right_Y;
      //			RoboControl_Struct.Robo_Target_Camera_X -=
      // Robo_Task_Dt * 100.0f * Remote.Right_X; 自动上台阶
      if (RoboControl_Struct.auto_up_flag == 1) {
        Acc_Slow(-48, &RoboControl_Struct.Robo_Target_Pitch2, 140.0f,
                 Robo_Task_Dt);
        Acc_Slow(-4, &RoboControl_Struct.Robo_Target_Yaw, 20.0f, Robo_Task_Dt);
        Limit_float(&RoboControl_Struct.Robo_Target_Vy, 0.9, -1.0);
        if (fabsf(INS.atanyz) < 157 && Remote.Left_Y > 0.2f) {
          //							Buzzer_Set_SoundEffect
          //(Buzzer_SoundEffect_Aim_ON);
          Acc_Slow(-185, &RoboControl_Struct.Robo_Target_Pitch1, 350.0f,
                   Robo_Task_Dt);
          Acc_Slow(-85, &RoboControl_Struct.Robo_Target_Pitch2, 150.0f,
                   Robo_Task_Dt);
          RoboControl_Struct.auto_up_flag_2 = 1;
        }
        if (RoboControl_Struct.Robo_Target_Pitch1 < -175 &&
            Remote.Left_Y > 0.2f && RoboControl_Struct.auto_up_flag_2 == 1) {
          Acc_Slow(-41, &RoboControl_Struct.Robo_Target_Leg_Angle, 45.0f,
                   Robo_Task_Dt);
          if (RoboControl_Struct.Robo_Target_Leg_Angle < -40.5) {
            RoboControl_Struct.auto_up_flag_2 = 0;
          }
        }

        if (fabsf(INS.atanyz) > 173 && Remote.Left_Y > 0.2f) {
          //							Buzzer_Set_SoundEffect
          //(Buzzer_SoundEffect_Aim_OFF);
          RoboControl_Struct.auto_up_flag_2 = 0;
          Acc_Slow(-1, &RoboControl_Struct.Robo_Target_Leg_Angle, 100.0f,
                   Robo_Task_Dt);
          Acc_Slow(-20, &RoboControl_Struct.Robo_Target_Pitch1, 100.0f,
                   Robo_Task_Dt);
          Acc_Slow(-10, &RoboControl_Struct.Robo_Target_Pitch2, 50.0f,
                   Robo_Task_Dt);
        }
      }
    } else {
      RoboControl_Struct.Robo_Target_Vx = 0;
      RoboControl_Struct.Robo_Target_Vy = 0;
      RoboControl_Struct.Robo_Target_Wz = 0;
    }

    if (Remote.Wheel < -0.6f) {
      RoboControl_Struct.Robo_Target_Pitch1 -=
          Robo_Task_Dt * 300.0f * (Remote.Left_Y);
      RoboControl_Struct.Robo_Target_Pitch2 -=
          Robo_Task_Dt * 300.0f * Remote.Right_Y;
      RoboControl_Struct.Robo_Target_J4 +=
          Robo_Task_Dt * 200.0f * Remote.Left_X;
      RoboControl_Struct.Robo_Target_J5 +=
          Robo_Task_Dt * 200.0f * Remote.Right_X;
    } else if (Remote.Wheel > 0.6f) {
      RoboControl_Struct.Robo_Target_J6 -=
          Robo_Task_Dt * 200.0f * Remote.Right_Y;
      RoboControl_Struct.Robo_Target_J7 -=
          Robo_Task_Dt * 200.0f * Remote.Right_X;
      //自动存矿
      if (Remote.Left_X > 0.6f &&
          Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle >= -149) {
        Acc_Slow(-168.2, &RoboControl_Struct.Robo_Target_Yaw, 300.0f, 0.001f);
        Acc_Slow(-150.2, &RoboControl_Struct.Robo_Target_Pitch1, 100.0f,
                 0.001f);
        Acc_Slow(-27.0, &RoboControl_Struct.Robo_Target_Pitch2, 50.0f, 0.001f);
        Acc_Slow(36.7, &RoboControl_Struct.Robo_Target_J4, 100.0f, 0.001f);
        Acc_Slow(-54.3, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
        Acc_Slow(-14.8, &RoboControl_Struct.Robo_Target_J6, 100.0f, 0.001f);
        Acc_Slow(-1.9, &RoboControl_Struct.Robo_Target_J7, 100.0f, 0.001f);
      } else if (Remote.Left_X > 0.6f &&
                 Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle < -149) {
        Acc_Slow(-33.8, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
      }

      if (Remote.Left_X < -0.6f &&
          Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle >= -147) {
        Acc_Slow(140.5, &RoboControl_Struct.Robo_Target_Yaw, 300.0f, 0.001f);
        Acc_Slow(-148.4, &RoboControl_Struct.Robo_Target_Pitch1, 100.0f,
                 0.001f);
        Acc_Slow(-23.1, &RoboControl_Struct.Robo_Target_Pitch2, 50.0f, 0.001f);
        Acc_Slow(-3.0, &RoboControl_Struct.Robo_Target_J4, 100.0f, 0.001f);
        Acc_Slow(-55.9, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
        Acc_Slow(-3.1, &RoboControl_Struct.Robo_Target_J6, 100.0f, 0.001f);
        Acc_Slow(15.2, &RoboControl_Struct.Robo_Target_J7, 100.0f, 0.001f);
      } else if (Remote.Left_X < -0.6f &&
                 Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle < -147) {
        Acc_Slow(-19.1, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
      }

      //自动取矿

      if (Remote.Left_Y > 0.6f &&
          Gimbal_Control_Struct.Clap.dev->data.Angle < 35) {
        Acc_Slow(-180.4, &RoboControl_Struct.Robo_Target_Yaw, 300.0f, 0.001f);
        Acc_Slow(-162.7, &RoboControl_Struct.Robo_Target_Pitch1, 100.0f,
                 0.001f);
        Acc_Slow(-12.2, &RoboControl_Struct.Robo_Target_Pitch2, 50.0f, 0.001f);
        Acc_Slow(39.5, &RoboControl_Struct.Robo_Target_J4, 100.0f, 0.001f);
        Acc_Slow(-60, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
        Acc_Slow(-10.3, &RoboControl_Struct.Robo_Target_J6, 100.0f, 0.001f);
        Acc_Slow(-1.9, &RoboControl_Struct.Robo_Target_J7, 100.0f, 0.001f);

      } else if (Remote.Left_Y > 0.6f &&
                 Gimbal_Control_Struct.Clap.dev->data.Angle > 35) {
        Acc_Slow(-90, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
      }

      if (Remote.Left_Y < -0.6f &&
          Gimbal_Control_Struct.Clap.dev->data.Angle < 35) {
        Acc_Slow(137.1, &RoboControl_Struct.Robo_Target_Yaw, 500.0f, 0.001f);
        Acc_Slow(-156.6, &RoboControl_Struct.Robo_Target_Pitch1, 200.0f,
                 0.001f);
        Acc_Slow(-15.5, &RoboControl_Struct.Robo_Target_Pitch2, 30.0f, 0.001f);
        Acc_Slow(11.8, &RoboControl_Struct.Robo_Target_J4, 110.0f, 0.001f);
        Acc_Slow(-40.0, &RoboControl_Struct.Robo_Target_J5, 40.0f, 0.001f);
        Acc_Slow(-3.1, &RoboControl_Struct.Robo_Target_J6, 110.0f, 0.001f);
        Acc_Slow(27.0, &RoboControl_Struct.Robo_Target_J7, 120.0f, 0.001f);
      } else if (Remote.Left_Y < -0.6f &&
                 Gimbal_Control_Struct.Clap.dev->data.Angle > 35) {
        Acc_Slow(-85, &RoboControl_Struct.Robo_Target_J5, 100.0f, 0.001f);
      }

      if (Remote_Press_Custom_L) {
        if (Gimbal_Control_Struct.Clap.dev->data.Angle > 35) {
          Acc_Slow(0, &RoboControl_Struct.Robo_Target_Yaw, 200.0f,
                   Robo_Task_Dt);
          Acc_Slow(-20, &RoboControl_Struct.Robo_Target_Pitch1, 350.0f,
                   Robo_Task_Dt);
          Acc_Slow(-10, &RoboControl_Struct.Robo_Target_Pitch2, 200.0f,
                   Robo_Task_Dt);
          Acc_Slow(88.5, &RoboControl_Struct.Robo_Target_J4, 200.0f,
                   Robo_Task_Dt);
          Acc_Slow(0, &RoboControl_Struct.Robo_Target_J5, 200.0f, Robo_Task_Dt);
          Acc_Slow(78.7, &RoboControl_Struct.Robo_Target_J6, 150.0f,
                   Robo_Task_Dt);
          Acc_Slow(81.0, &RoboControl_Struct.Robo_Target_J7, 200.0f,
                   Robo_Task_Dt);
        }
        if (RoboControl_Struct.Robo_Target_J6 > 78) {
          RoboControl_Struct.Robo_Target_Clap = 0;
          RoboControl_Struct.reset_flag = 1;
        }
      } else if (Remote_Release_Custom_L &&
                 RoboControl_Struct.reset_flag == 1) {
        RoboControl_Struct.Robo_Target_Clap = 65;
        RoboControl_Struct.reset_flag = 0;
      }
    }
  }
}

void RemoteControl_Bool(void) {
  /*===| 单击左键重置关节 |===*/
  //    if(Remote_Press_Custom_L)
  //    {
  //				Acc_Slow(0,
  //&RoboControl_Struct.Robo_Target_Yaw,9220.0f,Robo_Task_Dt);
  // Acc_Slow(-10, &RoboControl_Struct.Robo_Target_Pitch1,9220.0f,Robo_Task_Dt);
  // Acc_Slow(0, &RoboControl_Struct.Robo_Target_Pitch2,9220.0f,Robo_Task_Dt);
  // Acc_Slow(0, &RoboControl_Struct.Robo_Target_J4,9220.0f,Robo_Task_Dt);
  // Acc_Slow(0, &RoboControl_Struct.Robo_Target_J5,9220.0f,Robo_Task_Dt);
  // Acc_Slow(0, &RoboControl_Struct.Robo_Target_J6,9220.0f,Robo_Task_Dt);
  // Acc_Slow(0, &RoboControl_Struct.Robo_Target_J7,9220.0f,Robo_Task_Dt);

  //        RoboControl_Struct.Robo_Target_Yaw = 0;
  //        RoboControl_Struct.Robo_Target_Pitch1 = -10;
  //        RoboControl_Struct.Robo_Target_Pitch2 = 0;
  //        RoboControl_Struct.Robo_Target_J4 = 0;
  //        RoboControl_Struct.Robo_Target_J5 = 0;
  //        RoboControl_Struct.Robo_Target_J6 = 0;
  //        RoboControl_Struct.Robo_Target_J7 = 0;
  //    }

  if (Remote_PressSingle_Trigger) {
    if (Gimbal_Control_Struct.Clap.dev->data.Total_Angle > 15)
      RoboControl_Struct.Robo_Target_Clap = -4;
    else
      RoboControl_Struct.Robo_Target_Clap = 65;
  }

  /*===| 按住Pause并控制摇杆进入组合功能 |===*/
  if (Remote_Press_Pause) {
    /*===| 自定义功能-Pause+右按键 设置零点|===*/
    if (Remote_PressSingle_Custom_R) {
      RoboControl_Struct.IF_Seting_Zero = 1;
    }

    /*===| 自定义功能-左摇杆向右 |===*/
    if (Remote.Left_X > 0.8f && Remote_Last.Left_X < 0.8f) {
      /*===| 底盘跟随模式 |===*/
      RoboControl_Struct.Chassis_State = Chassis_STATIC;
    }
    /*===| 自定义功能-左摇杆向左 |===*/
    else if (Remote.Left_X < -0.8f && Remote_Last.Left_X > -0.8f) {
      /*===| 底盘静止模式 |===*/
      RoboControl_Struct.Chassis_State = Chassis_STATIC;
    }
    /*===| 自定义功能-左摇杆向上 |===*/
    if (Remote.Left_Y > 0.8f && Remote_Last.Left_Y < 0.8f) {
      RoboControl_Struct.Refresh_UI_Flag = 1;
    }
    /*===| 自定义功能-左摇杆向下 |===*/
    else if (Remote.Left_Y < -0.8f && Remote_Last.Left_Y > -0.8f) {
      RoboControl_Struct.Refresh_UI_Flag = 0;
    }
    /*===| 自定义功能-右摇杆向右 |===*/
    if (Remote.Right_X > 0.8f && Remote_Last.Right_X < 0.8f) {
      /*===| 速度等级+1 |===*/
      RoboControl_Struct.Chassis_Speed_Level += 1;
      if (RoboControl_Struct.Chassis_Speed_Level > 5)
        RoboControl_Struct.Chassis_Speed_Level = 5;
    }
    /*===| 自定义功能-右摇杆向左 |===*/
    else if (Remote.Right_X < -0.8f && Remote_Last.Right_X > -0.8f) {
      /*===| 速度等级-1 |===*/
      RoboControl_Struct.Chassis_Speed_Level -= 1;
      if (RoboControl_Struct.Chassis_Speed_Level < 1)
        RoboControl_Struct.Chassis_Speed_Level = 1;
    }
    /*===| 自定义功能-右摇杆向上 |===*/
    if (Remote.Right_Y > 0.8f && Remote_Last.Right_Y < 0.8f) {
      RoboControl_Struct.If_Enable_Leg = 1;
      //            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_ON);
    }
    /*===| 自定义功能-右摇杆向下 |===*/
    else if (Remote.Right_Y < -0.8f && Remote_Last.Right_Y > -0.8f) {
      RoboControl_Struct.If_Enable_Leg = 1;
      //            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_OFF);
    }
  }

  // static uint8_t Status = 0;
  if (Remote_PressSingle_Custom_R) {
    if (RoboControl_Struct.auto_up_flag == 0) {
      Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_ON);
      RoboControl_Struct.auto_up_flag = 1;
    }

    else
      RoboControl_Struct.auto_up_flag = 0;

    //        Status++;
    //        if(Status > 2)
    //        {
    //            Status = 0;
    //        }
  }

  //    if(Remote_Press_Custom_R)
  //    {
  //        if(Status == 0)
  //        {
  //            RoboControl_Struct.Robo_Target_Pitch1 = -240;
  //            RoboControl_Struct.Robo_Target_Pitch2 = -100;
  //        }
  //        else if(Status == 1)
  //        {
  //            RoboControl_Struct.Robo_Target_Leg_Angle = 1;
  //        }
  //        else if(Status == 2)
  //        {
  //            RoboControl_Struct.Robo_Target_Pitch1 = 0;
  //            RoboControl_Struct.Robo_Target_Pitch2 = 0;
  //        }
  //    }
}

/**
 * @brief 键盘控制
 */
void KeyControl_Float(void) {
  /*===| 底盘和云台运动控制 |===*/
  RoboControl_Struct.Robo_Target_Vx =
      Robo_Vx_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f *
      (Remote.Keyboard_D - Remote.Keyboard_A);
  RoboControl_Struct.Robo_Target_Vy =
      Robo_Vy_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level / 5.0f *
      (Remote.Keyboard_W - Remote.Keyboard_S);
  RoboControl_Struct.Robo_Target_Wz = Robo_Vx_MaxSpeed *
                                      RoboControl_Struct.Chassis_Speed_Level /
                                      5.0f * Remote.Mouse_Vx / 2.0f;
  RoboControl_Struct.Robo_Target_Leg_Angle +=
      Robo_Task_Dt * 90 * (Remote.Keyboard_R - Remote.Keyboard_F);
}
void KeyControl_Bool(void) {

  if (Remote_Press_Ctrl && Remote_Press_Q &&
      Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle >= -147) //向左存矿
  {
    Acc_Slow(140.5, &RoboControl_Struct.Robo_Target_Yaw,
             RoboControl_Struct.number * 200.0f, Robo_Task_Dt);
    Acc_Slow(-148.4, &RoboControl_Struct.Robo_Target_Pitch1,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-23.1, &RoboControl_Struct.Robo_Target_Pitch2,
             RoboControl_Struct.number * 50.0f, Robo_Task_Dt);
    Acc_Slow(-3.0, &RoboControl_Struct.Robo_Target_J4,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-55.9, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-3.1, &RoboControl_Struct.Robo_Target_J6,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(15.2, &RoboControl_Struct.Robo_Target_J7,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  } else if (Remote_Press_Ctrl && Remote_Press_Q &&
             Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle < -147) {
    Acc_Slow(-19.1, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  }

  if (Remote_Press_Ctrl && Remote_Press_E &&
      Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle >= -149) //向右存矿
  {
    Acc_Slow(-168.2, &RoboControl_Struct.Robo_Target_Yaw,
             RoboControl_Struct.number * 200.0f, Robo_Task_Dt);
    Acc_Slow(-150.2, &RoboControl_Struct.Robo_Target_Pitch1,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-27.0, &RoboControl_Struct.Robo_Target_Pitch2,
             RoboControl_Struct.number * 50.0f, Robo_Task_Dt);
    Acc_Slow(36.7, &RoboControl_Struct.Robo_Target_J4,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-54.3, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-14.8, &RoboControl_Struct.Robo_Target_J6,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-1.9, &RoboControl_Struct.Robo_Target_J7,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  } else if (Remote_Press_Ctrl && Remote_Press_E &&
             Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle < -149) {
    Acc_Slow(-33.8, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  }

  if (Remote_Press_Shift && Remote_Press_Q &&
      Gimbal_Control_Struct.Clap.dev->data.Angle < 35) //向左取矿
  {

    Acc_Slow(137.1, &RoboControl_Struct.Robo_Target_Yaw,
             RoboControl_Struct.number * 170.0f, Robo_Task_Dt);
    Acc_Slow(-156.6, &RoboControl_Struct.Robo_Target_Pitch1,
             RoboControl_Struct.number * 110.0f, Robo_Task_Dt);
    Acc_Slow(-15.5, &RoboControl_Struct.Robo_Target_Pitch2,
             RoboControl_Struct.number * 50.0f, Robo_Task_Dt);
    Acc_Slow(11.8, &RoboControl_Struct.Robo_Target_J4,
             RoboControl_Struct.number * 110.0f, Robo_Task_Dt);
    Acc_Slow(-40.0, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-3.1, &RoboControl_Struct.Robo_Target_J6,
             RoboControl_Struct.number * 110.0f, Robo_Task_Dt);
    Acc_Slow(27.0, &RoboControl_Struct.Robo_Target_J7,
             RoboControl_Struct.number * 120.0f, Robo_Task_Dt);
  } else if (Remote_Press_Shift && Remote_Press_Q &&
             Gimbal_Control_Struct.Clap.dev->data.Angle > 35) {
    Acc_Slow(-85, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  }

  if (Remote_Press_Shift && Remote_Press_E &&
      Gimbal_Control_Struct.Clap.dev->data.Angle < 35) //向右取矿
  {
    Acc_Slow(-180.4, &RoboControl_Struct.Robo_Target_Yaw,
             RoboControl_Struct.number * 200.0f, Robo_Task_Dt);
    Acc_Slow(-162.7, &RoboControl_Struct.Robo_Target_Pitch1,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-12.2, &RoboControl_Struct.Robo_Target_Pitch2,
             RoboControl_Struct.number * 50.0f, Robo_Task_Dt);
    Acc_Slow(39.5, &RoboControl_Struct.Robo_Target_J4,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-52.1, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-10.3, &RoboControl_Struct.Robo_Target_J6,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
    Acc_Slow(-1.9, &RoboControl_Struct.Robo_Target_J7,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  } else if (Remote_Press_Shift && Remote_Press_E &&
             Gimbal_Control_Struct.Clap.dev->data.Angle > 35) {
    Acc_Slow(-90, &RoboControl_Struct.Robo_Target_J5,
             RoboControl_Struct.number * 100.0f, Robo_Task_Dt);
  }

  /*===| 按B回到预设位置 |===*/
  if (Remote.Keyboard_B) {
    RoboControl_Struct.Refresh_UI_Flag = 1;
  } else {
    RoboControl_Struct.Refresh_UI_Flag = 0;
  }

  /*===| 按G重启 |===*/
  if (Remote_PressSingle_G) {
    Robo_Restart();
  }
}

/**
 * @brief 关闭机器人
 */
void Robo_Stop(void) {
  RoboControl_Struct.Robo_Enable = 0;

  RoboControl_Struct.Robo_Target_Vx = 0;
  RoboControl_Struct.Robo_Target_Vy = 0;
  RoboControl_Struct.Robo_Target_Wz = 0;
  RoboControl_Struct.Shoot_State = Shoot_State_Off;
  RoboControl_Struct.SuperCap_State = 0;
  RoboControl_Struct.Chassis_State = Chassis_OFF;

  /* 全部控制域失能。本函数只在 Remote_Rx_Task 里调用，遥控断连期间
     每拍都会进来；motor_disable_all 只处理还在使能的设备，不会重复发
     失能帧。别的任务不要直接调用本函数（断链检测只调 Remote_Clear） */
  for (int id = 0; id < (int)MOTOR_GROUP_MAX; id++)
    motor_disable_all((motor_group_id_t)id);

  robo_stop++;
}

/**
 * @brief 重启机器人
 */
void Robo_Restart(void) {
#if MOTOR_DM_READ_MODE
  /* 达妙寄存器读取模式：整机保持失能，遥控拨档也不拉起任何电机
     （见 Motor_DM_read.h） */
  return;
#endif

  /*===| 重启后Pitch回到0，Yaw更新到当前位置 |===*/
  RoboControl_Struct.Robo_Target_Yaw = 0;
  RoboControl_Struct.Robo_Target_Pitch1 = -15;
  RoboControl_Struct.Robo_Target_Pitch2 = -7;
  RoboControl_Struct.Robo_Target_J4 = 0;
  RoboControl_Struct.Robo_Target_J5 = 0;
  RoboControl_Struct.Robo_Target_J6 = 0;
  RoboControl_Struct.Robo_Target_J7 = 0;
  RoboControl_Struct.Robo_Target_Leg_Angle = 0;

  RoboControl_Struct.Robo_Target_Yaw_Last = 0;
  RoboControl_Struct.Robo_Target_Pitch1_Last = -15;
  RoboControl_Struct.Robo_Target_Pitch2_Last = -7;
  RoboControl_Struct.Robo_Target_J4_Last = 0;
  RoboControl_Struct.Robo_Target_J5_Last = 0;
  RoboControl_Struct.Robo_Target_J6_Last = 0;
  RoboControl_Struct.Robo_Target_J7_Last = 0;
  RoboControl_Struct.Robo_Target_Leg_Angle_Last = 0;
  /*===| 云台电机清除错误 |===*/

  /*===| 默认模块状态 |===*/
  RoboControl_Struct.Chassis_State = Chassis_STATIC;
  RoboControl_Struct.Shoot_State = Shoot_State_Off;
  RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
  RoboControl_Struct.Smooth_Start_K = 0;
  RoboControl_Struct.En_MODE_Flag_2 = 0;
  RoboControl_Struct.En_MODE_Flag_3 = 0;

  /* 全部控制域使能（已使能的设备 motor_enable_all 会跳过） */
  for (int id = 0; id < (int)MOTOR_GROUP_MAX; id++)
    motor_enable_all((motor_group_id_t)id);

  RoboControl_Struct.Robo_Enable = 1;
}
