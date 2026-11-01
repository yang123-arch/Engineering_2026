#include "UI_Task.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Motor_Driver.h"
#include "Referee_Unpack.h"
#include "RoboControl.h"


int UI_SELF_ID;
UI_State_StructTypedef UI_State;

void UI_Init(void) {
  ui_init_g_Static();
  ui_init_g_Dynamic();
}

void UI_Update(void) { ui_update_g_Dynamic(); }

void UI_Task(void *argument) {
  (void)argument;
  while (Robo_State.robot_id == 0)
    osDelay(1000);
  UI_SELF_ID = Robo_State.robot_id;

  UI_Init();

  while (1) {
    UI_SELF_ID = Robo_State.robot_id;

    /*===| 如果要求重刷UI，则重新初始化 |===*/
    if (RoboControl_Struct.Refresh_UI_Flag) {
      UI_Init();
    } else {
      /*===| 更新速度档位显示 |===*/
      ui_g_Dynamic_Speed->number = RoboControl_Struct.Chassis_Speed_Level;

      /*===| 更新自动上台阶模式 |===*/
      if (RoboControl_Struct.auto_up_flag) {
        ui_g_Dynamic_Stair_State_Text->number = 1;
        ui_g_Dynamic_Stair_State_Text->color = 2;
      } else {
        ui_g_Dynamic_Stair_State_Text->number = 0;
        ui_g_Dynamic_Stair_State_Text->color = 5;
      }

      /*===| 更新自定义控制器模式 |===*/
      if (RoboControl_Struct.Controler == Customer) {
        ui_g_Dynamic_Cus_State_Text->number = 1;
        ui_g_Dynamic_Cus_State_Text->color = 2;
      } else {
        ui_g_Dynamic_Cus_State_Text->number = 0;
        ui_g_Dynamic_Cus_State_Text->color = 5;
      }
      /*===| 更新旋转编码器是否锁定模式 |===*/
      if (RoboControl_Struct.En_MODE_Flag == 0) {
        ui_g_Dynamic_RE_State_Text->number = 0;
        ui_g_Dynamic_RE_State_Text->color = 5;
      } else if (RoboControl_Struct.En_MODE_Flag == 1) {
        ui_g_Dynamic_RE_State_Text->number = 1;
        ui_g_Dynamic_RE_State_Text->color = 2;
      }
      /*===| 更新力摇杆是否锁定模式 |===*/
      if (RoboControl_Struct.Joy_MODE_Flag == 1) {
        ui_g_Dynamic_Joy_State_Text->number = 0;
        ui_g_Dynamic_Joy_State_Text->color = 5;
      } else if (RoboControl_Struct.Joy_MODE_Flag == 0) {
        ui_g_Dynamic_Joy_State_Text->number = 1;
        ui_g_Dynamic_Joy_State_Text->color = 2;
      }
      /*===| 更新两个Pitch电机的角度 |===*/
      ui_g_Dynamic_Pitch1_Angle->number = (int)-Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle;
      ui_g_Dynamic_Pitch2_Angle->number = (int)-Gimbal_Control_Struct.Pitch2.dev->data.Total_Angle;
      /*===| 更新两个腿电机的角度 |===*/
      ui_g_Dynamic_Leg_Left_Angle->number =
          (int)Chassis.Leg[1].dev->data.Total_Angle;
      ui_g_Dynamic_Leg_Right_Angle->number =
          (int)-Chassis.Leg[0].dev->data.Total_Angle;
      /*===| 更新Vy速度值 |===*/
      ui_g_Dynamic_Vy_Speed->number =
          (int)(RoboControl_Struct.Robo_Target_Vy * 10.0f);

      //			UI_State.Left_Leg_Angle  =
      //-Chassis.Chassis_Leg_1->Total_Angle;
      //            UI_State.Right_Leg_Angle = -Chassis.Chassis_Leg_2->Total_Angle;
      //
      //
      //			/*===| Leg_Attitude |===*/
      //            //up     startpoint : joint;    endpoint : knee
      //            //down    startpoint : foot;    endpoint : knee
      //            //        sinA = 0.84; cosA = 0.55;
      //            //        OK = 99;     KF = 131;     OF =
      //            156;(可改为腿长，但余弦不为定值计算量增大)
      //
      //            /*===| Left_Leg_Attitude |===*/
      //            ui_g_Dynamic_Left_up_Leg->start_x = 1646;
      //            ui_g_Dynamic_Left_up_Leg->start_y = 865;
      //
      //            ui_g_Dynamic_Left_up_Leg->end_x = 1718 + 99 * (0.84 *
      //            cos(UI_State.Left_Leg_Angle * 2.0f*PI/360.0f) - 0.55 *
      //            sin(UI_State.Left_Leg_Angle * 2.0f*PI/360.0f));
      //            ui_g_Dynamic_Left_up_Leg->end_y = 797 + 99 * (0.55 *
      //            cos(UI_State.Left_Leg_Angle * 2.0f*PI/360.0f) - 0.84 *
      //            sin(UI_State.Left_Leg_Angle * 2.0f*PI/360.0f));
      //
      //            ui_g_Dynamic_Left_down_Leg->start_x = 1619 - 156 *
      //            sin(UI_State.Left_Leg_Angle * 2.0f*PI/360.0f);
      //            ui_g_Dynamic_Left_down_Leg->start_y = 710 - 156 *
      //            cos(UI_State.Left_Leg_Angle * 2.0f*PI/360.0f);
      //
      //            ui_g_Dynamic_Left_down_Leg->end_x =
      //            ui_g_Dynamic_Left_up_Leg->end_x;
      //            ui_g_Dynamic_Left_down_Leg->end_y =
      //            ui_g_Dynamic_Left_up_Leg->end_y;
      //
      ////            ui_g_Dynamic_Left_down_Leg->end_x = 1624 + 99 * (0.84 *
      ///cos(UI_State.Joint_Angle * 2.0f*PI/360.0f) - 0.55 *
      ///sin(UI_State.Joint_Angle * 2.0f*PI/360.0f)); /
      ///ui_g_Dynamic_Left_down_Leg->end_y = 807 + 99 * (0.55 *
      ///cos(UI_State.Joint_Angle * 2.0f*PI/360.0f) - 0.84 *
      ///sin(UI_State.Joint_Angle * 2.0f*PI/360.0f));
      //
      //            /*===| Right_Leg_Attitude |===*/
      //            ui_g_Dynamic_Right_up_Leg->start_x = 1648;
      //            ui_g_Dynamic_Right_up_Leg->start_y = 866;
      //
      //            ui_g_Dynamic_Right_up_Leg->end_x = 1731 + 99 * (0.84 *
      //            cos(UI_State.Right_Leg_Angle * 2.0f*PI/360.0f) - 0.55 *
      //            sin(UI_State.Right_Leg_Angle * 2.0f*PI/360.0f));
      //            ui_g_Dynamic_Right_up_Leg->end_y = 798 + 99 * (0.55 *
      //            cos(UI_State.Right_Leg_Angle * 2.0f*PI/360.0f) - 0.84 *
      //            sin(UI_State.Right_Leg_Angle * 2.0f*PI/360.0f));
      //
      //            ui_g_Dynamic_Right_down_Leg->start_x = 1620 - 156 *
      //            sin(UI_State.Right_Leg_Angle * 2.0f*PI/360.0f);
      //            ui_g_Dynamic_Right_down_Leg->start_y = 704 - 156 *
      //            cos(UI_State.Right_Leg_Angle * 2.0f*PI/360.0f);
      //
      //            ui_g_Dynamic_Right_down_Leg->end_x =
      //            ui_g_Dynamic_Right_up_Leg->end_x;
      //            ui_g_Dynamic_Right_down_Leg->end_y =
      //            ui_g_Dynamic_Right_up_Leg->end_y;
      //
      ////            ui_g_Dynamic_Right_down_Leg->end_x = 1708 + 99 * (0.84 *
      ///cos(UI_State.Joint_Angle * 2.0f*PI/360.0f) - 0.55 *
      ///sin(UI_State.Joint_Angle * 2.0f*PI/360.0f)); /
      ///ui_g_Dynamic_Right_down_Leg->end_y = 741 + 99 * (0.55 *
      ///cos(UI_State.Joint_Angle * 2.0f*PI/360.0f) - 0.84 *
      ///sin(UI_State.Joint_Angle * 2.0f*PI/360.0f));

      /*===| 更新UI |===*/
      UI_Update();
    }
  }
}
void String_Printf(char *String_Arr, const char *fmt, ...) {
  static va_list ap;
  va_start(ap, fmt);
  vsprintf((char *)String_Arr, fmt, ap);
  va_end(ap);
}
