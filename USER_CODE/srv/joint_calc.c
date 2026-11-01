//=====================================================================================
//                                    joint_calc[关节控制中间层]
//      说明见 joint_calc.h
//=====================================================================================
#include "joint_calc.h"

void joint_init_position(joint_t *j, float kp, float ki, float kd, float kf,
                         float i_max, float out_max) {
  PID_Init(&j->position_pid, kp, ki, kd, kf, i_max, out_max);
}

void joint_init_speed(joint_t *j, float kp, float ki, float kd, float kf,
                      float i_max, float out_max) {
  PID_Init(&j->speed_pid, kp, ki, kd, kf, i_max, out_max);
}

void joint_reset(joint_t *j) {
  j->position_pid.I_Output = 0;
  j->speed_pid.I_Output = 0;
  j->output = 0;
}

float joint_calc(joint_t *j, float target) {
  if (j == NULL || j->dev == NULL)
    return 0;

  const motor_data_t *data = &j->dev->data;
  float speed_set = target;

  /* 失能时也要算：PID 的积分步长是 DWT 测的两次调用间隔，停算一段再恢复，
     第一拍会按整段间隔积分 */
  if (j->mode == JOINT_MODE_CASCADE) {
    float position =
        (j->position_fb != NULL) ? *j->position_fb : data->Total_Angle;
    PID_Position_Calculate(&j->position_pid, target, position);
    speed_set = j->position_pid.Output;
  }
  PID_Position_Calculate(&j->speed_pid, speed_set, data->Speed_RPM);

  /* 失能期间不清积分，使能那一拍会带着积满的 ±I_Output_Max 冲出去 */
  if (j->dev->enabled == 0U)
    joint_reset(j);
  else
    j->output = j->speed_pid.Output;

  return j->output;
}
