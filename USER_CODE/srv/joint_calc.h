//=====================================================================================
//                                    joint_calc[关节控制中间层]
//
//      把"一个电机 + 它的 PID"绑成 joint_t，上层只填目标值：
//        JOINT_MODE_SPEED   —— 单速度环：target 是速度[RPM]
//        JOINT_MODE_CASCADE —— 位置环 -> 速度环串级：target 是角度[°]
//
//      用法：
//        1. 在应用的结构体里放 joint_t，初始化时填 .dev / .mode
//           （角度反馈不是 dev->data.Total_Angle 的再填 .position_fb）
//        2. 任务开头 joint_init_position / joint_init_speed 填 PID 参数
//        3. 每拍 joint_calc(j, target)，再把 j->output（可加前馈、缩放）
//           交给 motor_set_value(j->dev, ...)
//
//      joint_calc 只计算不下发：前馈、限幅、分频发送都留给应用层
//=====================================================================================
#ifndef __JOINT_CALC_H__
#define __JOINT_CALC_H__

#include "PID.h"
#include "motor_core.h"

/*===| 控制模式 |===*/
typedef enum {
  JOINT_MODE_SPEED = 0, // 单速度环
  JOINT_MODE_CASCADE,   // 位置环 -> 速度环串级
} joint_mode_t;

/**
 * @brief 关节：一个电机设备 + 位置环 + 速度环
 */
typedef struct {
  motor_device_t *dev; // 指向 g_motor_devices[] 里的设备
  joint_mode_t mode;
  /**
   * @brief 位置环反馈（可选）
   * @note  NULL 时用 dev->data.Total_Angle；带零偏/减速比的（相机、J5）
   *        指向应用自己算好的角度
   */
  const float *position_fb;

  PID_Struct_TypeDef position_pid; // 位置环（JOINT_MODE_CASCADE 时用）
  PID_Struct_TypeDef speed_pid;    // 速度环

  float output; // 速度环输出（未加前馈）；设备失能时为 0
} joint_t;

/**
 * @brief 填位置环 / 速度环参数（参数顺序同 PID_Init）
 */
void joint_init_position(joint_t *j, float kp, float ki, float kd, float kf,
                         float i_max, float out_max);
void joint_init_speed(joint_t *j, float kp, float ki, float kd, float kf,
                      float i_max, float out_max);

/**
 * @brief 按 mode 算一拍，结果写入 j->output 并返回
 * @note  设备失能（dev->enabled == 0）时照常算 PID，但清积分、output = 0
 */
float joint_calc(joint_t *j, float target);

/**
 * @brief 清积分、output = 0
 * @note  应用层有 dev->enabled 看不到的"不出力"开关（如 If_Enable_Leg）时，
 *        在 joint_calc 之后调用
 */
void joint_reset(joint_t *j);

#endif /* __JOINT_CALC_H__ */
