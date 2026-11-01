//=====================================================================================
//                    Function[常用函数]
//    Limit(float *Input, float Up, float Down)
//    Ignor(float *Input,  float Up, float Down)
//    Deadline(float *Input,  float Up, float Down)
//=====================================================================================

#include "Function.h"

/**
 * @brief 限幅
 *
 * @param 输入值指针
 * @param 限制范围
 */
void Limit_float(float *Input, float Up, float Down) {
  if (Up < Down)
    return;
  if (*Input > Up)
    *Input = Up;
  else if (*Input < Down)
    *Input = Down;
}
void Limit_int(int *Input, int Up, int Down) {
  if (Up < Down)
    return;
  if (*Input > Up)
    *Input = Up;
  else if (*Input < Down)
    *Input = Down;
}

/**
 * @brief 误差忽略
 *
 * @param 输入值指针
 * @param 忽略上边缘
 * @param 忽略下边缘
 */
void Ignor_float(float *Input, float Up, float Down) {
  if (*Input<Up && * Input> Down)
    *Input = 0;
}

/**
 * @brief 死区限制
 *
 * @param 输入值指针
 * @param 死区上边缘
 * @param 死区下边缘
 */
void Deadline_Float(float *Input, float Up, float Down) {
  if (Up < Down)
    return;
  if (*Input<Up && * Input> Down)
    *Input = 0;
}

/**
 * @brief 死区外从零开始线性映射
 *
 * @param 输入值指针
 * @param 死区边缘
 */
void Deadline_Zero_Float(float *Input, float DeadLine) {
  if (fabs(*Input) < DeadLine)
    *Input = 0;
  else {
    if (*Input > 0)
      *Input = (*Input - DeadLine) / (1.0f - DeadLine);
    else if (*Input < 0)
      *Input = (*Input + DeadLine) / (1.0f - DeadLine);
  }
}

/**
 * @brief 输出缓加速，减速
 *
 * @param 目标
 * @param 输出地址
 * @param 加速度
 * @param 减速度
 * @param 时间差
 */
void Acc_Slow(float Target, float *Output, float Acc, float Dt) {
  if (Target > *Output + Dt * Acc) {
    *Output += Dt * Acc;
  } else if (Target < *Output - Dt * Acc) {
    *Output -= Dt * Acc;
  } else {
    *Output = Target;
  }
}

float Caculate_Distence(float X, float Y) { return (sqrtf(X * X + Y * Y)); }
