//=====================================================================================
//                         chirp_pid_test[关节测试：扫频 / 匀速往返]
//      说明见 chirp_pid_test.h
//=====================================================================================
#include "chirp_pid_test.h"
#include "Function.h"
#include "USB_Driver.h"
#include "motor_init.h"
#include <math.h>
#include <string.h>

#define MOTOR_TEST_2PI 6.28318530717958647692f
#define MOTOR_TEST_DPS_TO_RPM (1.0f / 6.0f) // °/s -> RPM

/* 默认参数。安全区间和 Limit 默认不限（±1e6），按关节另填 */
Motor_Test_StructTypeDef Motor_Test = {
    .Test = MOTOR_TEST_TYPE_SWEEP,
    .Target = 0.0f,
    .Target_From_Enable = 1U,
    .Ramp_Speed = 20.0f,
    .Safe_Min = -1.0e6f,
    .Safe_Max = 1.0e6f,
    .Limit_Min = -1.0e6f,
    .Limit_Max = 1.0e6f,
    .Feedforward = 0.0f,

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

    .Ramp_Speeds = {1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 40.0f, 60.0f},
    .Ramp_Speed_Num = 7U,
    .Ramp_Passes = 4U,
    .Ramp_Pass_Time = 15.0f,
    .Ramp_Dwell = 0.5f,
};

/* USB 发送用两块缓冲轮流写：一块交给 USB 发送时，往另一块里攒帧 */
static uint8_t s_usb_buf[2][MOTOR_TEST_BUF_FRAMES * MOTOR_TEST_FRAME_BYTES];
static uint8_t s_usb_fill; // 正在写的那一块
static uint16_t s_usb_len; // 这一块已写的字节数

/*===|==================== USB 记录 ====================|===*/

/**
 * @brief 按小端写一个 float
 */
static void Motor_Test_Put_Float(uint8_t *p, float f) {
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
 *        之前不能动。返回 OK 说明上一块已经发完，这时换回去写才安全
 */
static void Motor_Test_Send(Motor_Test_StructTypeDef *t,
                            const float ch[MOTOR_TEST_CH]) {
  if (s_usb_len + MOTOR_TEST_FRAME_BYTES <= sizeof(s_usb_buf[0])) {
    uint8_t *p = &s_usb_buf[s_usb_fill][s_usb_len];
    for (uint8_t i = 0; i < MOTOR_TEST_CH; i++)
      Motor_Test_Put_Float(p + 4U * i, ch[i]);
    p[4U * MOTOR_TEST_CH + 0U] = 0x00U; /* JustFloat 帧尾 */
    p[4U * MOTOR_TEST_CH + 1U] = 0x00U;
    p[4U * MOTOR_TEST_CH + 2U] = 0x80U;
    p[4U * MOTOR_TEST_CH + 3U] = 0x7FU;
    s_usb_len += MOTOR_TEST_FRAME_BYTES;
  } else {
    t->Drop++;
  }

  if (s_usb_len > 0U &&
      USB_Send(s_usb_buf[s_usb_fill], (uint8_t)s_usb_len) == 0U) {
    s_usb_fill ^= 1U;
    s_usb_len = 0U;
  }
}

/*===|==================== 匀速往返 ====================|===*/

static float Motor_Test_Clamp(const Motor_Test_StructTypeDef *t, float a) {
  if (a > t->Safe_Max)
    return t->Safe_Max;
  if (a < t->Safe_Min)
    return t->Safe_Min;
  return a;
}

/**
 * @brief 第 i 档的半行程[°]：min(安全区间一半, 速度 × 每趟时间 / 2)
 */
static float Motor_Test_Ramp_Half(const Motor_Test_StructTypeDef *t,
                                  uint8_t i) {
  float half = 0.5f * (t->Safe_Max - t->Safe_Min);
  float h = 0.5f * t->Ramp_Speeds[i] * t->Ramp_Pass_Time;
  return (h < half) ? h : half;
}

static float Motor_Test_Ramp_Center(const Motor_Test_StructTypeDef *t) {
  return 0.5f * (t->Safe_Min + t->Safe_Max);
}

/**
 * @brief 第 i 档离 from 近的那一端
 */
static float Motor_Test_Ramp_Near_End(const Motor_Test_StructTypeDef *t,
                                      uint8_t i, float from) {
  float c = Motor_Test_Ramp_Center(t);
  float h = Motor_Test_Ramp_Half(t, i);
  return (from >= c) ? c + h : c - h;
}

/**
 * @brief 参数检查通过后从第 0 档开始；不合法返回 -1
 */
static int Motor_Test_Ramp_Begin(Motor_Test_StructTypeDef *t, float angle) {
  if (t->Ramp_Speed_Num == 0U || t->Ramp_Speed_Num > MOTOR_TEST_RAMP_SPEED_MAX ||
      t->Ramp_Passes == 0U || !(t->Ramp_Pass_Time > 0.0f) ||
      !(t->Ramp_Speed > 0.0f) || !(t->Safe_Max > t->Safe_Min))
    return -1;
  for (uint8_t i = 0; i < t->Ramp_Speed_Num; i++) {
    if (!(t->Ramp_Speeds[i] > 0.0f))
      return -1;
  }

  t->Ramp_Index = 0U;
  t->Ramp_Pass = 0U;
  t->Ramp_Timer = 0U;
  t->Ramp_Goal = Motor_Test_Ramp_Near_End(t, 0U, angle);
  t->Ramp_Step = MOTOR_TEST_RAMP_MOVE;
  return 0;
}

/**
 * @brief 往返的子阶段切换；返回 1 表示全部走完（已回到 Target）
 * @note  "走到"按斜坡目标 Angle_Set 判断（Acc_Slow 到点时精确等于目标），
 *        不按实际角度，免得带载静差让它永远到不了
 */
static uint8_t Motor_Test_Ramp_Update(Motor_Test_StructTypeDef *t) {
  switch (t->Ramp_Step) {
  case MOTOR_TEST_RAMP_MOVE:
    if (t->Angle_Set == t->Ramp_Goal) {
      t->Ramp_Step = MOTOR_TEST_RAMP_DWELL;
      t->Ramp_Timer = 0U;
    }
    break;

  case MOTOR_TEST_RAMP_DWELL:
    t->Ramp_Timer++;
    if ((float)t->Ramp_Timer * t->Dt < t->Ramp_Dwell)
      break;
    if (t->Ramp_Pass < t->Ramp_Passes) {
      /* 这一档还没走完：走向另一端 */
      float c = Motor_Test_Ramp_Center(t);
      float h = Motor_Test_Ramp_Half(t, t->Ramp_Index);
      t->Ramp_Goal = (t->Angle_Set >= c) ? c - h : c + h;
      t->Ramp_Step = MOTOR_TEST_RAMP_PASS;
    } else if (t->Ramp_Index + 1U < t->Ramp_Speed_Num) {
      /* 换下一档：行程可能变了，先移到新行程同一侧的端点 */
      t->Ramp_Index++;
      t->Ramp_Pass = 0U;
      t->Ramp_Goal = Motor_Test_Ramp_Near_End(t, t->Ramp_Index, t->Angle_Set);
      t->Ramp_Step = MOTOR_TEST_RAMP_MOVE;
    } else {
      t->Ramp_Goal = Motor_Test_Clamp(t, t->Target);
      t->Ramp_Step = MOTOR_TEST_RAMP_RETURN;
    }
    break;

  case MOTOR_TEST_RAMP_PASS:
    if (t->Angle_Set == t->Ramp_Goal) {
      t->Ramp_Pass++;
      t->Ramp_Step = MOTOR_TEST_RAMP_DWELL;
      t->Ramp_Timer = 0U;
    }
    break;

  default: /* MOTOR_TEST_RAMP_RETURN */
    if (t->Angle_Set == t->Ramp_Goal)
      return 1U;
    break;
  }
  return 0U;
}

/*===|==================== 对外接口 ====================|===*/

int Motor_Test_Init(Motor_Test_StructTypeDef *t, joint_t *joint, float dt) {
  if (t == NULL || joint == NULL || joint->dev == NULL || !(dt > 0.0f))
    return -1;

  t->Joint = joint;
  t->Dt = dt;
  t->Phase = MOTOR_TEST_HOLD;
  t->Hold_Torque = 0.0f;
  t->Sweep_Center = 0.0f;
  t->Angle_Set = 0.0f;
  t->Chirp_Out = 0.0f;
  t->Ctrl_Out = 0.0f;
  t->Total_Out = 0.0f;
  t->Speed = 0.0f;
  t->V_Ref = 0.0f;
  t->Run_Last = 0U;
  t->Req_Last = 0U;
  t->Settle_Cnt = 0U;
  t->Settle_Sum = 0.0f;
  t->Ramp_Step = MOTOR_TEST_RAMP_MOVE;
  t->Ramp_Index = 0U;
  t->Ramp_Pass = 0U;
  t->Ramp_Goal = 0.0f;
  t->Ramp_Timer = 0U;
  Chirp_Stop(&t->Chirp);
  return 0;
}

void Motor_Test_Disable_Others(const motor_device_t *keep) {
  for (uint16_t i = 0; i < g_motor_device_count; i++) {
    motor_device_t *dev = &g_motor_devices[i];
    if (dev != keep && dev->enabled != 0U)
      motor_disable(dev);
  }
}

void Motor_Test_Step(Motor_Test_StructTypeDef *t, uint8_t req) {
  if (t == NULL || t->Joint == NULL || t->Joint->dev == NULL)
    return;

  joint_t *j = t->Joint;
  motor_device_t *dev = j->dev;

  /*===| 反馈快照：PID 计算和发送用同一份 |===*/
  float angle =
      (j->position_fb != NULL) ? *j->position_fb : dev->data.Total_Angle;
  t->Speed = dev->data.Speed_RPM;

  /* 只看 dev->enabled：使能后到电机回"已使能"之间驱动照发 0 力矩，
     这几拍 PID 照算（Ki ≠ 0 时会积一点分，同 joint_calc） */
  uint8_t run = (dev->enabled != 0U) ? 1U : 0U;
  uint8_t start = (run == 0U || t->Run_Last == 0U) ? 1U : 0U;

  /*===| 使能那一拍：目标从当前位置起步 |===*/
  if (start != 0U) {
    t->Angle_Set = angle;
    if (run != 0U && t->Target_From_Enable != 0U)
      t->Target = Motor_Test_Clamp(t, angle);
    j->position_pid.I_Output = 0;
    j->speed_pid.I_Output = 0;
  }

  /*===| 阶段切换 |===*/
  if (t->Phase == MOTOR_TEST_FAULT) {
    /* 超限后不出力，等 req 拉回 0 再从当前位置重新开始 */
    if (req == 0U) {
      t->Phase = MOTOR_TEST_HOLD;
      t->Angle_Set = angle;
    }
  } else if (run != 0U && (angle < t->Limit_Min || angle > t->Limit_Max)) {
    Chirp_Stop(&t->Chirp);
    t->Abort++;
    t->Phase = MOTOR_TEST_FAULT;
  } else if (start != 0U || req == 0U) {
    /* 没在控制、刚开始控制、或没请求测试：位置保持，到位计时重来 */
    Chirp_Stop(&t->Chirp);
    t->Phase = MOTOR_TEST_HOLD;
    t->Settle_Cnt = 0U;
    t->Settle_Sum = 0.0f;
  } else if (t->Phase == MOTOR_TEST_HOLD && t->Test == MOTOR_TEST_TYPE_RAMP) {
    t->Phase = (Motor_Test_Ramp_Begin(t, t->Angle_Set) == 0) ? MOTOR_TEST_RAMP
                                                            : MOTOR_TEST_DONE;
  } else if (t->Phase == MOTOR_TEST_HOLD) {
    /* 扫频：斜坡走完、离 Target 够近、基本静止，连续 Settle_Time 算到位。
       P 环带载会有静差，离得远进不了扫频就调大 Settle_Tol */
    float target = Motor_Test_Clamp(t, t->Target);
    if (t->Angle_Set == target && fabsf(angle - target) <= t->Settle_Tol &&
        fabsf(t->Speed) <= t->Settle_Speed) {
      t->Settle_Cnt++;
      t->Settle_Sum += t->Ctrl_Out; /* 上一拍的输出 */
    } else {
      t->Settle_Cnt = 0U;
      t->Settle_Sum = 0.0f;
    }

    /* 换成弱位置环后，它的刚度扛不住负载，所以把到位期间的平均输出
       当常值力矩加上，代替原来位置环扛着的那部分；弱位置环只负责
       拉住保持力矩估偏（静摩擦）带来的漂移 */
    if (t->Settle_Cnt > 0U &&
        (float)t->Settle_Cnt * t->Dt >= t->Settle_Time &&
        Chirp_Init(&t->Chirp, t->F0, t->F1, t->Duration, t->Amp, t->Type) ==
            0) {
      t->Hold_Torque = t->Settle_Sum / (float)t->Settle_Cnt;
      t->Sweep_Center = angle;
      j->speed_pid.I_Output = 0; /* 积分已算进保持力矩 */
      Chirp_Start(&t->Chirp);
      t->Phase = MOTOR_TEST_SWEEP;
    }
  } else if (t->Phase == MOTOR_TEST_SWEEP) {
    if (fabsf(angle - t->Sweep_Center) > t->Guard) {
      Chirp_Stop(&t->Chirp);
      t->Abort++;
      t->Phase = MOTOR_TEST_DONE;
    } else if (t->Chirp.Running == 0U) {
      t->Phase = MOTOR_TEST_DONE;
    }
  } else if (t->Phase == MOTOR_TEST_RAMP) {
    if (Motor_Test_Ramp_Update(t) != 0U)
      t->Phase = MOTOR_TEST_DONE;
  }
  t->Run_Last = run;
  t->Req_Last = req;

  /*===| 斜坡目标：往返时按当前档速度走向 Ramp_Goal，其余走向 Target |===*/
  float goal = Motor_Test_Clamp(t, t->Target);
  float speed = t->Ramp_Speed;
  if (t->Phase == MOTOR_TEST_RAMP) {
    goal = t->Ramp_Goal;
    if (t->Ramp_Step == MOTOR_TEST_RAMP_PASS)
      speed = t->Ramp_Speeds[t->Ramp_Index];
  }
  float set_last = t->Angle_Set;
  if (start == 0U && t->Phase != MOTOR_TEST_FAULT &&
      t->Phase != MOTOR_TEST_SWEEP)
    Acc_Slow(goal, &t->Angle_Set, speed, t->Dt);
  float v_ff = (t->Angle_Set - set_last) / t->Dt; /* 目标的变化率[°/s] */
  t->V_Ref = (t->Phase == MOTOR_TEST_RAMP &&
              t->Ramp_Step == MOTOR_TEST_RAMP_PASS)
                 ? v_ff
                 : 0.0f;

  /*===| PID |===
   * 角度环每拍都算，扫频时只是不用它的输出：恢复位置保持那一拍
   * Error_Last 是新的，D 项不会因为误差跳变冲一下。
   * 斜坡运动时把目标变化率作为速度前馈加到速度目标上（外环 P + v_ref），
   * 匀速段不靠角度误差去推，速度更稳。
   * 扫频时速度目标来自弱位置环：只有 P，目标是扫频起始角（不是 Target，
   * 否则带载静差会变成一个速度目标，把关节往 Target 拽）。
   * 2π·BW 是 (rad/s)/rad，换成 RPM/° 要 ÷6。
   * 扫频前后控制器总输出是连续的：关节停在扫频起点附近，位置环恢复后
   * 输出的就是原来那份保持力矩，正好接替 Hold_Torque */
  PID_Position_Calculate(&j->position_pid, t->Angle_Set, angle);
  float speed_set = j->position_pid.Output + v_ff * MOTOR_TEST_DPS_TO_RPM;
  if (t->Phase == MOTOR_TEST_SWEEP)
    speed_set = MOTOR_TEST_2PI * t->Sweep_Pos_BW * MOTOR_TEST_DPS_TO_RPM *
                (t->Sweep_Center - angle);
  PID_Position_Calculate(&j->speed_pid, speed_set, t->Speed);

  t->Ctrl_Out = 0.0f;
  j->output = 0.0f;
  float ff = 0.0f;
  if (run != 0U && t->Phase != MOTOR_TEST_FAULT) {
    j->output = j->speed_pid.Output;
    t->Ctrl_Out = j->output;
    if (t->Phase == MOTOR_TEST_SWEEP)
      t->Ctrl_Out += t->Hold_Torque;
    ff = t->Feedforward;
  } else {
    j->position_pid.I_Output = 0;
    j->speed_pid.I_Output = 0;
  }
  t->Chirp_Out = Chirp_Calculate(&t->Chirp, t->Dt);

  /*===| 发送：失能时驱动自己改发 0 力矩帧，照常调用以保持有反馈 |===*/
  motor_set_value(dev, t->Ctrl_Out + t->Chirp_Out + ff);
  t->Total_Out = dev->output;

  /*===| USB 记录 |===*/
  float ch[MOTOR_TEST_CH] = {t->Chirp_Out, t->Ctrl_Out, t->Total_Out, t->Speed,
                             angle,        t->V_Ref,    (float)t->Frame};
  Motor_Test_Send(t, ch);
  t->Frame++;
}
