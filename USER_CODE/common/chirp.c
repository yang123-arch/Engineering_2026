//=====================================================================================
//                    chirp[扫频信号]
// Chirp_Init(Chirp_Struct_TypeDef *Chirp, float F0, float F1, float Duration,
// float Amp, Chirp_Type_EnumTypeDef Type) Chirp_Start(Chirp_Struct_TypeDef
// *Chirp) / Chirp_Stop(Chirp_Struct_TypeDef *Chirp)
// Chirp_Calculate(Chirp_Struct_TypeDef *Chirp, float Dt)
//=====================================================================================

#include "chirp.h"
#include "math.h"
#include <stddef.h>

#define CHIRP_2PI 6.28318530717958647692f
#define CHIRP_FADE_PERIODS 2.0f // 默认淡入 / 淡出的周期数

/**
 * @brief 时刻 t 的瞬时频率[Hz]
 */
static float Chirp_Frequency(const Chirp_Struct_TypeDef *Chirp, float t) {
  if (Chirp->Type == CHIRP_LOG)
    return Chirp->F0 * expf(Chirp->Log_K * t);
  return Chirp->F0 + (Chirp->F1 - Chirp->F0) * t / Chirp->Duration;
}

/**
 * @brief 时刻 t 的幅值包络（0~1），开头、结尾用半个余弦过渡
 */
static float Chirp_Envelope(const Chirp_Struct_TypeDef *Chirp, float t) {
  float w = 1.0f;
  float t_end = Chirp->Duration - t;

  if (Chirp->Fade_In > 0.0f && t < Chirp->Fade_In)
    w = 0.5f * (1.0f - cosf(CHIRP_2PI * 0.5f * t / Chirp->Fade_In));
  if (Chirp->Fade_Out > 0.0f && t_end < Chirp->Fade_Out)
    w *= 0.5f * (1.0f - cosf(CHIRP_2PI * 0.5f * t_end / Chirp->Fade_Out));
  return w;
}

/**
 * @brief 扫频参数初始化（不启动）
 */
int Chirp_Init(Chirp_Struct_TypeDef *Chirp, float F0, float F1, float Duration,
               float Amp, Chirp_Type_EnumTypeDef Type) {
  if (Chirp == NULL || !(F0 > 0.0f) || !(F1 > 0.0f) || !(Duration > 0.0f))
    return -1;

  Chirp->F0 = F0;
  Chirp->F1 = F1;
  Chirp->Duration = Duration;
  Chirp->Amp = Amp;
  Chirp->Type = Type;
  Chirp->Log_K = (Type == CHIRP_LOG) ? logf(F1 / F0) / Duration : 0.0f;

  Chirp->Fade_In = fminf(CHIRP_FADE_PERIODS / F0, 0.25f * Duration);
  Chirp->Fade_Out = fminf(CHIRP_FADE_PERIODS / F1, 0.25f * Duration);

  Chirp->Time = 0.0f;
  Chirp->Phase = 0.0f;
  Chirp->Frequency = F0;
  Chirp->Output = 0.0f;
  Chirp->Running = 0U;
  return 0;
}

/**
 * @brief 从头开始扫频
 */
void Chirp_Start(Chirp_Struct_TypeDef *Chirp) {
  if (Chirp == NULL || !(Chirp->Duration > 0.0f))
    return;

  Chirp->Time = 0.0f;
  Chirp->Phase = 0.0f;
  Chirp->Frequency = Chirp->F0;
  Chirp->Output = 0.0f;
  Chirp->Running = 1U;
}

/**
 * @brief 立即停止，输出置 0
 */
void Chirp_Stop(Chirp_Struct_TypeDef *Chirp) {
  if (Chirp == NULL)
    return;

  Chirp->Running = 0U;
  Chirp->Output = 0.0f;
}

/**
 * @brief 推进 Dt 并返回本拍输出
 * @note  相位用区间中点的频率积分（误差是 Dt² 量级），并折回 0~2π，
 *        扫很久也不会因为相位变大丢精度
 */
float Chirp_Calculate(Chirp_Struct_TypeDef *Chirp, float Dt) {
  if (Chirp == NULL)
    return 0.0f;
  if (!Chirp->Running) {
    Chirp->Output = 0.0f;
    return 0.0f;
  }
  if (!(Dt > 0.0f))
    return Chirp->Output;

  /*===| 相位积分 |===*/
  float f_mid = Chirp_Frequency(Chirp, Chirp->Time + 0.5f * Dt);
  Chirp->Phase = fmodf(Chirp->Phase + CHIRP_2PI * f_mid * Dt, CHIRP_2PI);
  Chirp->Time += Dt;

  /*===| 扫完：停止，输出回 0 |===*/
  if (Chirp->Time >= Chirp->Duration) {
    Chirp->Frequency = Chirp->F1;
    Chirp_Stop(Chirp);
    return 0.0f;
  }

  /*===| 输出 |===*/
  Chirp->Frequency = Chirp_Frequency(Chirp, Chirp->Time);
  Chirp->Output =
      Chirp->Amp * Chirp_Envelope(Chirp, Chirp->Time) * sinf(Chirp->Phase);
  return Chirp->Output;
}
