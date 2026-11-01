//=====================================================================================
//                    chirp[扫频信号]
// Chirp_Init(Chirp_Struct_TypeDef *Chirp, float F0, float F1, float Duration, float Amp, Chirp_Type_EnumTypeDef Type)
// Chirp_Start(Chirp_Struct_TypeDef *Chirp) / Chirp_Stop(Chirp_Struct_TypeDef *Chirp)
// Chirp_Calculate(Chirp_Struct_TypeDef *Chirp, float Dt)
//
//      速度环辨识用：闭环稳住后，在力矩指令上叠加一段扫频正弦（见 plan/draft.md
//      "PID 整定方法"）。输出单位跟 Amp 一致，叠加在力矩上就是 N·m。
//      求频响时用"实际下发的总力矩"（dev->output）对实测速度，不要只用本模块的输出。
//=====================================================================================

#ifndef __CHIRP__
#define __CHIRP__

#include <stdint.h>

/*===| 扫频方式 |===*/
typedef enum {
  CHIRP_LOG = 0,    // 对数扫频：每个倍频程用时相同，低频段周期多（辨识常用）
  CHIRP_LINEAR = 1, // 线性扫频：频率随时间线性增加，时间大多花在高频段
} Chirp_Type_EnumTypeDef;

/*===| 扫频数据结构体 |===*/
typedef struct {
  /* 配置（Chirp_Init 写入） */
  float F0;       // 起始频率[Hz]
  float F1;       // 终止频率[Hz]
  float Duration; // 扫频时长[s]
  float Amp;      // 幅值（单位跟叠加对象一致）
  float Fade_In;  // 开头余弦淡入时长[s]，0 = 不淡入
  float Fade_Out; // 结尾余弦淡出时长[s]，0 = 不淡出（结尾相位任意，不淡出会有力矩阶跃）
  Chirp_Type_EnumTypeDef Type;
  float Log_K; // 对数扫频的指数系数 ln(F1/F0)/Duration

  /* 状态（记录数据时一起记，方便离线对齐） */
  float Time;      // 已扫时间[s]
  float Phase;     // 当前相位[rad]，0~2π
  float Frequency; // 当前瞬时频率[Hz]
  float Output;    // 当前输出
  uint8_t Running; // 1 = 正在扫频
} Chirp_Struct_TypeDef;

/**
 * @brief 扫频参数初始化（不启动）
 * @param F0, F1   起止频率[Hz]，都要 > 0；F1 < F0 时从高往低扫
 * @param Duration 扫频时长[s]
 * @param Amp      幅值
 * @param Type     CHIRP_LOG / CHIRP_LINEAR
 * @note  淡入默认 2 个 F0 周期、淡出默认 2 个 F1 周期，各自不超过 Duration/4；
 *        可在 Init 后直接改 Fade_In / Fade_Out
 * @return 0 成功，-1 参数不合法（结构体不改动）
 */
int Chirp_Init(Chirp_Struct_TypeDef *Chirp, float F0, float F1, float Duration,
               float Amp, Chirp_Type_EnumTypeDef Type);

/**
 * @brief 从头开始扫频（时间、相位清零）
 */
void Chirp_Start(Chirp_Struct_TypeDef *Chirp);

/**
 * @brief 立即停止，输出置 0（急停用，不做淡出）
 */
void Chirp_Stop(Chirp_Struct_TypeDef *Chirp);

/**
 * @brief 推进 Dt 并返回本拍输出；没在扫频或已扫完时返回 0
 * @param Dt 距上次调用的时间[s]。每拍都要调，传任务周期即可
 *           （Gimbal_Task 1kHz 传 0.001f；只在本电机发帧那拍调用就传 0.002f）
 * @note  相位按瞬时频率积分，Dt 有抖动也不会让频率跑偏
 */
float Chirp_Calculate(Chirp_Struct_TypeDef *Chirp, float Dt);

#endif
