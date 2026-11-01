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

#include "chirp.h"
#include "stm32g4xx_hal.h"

#define LEG_TEST_DT 0.002f
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
} Motor_Test_StructTypeDef;