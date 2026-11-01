//=====================================================================================
//                         chirp_pid_test[关节测试：扫频 / 匀速往返]
//
//      对一个关节（joint_t：电机 + 位置环 + 速度环）做两种测试，数据经 USB 发给
//      电脑离线处理：
//        MOTOR_TEST_TYPE_SWEEP  扫频：辨识对象、算速度环参数
//                               （scripts/speed_loop_id.py）
//        MOTOR_TEST_TYPE_RAMP   匀速往返：辨识摩擦、重力曲线、惯量
//                               （scripts/friction_id.py）
//
//      每拍调一次 Motor_Test_Step(t, req)，类型由 t->Test 选：
//        req = 0：位置保持。目标从使能时的位置按斜坡走到 Target
//        req = 1，SWEEP：先位置保持；到位并静止 Settle_Time 后换成弱位置环
//                 （带宽 Sweep_Pos_BW，目标是扫频起始角），加上到位期间测到的
//                 保持力矩，叠加扫频。扫完、或离起始角超过 Guard，回到位置保持
//        req = 1，RAMP：按 Ramp_Speeds[] 逐个速度，在安全区间里来回匀速走
//                 Ramp_Passes 趟（每趟最长 Ramp_Pass_Time，低速时行程按时间缩短，
//                 以区间中点为中心），两端各停 Ramp_Dwell。走完回到 Target 保持
//        两种测试结束后都停在 DONE（位置保持）；再测一次要先把 req 拉回 0
//      安全：目标角一律截在 [Safe_Min, Safe_Max]；实际角度超出
//      [Limit_Min, Limit_Max] 立刻进 FAULT，不出力，req 拉回 0 后从当前位置重新开始。
//      设备失能（dev->enabled == 0）时目标跟着实际位置走、清积分，不出力。
//
//      每拍经 USB 按 JustFloat 发 7 个小端 float + 帧尾 00 00 80 7F：
//        注入信号[N·m]、控制器输出[N·m]、实际下发总力矩[N·m]、速度反馈[RPM]、
//        角度反馈[°]、匀速段参考速度[°/s]（不在匀速段时为 0）、
//        帧序号（每拍加 1，× Dt 即时间）
//
//      用法（应用层任务里，周期必须等于 Dt）：
//        Motor_Test_Init(&Motor_Test, &joint, 0.002f);
//        Motor_Test.Safe_Min = -70; Motor_Test.Safe_Max = 70;  // 按关节填
//        Motor_Test.Limit_Min = -75; Motor_Test.Limit_Max = 75;
//        每拍：Motor_Test.Feedforward = 重力补偿等前馈[N·m];
//              Motor_Test_Step(&Motor_Test, 请求);
//      USB 发送缓冲只有一份，同一时间只能有一个实例在跑。
//      Motor_Test_Disable_Others 用于"只让被测电机出力"的场合（如腿）；
//      机械臂别的关节要撑住姿态，不要调它
//=====================================================================================
#ifndef __CHIRP_PID_TEST_H__
#define __CHIRP_PID_TEST_H__

#include "chirp.h"
#include "joint_calc.h"
#include <stdint.h>

#define MOTOR_TEST_CH 7U                                 // 每帧 float 个数
#define MOTOR_TEST_FRAME_BYTES (MOTOR_TEST_CH * 4U + 4U) // 一帧 32 字节
/* 每块能放的帧数：USB_Send 长度是 uint8_t，一块不能超过 255 字节 */
#define MOTOR_TEST_BUF_FRAMES (255U / MOTOR_TEST_FRAME_BYTES)
#define MOTOR_TEST_RAMP_SPEED_MAX 8U // Ramp_Speeds[] 最多几档

/**
 * @brief 测试类型（req = 1 时做哪一种）
 */
typedef enum {
  MOTOR_TEST_TYPE_SWEEP = 0, // 扫频
  MOTOR_TEST_TYPE_RAMP,      // 匀速往返
} Motor_Test_Type_EnumTypeDef;

/**
 * @brief 测试阶段
 */
typedef enum {
  MOTOR_TEST_HOLD = 0, // 位置保持；SWEEP 请求时同时做到位判定
  MOTOR_TEST_SWEEP,    // 弱位置环 + 保持力矩 + 扫频
  MOTOR_TEST_RAMP,     // 匀速往返
  MOTOR_TEST_DONE,     // 测完或中止，位置保持，等 req 回 0
  MOTOR_TEST_FAULT,    // 超出 Limit：不出力，等 req 回 0
} Motor_Test_Phase_EnumTypeDef;

/**
 * @brief 匀速往返的子阶段
 */
typedef enum {
  MOTOR_TEST_RAMP_MOVE = 0, // 以 Ramp_Speed 走到这一档的起点（不算数据）
  MOTOR_TEST_RAMP_DWELL,    // 端点停留
  MOTOR_TEST_RAMP_PASS,     // 匀速走一趟（记数据）
  MOTOR_TEST_RAMP_RETURN,   // 全部测完，以 Ramp_Speed 回到 Target
} Motor_Test_Ramp_Step_EnumTypeDef;

/**
 * @brief 测试数据（调试器里看 / 改）
 */
typedef struct {
  /* ---- 通用参数 ---- */
  Motor_Test_Type_EnumTypeDef Test; // req = 1 时做哪种测试
  float Target;              // 保持角[°]（和位置环反馈同一量纲）
  uint8_t Target_From_Enable; // 1 = 使能那一拍把 Target 设成当前位置（截进安全区间）
  float Ramp_Speed;  // 保持 / 移动时的目标斜坡速度[°/s]
  float Safe_Min;    // 目标角下限[°]：所有目标都截在安全区间里
  float Safe_Max;    // 目标角上限[°]
  float Limit_Min;   // 实际角度低于它进 FAULT[°]
  float Limit_Max;   // 实际角度高于它进 FAULT[°]
  float Feedforward; // 前馈力矩[N·m]（如重力补偿），应用层每拍写，出力时加上

  /* ---- 扫频参数：在开始扫频那一拍生效 ---- */
  float Settle_Tol;   // 到位判定：离 Target 不超过[°]
  float Settle_Speed; // 到位判定：速度不超过[RPM]
  float Settle_Time;  // 到位判定：连续满足的时长[s]
  float Guard;        // 扫频中离起始角超过[°]就中止
  float Sweep_Pos_BW; // 扫频时弱位置环带宽[Hz]：拉住漂移，取 F0 的 1/5 左右
  float F0;           // 扫频起始频率[Hz]
  float F1;           // 扫频终止频率[Hz]
  float Duration;     // 扫频时长[s]
  float Amp;          // 扫频幅值[N·m]
  Chirp_Type_EnumTypeDef Type;

  /* ---- 匀速往返参数：在开始测试那一拍生效 ---- */
  float Ramp_Speeds[MOTOR_TEST_RAMP_SPEED_MAX]; // 各档速度[°/s]，从低到高
  uint8_t Ramp_Speed_Num; // 用前几档
  uint8_t Ramp_Passes;    // 每档走几趟（一趟 = 一个方向）
  float Ramp_Pass_Time;   // 每趟最长时间[s]：行程 = min(安全区间, 速度 × 时间)
  float Ramp_Dwell;       // 端点停留[s]

  /* ---- 绑定（Motor_Test_Init 写入） ---- */
  joint_t *Joint; // 被测关节：电机设备 + 位置环 + 速度环（PID 参数由应用层填）
  float Dt;       // 调用周期[s]，必须等于任务实际周期

  /* ---- 状态 ---- */
  Motor_Test_Phase_EnumTypeDef Phase;
  float Hold_Torque;   // 扫频：到位期间控制器输出的平均值[N·m]，扫频时当常值前馈
  float Sweep_Center;  // 扫频：开始时的角度[°]
  uint32_t Abort;      // 扫频越过 Guard、或超出 Limit 中止的次数
  float Angle_Set;     // 斜坡后的角度目标[°]
  float Chirp_Out;     // 注入信号[N·m]
  float Ctrl_Out;      // 控制器输出[N·m]（速度环输出，扫频时再加 Hold_Torque）
  float Total_Out;     // 实际下发总力矩[N·m]（驱动限幅、失能置 0 之后的 dev->output）
  float Speed;         // 本拍 PID 用的速度反馈[RPM]
  float V_Ref;         // 匀速段参考速度[°/s]，不在匀速段时为 0
  uint32_t Frame;      // 帧序号
  uint32_t Drop;       // USB 一直没空、缓冲满了丢掉的帧数
  uint8_t Run_Last;    // 上一拍设备是否使能
  uint8_t Req_Last;    // 上一拍的 req
  uint32_t Settle_Cnt; // 扫频：到位条件连续满足的拍数
  float Settle_Sum;    // 扫频：到位期间控制器输出的累加
  Motor_Test_Ramp_Step_EnumTypeDef Ramp_Step;
  uint8_t Ramp_Index;   // 当前第几档
  uint8_t Ramp_Pass;    // 这一档已走完几趟
  float Ramp_Goal;      // 当前要走到的角度[°]
  uint32_t Ramp_Timer;  // 端点停留计数[拍]
  Chirp_Struct_TypeDef Chirp;
} Motor_Test_StructTypeDef;

/* 默认实例：参数已填默认值；安全区间、Limit 默认不限，必须按关节填 */
extern Motor_Test_StructTypeDef Motor_Test;

/**
 * @brief 绑定被测关节、设调用周期，清状态（参数不动）
 * @param joint 被测关节；PID 参数、position_fb 由应用层先填好
 * @param dt    调用周期[s]
 * @return 0 成功，-1 参数不合法
 */
int Motor_Test_Init(Motor_Test_StructTypeDef *t, joint_t *joint, float dt);

/**
 * @brief 跑一拍：阶段切换、PID、扫频 / 往返、下发力矩、USB 记录一帧
 * @param req 1 = 请求测试（类型由 t->Test 选），0 = 位置保持
 * @note  失能时照常调用：驱动会改发 0 力矩帧，被测电机因此一直有反馈
 */
void Motor_Test_Step(Motor_Test_StructTypeDef *t, uint8_t req);

/**
 * @brief 除 keep 以外，把设备表里还使能着的电机全部失能
 * @note  只测一个电机、别的都不需要出力时用（Robo_Restart 会把所有组都使能，
 *        每拍调一次）
 */
void Motor_Test_Disable_Others(const motor_device_t *keep);

#endif /* __CHIRP_PID_TEST_H__ */
