//=====================================================================================
//                                Motor_DAMIAO_Driver[达妙电机驱动]
//
//      本文件提供两种达妙驱动：
//
//        Motor_DM_4005/3507/8009/8009P/8006/4340/4310_Driver
//        单机：一条报文一个电机，按型号给 q/v/t 量程
//        Motor_DAMIAO_1TO4_Driver
//        一拖四：一条报文 4 个电机
//                                  （rx_id 推出通道号，提交管理器只发一帧）
//
//      MIT 模式的位置 / 速度 / 力矩量程是"每设备参数"，不再写死在协议宏里，
//      各型号数值见 Motor_DMAIAO_Type.h 的 MOTOR_DM_XXX_Q/V/T_LIMIT。
//
//      probe 阶段按型号把三段量程写进 ctx。量程等于电机寄存器
//      PMAX / VMAX / TMAX 时，力矩单位就是 N·m，动力学解算出的力矩直接
//      motor_set_value() 即可。t_limit 处的限幅只防编码溢出，TMAX 一般大于
//      峰值力矩，额定 / 峰值限幅由上层做。
//
//      电机寄存器 TMAX 改了之后，同步改本设备的 t_limit：
//        Motor_DM_SetTorqueRange(&g_motor_devices[IDX_XXX], 新 TMAX);
//=====================================================================================

#include "Motor_DAMIAO_Driver.h"
#include "CAN_Driver.h"
#include "DWT.h"
#include "Define.h"
#include "motor_core.h"

/*===| 上下文池大小（单机 / 一拖四各一个池）|===*/
#define MOTOR_DM_CTX_MAX 16U
#define MOTOR_DM_1TO4_CTX_MAX 8U
#define MOTOR_DM_1TO4_CH_MAX 4U

/*===| 失联恢复参数（单机与一拖四共用）|===*/
#define MOTOR_DM_RECOVER_TIMEOUT_US 2000U  // 每一步等回码的最长时间
#define MOTOR_DM_1TO4_CLEAR_WAIT_US 1000U  // 一拖四清故障到使能的固定间隔
#define MOTOR_DM_RECOVER_RETRY_MAX 3U      // 连续失败几次后进入退避
#define MOTOR_DM_RECOVER_HOLDOFF_US 30000U // 退避时长：失联电机不刷屏总线

/**
 * @brief 恢复状态机的阶段
 */
typedef enum {
  DM_RECOVER_IDLE = 0,    // 空闲：下一次进来发清故障
  DM_RECOVER_WAIT_CLEAR,  // 已发清故障，等回码 / 等间隔
  DM_RECOVER_WAIT_ENABLE, // 已发使能，等"已使能"回码
  DM_RECOVER_HOLDOFF,     // 连续失败，退避中
} dm_recover_state_t;

/**
 * @brief 恢复状态机（单机一份，一拖四每通道一份）
 * @note  "有没有新回码"用驱动自己的回码计数判断：计数只在接收中断里加，
 *        任务里只读、跟发命令时记下的快照比较。不看 If_Online：它会被
 *        error_core 的 5ms 轮询一次性清零，读到的不一定是真相
 */
typedef struct {
  uint8_t state;    // dm_recover_state_t
  uint8_t fail;     // 连续失败次数（到 RETRY_MAX 后饱和）
  uint32_t rx_snap; // 发命令时的回码计数快照
  uint64_t t0_us;   // 当前阶段起始时间[us]
} dm_recover_t;

/**
 * @brief 恢复成功 / 应用失能：状态机回到空闲，失败计数清零
 */
static void dm_recover_reset(dm_recover_t *rc) {
  rc->state = DM_RECOVER_IDLE;
  rc->fail = 0U;
}

/**
 * @brief 一轮尝试失败：没到次数就立刻重试，否则进入退避
 * @note  失败计数饱和在 RETRY_MAX：前几次快速重试，之后每次失败都退避，
 *        即"连续失败 3 次后每 30ms 试一次"。退避期间 rx_snap 保留发使能时
 *        的快照，迟到的"已使能"回码照样算成功
 * @return 1 调用方马上开始新一轮；0 已进入退避
 */
static uint8_t dm_recover_fail(dm_recover_t *rc, uint64_t now) {
  if (rc->fail < MOTOR_DM_RECOVER_RETRY_MAX)
    rc->fail++;
  if (rc->fail < MOTOR_DM_RECOVER_RETRY_MAX)
    return 1U;

  rc->t0_us = now;
  rc->state = DM_RECOVER_HOLDOFF;
  return 0U;
}

/*===|==================== 单机：底层协议函数 ====================|===*/

/**
 * @brief 达妙电机使能
 */
void Motor_DM_CMD_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t DM_Enable_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN_Send_Data(hfdcan, CAN_ID, DM_Enable_Data);
}

/**
 * @brief 达妙电机失能
 */
void Motor_DM_CMD_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t DM_Disable_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  CAN_Send_Data(hfdcan, CAN_ID, DM_Disable_Data);
}

/**
 * @brief 达妙电机设置零点
 */
void Motor_DM_CMD_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t DM_SetZero_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  CAN_Send_Data(hfdcan, CAN_ID, DM_SetZero_Data);
}

/**
 * @brief 达妙电机清除错误
 */
void Motor_DM_CMD_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t DM_ClearErr_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                 0xFF, 0xFF, 0xFF, 0xFB};
  CAN_Send_Data(hfdcan, CAN_ID, DM_ClearErr_Data);
}

/**
 * @brief 达妙电机 MIT 控制
 *
 * @param _torq    力矩指令[N·m]，越界被钳位到 ±t_limit
 * @param t_limit  力矩限幅[N·m]，编码区间铺满 ±t_limit
 * @param q_limit  位置编码量程[rad]（±），pos 越界被钳位
 * @param v_limit  速度编码量程[rad/s]（±），vel 越界被钳位
 * @param pos/vel/kp/kd  MIT 其余四段；全 0 即旧版纯力矩模式
 */
void Motor_DM_CMD_MIT(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID, float _torq,
                      float t_limit, float q_limit, float v_limit, float pos,
                      float vel, float kp, float kd) {
  uint8_t DM_Send_Data[8];

  if (t_limit <= 0.0f)
    return;
  if (q_limit <= 0.0f)
    q_limit = MOTOR_DM_4005_Q_LIMIT;
  if (v_limit <= 0.0f)
    v_limit = MOTOR_DM_4005_V_LIMIT;

  if (_torq > t_limit)
    _torq = t_limit;
  else if (_torq < -t_limit)
    _torq = -t_limit;
  if (pos > q_limit)
    pos = q_limit;
  else if (pos < -q_limit)
    pos = -q_limit;
  if (vel > v_limit)
    vel = v_limit;
  else if (vel < -v_limit)
    vel = -v_limit;
  if (kp > KP_MAX)
    kp = KP_MAX;
  else if (kp < KP_MIN)
    kp = KP_MIN;
  if (kd > KD_MAX)
    kd = KD_MAX;
  else if (kd < KD_MIN)
    kd = KD_MIN;

  /* pos/vel/kp/kd 全 0 时与旧版纯力矩模式逐字节一致 */
  uint16_t pos_tmp = (uint16_t)DM_float_to_uint(pos, -q_limit, q_limit, 16);
  uint16_t vel_tmp = (uint16_t)DM_float_to_uint(vel, -v_limit, v_limit, 12);
  uint16_t kp_tmp = (uint16_t)DM_float_to_uint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kd_tmp = (uint16_t)DM_float_to_uint(kd, KD_MIN, KD_MAX, 12);
  uint16_t tor_tmp = (uint16_t)DM_float_to_uint(_torq, -t_limit, t_limit, 12);

  DM_Send_Data[0] = (uint8_t)(pos_tmp >> 8);
  DM_Send_Data[1] = (uint8_t)pos_tmp;
  DM_Send_Data[2] = (uint8_t)(vel_tmp >> 4);
  DM_Send_Data[3] = (uint8_t)(((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
  DM_Send_Data[4] = (uint8_t)kp_tmp;
  DM_Send_Data[5] = (uint8_t)(kd_tmp >> 4);
  DM_Send_Data[6] = (uint8_t)(((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
  DM_Send_Data[7] = (uint8_t)tor_tmp;

  CAN_Send_Data(hfdcan, CAN_ID, DM_Send_Data);
}

/**
 * @brief 达妙电机位置控制
 */
void Motor_DM_CMD_Position(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                           float Position, float Speed) {
  float Position_Speed_Buf[2] = {Position, Speed};

  CAN_Send_Data(hfdcan, CAN_ID, (uint8_t *)Position_Speed_Buf);
}

/**
 * @brief 达妙电机速度控制
 */
void Motor_DM_CMD_Speed(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                        float Speed) {
  float Speed_Buf[1] = {Speed};

  CAN_Send_Data(hfdcan, CAN_ID, (uint8_t *)Speed_Buf);
}

float DM_uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int DM_float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 达妙单机反馈数据解包
 *
 * @param data     电调发送的数组
 * @param q_limit  本型号位置解码量程[rad]（±）
 * @param v_limit  本型号速度解码量程[rad/s]（±）
 * @param t_limit  本型号力矩解码量程[N·m]（±）
 * @note  协议 16 位位置 / 12 位速度 / 12 位力矩字段本身不带量纲，
 *        必须按本型号 ctx 里的 q/v/t_limit 解码；未确认的型号
 *        q/v 用达妙默认（±π、±30）、t 走 T_MAX 兜底
 */
void Motor_DM_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *data,
                           float q_limit, float v_limit, float t_limit) {
  /*===| 协议解包 |===*/
  uint8_t ErrCode = Data[0] >> 4;
  int p_int = (Data[1] << 8) | Data[2];
  int v_int = (Data[3] << 4) | (Data[4] >> 4);
  int t_int = ((Data[4] & 0xF) << 8) | Data[5];
  int temperature = Data[6];

  /*===| 转移数据 |===*/
  data->Angle =
      DM_uint_to_float(p_int, -q_limit, q_limit, 16) / 2.0f / PI * 360.0f;
  data->Speed_RPM =
      DM_uint_to_float(v_int, -v_limit, v_limit, 12) / 2.0f / PI * 60.0f;

  if (t_limit <= 0.0f) { // 型号没给额定力矩：退回名义量程
    t_limit = T_MAX;
  }
  data->Torque = DM_uint_to_float(t_int, -t_limit, t_limit, 12);
  data->Temperature = (int8_t)temperature;

  /*===| 得到总角度值 |===*/
  if (data->Angle - data->Angle_Last > 180.0f)
    data->Round--;
  else if (data->Angle - data->Angle_Last < -180.0f)
    data->Round++;
  data->Total_Angle = 360.0f * data->Round + data->Angle;

  /*===| 记录状态 |===*/
  data->Angle_Last = data->Angle;
  data->Error = ErrCode;

  /* 在线标志不在这里置：算不算在线要和"应用想让它使能还是失能"比，
     本函数拿不到设备，由 Motor_DM_Ops_StorageData 判 */
}

/*===|==================== 一拖四：底层协议函数 ====================|===*/

/**
 * @brief 达妙一拖四电机发送电流（一帧 4 个电机，小端）
 */
void Motor_DM1to4_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                              int16_t ID1_Currnet, int16_t ID2_Currnet,
                              int16_t ID3_Currnet, int16_t ID4_Currnet) {
  uint8_t Motor_Tx_Data[8];

  Motor_Tx_Data[0] = (uint8_t)(ID1_Currnet & 0xFF);
  Motor_Tx_Data[1] = (uint8_t)(ID1_Currnet >> 8);
  Motor_Tx_Data[2] = (uint8_t)(ID2_Currnet & 0xFF);
  Motor_Tx_Data[3] = (uint8_t)(ID2_Currnet >> 8);
  Motor_Tx_Data[4] = (uint8_t)(ID3_Currnet & 0xFF);
  Motor_Tx_Data[5] = (uint8_t)(ID3_Currnet >> 8);
  Motor_Tx_Data[6] = (uint8_t)(ID4_Currnet & 0xFF);
  Motor_Tx_Data[7] = (uint8_t)(ID4_Currnet >> 8);

  CAN_Send_Data(hfdcan, CAN_ID, Motor_Tx_Data);
}

/**
 * @brief 达妙一拖四清故障：ID 放在数据段里，报文固定发到 0x7FF
 */
void Motor_DM1to4_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t Motor_Tx_Data[8];

  Motor_Tx_Data[0] = (uint8_t)(CAN_ID & 0xFF);
  Motor_Tx_Data[1] = (uint8_t)(CAN_ID >> 8);
  Motor_Tx_Data[2] = 0x55;
  Motor_Tx_Data[3] = 0x3C;
  Motor_Tx_Data[4] = 0;
  Motor_Tx_Data[5] = 0;
  Motor_Tx_Data[6] = 0;
  Motor_Tx_Data[7] = 0;

  CAN_Send_Data(hfdcan, 0x7FF, Motor_Tx_Data);
}

/**
 * @brief 达妙一拖四反馈数据解包
 */
void Motor_DM1to4_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *data) {
  /*===| 协议解包 |===*/
  data->Encoder = (int16_t)(Data[0] << 8 | Data[1]);
  data->Speed_RPM = (int16_t)(Data[2] << 8 | Data[3]) / 100.0f;
  data->Torque = (int16_t)(Data[4] << 8 | Data[5]);
  data->Temperature = (int8_t)(Data[6]);

  data->Angle = (data->Encoder - 4096.0f) * 180.0f / 4096.0f;

  /*===| 得到总角度值 |===*/
  if (data->Encoder - data->Encoder_Last > 4096)
    data->Round--;
  else if (data->Encoder - data->Encoder_Last < -4096)
    data->Round++;
  data->Total_Angle = 360.0f * (data->Round + data->Encoder / 8192.0f);

  /*===| 记录状态 |===*/
  data->Encoder_Last = data->Encoder;
  data->Angle_Last = data->Angle;

  data->If_Online = 1;
}

/*===|==================== 单机：上下文、操作表、probe
 * ====================|===*/

/**
 * @brief 单机设备的驱动上下文
 * @note  q_limit / v_limit / t_limit 是输出限幅 + MIT 编码区间 + 反馈解码
 *        三段共用的"本型号额定量程"，probe 时按 type_id 写死；
 *        应用想收窄可通过 Motor_DM_SetTorqueRange 等 API 改 t_limit。
 */
typedef struct {
  motor_ctx_t base;
  float q_limit; // 位置量程 [rad]（±），MIT 编码 + 反馈解码共用
  float v_limit; // 速度量程 [rad/s]（±），MIT 编码 + 反馈解码共用
  float t_limit; // 力矩量程 [N·m]（±），输出限幅 + MIT 编码 + 反馈解码共用
  float p_des;                     // MIT 位置目标（默认 0，纯力矩）
  float v_des;                     // MIT 速度目标（默认 0）
  float kp;                        // MIT Kp（默认 0）
  float kd;                        // MIT Kd（默认 0）
  volatile uint32_t rx_count;      // 回码计数：接收中断里加，恢复流程只读
  dm_recover_t recover;            // 失联恢复状态机
} dm_ctx_t;

static dm_ctx_t s_dm_ctx[MOTOR_DM_CTX_MAX];
static uint8_t s_dm_ctx_used;

static void Motor_DM_Ops_StorageData(motor_device_t *dev, const uint8_t *data) {
  dm_ctx_t *ctx = (dm_ctx_t *)dev->ctx;
  /* 反馈按本型号 ctx 里的 q/v/t_limit 解码；ctx 缺失时全部走默认 */
  float q = (ctx != NULL) ? ctx->q_limit : MOTOR_DM_4005_Q_LIMIT;
  float v = (ctx != NULL) ? ctx->v_limit : MOTOR_DM_4005_V_LIMIT;
  float t = (ctx != NULL) ? ctx->t_limit : 0.0f;
  Motor_DM_Storage_Data((uint8_t *)data, &dev->data, q, v, t);

  /* 在线 = 状态码和应用期望一致：应用使能时要回 1，应用失能时回 0 也算。
     达妙失能后照样回帧，停机期间不会被 error_core 判成失联；故障码、
     或者该使能却没使能，都判失联，交给恢复流程 */
  uint8_t expect = (dev->enabled != 0U) ? MOTOR_DM_STATE_ENABLED
                                        : MOTOR_DM_STATE_DISABLED;
  if (dev->data.Error == expect)
    dev->data.If_Online = 1U;

  /* 回码计数：恢复流程拿它和发命令时的快照比，判断有没有收到新回码
     （中断整段跑完任务才能继续，任务看到新计数时 data.Error 已经更新） */
  if (ctx != NULL)
    ctx->rx_count++;
}

static int Motor_DM_Ops_Enable(motor_device_t *dev) {
  Motor_DM_CMD_Enable(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_DM_Ops_Disable(motor_device_t *dev) {
  Motor_DM_CMD_Disable(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_DM_Ops_Clear(motor_device_t *dev) {
  Motor_DM_CMD_ClearErr(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_DM_Ops_SetZero(motor_device_t *dev) {
  Motor_DM_CMD_SetZero(dev->hfdcan, dev->tx_id);
  return 0;
}

/**
 * @brief 单机 set_value：按 ctx->t_limit 限幅后发 MIT
 * 帧（一机一帧，无需提交管理器）
 * @note  达妙一问一答，不发帧就没有反馈，所以任何状态下都照常发一帧：
 *          应用已失能、电机却还回"已使能" -> 改发失能帧（失能帧丢了，或被
 *                                            恢复流程后到的使能帧盖掉）
 *          应用未使能 / 电机状态码不是 1   -> 力矩、Kp、Kd 全填 0
 *          其余                            -> 正常下发
 *        这样第一帧非零力矩一定在确认电机已使能之后才发出去
 */
static int Motor_DM_Ops_SetValue(motor_device_t *dev, float value) {
  dm_ctx_t *ctx = (dm_ctx_t *)dev->ctx;

  if (ctx == NULL)
    return -1;

  uint8_t state = dev->data.Error;

  if (dev->enabled == 0U && state == MOTOR_DM_STATE_ENABLED) {
    dev->output = 0.0f;
    Motor_DM_CMD_Disable(dev->hfdcan, dev->tx_id);
    return 0;
  }

  if (dev->enabled == 0U || state != MOTOR_DM_STATE_ENABLED) {
    dev->output = 0.0f;
    Motor_DM_CMD_MIT(dev->hfdcan, dev->tx_id, 0.0f, ctx->t_limit, ctx->q_limit,
                     ctx->v_limit, 0.0f, 0.0f, 0.0f, 0.0f);
    return 0;
  }

  /* 按本型号额定力矩限幅（t_limit 在 ctx），并回写为"实际下发值" */
  float t_max = ctx->t_limit;
  if (value > t_max)
    value = t_max;
  else if (value < -t_max)
    value = -t_max;
  dev->output = value;

  Motor_DM_CMD_MIT(dev->hfdcan, dev->tx_id, value, ctx->t_limit, ctx->q_limit,
                   ctx->v_limit, ctx->p_des, ctx->v_des, ctx->kp, ctx->kd);
  return 0;
}

/**
 * @brief 恢复（错误处理）：按状态码确认的非阻塞状态机
 *
 *          IDLE         发清故障 -> WAIT_CLEAR
 *          WAIT_CLEAR   回码 0（故障已清、处于失能）或等满 2ms
 *                       -> 发使能 -> WAIT_ENABLE
 *          WAIT_ENABLE  2ms 内没等到回码 1 -> 本轮失败
 *          HOLDOFF      连续失败 3 次后，每轮先等 30ms
 *
 *        除 IDLE 外，任何阶段收到新的回码 1 都算成功（包括上一轮迟到的
 *        回码）；失败只由超时判定。清故障后电机回不回码没有实测过，
 *        所以 2ms 没等到也照样发使能，不会卡死在清故障这一步。
 * @note  应用失能（dev->enabled == 0）时不拉起电机、直接算完成；
 *        已经发过使能帧的补发一帧失能，抵消可能后到的使能
 * @return 1 流程走完（恢复成功 / 应用已失能）；0 未走完
 */
static int Motor_DM_Ops_Recover(motor_device_t *dev) {
  dm_ctx_t *ctx = (dm_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return 1; /* 上下文丢了：算完成，别让上层死等 */

  dm_recover_t *rc = &ctx->recover;

  if (dev->enabled == 0U) {
    if (rc->state == DM_RECOVER_WAIT_ENABLE)
      Motor_DM_CMD_Disable(dev->hfdcan, dev->tx_id);
    dm_recover_reset(rc);
    return 1;
  }

  /* 先读计数再读状态码：计数变了，读到的状态码就不会比它旧 */
  uint32_t rx = ctx->rx_count;
  uint8_t state = dev->data.Error;
  uint8_t got = (rx != rc->rx_snap) ? 1U : 0U;
  uint64_t now = DWT_GetTimeline_us();

  /* 先看回码再看超时：错误处理任务两次调用的间隔可能比窗口还长 */
  if (rc->state != DM_RECOVER_IDLE && got != 0U &&
      state == MOTOR_DM_STATE_ENABLED) {
    dm_recover_reset(rc);
    return 1;
  }

  switch (rc->state) {
  case DM_RECOVER_WAIT_CLEAR:
    if ((got != 0U && state == MOTOR_DM_STATE_DISABLED) ||
        (now - rc->t0_us) >= MOTOR_DM_RECOVER_TIMEOUT_US) {
      rc->rx_snap = rx;
      rc->t0_us = now;
      rc->state = DM_RECOVER_WAIT_ENABLE;
      Motor_DM_CMD_Enable(dev->hfdcan, dev->tx_id);
    }
    return 0;

  case DM_RECOVER_WAIT_ENABLE:
    if ((now - rc->t0_us) < MOTOR_DM_RECOVER_TIMEOUT_US)
      return 0;
    if (dm_recover_fail(rc, now) == 0U)
      return 0; /* 进入退避 */
    break;      /* 立刻开始新一轮 */

  case DM_RECOVER_HOLDOFF:
    if ((now - rc->t0_us) < MOTOR_DM_RECOVER_HOLDOFF_US)
      return 0;
    break;

  default: /* DM_RECOVER_IDLE */
    break;
  }

  /* 新一轮：清故障 */
  rc->rx_snap = rx;
  rc->t0_us = now;
  rc->state = DM_RECOVER_WAIT_CLEAR;
  Motor_DM_CMD_ClearErr(dev->hfdcan, dev->tx_id);
  return 0;
}

static const motor_ops_t s_dm_ops = {
    .enable = Motor_DM_Ops_Enable,
    .disable = Motor_DM_Ops_Disable,
    .clear = Motor_DM_Ops_Clear,
    .set_zero = Motor_DM_Ops_SetZero,
    .recover = Motor_DM_Ops_Recover,
    .storage_data = Motor_DM_Ops_StorageData,
    .set_value = Motor_DM_Ops_SetValue,
};

/**
 * @brief 设置某台达妙电机的力矩量程（写本设备 ctx，输出限幅 + MIT 编码 +
 * 反馈解码）
 * @note  区间越窄，单位力矩占用的编码位越多、分辨率越高；只接受对称区间
 * ±t_limit
 * @return 0 成功，-1 失败
 */
int Motor_DM_SetTorqueRange(motor_device_t *dev, float t_limit) {
  /* 只对达妙单机有效（一拖四走的是电流环，没有 MIT 编码区间） */
  if (dev == NULL || dev->driver == NULL ||
      dev->driver->type_id >= MOTOR_TYPE_DM_1TO4)
    return -1;
  if (t_limit <= 0.0f)
    return -1;

  dm_ctx_t *ctx = (dm_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return -1;

  ctx->t_limit = t_limit;
  return 0;
}

/**
 * @brief 动态设置某台达妙电机的 MIT 位置/速度/Kp/Kd（默认全 0 = 纯力矩）
 * @note  写入 dm_ctx_t，下次 set_value 时随力矩一起下发；想恢复纯力矩
 *        就再调用一次传全 0。
 * @return 0 成功，-1 失败
 */
int Motor_DM_SetMITParams(motor_device_t *dev, float p, float v, float kp,
                          float kd) {
  if (dev == NULL || dev->driver == NULL ||
      dev->driver->type_id >= MOTOR_TYPE_DM_1TO4)
    return -1;

  dm_ctx_t *ctx = (dm_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return -1;

  ctx->p_des = p;
  ctx->v_des = v;
  ctx->kp = kp;
  ctx->kd = kd;
  return 0;
}

/**
 * @brief probe：给单机设备挂上下文，并按型号写死 q/v/t 三段限幅
 * @note  3507 / 8009 / 8009P / 8006 / 4340 / 4310 的 q/v/t 等于电机寄存器读出值；
 *        4005 没读过寄存器，沿用达妙默认（q=±π、v=±30）；
 *        DM_DJI 沿用旧工程 16384 原始值区间，motor_init.c 现有
 *        设备表全部走该兼容型号，与旧版 MIT 编码行为一致。
 */
static int Motor_DM_Probe(motor_device_t *dev) {
  if (s_dm_ctx_used >= MOTOR_DM_CTX_MAX)
    return -1;

  dm_ctx_t *ctx = &s_dm_ctx[s_dm_ctx_used++];
  ctx->base.dev = dev;
  ctx->base.drv = dev->driver;
  ctx->p_des = 0.0f;
  ctx->v_des = 0.0f;
  ctx->kp = 0.0f;
  ctx->kd = 0.0f;
  ctx->rx_count = 0U;
  ctx->recover = (dm_recover_t){0};

  /* 按型号写死三段限幅（不再从 motor_driver_t 读） */
  switch (dev->type_id) {
  case MOTOR_TYPE_DM_4005:
    ctx->q_limit = MOTOR_DM_4005_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_4005_V_LIMIT;
    ctx->t_limit = MOTOR_DM_4005_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_3507:
    ctx->q_limit = MOTOR_DM_3507_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_3507_V_LIMIT;
    ctx->t_limit = MOTOR_DM_3507_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_8009:
    ctx->q_limit = MOTOR_DM_8009_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_8009_V_LIMIT;
    ctx->t_limit = MOTOR_DM_8009_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_8009P:
    ctx->q_limit = MOTOR_DM_8009P_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_8009P_V_LIMIT;
    ctx->t_limit = MOTOR_DM_8009P_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_8006:
    ctx->q_limit = MOTOR_DM_8006_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_8006_V_LIMIT;
    ctx->t_limit = MOTOR_DM_8006_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_4340:
    ctx->q_limit = MOTOR_DM_4340_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_4340_V_LIMIT;
    ctx->t_limit = MOTOR_DM_4340_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_4310:
    ctx->q_limit = MOTOR_DM_4310_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_4310_V_LIMIT;
    ctx->t_limit = MOTOR_DM_4310_T_LIMIT;
    break;
  case MOTOR_TYPE_DM_DJI:
    /* 兼容型号：沿用旧工程在用的 16384 原始值区间，
       设备表里旧 DM_8009 行可直接换 type_id 即恢复旧行为 */
    ctx->q_limit = MOTOR_DM_DJI_Q_LIMIT;
    ctx->v_limit = MOTOR_DM_DJI_V_LIMIT;
    ctx->t_limit = MOTOR_DM_DJI_T_LIMIT;
    break;
  default:
    return -1;
  }

  dev->ctx = (motor_ctx_t *)ctx;
  dev->channel = 0;

  return 0;
}

/*===| 每种型号一个驱动：额定力矩由 probe 按 type_id 写死到 ctx ===|*/
motor_driver_t Motor_DM_4005_Driver = {
    .name = "damiao_4005",
    .type_id = MOTOR_TYPE_DM_4005,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_DM_3507_Driver = {
    .name = "damiao_3507",
    .type_id = MOTOR_TYPE_DM_3507,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_DM_8009_Driver = {
    .name = "damiao_8009",
    .type_id = MOTOR_TYPE_DM_8009,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_DM_8009P_Driver = {
    .name = "damiao_8009p",
    .type_id = MOTOR_TYPE_DM_8009P,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_DM_8006_Driver = {
    .name = "damiao_8006",
    .type_id = MOTOR_TYPE_DM_8006,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_DM_4340_Driver = {
    .name = "damiao_4340",
    .type_id = MOTOR_TYPE_DM_4340,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_DM_4310_Driver = {
    .name = "damiao_4310",
    .type_id = MOTOR_TYPE_DM_4310,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

/* 兼容型号：沿用旧工程 16384 原始值区间，旧 DM_8009 设备表项换 type_id
   即恢复原有行为；新写的设备请直接用 4005/3507 等已确认额定力矩的型号 */
motor_driver_t Motor_DM_DJI_Driver = {
    .name = "damiao_dji",
    .type_id = MOTOR_TYPE_DM_DJI,
    .ops = &s_dm_ops,
    .probe = Motor_DM_Probe,
    .remove = NULL,
    .next = NULL,
};

/**
 * @brief 注册本文件支持的全部达妙驱动（单机 + 兼容型号 + 一拖四）
 */
int Motor_DAMIAO_Register(void) {
  int ret = 0;

  ret |= motor_driver_register(&Motor_DM_4005_Driver);
  ret |= motor_driver_register(&Motor_DM_3507_Driver);
  ret |= motor_driver_register(&Motor_DM_8009_Driver);
  ret |= motor_driver_register(&Motor_DM_8009P_Driver);
  ret |= motor_driver_register(&Motor_DM_8006_Driver);
  ret |= motor_driver_register(&Motor_DM_4340_Driver);
  ret |= motor_driver_register(&Motor_DM_4310_Driver);
  ret |= motor_driver_register(&Motor_DM_DJI_Driver);
  ret |= motor_driver_register(&Motor_DAMIAO_1TO4_Driver);

  return ret;
}

/*===|==================== 一拖四：上下文、操作表、probe
 * ====================|===*/

/**
 * @brief 一拖四设备的驱动上下文
 */
typedef struct {
  motor_ctx_t base;
  motor_device_t *devs[MOTOR_DM_1TO4_CH_MAX]; // 按通道号索引
  int16_t pending[MOTOR_DM_1TO4_CH_MAX];      // 当前电流指令
  volatile uint8_t staged;                    // 曾经写过值的通道
  volatile uint8_t dirty; // 自上次提交后被更新过的通道
  float commit_ms;        // 上次发帧时的 DWT 时间戳[ms]
  float current_max;      // 电流限幅 [A]，probe 时按型号写死
  /* 恢复状态按通道分：4 个通道可能各自失联，谁先恢复谁先走完 */
  volatile uint32_t rx_count[MOTOR_DM_1TO4_CH_MAX]; // 每通道回码计数（中断里加）
  dm_recover_t recover[MOTOR_DM_1TO4_CH_MAX];
} dm1to4_ctx_t;

static dm1to4_ctx_t s_dm1to4_ctx[MOTOR_DM_1TO4_CTX_MAX];
static uint8_t s_dm1to4_ctx_used;

/*===|==================== 一拖四：提交管理器 ====================|===*/

static void Motor_DM_Ops_StorageData1to4(motor_device_t *dev,
                                         const uint8_t *data) {
  dm1to4_ctx_t *ctx = (dm1to4_ctx_t *)dev->ctx;
  Motor_DM1to4_Storage_Data((uint8_t *)data, &dev->data);

  /* 本通道回码计数：恢复流程据此判断使能后有没有收到新回码 */
  if (ctx != NULL && dev->channel >= 0 &&
      dev->channel < (int8_t)MOTOR_DM_1TO4_CH_MAX)
    ctx->rx_count[dev->channel]++;
}

static int Motor_DM1to4_Ops_Enable(motor_device_t *dev) {
  uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN_Send_Data(dev->hfdcan, dev->tx_id, enable_data);
  return 0;
}

/**
 * @brief 失能：本通道电流置 0 并置 dirty，下一帧发 0
 */
static int Motor_DM1to4_Ops_Disable(motor_device_t *dev) {
  if (dev->channel < 0)
    return -1;

  dm1to4_ctx_t *ctx = (dm1to4_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return -1;

  ctx->pending[dev->channel] = 0;
  ctx->staged |= (uint8_t)(1U << dev->channel);
  ctx->dirty |= (uint8_t)(1U << dev->channel);
  return 0;
}

static int Motor_DM1to4_Ops_Clear(motor_device_t *dev) {
  Motor_DM1to4_ClearErr(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_DM1to4_Ops_SetValue(motor_device_t *dev, float value) {
  if (dev->channel < 0)
    return -1;

  dm1to4_ctx_t *ctx = (dm1to4_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return -1;

  /* 按型号限幅（单位 A，从共享 ctx 读），并回写为"实际下发值" */
  float i_max = ctx->current_max;
  if (value > i_max)
    value = i_max;
  else if (value < -i_max)
    value = -i_max;
  dev->output = value;

  /* 换算成协议原始值并顶一层量程：16384 对应 20 A */
  int32_t current = (int32_t)(value * MOTOR_DM_1TO4_RAW_PER_AMP);
  if (current > MOTOR_DM_1TO4_CURRENT_MAX)
    current = MOTOR_DM_1TO4_CURRENT_MAX;
  else if (current < MOTOR_DM_1TO4_CURRENT_MIN)
    current = MOTOR_DM_1TO4_CURRENT_MIN;

  ctx->pending[dev->channel] = (int16_t)current;
  ctx->staged |= (uint8_t)(1U << dev->channel);
  ctx->dirty |= (uint8_t)(1U << dev->channel);

  return 0;
}

/**
 * @brief 恢复（错误处理）：本通道 清故障 -> 等 1ms -> 使能 -> 等回码确认
 * @note  一拖四的反馈里没有状态码，判不出"已使能"，只能按"使能之后收到过
 *        本通道回码"算成功；2ms 内没收到算失败，重试与退避规则同单机。
 *        4 个通道各走各的状态机；使能帧是整板广播帧，可能重复发几次，
 *        对电调是幂等的
 * @return 1 流程走完（恢复成功 / 应用已失能）；0 未走完
 */
static int Motor_DM1to4_Ops_Recover(motor_device_t *dev) {
  dm1to4_ctx_t *ctx = (dm1to4_ctx_t *)dev->ctx;
  if (ctx == NULL || dev->channel < 0 ||
      dev->channel >= (int8_t)MOTOR_DM_1TO4_CH_MAX)
    return 1; /* 通道没分配：算完成，别让上层死等 */

  dm_recover_t *rc = &ctx->recover[dev->channel];

  /* 应用失能：不拉起电机（一拖四失能只是本通道电流置 0，无需补发帧） */
  if (dev->enabled == 0U) {
    dm_recover_reset(rc);
    return 1;
  }

  uint32_t rx = ctx->rx_count[dev->channel];
  uint64_t now = DWT_GetTimeline_us();

  /* 发过使能之后（含退避期间迟到的）收到本通道回码：恢复成功 */
  if ((rc->state == DM_RECOVER_WAIT_ENABLE ||
       rc->state == DM_RECOVER_HOLDOFF) &&
      rx != rc->rx_snap) {
    dm_recover_reset(rc);
    return 1;
  }

  switch (rc->state) {
  case DM_RECOVER_WAIT_CLEAR:
    if ((now - rc->t0_us) >= MOTOR_DM_1TO4_CLEAR_WAIT_US) {
      rc->rx_snap = rx;
      rc->t0_us = now;
      rc->state = DM_RECOVER_WAIT_ENABLE;
      Motor_DM1to4_Ops_Enable(dev);
    }
    return 0;

  case DM_RECOVER_WAIT_ENABLE:
    if ((now - rc->t0_us) < MOTOR_DM_RECOVER_TIMEOUT_US)
      return 0;
    if (dm_recover_fail(rc, now) == 0U)
      return 0; /* 进入退避 */
    break;      /* 立刻开始新一轮 */

  case DM_RECOVER_HOLDOFF:
    if ((now - rc->t0_us) < MOTOR_DM_RECOVER_HOLDOFF_US)
      return 0;
    break;

  default: /* DM_RECOVER_IDLE */
    break;
  }

  /* 新一轮：清故障 */
  rc->t0_us = now;
  rc->state = DM_RECOVER_WAIT_CLEAR;
  Motor_DM1to4_Ops_Clear(dev);
  return 0;
}

static const motor_ops_t s_dm1to4_ops = {
    .enable = Motor_DM1to4_Ops_Enable,
    .disable = Motor_DM1to4_Ops_Disable,
    .clear = Motor_DM1to4_Ops_Clear,
    .set_zero = NULL, // 一拖四无设置零点指令
    .recover = Motor_DM1to4_Ops_Recover,
    .storage_data = Motor_DM_Ops_StorageData1to4,
    .set_value = Motor_DM1to4_Ops_SetValue,
};

/**
 * @brief probe：按 rx_id 推出通道号，同一帧的 4 个设备共用一个上下文
 */
static int Motor_DM1to4_Probe(motor_device_t *dev) {
  uint16_t channel = (uint16_t)(dev->rx_id - MOTOR_DM_1TO4_RX_ID_BASE);
  if (channel >= MOTOR_DM_1TO4_CH_MAX)
    return -1;

  /* 先找同一帧上已经认领过的上下文（tx_id + 总线相同） */
  dm1to4_ctx_t *ctx = NULL;
  for (uint8_t i = 0; i < s_dm1to4_ctx_used; i++) {
    dm1to4_ctx_t *c = &s_dm1to4_ctx[i];
    if (c->base.dev != NULL && c->base.dev->hfdcan == dev->hfdcan &&
        c->base.dev->tx_id == dev->tx_id) {
      ctx = c;
      break;
    }
  }

  if (ctx == NULL) {
    if (s_dm1to4_ctx_used >= MOTOR_DM_1TO4_CTX_MAX)
      return -1;
    ctx = &s_dm1to4_ctx[s_dm1to4_ctx_used++];
    ctx->base.dev = dev;
    ctx->base.drv = &Motor_DAMIAO_1TO4_Driver;
    ctx->staged = 0U;
    ctx->dirty = 0U;
    ctx->commit_ms = 0.0f;
    /* 一拖四的电流上限按板子规格写死；4 个通道同型号共享 */
    ctx->current_max = MOTOR_DM_1TO4_OUTPUT_A_MAX;
    for (uint8_t i = 0; i < MOTOR_DM_1TO4_CH_MAX; i++) {
      ctx->devs[i] = NULL;
      ctx->pending[i] = 0;
      ctx->rx_count[i] = 0U;
      ctx->recover[i] = (dm_recover_t){0};
    }
  }

  ctx->devs[channel] = dev;
  dev->channel = (int8_t)channel;
  dev->ctx = (motor_ctx_t *)ctx;

  return 0;
}

/**
 * @brief 达妙一拖四提交入口
 * @note  由 motor_init.c 的 motor_commit_all() 调用（大疆之外的多机帧）
 */
void Motor_DM1to4_CommitAll(void) {
  uint8_t all_channels = (uint8_t)((1U << MOTOR_DM_1TO4_CH_MAX) - 1U);

  for (uint8_t i = 0; i < s_dm1to4_ctx_used; i++) {
    dm1to4_ctx_t *ctx = &s_dm1to4_ctx[i];
    if (ctx->base.dev == NULL || (ctx->staged & all_channels) != all_channels)
      continue;

    uint8_t changed = ctx->dirty != 0U;
    uint8_t timeout = 0;
#if MOTOR_FRAME_REFRESH_MS > 0
    /* 值没变也定期重发一次，防止电调长时间收不到指令；时间基准是
       DWT 毫秒时间戳（只比较差值） */
    timeout = (DWT_GetTimeline_ms() - ctx->commit_ms) >=
              (float)MOTOR_FRAME_REFRESH_MS;
#endif
    if (!changed && !timeout)
      continue;

    Motor_DM1to4_SendCurrent(ctx->base.dev->hfdcan, ctx->base.dev->tx_id,
                             ctx->pending[0], ctx->pending[1], ctx->pending[2],
                             ctx->pending[3]);
    ctx->dirty = 0U;
    ctx->commit_ms = DWT_GetTimeline_ms();
  }
}

motor_driver_t Motor_DAMIAO_1TO4_Driver = {
    .name = "damiao_1to4",
    .type_id = MOTOR_TYPE_DM_1TO4,
    .ops = &s_dm1to4_ops,
    .probe = Motor_DM1to4_Probe,
    .remove = NULL,
    .next = NULL,
};
