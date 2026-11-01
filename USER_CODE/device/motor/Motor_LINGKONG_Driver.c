//=====================================================================================
//                                Motor_LINGKONG_Driver[凌空电机驱动]
//
//      单通道驱动：一条报文一个电机，set_value 直接发转矩电流，不参与提交管理器。
//      驱动内部无 per-device static 状态：每台电机一块上下文，probe 阶段挂上。
//=====================================================================================

#include "Motor_LINGKONG_Driver.h"
#include "CAN_Driver.h"
#include "DWT.h"

#define MOTOR_LK_CTX_MAX 16U

/*===| 失联恢复参数 |===*/
#define MOTOR_LK_RECOVER_CLEAR_WAIT_US 1000U // 清故障到使能的间隔（同老工程）
#define MOTOR_LK_RECOVER_TIMEOUT_US 2000U    // 使能后等回码的最长时间
#define MOTOR_LK_RECOVER_RETRY_MAX 3U        // 连续失败几次后进入退避
#define MOTOR_LK_RECOVER_HOLDOFF_US 30000U   // 退避时长：失联电机不刷屏总线

/*===|==================== 底层协议函数 ====================|===*/

/**
 * @brief 读取凌空电机状态2命令
 */
void Motor_LK_Send_Storage_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t LK_Send_Storage_Data[8] = {0x9C, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00};
  CAN_Send_Data(hfdcan, CAN_ID, LK_Send_Storage_Data);
}

/**
 * @brief 凌空电机反馈数据
 *
 * @param 电机发送的数组
 * @param 要存储到的电机数据结构体
 */
void Motor_LK_Storage_Data(uint8_t *Data, Motor_Data_StructTypeDef *data) {
  if (Data[0] != 0xA1)
    return;

  /*===| 协议解包 |===*/
  data->Temperature = (int8_t)Data[1];
  data->Torque = (int16_t)(Data[3] << 8 | Data[2]);
  data->Speed_RPM = (int16_t)(Data[5] << 8 | Data[4]) / 60.0f; // °/s -> RPM
  data->Encoder = (int16_t)(Data[7] << 8 | Data[6]);           // 0~65535

  data->Angle = (data->Encoder) * 360.0f / 65536.0f;

  /*===| 得到总角度值 |===*/
  if (data->Encoder - data->Encoder_Last > 32768)
    data->Round--;
  else if (data->Encoder - data->Encoder_Last < -32768)
    data->Round++;
  data->Total_Angle = 360.0f * (data->Round + data->Encoder / 65536.0f);

  /*===| 记录状态 |===*/
  data->Encoder_Last = data->Encoder;
  data->Angle_Last = data->Angle;

  data->If_Online = 1;
}

/**
 * @brief 凌空电机使能
 */
void Motor_LK_CMD_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t LK_Enable_Data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN_Send_Data(hfdcan, CAN_ID, LK_Enable_Data);
}

/**
 * @brief 凌空电机失能
 */
void Motor_LK_CMD_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t LK_Disable_Data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN_Send_Data(hfdcan, CAN_ID, LK_Disable_Data);
}

/**
 * @brief 凌空电机清除错误
 */
void Motor_LK_CMD_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t LK_ClearErr_Data[8] = {0x9B, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00};
  CAN_Send_Data(hfdcan, CAN_ID, LK_ClearErr_Data);
}

/**
 * @brief 凌空电机设置零点
 */
void Motor_LK_CMD_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID) {
  uint8_t LK_SetZero_Data[8] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN_Send_Data(hfdcan, CAN_ID, LK_SetZero_Data);
}

/**
 * @brief 凌空电机转矩电流控制
 *
 * @param Current 转矩电流指令
 */
void Motor_LK_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                          int16_t Current) {
  uint8_t LK_SendCurrent_Data[8] = {0xA1, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00};
  LK_SendCurrent_Data[4] = (uint8_t)(Current & 0xFF);
  LK_SendCurrent_Data[5] = (uint8_t)((Current >> 8) & 0xFF);
  CAN_Send_Data(hfdcan, CAN_ID, LK_SendCurrent_Data);
}

/*===|==================== 驱动上下文与操作表 ====================|===*/

/**
 * @brief 恢复状态机的阶段
 */
typedef enum {
  LK_RECOVER_IDLE = 0,    // 空闲：下一次进来发清故障
  LK_RECOVER_WAIT_CLEAR,  // 已发清故障，等固定间隔
  LK_RECOVER_WAIT_ENABLE, // 已发使能，等回码
  LK_RECOVER_HOLDOFF,     // 连续失败，退避中
} lk_recover_state_t;

/**
 * @brief 恢复状态机
 * @note  "有没有新回码"用回码计数和发使能时的快照比较，不看 If_Online
 *        （会被 error_core 的轮询一次性清零）
 */
typedef struct {
  uint8_t state;    // lk_recover_state_t
  uint8_t fail;     // 连续失败次数（到 RETRY_MAX 后饱和）
  uint32_t rx_snap; // 发使能时的回码计数快照
  uint64_t t0_us;   // 当前阶段起始时间[us]
} lk_recover_t;

typedef struct {
  motor_ctx_t base;
  float current_max; // set_value 限幅（单位同 set_value 入参），probe 时按型号写死
  float raw_per_unit; // set_value 入参 × 它 = 协议原始值；原始值型号为 1
  volatile uint32_t rx_count; // 回码计数：接收中断里加，恢复流程只读
  lk_recover_t recover;       // 失联恢复状态机
} lk_ctx_t;

static lk_ctx_t s_lk_ctx[MOTOR_LK_CTX_MAX];
static uint8_t s_lk_ctx_used;

static void Motor_LK_Ops_StorageData(motor_device_t *dev,
                                     const uint8_t *data) {
  lk_ctx_t *ctx = (lk_ctx_t *)dev->ctx;
  Motor_LK_Storage_Data((uint8_t *)data, &dev->data);

  /* 反馈力矩换成和 set_value 同一单位（只有转矩帧 0xA1 才更新 Torque） */
  if (ctx != NULL && data[0] == 0xA1 && ctx->raw_per_unit != 1.0f)
    dev->data.Torque /= ctx->raw_per_unit;

  /* 任何回帧都计数（包括命令回显）：能回帧就说明电机在总线上 */
  if (ctx != NULL)
    ctx->rx_count++;
}

static int Motor_LK_Ops_Enable(motor_device_t *dev) {
  Motor_LK_CMD_Enable(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_LK_Ops_Disable(motor_device_t *dev) {
  Motor_LK_CMD_Disable(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_LK_Ops_Clear(motor_device_t *dev) {
  Motor_LK_CMD_ClearErr(dev->hfdcan, dev->tx_id);
  return 0;
}

static int Motor_LK_Ops_SetZero(motor_device_t *dev) {
  Motor_LK_CMD_SetZero(dev->hfdcan, dev->tx_id);
  return 0;
}

/**
 * @brief 单通道提交：按本型号限幅后直接发转矩电流帧（无需等待 commit_all）
 */
static int Motor_LK_Ops_SetValue(motor_device_t *dev, float value) {
  lk_ctx_t *ctx = (lk_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return -1;

  /* 按本型号限幅并回写为"实际下发值"（单位同入参），再换成 int16 原始值 */
  float i_max = ctx->current_max;
  if (value > i_max)
    value = i_max;
  else if (value < -i_max)
    value = -i_max;
  dev->output = value;

  Motor_LK_SendCurrent(dev->hfdcan, dev->tx_id,
                       (int16_t)(value * ctx->raw_per_unit));
  return 0;
}

/**
 * @brief 一轮尝试失败：没到次数就立刻重试，否则进入退避
 * @note  失败计数饱和在 RETRY_MAX：连续失败 3 次后每 30ms 试一次
 * @return 1 调用方马上开始新一轮；0 已进入退避
 */
static uint8_t lk_recover_fail(lk_recover_t *rc, uint64_t now) {
  if (rc->fail < MOTOR_LK_RECOVER_RETRY_MAX)
    rc->fail++;
  if (rc->fail < MOTOR_LK_RECOVER_RETRY_MAX)
    return 1U;

  rc->t0_us = now;
  rc->state = LK_RECOVER_HOLDOFF;
  return 0U;
}

/**
 * @brief 恢复（错误处理）：清故障 -> 等 1ms -> 使能 -> 等回码确认
 * @note  凌空失能后不会自己恢复输出，所以失联自愈必须走"清故障 + 重新使能"。
 *        这里的反馈没有使能状态字段，只能按"使能之后收到过回码"算成功；
 *        2ms 内没收到算失败，连续失败 3 次后每轮先退避 30ms。
 *        应用失能（dev->enabled == 0）时不拉起电机；已经发过使能帧的补发
 *        一帧失能，抵消可能后到的使能
 * @return 1 流程走完（恢复成功 / 应用已失能）；0 未走完
 */
static int Motor_LK_Ops_Recover(motor_device_t *dev) {
  lk_ctx_t *ctx = (lk_ctx_t *)dev->ctx;
  if (ctx == NULL)
    return 1; /* 上下文丢了：算完成，别让上层死等 */

  lk_recover_t *rc = &ctx->recover;

  if (dev->enabled == 0U) {
    if (rc->state == LK_RECOVER_WAIT_ENABLE)
      Motor_LK_CMD_Disable(dev->hfdcan, dev->tx_id);
    rc->state = LK_RECOVER_IDLE;
    rc->fail = 0U;
    return 1;
  }

  uint32_t rx = ctx->rx_count;
  uint64_t now = DWT_GetTimeline_us();

  /* 发过使能之后（含退避期间迟到的）收到回码：恢复成功 */
  if ((rc->state == LK_RECOVER_WAIT_ENABLE ||
       rc->state == LK_RECOVER_HOLDOFF) &&
      rx != rc->rx_snap) {
    rc->state = LK_RECOVER_IDLE;
    rc->fail = 0U;
    return 1;
  }

  switch (rc->state) {
  case LK_RECOVER_WAIT_CLEAR:
    if ((now - rc->t0_us) >= MOTOR_LK_RECOVER_CLEAR_WAIT_US) {
      rc->rx_snap = rx;
      rc->t0_us = now;
      rc->state = LK_RECOVER_WAIT_ENABLE;
      Motor_LK_CMD_Enable(dev->hfdcan, dev->tx_id);
    }
    return 0;

  case LK_RECOVER_WAIT_ENABLE:
    if ((now - rc->t0_us) < MOTOR_LK_RECOVER_TIMEOUT_US)
      return 0;
    if (lk_recover_fail(rc, now) == 0U)
      return 0; /* 进入退避 */
    break;      /* 立刻开始新一轮 */

  case LK_RECOVER_HOLDOFF:
    if ((now - rc->t0_us) < MOTOR_LK_RECOVER_HOLDOFF_US)
      return 0;
    break;

  default: /* LK_RECOVER_IDLE */
    break;
  }

  /* 新一轮：清故障 */
  rc->t0_us = now;
  rc->state = LK_RECOVER_WAIT_CLEAR;
  Motor_LK_CMD_ClearErr(dev->hfdcan, dev->tx_id);
  return 0;
}

static const motor_ops_t s_lk_ops = {
    .enable = Motor_LK_Ops_Enable,
    .disable = Motor_LK_Ops_Disable,
    .clear = Motor_LK_Ops_Clear,
    .set_zero = Motor_LK_Ops_SetZero,
    .recover = Motor_LK_Ops_Recover,
    .storage_data = Motor_LK_Ops_StorageData,
    .set_value = Motor_LK_Ops_SetValue,
};

static int Motor_LK_Probe(motor_device_t *dev) {
  if (s_lk_ctx_used >= MOTOR_LK_CTX_MAX)
    return -1;

  lk_ctx_t *ctx = &s_lk_ctx[s_lk_ctx_used++];
  ctx->base.dev = dev;
  ctx->base.drv = dev->driver;
  if (dev->type_id == MOTOR_TYPE_LK_MG4010E) {
    /* 收 N·m，按 MOTOR_LK_MG4010E_T_MAX 限幅，满量程换成原始值 ±2048 */
    ctx->current_max = MOTOR_LK_MG4010E_T_MAX;
    ctx->raw_per_unit = MOTOR_LK_MG4010E_RAW_MAX / MOTOR_LK_MG4010E_T_MAX;
  } else {
    /* 指令直接是协议原始值，按 int16 满量程限幅 */
    ctx->current_max = (float)MOTOR_LK_CURRENT_MAX;
    ctx->raw_per_unit = 1.0f;
  }
  ctx->rx_count = 0U;
  ctx->recover = (lk_recover_t){0};

  dev->ctx = (motor_ctx_t *)ctx;
  dev->channel = 0;

  return 0;
}

/*===| 每种型号一个驱动 ===|*/
motor_driver_t Motor_LK_MG6010_Driver = {
    .name = "lingkong_mg6010",
    .type_id = MOTOR_TYPE_LK_MG6010,
    .ops = &s_lk_ops,
    .probe = Motor_LK_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_LK_MG8010_Driver = {
    .name = "lingkong_mg8010",
    .type_id = MOTOR_TYPE_LK_MG8010,
    .ops = &s_lk_ops,
    .probe = Motor_LK_Probe,
    .remove = NULL,
    .next = NULL,
};

motor_driver_t Motor_LK_MG4010E_Driver = {
    .name = "lingkong_mg4010e",
    .type_id = MOTOR_TYPE_LK_MG4010E,
    .ops = &s_lk_ops,
    .probe = Motor_LK_Probe,
    .remove = NULL,
    .next = NULL,
};

/**
 * @brief 注册本文件支持的全部凌空驱动
 */
int Motor_LINGKONG_Register(void) {
  int ret = 0;

  ret |= motor_driver_register(&Motor_LK_MG6010_Driver);
  ret |= motor_driver_register(&Motor_LK_MG8010_Driver);
  ret |= motor_driver_register(&Motor_LK_MG4010E_Driver);

  return ret;
}
