//=====================================================================================
//                                Motor_DJI_Driver[大疆电机驱动]
//
//      协议（RoboMaster 电调，电流环）：
//        标识符 0x200 -> 电调 ID 1~4，DATA[0..7] 依次为 4 个电调电流的高/低字节
//        标识符 0x1FF -> 电调 ID 5~8，格式同上
//        电流量程 -16384 ~ 0 ~ 16384 对应输出转矩电流 -20 ~ 0 ~ 20 A（大端）
//
//      这条协议天然是"一帧多机"，所以驱动自带一个提交管理器：
//
//        probe       : 按 rx_id(0x201~0x208) 把设备挂到 (总线, 电调ID) 槽位上
//        set_value   : 换算成电调电流 + 写 cache + 置 dirty，不发报文
//        commit_hook : 由 motor_commit_all() 调用，把每条总线上两条帧发出去
//
//      上层只需要 motor_set_value() + motor_commit_all(&group)，
//      完全不需要知道 0x200/0x1FF 或 dirty 的存在。
//=====================================================================================

#include "Motor_DJI_Driver.h"
#include "CAN_Driver.h"
#include "DWT.h"
#include "Define.h"
#include "motor_core.h"

/*===| 电调 ID 与反馈 ID 的映射：反馈 ID = 0x200 + 电调 ID ===|*/
#define MOTOR_DJI_RX_ID_BASE 0x200U
#define MOTOR_DJI_ID_MIN 1U
#define MOTOR_DJI_ID_MAX 8U

/*===| 两条广播帧 ===|*/
#define MOTOR_DJI_TX_ID_LOW 0x200U  // 电调 1~4
#define MOTOR_DJI_TX_ID_HIGH 0x1FFU // 电调 5~8
#define MOTOR_DJI_ID_PER_FRAME 4U   // 每条帧 4 个电调
#define MOTOR_DJI_IDS_PER_BUS 8U    // 每条总线 8 个电调

/**
 * @brief 一条总线上的大疆槽位管理器
 * @note  motors 下标 = 电调 ID - 1；cache/dirty 与之一一对应
 */
typedef struct {
  motor_device_t *motors[MOTOR_DJI_IDS_PER_BUS]; // 索引 0~7 对应电调 ID 1~8
  int16_t cache[MOTOR_DJI_IDS_PER_BUS]; // 当前控制值（电调原始单位）
  volatile uint8_t staged;              // 曾经写过值的槽位
  volatile uint8_t dirty; // 自上次提交后被更新过的槽位
  float commit_ms;        // 上次发帧时的 DWT 时间戳[ms]
  uint8_t used;
} dji_can_slot_t;

/**
 * @brief 大疆 per-device 私有上下文
 * @note  slot 共享（一帧多机），但限幅是 per-device 的：
 *        上层 motor_set_value 收力矩 [N·m]，按本型号 torque_max 放缩到
 *        ±16384 写入 slot cache。
 */
typedef struct {
  motor_ctx_t base;
  float torque_max;     // 力矩上限 [N·m]，probe 时按型号写死
  dji_can_slot_t *slot; // 回指共享槽位管理器
} dji_dev_ctx_t;

#define MOTOR_DJI_DEV_CTX_MAX 16U
static dji_dev_ctx_t s_dji_dev_ctx[MOTOR_DJI_DEV_CTX_MAX];
static uint8_t s_dji_dev_ctx_used;

static dji_can_slot_t s_dji_slots[3]; // CAN1 / CAN2 / CAN3

/*===|==================== 底层协议函数 ====================|===*/

/**
 * @brief 总线指针 -> 槽位下标（不是这三条总线则返回 -1）
 */
static int8_t Motor_DJI_BusIndex(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == &hfdcan1)
    return 0;
  if (hfdcan == &hfdcan2)
    return 1;
  if (hfdcan == &hfdcan3)
    return 2;
  return -1;
}

/**
 * @brief 大疆电机发送电流（原始接口，需自己指定总线与 ID）
 */
void Motor_DJI_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint16_t CAN_ID,
                           int16_t ID1_Currnet, int16_t ID2_Currnet,
                           int16_t ID3_Currnet, int16_t ID4_Currnet) {
  uint8_t Motor_Tx_Data[8];

  Motor_Tx_Data[0] = (uint8_t)(ID1_Currnet >> 8);
  Motor_Tx_Data[1] = (uint8_t)(ID1_Currnet & 0xFF);
  Motor_Tx_Data[2] = (uint8_t)(ID2_Currnet >> 8);
  Motor_Tx_Data[3] = (uint8_t)(ID2_Currnet & 0xFF);
  Motor_Tx_Data[4] = (uint8_t)(ID3_Currnet >> 8);
  Motor_Tx_Data[5] = (uint8_t)(ID3_Currnet & 0xFF);
  Motor_Tx_Data[6] = (uint8_t)(ID4_Currnet >> 8);
  Motor_Tx_Data[7] = (uint8_t)(ID4_Currnet & 0xFF);

  CAN_Send_Data(hfdcan, CAN_ID, Motor_Tx_Data);
}

/**
 * @brief 存储大疆电机状态参数并计算得到总角度
 *
 * @param 电调发送的数组
 * @param 要存储到的电机数据结构体
 */
void Motor_DJI_Storage_Data(uint8_t *Data,
                            Motor_Data_StructTypeDef *Motor_Data_Struct) {
  /*===| 协议解包 |===*/
  Motor_Data_Struct->Encoder = (int16_t)(Data[0] << 8 | Data[1]);
  Motor_Data_Struct->Speed_RPM = (int16_t)(Data[2] << 8 | Data[3]);
  Motor_Data_Struct->Torque = (int16_t)(Data[4] << 8 | Data[5]);
  Motor_Data_Struct->Temperature = (int8_t)(Data[6]);

  Motor_Data_Struct->Angle =
      (Motor_Data_Struct->Encoder - 4096.0f) * 180.0f / 4096.0f;

  /*===| 得到总角度值 |===*/
  if (Motor_Data_Struct->Encoder - Motor_Data_Struct->Encoder_Last > 4096)
    Motor_Data_Struct->Round--;
  else if (Motor_Data_Struct->Encoder - Motor_Data_Struct->Encoder_Last < -4096)
    Motor_Data_Struct->Round++;
  Motor_Data_Struct->Total_Angle =
      360.0f *
      (Motor_Data_Struct->Round + Motor_Data_Struct->Encoder / 8192.0f);

  /*===| 记录编码器值 |===*/
  Motor_Data_Struct->Encoder_Last = Motor_Data_Struct->Encoder;
  Motor_Data_Struct->Angle_Last = Motor_Data_Struct->Angle;

  Motor_Data_Struct->If_Online = 1;
}

/*===|==================== 提交管理器 ====================|===*/

/**
 * @brief 把一条总线上的一条帧（4 个电调）发出去
 * @param slot  槽位管理器
 * @param tx_id 本帧标识符（0x200 或 0x1FF）
 * @param first 本帧第一个电调的下标（0 表示 ID 1~4，4 表示 ID 5~8）
 * @note  已失能的电机 cache 已被驱动置 0，这里直接取 cache
 */
static void Motor_DJI_CommitFrame(FDCAN_HandleTypeDef *hfdcan, uint16_t tx_id,
                                  dji_can_slot_t *slot, uint8_t first) {
  uint8_t mask = 0;
  uint8_t ready = 0;
  uint16_t frame_tx_id = tx_id;

  for (uint8_t i = 0; i < MOTOR_DJI_ID_PER_FRAME; i++) {
    motor_device_t *m = slot->motors[first + i];
    uint8_t bit = (uint8_t)(1U << (first + i));

    /* 空槽位不是本帧的受控通道。高帧常见只有 ID6 等稀疏设备，
       不能因为 ID5/7/8 未注册就把整帧判为未就绪。 */
    if (m == NULL)
      continue;

    mask |= bit;
    /* 设备表允许为兼容旧硬件配置非标准广播 ID（相机 X 使用 0x1FE）。
       同一帧内的设备应共享发送 ID，取本帧任一已注册设备的配置即可。 */
    frame_tx_id = m->tx_id;

    /* 已失能的电机：本周期直接输出 0，避免把失能前的旧电流又发出去 */
    if (m != NULL && m->enabled == 0U)
      slot->cache[first + i] = 0;

    /* 该槽位挂了电机且已经写过值，才算"本帧这一路可用" */
    if (m != NULL && (slot->staged & bit) != 0U)
      ready |= bit;
  }

  if (mask == 0U)
    return;

  /* 本帧有电调还没进入控制环：整帧不发，避免把没控的电机写成 0 电流 */
  if (ready != mask)
    return;

  /* 值没变就不发；但超过 MOTOR_FRAME_REFRESH_MS 没发过则强制刷新一次，
     防止电调长时间收不到指令（也兜住 CAN 丢帧）。时间基准是 DWT 毫秒
     时间戳，只比较差值，回绕也不怕 */
  uint8_t changed = (slot->dirty & mask) != 0U;
  uint8_t timeout = 0;
#if MOTOR_FRAME_REFRESH_MS > 0
  timeout =
      (DWT_GetTimeline_ms() - slot->commit_ms) >= (float)MOTOR_FRAME_REFRESH_MS;
#endif
  if (!changed && !timeout)
    return;

  Motor_DJI_SendCurrent(hfdcan, frame_tx_id, slot->cache[first],
                        slot->cache[first + 1U], slot->cache[first + 2U],
                        slot->cache[first + 3U]);
  slot->dirty &= (uint8_t)~mask;
  slot->commit_ms = DWT_GetTimeline_ms();
}

/**
 * @brief 提交一条总线上的两条帧
 */
static void Motor_DJI_CommitBus(FDCAN_HandleTypeDef *hfdcan,
                                dji_can_slot_t *slot) {
  if (slot == NULL || slot->used == 0U)
    return;

  Motor_DJI_CommitFrame(hfdcan, MOTOR_DJI_TX_ID_LOW, slot, 0);
  Motor_DJI_CommitFrame(hfdcan, MOTOR_DJI_TX_ID_HIGH, slot,
                        MOTOR_DJI_ID_PER_FRAME);
}

/**
 * @brief 大疆提交：把 3 条总线上更新过的帧发出去
 * @note  由 motor_init.c 的 motor_commit_all() 直接调用。
 *        用固定 ID 调用是因为帧 cache 是按 (总线, 电调ID) 组织的，
 *        与"这一组里有哪几个大疆设备"无关。
 */
void Motor_DJI_CommitAll(void) {
  Motor_DJI_CommitBus(&hfdcan1, &s_dji_slots[0]);
  Motor_DJI_CommitBus(&hfdcan2, &s_dji_slots[1]);
  Motor_DJI_CommitBus(&hfdcan3, &s_dji_slots[2]);
}

/*===|==================== 操作表实现 ====================|===*/

/**
 * @brief 反馈解包：中断上下文调用，要短
 */
static void Motor_DJI_Ops_StorageData(motor_device_t *dev,
                                      const uint8_t *data) {
  Motor_DJI_Storage_Data((uint8_t *)data, &dev->data);
}

/**
 * @brief 力矩[N·m] -> 电调原始值，写 cache + 置 dirty（不发报文，由
 *        commit_hook 统一发）
 * @note  输入语义：力矩 [N·m]。驱动按本型号 torque_max 限幅并放缩到
 *        ±16384 电调原始值。
 */
static int Motor_DJI_Ops_SetValue(motor_device_t *dev, float value) {
  if (dev->channel < 0 || dev->channel >= (int8_t)MOTOR_DJI_IDS_PER_BUS)
    return -1;

  dji_dev_ctx_t *ctx = (dji_dev_ctx_t *)dev->ctx;
  if (ctx == NULL || ctx->slot == NULL)
    return -1;

  /* 1. 按本型号力矩上限限幅，并回写为"实际下发值" */
  float t_max = ctx->torque_max;
  if (value > t_max)
    value = t_max;
  else if (value < -t_max)
    value = -t_max;
  dev->output = value;

  /* 2. 放缩到电调原始单位：±torque_max 对应 ±16384 */
  int32_t raw = 0;
  if (t_max > 0.0f)
    raw = (int32_t)(value * ((float)MOTOR_DJI_CURRENT_MAX / t_max));

  if (raw > MOTOR_DJI_CURRENT_MAX)
    raw = MOTOR_DJI_CURRENT_MAX;
  else if (raw < MOTOR_DJI_CURRENT_MIN)
    raw = MOTOR_DJI_CURRENT_MIN;

  ctx->slot->cache[dev->channel] = (int16_t)raw;
  ctx->slot->staged |= (uint8_t)(1U << dev->channel);
  ctx->slot->dirty |= (uint8_t)(1U << dev->channel);

  return 0;
}

/**
 * @brief 使能：大疆电调没有使能指令，标记后按正常值发帧即可
 */
static int Motor_DJI_Ops_Enable(motor_device_t *dev) {
  /* 大疆没有使能指令，只把标志置位；框架会在返回 0 后写 dev->enabled */
  (void)dev;
  return 0;
}

/**
 * @brief 失能：把本槽位置 0 并置 dirty，下一帧起输出 0 电流
 */
static int Motor_DJI_Ops_Disable(motor_device_t *dev) {
  if (dev->channel < 0 || dev->channel >= (int8_t)MOTOR_DJI_IDS_PER_BUS)
    return -1;

  dji_dev_ctx_t *ctx = (dji_dev_ctx_t *)dev->ctx;
  if (ctx == NULL || ctx->slot == NULL)
    return -1;

  ctx->slot->cache[dev->channel] = 0;
  ctx->slot->staged |= (uint8_t)(1U << dev->channel);
  ctx->slot->dirty |= (uint8_t)(1U << dev->channel);
  ctx->slot->commit_ms = DWT_GetTimeline_ms();

  return 0;
}

/**
 * @brief 恢复（错误处理）：大疆电调没有清故障/使能指令，失联后的处理就是
 *        立刻停机 —— 把本槽位电流置 0 并置 dirty，下一帧先输出 0 电流。
 *        应用层的 enabled 状态保持不变，反馈恢复后控制环可继续下发。
 * @note  单步流程，没有延时，所以一次就返回 1。
 */
static int Motor_DJI_Ops_Recover(motor_device_t *dev) {
  /* 大疆没有独立的失能协议。只清零缓存并标记 dirty，保留应用层的
     enabled 状态，这样短暂失联恢复后控制环可以继续下发电流。 */
  (void)Motor_DJI_Ops_Disable(dev);
  return 1;
}

/*===|==================== 驱动注册 ====================|===*/

/**
 * @brief probe：认领设备并挂到 (总线, 电调ID) 槽位；按型号写死力矩上限
 * @note  电调 ID 由反馈 ID 推出（0x201~0x208 -> 1~8），其余 ID 直接拒绝
 */
static int Motor_DJI_Probe(motor_device_t *dev) {
  int8_t bus = Motor_DJI_BusIndex(dev->hfdcan);
  if (bus < 0)
    return -1;

  uint16_t id = (uint16_t)(dev->rx_id - MOTOR_DJI_RX_ID_BASE);
  if (id < MOTOR_DJI_ID_MIN || id > MOTOR_DJI_ID_MAX)
    return -1;

  if (s_dji_dev_ctx_used >= MOTOR_DJI_DEV_CTX_MAX)
    return -1;

  uint8_t channel = (uint8_t)(id - 1U);
  dji_can_slot_t *slot = &s_dji_slots[bus];
  dji_dev_ctx_t *ctx = &s_dji_dev_ctx[s_dji_dev_ctx_used++];

  slot->motors[channel] = dev;
  slot->used = 1U;

  ctx->base.dev = dev;
  ctx->base.drv = dev->driver;
  ctx->slot = slot;

  /* 按型号写死力矩上限 [N·m]（不再从 motor_driver_t 读） */
  switch (dev->type_id) {
  case MOTOR_TYPE_DJI_3508:
    ctx->torque_max = MOTOR_DJI_3508_TORQUE_MAX;
    break;
  case MOTOR_TYPE_DJI_2006:
    ctx->torque_max = MOTOR_DJI_2006_TORQUE_MAX;
    break;
  case MOTOR_TYPE_DJI_6020:
    ctx->torque_max = MOTOR_DJI_6020_TORQUE_MAX;
    break;
  case MOTOR_TYPE_DJI_LEGACY:
    /* 兼容型号：±20 等效电流 -> ±16384 raw，与旧版行为一致 */
    ctx->torque_max = MOTOR_DJI_LEGACY_TORQUE_MAX;
    break;
  default:
    return -1;
  }

  dev->channel = (int8_t)channel;
  dev->ctx = (motor_ctx_t *)ctx;

  return 0;
}

static int Motor_DJI_Remove(motor_device_t *dev) {
  dji_dev_ctx_t *ctx = (dji_dev_ctx_t *)dev->ctx;

  if (ctx != NULL && ctx->slot != NULL && dev->channel >= 0) {
    dji_can_slot_t *slot = ctx->slot;
    slot->motors[dev->channel] = NULL;
    slot->cache[dev->channel] = 0;
    slot->staged &= (uint8_t) ~(1U << dev->channel);
    slot->dirty &= (uint8_t) ~(1U << dev->channel);
    ctx->slot = NULL;
  }

  dev->ctx = NULL;
  dev->channel = -1;
  return 0;
}

static const motor_ops_t s_dji_ops = {
    .enable = Motor_DJI_Ops_Enable,
    .disable = Motor_DJI_Ops_Disable,
    .clear = NULL,    // 大疆电调无清故障指令
    .set_zero = NULL, // 无设置零点指令
    .recover = Motor_DJI_Ops_Recover,
    .storage_data = Motor_DJI_Ops_StorageData,
    .set_value = Motor_DJI_Ops_SetValue,
};

/*===| 每种型号一个驱动：协议与 ops 相同，仅识别号与额定力矩不同 ===|*/
motor_driver_t Motor_DJI_3508_Driver = {
    .name = "dji_3508",
    .type_id = MOTOR_TYPE_DJI_3508,
    .ops = &s_dji_ops,
    .probe = Motor_DJI_Probe,
    .remove = Motor_DJI_Remove,
    .next = NULL,
};

motor_driver_t Motor_DJI_2006_Driver = {
    .name = "dji_2006",
    .type_id = MOTOR_TYPE_DJI_2006,
    .ops = &s_dji_ops,
    .probe = Motor_DJI_Probe,
    .remove = Motor_DJI_Remove,
    .next = NULL,
};

motor_driver_t Motor_DJI_6020_Driver = {
    .name = "dji_6020",
    .type_id = MOTOR_TYPE_DJI_6020,
    .ops = &s_dji_ops,
    .probe = Motor_DJI_Probe,
    .remove = Motor_DJI_Remove,
    .next = NULL,
};

/* 兼容型号：沿用旧工程 ±20A 满量程语义（±20 -> ±16384 raw），
   旧设备表项换 type_id 即恢复原参数；新写的设备请用具体型号 */
motor_driver_t Motor_DJI_Legacy_Driver = {
    .name = "dji_legacy",
    .type_id = MOTOR_TYPE_DJI_LEGACY,
    .ops = &s_dji_ops,
    .probe = Motor_DJI_Probe,
    .remove = Motor_DJI_Remove,
    .next = NULL,
};

/**
 * @brief 注册本文件支持的全部大疆电机驱动
 */
int Motor_DJI_Register(void) {
  int ret = 0;

  ret |= motor_driver_register(&Motor_DJI_3508_Driver);
  ret |= motor_driver_register(&Motor_DJI_2006_Driver);
  ret |= motor_driver_register(&Motor_DJI_6020_Driver);
  ret |= motor_driver_register(&Motor_DJI_Legacy_Driver);

  return ret;
}
