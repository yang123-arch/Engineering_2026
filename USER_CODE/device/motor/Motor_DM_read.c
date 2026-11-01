//=====================================================================================
//                                Motor_DM_read[达妙寄存器读取（临时）]
//
//      协议与用法见 Motor_DM_read.h。
//
//      ops 只有两个：set_value 发读命令（入参是寄存器地址 RID），
//      storage_data 收应答。其余为 NULL，框架调用时直接返回 -1：
//      没有 enable，dev->enabled 一直是 0；没有 recover，error_core 判失联
//      也不会拉起电机。
//
//      Motor_DM_Read_All 一次只让一条读命令在路上：逐台、逐个寄存器发命令，
//      等 ok_mask 对应位被接收中断置上；没等到就重发，重发次数用完记为失败、
//      接着读下一个。
//=====================================================================================

#include "Motor_DM_read.h"
#include "CAN_Driver.h"
#include "cmsis_os2.h"
#include <string.h>

/*===| 读取参数 |===*/
#define DM_READ_TIMEOUT_MS 10U      // 每次读命令等应答的最长时间
#define DM_READ_RETRY_MAX 3U        // 每个寄存器最多发几次
#define DM_READ_DISABLE_WAIT_MS 10U // 强制失能后，等失能帧发完

/**
 * @brief 读取表：寄存器地址与 ok_mask 位一一对应
 */
typedef struct {
  uint8_t rid;
  uint8_t ok_bit;
} dm_read_reg_t;

static const dm_read_reg_t s_dm_read_regs[] = {
    {DM_REG_PMAX, DM_READ_OK_PMAX}, {DM_REG_VMAX, DM_READ_OK_VMAX},
    {DM_REG_TMAX, DM_READ_OK_TMAX}, {DM_REG_ACC, DM_READ_OK_ACC},
    {DM_REG_DEC, DM_READ_OK_DEC},
};
#define DM_READ_REG_NUM (sizeof(s_dm_read_regs) / sizeof(s_dm_read_regs[0]))

DM_MotorRangeParam_t g_dm_range[MOTOR_IDX_MAX];

/**
 * @brief 寄存器地址 -> 读取表项（不在表里返回 NULL）
 */
static const dm_read_reg_t *dm_read_reg_find(uint8_t rid) {
  for (uint32_t k = 0; k < DM_READ_REG_NUM; k++) {
    if (s_dm_read_regs[k].rid == rid)
      return &s_dm_read_regs[k];
  }
  return NULL;
}

/**
 * @brief 寄存器地址 -> 结构体字段
 */
static float *dm_read_field(DM_MotorRangeParam_t *p, uint8_t rid) {
  switch (rid) {
  case DM_REG_ACC:
    return &p->acc;
  case DM_REG_DEC:
    return &p->dec;
  case DM_REG_PMAX:
    return &p->p_max;
  case DM_REG_VMAX:
    return &p->v_max;
  case DM_REG_TMAX:
    return &p->t_max;
  default:
    return NULL;
  }
}

/**
 * @brief 设备 -> 结果槽位
 * @note  结果按 g_motor_devices[] 下标存；设备表有空行时注册序号会和下标
 *        错开，这时对不上就不存（该电机最后记为失败）
 */
static DM_MotorRangeParam_t *dm_read_slot(const motor_device_t *dev) {
  if (dev->index >= g_motor_device_count || dev->index >= MOTOR_IDX_MAX)
    return NULL;
  if (&g_motor_devices[dev->index] != dev)
    return NULL;
  return &g_dm_range[dev->index];
}

/*===|==================== 操作表、probe ====================|===*/

/**
 * @brief set_value：入参是寄存器地址 RID，发一帧读命令（不等应答）
 * @return 0 已发出；-1 不是读取表里的寄存器
 */
static int Motor_DM_Read_Ops_SetValue(motor_device_t *dev, float value) {
  if (value < 0.0f || value > 255.0f)
    return -1;

  uint8_t rid = (uint8_t)value;
  if (dm_read_reg_find(rid) == NULL)
    return -1;

  uint8_t data[8] = {(uint8_t)(dev->tx_id & 0xFFU),
                     (uint8_t)(dev->tx_id >> 8),
                     DM_READ_PARAM_CMD,
                     rid,
                     0U,
                     0U,
                     0U,
                     0U};
  CAN_Send_Data(dev->hfdcan, DM_READ_PARAM_CAN_ID, data);
  return 0;
}

/**
 * @brief storage_data：读应答存进 g_dm_range[设备下标]
 * @note  达妙一问一答，本驱动只发读命令，收到的只会是读应答；
 *        格式对不上的帧直接丢掉
 */
static void Motor_DM_Read_Ops_StorageData(motor_device_t *dev,
                                          const uint8_t *data) {
  if (data[2] != DM_READ_PARAM_CMD ||
      data[0] != (uint8_t)(dev->tx_id & 0xFFU) ||
      data[1] != (uint8_t)(dev->tx_id >> 8))
    return;

  dev->data.If_Online = 1U;

  const dm_read_reg_t *reg = dm_read_reg_find(data[3]);
  DM_MotorRangeParam_t *p = dm_read_slot(dev);
  if (reg == NULL || p == NULL)
    return;

  float *field = dm_read_field(p, reg->rid);
  if (field == NULL)
    return;

  memcpy(field, &data[4], sizeof(float)); /* D4~D7 小端，和本机字节序一致 */
  p->ok_mask |= reg->ok_bit;
}

static const motor_ops_t s_dm_read_ops = {
    .enable = NULL,
    .disable = NULL,
    .clear = NULL,
    .set_zero = NULL,
    .recover = NULL,
    .storage_data = Motor_DM_Read_Ops_StorageData,
    .set_value = Motor_DM_Read_Ops_SetValue,
};

static int Motor_DM_Read_Probe(motor_device_t *dev) {
  if (dev->hfdcan == NULL)
    return -1;

  dev->ctx = NULL; /* 结果存在 g_dm_range，不需要私有上下文 */
  dev->channel = 0;
  return 0;
}

motor_driver_t Motor_DM_Read_Driver = {
    .name = "damiao_read",
    .type_id = MOTOR_TYPE_DM_READ,
    .ops = &s_dm_read_ops,
    .probe = Motor_DM_Read_Probe,
    .remove = NULL,
    .next = NULL,
};

int Motor_DM_Read_Register(void) {
  return motor_driver_register(&Motor_DM_Read_Driver);
}

/*===|==================== 读取流程 ====================|===*/

int Motor_DM_Read_All(void) {
  /* 整机强制失能：motor_disable 不看 enabled 标志，有 disable 操作的驱动
     （凌空等）一定会发失能帧。挂本驱动的达妙没有 disable/enable，
     上电就是失能，也没有人能把它拉起来 */
  for (uint16_t i = 0; i < g_motor_device_count; i++)
    motor_disable(&g_motor_devices[i]);
  osDelay(DM_READ_DISABLE_WAIT_MS);

  int done = 0;
  for (uint16_t i = 0; i < g_motor_device_count && i < MOTOR_IDX_MAX; i++) {
    motor_device_t *dev = &g_motor_devices[i];
    DM_MotorRangeParam_t *p = &g_dm_range[i];

    if (dev->ops != &s_dm_read_ops)
      continue;

    memset(p, 0, sizeof(*p));
    p->state = DM_READ_BUSY;

    for (uint32_t k = 0; k < DM_READ_REG_NUM; k++) {
      uint8_t bit = s_dm_read_regs[k].ok_bit;

      for (uint32_t n = 0; n < DM_READ_RETRY_MAX && (p->ok_mask & bit) == 0U;
           n++) {
        motor_set_value(dev, (float)s_dm_read_regs[k].rid);
        for (uint32_t t = 0; t < DM_READ_TIMEOUT_MS && (p->ok_mask & bit) == 0U;
             t++)
          osDelay(1);
      }
    }

    if (p->ok_mask == DM_READ_OK_ALL) {
      p->state = DM_READ_DONE;
      done++;
    } else {
      p->state = DM_READ_FAIL;
    }
  }

  return done;
}
