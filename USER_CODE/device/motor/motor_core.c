//=====================================================================================
//                                 motor_core[电机框架层]
//      设备/驱动/分组 的注册、匹配(probe)与反馈分发。不含任何具体电机协议。
//=====================================================================================

#include "motor_core.h"
#include "CAN_Driver.h"
#include "DWT.h"

/*===| 全局链表头：驱动表与设备表 |===*/
static motor_driver_t *s_driver_head = NULL;
static motor_driver_t *s_driver_tail = NULL;
static motor_device_t *s_device_head = NULL;
static motor_device_t *s_device_tail = NULL;

static uint16_t s_driver_count = 0;
static uint16_t s_device_count = 0;

/*===| 注册期临界区：表只在初始化阶段被写，用关中断保证链表一致 |===*/
static inline uint32_t motor_enter_critical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void motor_exit_critical(uint32_t primask) {
  if (primask == 0U)
    __enable_irq();
}

int motor_driver_register(motor_driver_t *drv) {
  if (drv == NULL || drv->name == NULL || drv->ops == NULL)
    return -1;
  if (drv->type_id == 0U)
    return -1;

  uint32_t primask = motor_enter_critical();
  drv->next = NULL;
  if (s_driver_tail == NULL) {
    s_driver_head = drv;
    s_driver_tail = drv;
  } else {
    s_driver_tail->next = drv;
    s_driver_tail = drv;
  }
  s_driver_count++;
  motor_exit_critical(primask);

  return 0;
}

int motor_device_register(motor_device_t *dev, uint16_t count) {
  if (dev == NULL || count == 0U)
    return -1;

  uint32_t primask = motor_enter_critical();
  for (uint16_t i = 0; i < count; i++) {
    motor_device_t *d = &dev[i];

    if (d->type_id == 0U)
      continue;

    /* 框架只填"框架自己的"字段；group/上下限/通道/上下文分别由
       motor_init.c 的声明与型号表、驱动 probe 负责 */
    d->ops = NULL;
    d->driver = NULL;
    d->ctx = NULL;
    d->channel = -1;
    /* 上电默认失能：应用显式调用 motor_enable(_all) 才算使能，
       恢复流程也不会在此之前把电机拉起来（对应老工程的 Robo_Enable 门控） */
    d->enabled = 0U;
    d->index = (uint8_t)s_device_count;
    d->output = 0.0f;
    d->next = NULL;

    if (s_device_tail == NULL) {
      s_device_head = d;
      s_device_tail = d;
    } else {
      s_device_tail->next = d;
      s_device_tail = d;
    }
    s_device_count++;
  }
  motor_exit_critical(primask);

  return 0;
}

int motor_driver_match(void) {
  int matched = 0;

  for (motor_device_t *dev = s_device_head; dev != NULL; dev = dev->next) {
    for (motor_driver_t *drv = s_driver_head; drv != NULL; drv = drv->next) {
      if (drv->type_id != dev->type_id)
        continue;

      /* 1. 已经 probe 过的设备只重新绑定，不重复执行 probe：
            probe 会分配上下文，重复执行会造成上下文泄漏与参数被覆盖 */
      if (dev->ops != NULL) {
        dev->driver = drv;
        matched++;
        break;
      }

      /* 2. 先绑定，再 probe：probe 里要读 dev->driver 的默认上下限 */
      dev->ops = drv->ops;
      dev->driver = drv;

      if (drv->probe != NULL && drv->probe(dev) != 0) {
        /* 驱动拒绝该设备：回滚，保持未匹配 */
        dev->ops = NULL;
        dev->driver = NULL;
        dev->ctx = NULL;
        dev->channel = -1;
        break;
      }

      /* 3. 具体型号参数由 motor_init.c 的型号表随后覆盖 */
      matched++;
      break;
    }
  }

  return matched;
}

motor_device_t *motor_find_by_name(const char *name) {
  if (name == NULL)
    return NULL;

  for (motor_device_t *dev = s_device_head; dev != NULL; dev = dev->next) {
    const char *a = dev->name;
    const char *b = name;
    if (a == NULL)
      continue;
    while (*a != '\0' && *a == *b) {
      a++;
      b++;
    }
    if (*a == '\0' && *b == '\0')
      return dev;
  }

  return NULL;
}

int motor_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                  const uint8_t *data) {
  if (hfdcan == NULL || data == NULL)
    return -1;

  for (motor_device_t *dev = s_device_head; dev != NULL; dev = dev->next) {
    if (dev->hfdcan != hfdcan)
      continue;
    if (dev->rx_id != id)
      continue;
    if (dev->ops == NULL || dev->ops->storage_data == NULL)
      return -1;

    /* If_Online 由驱动按协议置位，这里不能无条件置 1：达妙失能/报故障时
       照样回帧，框架一覆盖就永远判不出失联，恢复流程也就不会触发 */
    dev->ops->storage_data(dev, data);
    return 0;
  }

  return -1;
}

uint16_t motor_driver_count(void) { return s_driver_count; }

uint16_t motor_device_count(void) { return s_device_count; }

void Get_TotalAngle_Speed(motor_data_t *data, float K) {
  if (data == NULL)
    return;

  float dt = DWT_GetDeltaT(&data->Total_Angle_DWT_Count);
  if (dt <= 0.0f)
    return;

  float speed =
      (data->Total_Angle - data->Total_Angle_Last) / dt * 60.0f / 360.0f;

  data->Total_Angle_Speed_RPM =
      K * speed + (1.0f - K) * data->Total_Angle_Speed_RPM_Last;
  data->Total_Angle_Speed_RPM_Last = data->Total_Angle_Speed_RPM;

  data->Total_Angle_Last = data->Total_Angle;
}
