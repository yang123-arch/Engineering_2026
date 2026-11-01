//=====================================================================================
//                                 error_core[错误监控与恢复框架]
//
//      建表 + 状态更新 + 发恢复通知。设计说明见 error_core.h 顶部注释。
//      文件放在 device/ 下面：它属于设备层（要知道有哪些设备），
//      应用层只按周期调 error_core_update_all()、按位调 recover_fn。
//=====================================================================================

#include "error_core.h"
#include "motor_core.h"
#include "motor_init.h"

/*===|==================== 一、监控表 ====================|===*/

/* 恢复任务句柄：由 Error_Deal_task 启动时填自己 */
TaskHandle_t g_recovery_task_handle = NULL;

/**
 * @brief 监控表
 * @note  设备数量 g_motor_device_count 是别的翻译单元里的 const 变量，C 语言
 *        不允许拿它初始化 static 数组（不是常量表达式，C++ 才可以），所以
 *        这里按编译期上限开数组，实际数量在 error_core_init() 里取。
 *        静态存储 → 全 0，初始化前也不会是一堆野指针。
 */
static error_ctx_t s_error_ctx[ERROR_DEV_MAX];
static uint16_t s_error_count = 0U;
static uint8_t s_error_inited = 0U;

/*===|==================== 二、电机适配层 ====================|===*/

/**
 * @brief 电机设备的恢复入口：error ctx -> motor device -> ops->recover
 * @note  目前 ctx->id 就是 g_motor_devices[] 的下标（建表时就是这么赋的）；
 *        以后接别的设备类型，在这里按类型分派即可
 */
static int Error_Motor_Recover(error_ctx_t *ctx) {
  if (ctx == NULL || ctx->id >= g_motor_device_count)
    return 1; /* 没有对应设备：算完成，别让恢复任务反复自旋 */

  /* 该型号没实现 recover 时 motor_recover() 返回 -1：同样算完成 */
  int ret = motor_recover(&g_motor_devices[ctx->id]);
  return (ret < 0) ? 1 : ret;
}

/*===|==================== 三、建表 ====================|===*/

int error_core_init(void) {
  if (s_error_inited != 0U)
    return 0; /* 幂等：重复调用直接返回 */

  uint16_t count = g_motor_device_count;
  if (count > ERROR_DEV_MAX)
    count = ERROR_DEV_MAX; /* 通知位只有 32 个，多出来的设备不监控 */

  s_error_count = count;

  for (uint16_t i = 0; i < count; i++) {
    error_ctx_t *ctx = &s_error_ctx[i];

    ctx->id = (uint8_t)i;
    ctx->state = (uint8_t)ERR_NORMAL;
    ctx->counter = 0U;
    ctx->counter_limit = ERROR_COUNT_LIMIT_DEFAULT;
    ctx->if_online = &g_motor_devices[i].data.If_Online;
    ctx->recover_fn = Error_Motor_Recover;
  }

  s_error_inited = 1U;
  return 0;
}

uint16_t error_core_count(void) { return s_error_count; }

error_ctx_t *error_core_get(uint16_t id) {
  if (id >= s_error_count)
    return NULL;
  return &s_error_ctx[id];
}

/*===|==================== 四、状态更新 ====================|===*/

void error_core_update(error_ctx_t *ctx) {
  if (ctx == NULL || ctx->if_online == NULL || ctx->recover_fn == NULL)
    return;

  /* --- 读到 1：这一轮收到过反馈，读到后顺手清零（一次性消费）---
     清零与接收中断的置 1 之间存在一个极短的竞争窗口，最坏情况是少算
     一次"有数据"，相对 counter_limit（20 次）可以忽略 */
  if (*ctx->if_online != 0U) {
    *ctx->if_online = 0U;
    ctx->counter = 0U;
    return;
  }

  /* --- 读到 0：累计没收到数据的次数 --- */
  if (ctx->counter < ctx->counter_limit)
    ctx->counter++;

  if (ctx->counter < ctx->counter_limit)
    return;

  /* --- 计数到上限：改状态 + 通知恢复任务 ---
     计数清零后重新开始数：失联期间每 counter_limit 次（约 100ms）重试一次
     恢复，直到恢复流程走完（Error_Deal_task 会把 state 复位成 ERR_NORMAL）；
     中间设备要是自己回来了，上面的分支会把计数清零，不再打扰恢复任务 */
  ctx->state = (uint8_t)ERR_LOST;
  ctx->counter = 0U;

  if (g_recovery_task_handle != NULL)
    xTaskNotify(g_recovery_task_handle, ERROR_NOTIFY_BIT(ctx->id), eSetBits);
}

void error_core_update_all(void) {
  for (uint16_t i = 0; i < s_error_count; i++)
    error_core_update(&s_error_ctx[i]);
}
