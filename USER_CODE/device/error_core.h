//=====================================================================================
//                                 error_core[错误监控与恢复框架]
//
//      把"设备失联 -> 通知恢复任务 -> 跑设备自己的恢复流程 -> 状态复位"这套逻辑
//      从应用层里剥出来，上层只需要做三件事：
//
//        1. 调度器启动前调用一次 error_core_init()
//           —— 按设备数量建表、把 if_online 指针与恢复函数挂上
//        2. 一个 200Hz 的任务周期调用 error_core_update_all()
//        3. 一个阻塞在任务通知上的任务（Error_Deal_task）按位调用 recover_fn
//
//      数据流：
//
//        设备接收中断:  data.If_Online = 1              （收到一帧反馈置 1）
//
//        200Hz 监控:  error_core_update_all()
//                       └─ error_core_update(ctx)
//                            ├─ 读 *if_online，读到 1 后清零（一次性消费）
//                            ├─ 读到 1 -> counter = 0
//                            └─ 读到 0 -> counter++；计数到 counter_limit
//                                         -> state = ERR_LOST
//                                         -> xTaskNotify(1 << id)
//
//        恢复任务:    xTaskNotifyWait() 取出 notified_bits
//                       按位 i：ret = error_ctx[i].recover_fn(&error_ctx[i])
//                            ├─ 1 -> state = ERR_NORMAL, counter = 0
//                            └─ 0 -> 重新置位 (1 << i)，下一轮接着跑
//                                    （多步恢复流程靠这个非阻塞地推进）
//
//      目前只适配电机：error_core.c 的适配层把 ctx->id 当作 g_motor_devices[]
//      的下标，recover_fn 走设备的 ops->recover（见 motor_core.h）。
//      以后要监控别的设备（超电、遥控器……），只要在 error_core_init() 里给它
//      挂一个 if_online 指针和一个 recover_fn 就行，本文件不用动。
//=====================================================================================

#ifndef __ERROR_CORE_H__
#define __ERROR_CORE_H__

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/*===|==================== 一、状态与结构体 ====================|===*/

/**
 * @brief 设备状态
 */
typedef enum {
  ERR_NORMAL = 0, // 正常：周期内读到过在线标志
  ERR_LOST = 1,   // 失联：连续 counter_limit 次没读到在线标志
} error_state_t;

/**
 * @brief 失联判定计数上限（200Hz 轮询 × 20 = 100ms 没有反馈就判失联）
 * @note  可以按设备单独改：ctx->counter_limit = xxx
 */
#ifndef ERROR_COUNT_LIMIT_DEFAULT
#define ERROR_COUNT_LIMIT_DEFAULT 20U
#endif

/**
 * @brief 一次任务通知有 32 个 bit，一个 bit 对应一个设备，所以最多监控 32 台
 */
#define ERROR_DEV_MAX 32U

/** 第 id 台设备对应的任务通知位 */
#define ERROR_NOTIFY_BIT(id) (1UL << (id))

/**
 * @brief 一个被监控设备的错误上下文
 * @note  if_online 指向设备自己的在线标志（收到数据置 1）；recover_fn 由
 *        适配层挂上，签名收本结构体，方便恢复函数回读 id/状态
 */
typedef struct error_ctx_t {
  uint8_t id;              // 设备序号（= error 数组下标 = 设备表下标）
  volatile uint8_t state;  // error_state_t 当前状态
  uint16_t counter;        // 连续"读到离线"的次数
  uint16_t counter_limit;  // 判定失联的计数上限（到了就改状态 + 发通知）

  volatile uint8_t *if_online; // 设备的在线标志（设备收到数据后置 1）

  /**
   * @brief 恢复函数
   * @return 1 恢复流程已走完（状态复位）；0 流程未完成（重新置位，下次继续）
   * @note   必须保证有限步内返回 1：一直返回 0 会让恢复任务反复自旋。
   *         没有恢复能力的设备请直接返回 1。
   */
  int (*recover_fn)(struct error_ctx_t *ctx);
} error_ctx_t;

/*===|==================== 二、恢复任务句柄 ====================|===*/

/**
 * @brief 恢复任务（Error_Deal_task）的句柄，失联时往它的通知位里置位
 * @note  由 Error_Deal_task 启动时用 xTaskGetCurrentTaskHandle() 填自己；
 *        填之前 error_core_update() 不会发通知（不会误发给 NULL 句柄）
 */
extern TaskHandle_t g_recovery_task_handle;

/*===|==================== 三、API ====================|===*/

/**
 * @brief  错误监控初始化：按设备数量建表、赋予 id、挂在线指针与恢复函数
 * @note   调度器启动前调用一次（可重复调用，重复只是原样重填一遍）；
 *         必须在 CAN 等接收中断打开之前调用
 * @return 0 成功
 */
int error_core_init(void);

/**
 * @brief 当前监控的设备数量（= 建表时的设备数量）
 */
uint16_t error_core_count(void);

/**
 * @brief 取第 id 台设备的错误上下文，越界返回 NULL
 */
error_ctx_t *error_core_get(uint16_t id);

/**
 * @brief 更改状态：读一次在线标志 -> 计数 -> 到上限就改状态并发恢复通知
 * @note  约定 200Hz 调用；读到 1 会顺手把标志清零（一次性消费），
 *        所以计数反映的是"这一轮有没有新数据"
 */
void error_core_update(error_ctx_t *ctx);

/**
 * @brief 遍历整张表，逐个 error_core_update()（Error_Check_Task 调用）
 */
void error_core_update_all(void);

#endif /* __ERROR_CORE_H__ */
