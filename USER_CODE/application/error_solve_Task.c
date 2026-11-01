#if 0
//=====================================================================================
//                            error_solve_Task[错误监控与恢复任务]
//
//      Error_Check_Task : 1kHz，按老工程
//      tests/Project/Robot_Application/ErrorHandle.c
//                         逐行移植：
//                           - 遥控器 3s 断连 -> Robo_Stop
//                           - CAN 总线 100ms 无接收 -> 重新初始化该总线
//                           - DM/LK 电机离线 110ms -> 清故障 +
//                           重新使能（Robo_Enable 门控）
//                         发送直接走底层
//                         CAN_Send_Data（当前为入队非阻塞），与老工程
//                         的区别仅在"阻塞直发 ->
//                         入队"，用于测试发送框架是否可靠。
//
//      Error_Deal_Task  : 保留电机框架 error_core 的恢复通道（闲置兜底）：
//                         阻塞等通知 -> 按位调设备 ops->recover。
//                         正常情况下老工程移植逻辑已经覆盖了所有电机的恢复，
//                         这个任务不会收到通知；保留它是给以后"按型号自定义恢复
//                         流程"留的口子。
//=====================================================================================

#include "FreeRTOS.h"
#include "task.h"

#include "Remote_Control.h"
#include "RoboControl.h"
#include "error_core.h"
#include "motor_init.h"

#include "CAN_Driver.h"
#include "Define.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_LINGKONG_Driver.h"
#include "fdcan.h"
#include "main.h"

#define REMOTE_TIMEOUT_MS 3000U

uint8_t Err_total = 0;

/*===| 离线判定/恢复阈值（1kHz 计数，与老工程一致） |===*/
#define MOTOR_OFFLINE_MS 100U // 100ms 无反馈判离线
#define MOTOR_RECOVER_MS 110U // 110ms 时发 清故障+使能

extern uint16_t CAN1_Err_Ticker;
extern uint16_t CAN2_Err_Ticker;
extern uint16_t CAN3_Err_Ticker;

uint8_t hfdcan1_Err_total = 0;
uint8_t hfdcan2_Err_total = 0;
uint8_t hfdcan3_Err_total = 0;

/* 调试观察：每台电机 110ms 重使能触发次数（下标 = g_motor_devices[] 序号，
   腿电机 MOTOR_IDX_CHASSIS_LEG1/2 = 4/5，云台从 MOTOR_IDX_GIMBAL_YAW = 6 起）
 */
volatile uint32_t err_recover_fire[MOTOR_IDX_MAX];

/**
 * @brief 错误监控与恢复任务：1kHz 定点周期（老工程 ErrorHandle_Task 移植）
 */
void Error_Check_Task(void *argument) {
  (void)argument;

  /* error_core 建表（幂等）：Error_Deal_Task 的兜底恢复通道要用 */
  error_core_init();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); /* 1kHz 定点周期 */

  /* 每台电机一个离线计数器（下标 = g_motor_devices[] 下标），
     对应老工程 Motor_Data_StructTypeDef.Ticker */
  static uint16_t s_motor_ticker[MOTOR_IDX_MAX];

  for (;;) {
    /*===| 遥控器断连处理 |===*/
    Remote.Remote_GetData_Ticker++;
    if (Remote.Remote_GetData_Ticker >= REMOTE_TIMEOUT_MS) {
      Robo_Stop();
      Err_total++;
    }

    /*===| CAN 总线错误处理：100ms 无接收 -> 重新初始化该总线 |===*/
    CAN1_Err_Ticker++;
    CAN2_Err_Ticker++;
    CAN3_Err_Ticker++;
    if (CAN1_Err_Ticker > 100) {
      MX_FDCAN1_Init();
      CAN_Filter_Init(&hfdcan1);
      HAL_FDCAN_Start(&hfdcan1);
      CAN1_Err_Ticker = 0;
      hfdcan1_Err_total++;
    }
    if (CAN2_Err_Ticker > 100) {
      MX_FDCAN2_Init();
      CAN_Filter_Init(&hfdcan2);
      HAL_FDCAN_Start(&hfdcan2);
      CAN2_Err_Ticker = 0;
      hfdcan2_Err_total++;
    }
    if (CAN3_Err_Ticker > 100) {
      MX_FDCAN3_Init();
      CAN_Filter_Init(&hfdcan3);
      HAL_FDCAN_Start(&hfdcan3);
      CAN3_Err_Ticker = 0;
      hfdcan3_Err_total++;
    }

    /*===| 电机离线检测：计数窗口 100~110ms（与老工程一致） |===*/
    for (uint16_t i = 0; i < g_motor_device_count && i < MOTOR_IDX_MAX; i++) {
      s_motor_ticker[i]++;
      if (s_motor_ticker[i] > MOTOR_RECOVER_MS)
        s_motor_ticker[i] = MOTOR_OFFLINE_MS;

      /* 收到反馈（If_Online == 1）就归零并清零标志，等价老工程
         解码函数里的 Ticker = 0（一次性消费） */
      if (g_motor_devices[i].data.If_Online != 0U) {
        g_motor_devices[i].data.If_Online = 0U;
        s_motor_ticker[i] = 0U;
      }
    }

    /*===| 电机离线后重使能 —— 仅 Robo_Enable 时操作，防止上电/停机时
     *      误使能电机乱动（与老工程门控一致）。DM/LK 一机一帧，直接按
     *      设备表里的总线/发送 ID 发协议命令；DJI 电调无使能协议，跳过 |===*/
    if (RoboControl_Struct.Robo_Enable) {
      for (uint16_t i = 0; i < g_motor_device_count && i < MOTOR_IDX_MAX; i++) {
        if (s_motor_ticker[i] != MOTOR_RECOVER_MS)
          continue;

        motor_device_t *dev = &g_motor_devices[i];
        err_recover_fire[i]++;

        switch (dev->type_id) {
        case MOTOR_TYPE_DM_4005:
        case MOTOR_TYPE_DM_3507:
        case MOTOR_TYPE_DM_8009:
        case MOTOR_TYPE_DM_DJI:
          Motor_DM_CMD_ClearErr(dev->hfdcan, dev->tx_id);
          osDelay(1);
          Motor_DM_CMD_Enable(dev->hfdcan, dev->tx_id);
          osDelay(1);
          break;

        case MOTOR_TYPE_LK_MG6010:
        case MOTOR_TYPE_LK_MG8010:
          Motor_LK_CMD_ClearErr(dev->hfdcan, dev->tx_id);
          osDelay(1);
          Motor_LK_CMD_Enable(dev->hfdcan, dev->tx_id);
          osDelay(1);
          break;

        default:
          /* DJI 电调无清故障/使能指令；一拖四不在当前设备表 */
          break;
        }
      }
    } /* Robo_Enable 门控结束 */

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief 错误恢复任务（error_core 兜底通道）：等通知 -> 按位跑恢复函数
 * @note  老工程移植逻辑（Error_Check_Task）已经覆盖全部电机的离线重使能，
 *        本任务正常情况下收不到通知，处于长期阻塞，不占 CPU；
 *        保留它是为了给"按型号自定义多步恢复流程"留通道
 */
void Error_Deal_Task(void *argument) {
  (void)argument;

  /* 通知目标就是本任务：设备失联时 error_core_update() 往这里置位 */
  g_recovery_task_handle = xTaskGetCurrentTaskHandle();

  uint32_t notified_bits = 0U;

  for (;;) {
    /* 阻塞等通知：一个 bit 对应 error 数组里的一个下标
       （ulBitsToClearOnExit = 0xFFFFFFFF：取完就把位清掉） */
    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notified_bits, portMAX_DELAY) !=
        pdTRUE)
      continue;

    uint16_t count = error_core_count();

    for (uint16_t i = 0; i < count; i++) {
      error_ctx_t *ctx = error_core_get(i);
      if (ctx == NULL || ctx->recover_fn == NULL)
        continue;

      /* 位没置起来、状态也不是失联的：跳过 */
      if ((notified_bits & ERROR_NOTIFY_BIT(i)) == 0U &&
          ctx->state != (uint8_t)ERR_LOST)
        continue;

      /* 状态已经不是失联（自己恢复了 / 上一轮刚复位）：丢掉过期通知 */
      if (ctx->state != (uint8_t)ERR_LOST)
        continue;

      int ret = ctx->recover_fn(ctx);

      if (ret == 1) {
        /* 恢复流程走完：状态复位，等下一次真的再失联才重新计数 */
        ctx->state = (uint8_t)ERR_NORMAL;
        ctx->counter = 0U;
      } else {
        /* 未完成：重新置位，下一轮继续跑剩下的步骤 */
        vTaskDelay(pdMS_TO_TICKS(1U));
        xTaskNotify(g_recovery_task_handle, ERROR_NOTIFY_BIT(i), eSetBits);
      }
    }
  }
}
#endif                        /* 0：error_core 框架方案备份 */

#if 1
//=====================================================================================
//          【备份】error_core 框架方案（200Hz 轮询 + 通知式恢复）
//
//      Error_Check_Task : 200Hz 定点周期，调 error_core_update_all() 轮询
//                         所有设备的在线标志，失联计数到上限后由 error_core
//                         往 Error_Deal_Task 发任务通知
//
//      Error_Deal_Task  : 阻塞在任务通知上，被唤醒后按位调用设备的
//                         recover_fn（error_core 适配层 -> motor_recover ->
//                         设备 ops->recover）：
//                           ret == 1 -> 恢复流程走完，状态复位
//                           ret == 0 -> 重新置位，下一轮接着推进
//
//      遥控器断连保护：Remote_GetData_Ticker 3s 无更新 -> Robo_Stop()
//=====================================================================================

#include "FreeRTOS.h"
#include "task.h"

#include "Remote_Control.h"
#include "RoboControl.h"
#include "error_core.h"

#define REMOTE_TIMEOUT_MS 1000U
/* Error_Check_Task 周期 5ms（200Hz），3s = 600 个周期 */
#define REMOTE_TIMEOUT_TICKS (REMOTE_TIMEOUT_MS / 5U)

uint8_t Err_total = 0;

/**
 * @brief 错误监控任务：200Hz 定点周期，刷新所有设备的失联计数
 */
void Error_Check_Task(void *argument) {
  (void)argument;

  /* error_core 建表（幂等）：挂 if_online 指针与恢复函数 */
  error_core_init();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); /* 200Hz 定点周期 */

  for (;;) {
    /*===| 遥控器断连处理：只清遥控数据，不在这里停机 |===
     *    停机交给 Remote_Rx_Task：它读到 If_Remote_Connect == 0 后调用
     *    Robo_Stop。两个任务都去改机器状态时，断链期间 Mode 停在旧档位，
     *    这里 Stop、那边立刻 Restart，电机被 200Hz 反复失能/使能 */
    Remote.Remote_GetData_Ticker++;
    if (Remote.Remote_GetData_Ticker >= REMOTE_TIMEOUT_TICKS) {
      Remote_Clear();
      Err_total++;
    }

    /*===| 设备失联轮询：读到在线标志清计数，连续离线到上限发恢复通知 |===*/
    error_core_update_all();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

volatile uint32_t recover_data = 0;

/**
 * @brief 错误恢复任务：等通知 -> 按位跑恢复函数
 * @note  长期阻塞在任务通知上，不占 CPU；有设备失联时被 error_core 唤醒
 */
void Error_Deal_Task(void *argument) {
  (void)argument;

  /* 通知目标就是本任务：设备失联时 error_core_update() 往这里置位 */
  g_recovery_task_handle = xTaskGetCurrentTaskHandle();

  uint32_t notified_bits = 0U;

  for (;;) {
    /* 阻塞等通知：一个 bit 对应 error 数组里的一个下标
       （ulBitsToClearOnExit = 0xFFFFFFFF：取完就把位清掉） */
    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notified_bits, portMAX_DELAY) !=
        pdTRUE)
      continue;

    uint16_t count = error_core_count();
    uint32_t pending = 0U; /* 本轮没走完、下一轮还要接着跑的设备 */

    for (uint16_t i = 0; i < count; i++) {
      error_ctx_t *ctx = error_core_get(i);
      if (ctx == NULL || ctx->recover_fn == NULL)
        continue;

      /* 位没置起来、状态也不是失联的：跳过 */
      if ((notified_bits & ERROR_NOTIFY_BIT(i)) == 0U &&
          ctx->state != (uint8_t)ERR_LOST)
        continue;

      /* 状态已经不是失联（自己恢复了 / 上一轮刚复位）：丢掉过期通知 */
      if (ctx->state != (uint8_t)ERR_LOST)
        continue;

      int ret = ctx->recover_fn(ctx);
      recover_data++;

      if (ret == 1) {
        /* 恢复流程走完：状态复位，等下一次真的再失联才重新计数 */
        ctx->state = (uint8_t)ERR_NORMAL;
        ctx->counter = 0U;
      } else {
        /* 未完成：记下来，整轮跑完后统一延时再继续 */
        pending |= ERROR_NOTIFY_BIT(i);
      }
    }

    /* 延时放在循环外：放在里面的话，N 台同时恢复时每台要隔 N ms 才轮到
       一次，比驱动恢复流程的 2ms 超时窗口还长 */
    if (pending != 0U) {
      vTaskDelay(pdMS_TO_TICKS(1U));
      xTaskNotify(g_recovery_task_handle, pending, eSetBits);
    }
  }
}
#endif /* 0：error_core 框架方案备份 */
