//=====================================================================================
//                    CAN_Driver[CAN通信驱动]
//
//      CAN初始化  CAN_Init(void)
//      CAN发送    CAN_Send_Data(...)   非阻塞：入软件队列，立即返回
//      CAN发送任务 CAN_TX_Task         遍历 3 条 CAN，把队列里的帧塞进硬件 FIFO
//      CAN接收回调 CAN_RxCallback      在 INT/can_callback.c 里统一处理
//
//      设计要点：
//        1. 每条 FDCAN 一个 RingBuffer（深度 64），消费者 = CAN_TX_Task；
//           生产者不止一个（控制任务、错误恢复任务、RoboControl 的使能/失能），
//           入队、出队都在临界区里做。
//        2. 队列满时覆盖最旧帧（UDP 风格，保证新指令永远进得来）。
//        3. 唤醒源有两个：
//             - 控制任务调 CAN_Send_Data 入队后 osThreadFlagsSet
//             - TX FIFO 空中断里 osThreadFlagsSet（中断里走 *FromISR 路径，
//               CMSIS-RTOS v2 的 osThreadFlagsSet 内部就是 xTaskNotifyFromISR）
//        4. CAN_Init() 里启用 FDCAN_IT_TX_FIFO_EMPTY。
//=====================================================================================

#include "CAN_Driver.h"
#include "Define.h"
#include "RingBuffer.h"
#include "cmsis_os2.h"
#include "fdcan.h"

/*===| 队列深度（必须是 2 的幂）与任务通知位 |===*/
#define CAN_TX_QUEUE_DEPTH 64U
#define CAN_TX_FLAG_WAKEUP 0x01U

/*===| 每条 FDCAN 一个软件发送队列 |===*/
RING_BUFFER_DEFINE(s_can1_tx, can_tx_msg_t, CAN_TX_QUEUE_DEPTH);
RING_BUFFER_DEFINE(s_can2_tx, can_tx_msg_t, CAN_TX_QUEUE_DEPTH);
RING_BUFFER_DEFINE(s_can3_tx, can_tx_msg_t, CAN_TX_QUEUE_DEPTH);

typedef struct {
  FDCAN_HandleTypeDef *hfdcan;
  RingBuffer_t *queue;
} can_bus_tx_t;

static can_bus_tx_t s_can_buses[3] = {
    {&hfdcan1, &s_can1_tx},
    {&hfdcan2, &s_can2_tx},
    {&hfdcan3, &s_can3_tx},
};

/* CAN_TX 任务句柄：app_freertos.c 创建，CAN_Driver 用它发通知 */
extern osThreadId_t CAN_TXHandle;

/**
 * @brief 按 hfdcan 找到对应的软件发送队列；不在表里返回 NULL
 */
static RingBuffer_t *CAN_FindQueue(FDCAN_HandleTypeDef *hfdcan) {
  for (uint8_t i = 0; i < 3U; i++) {
    if (s_can_buses[i].hfdcan == hfdcan)
      return s_can_buses[i].queue;
  }
  return NULL;
}

/*===| 队列临界区 |===
 *      RingBuffer 本身只支持单生产者单消费者，而同一条总线会有多个任务入队；
 *      队满时入队方还要挪读指针，和 CAN_TX_Task 出队冲突。临界区只包住
 *      一帧（十几字节）的拷贝，关中断时间很短。用 PRIMASK 保存/恢复，
 *      可以嵌套，调度器启动前调用也安全 */
static inline uint32_t CAN_EnterCritical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void CAN_ExitCritical(uint32_t primask) {
  if (primask == 0U)
    __enable_irq();
}

/**
 * @brief 非阻塞发送：消息入队，立即返回；队列满时覆盖最旧帧
 * @note  任务上下文调用（可多个任务同时调用）；由 CAN_TX_Task 消费
 */
void CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *TX_Data) {
  RingBuffer_t *q = CAN_FindQueue(hfdcan);
  if (q == NULL || TX_Data == NULL)
    return;

  can_tx_msg_t msg;
  msg.id = ID;
  for (uint8_t i = 0; i < 8U; i++)
    msg.data[i] = TX_Data[i];

  uint32_t primask = CAN_EnterCritical();

  /* 入队前是否为空：空->非空才需要唤醒发送任务；
     非空说明 CAN_TX_Task 已被唤醒过（或正在跑），重复 set flags 只会
     在高优先级 CAN_TX 存在时白白触发一次上下文切换 */
  uint8_t was_empty = (RingBuffer_Count(q) == 0U) ? 1U : 0U;

  /* 队列满时丢最旧帧，给新指令腾位置（UDP 风格） */
  if (RingBuffer_Free(q) == 0U) {
    RingBuffer_Skip(q, 1U);
  }
  (void)RingBuffer_Write(q, &msg, 1U);

  CAN_ExitCritical(primask);

  /* 唤醒发送任务；任务还没创建时 osThreadFlagsSet 会返回错误，忽略 */
  if (was_empty != 0U && CAN_TXHandle != NULL)
    (void)osThreadFlagsSet(CAN_TXHandle, CAN_TX_FLAG_WAKEUP);
}

/**
 * @brief TX FIFO 空中断：硬件 FIFO 全部发完，可以往里塞新数据
 * @note  ISR 上下文：osThreadFlagsSet 内部走 xTaskNotifyFromISR，安全
 */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
  (void)hfdcan;
  if (CAN_TXHandle != NULL)
    (void)osThreadFlagsSet(CAN_TXHandle, CAN_TX_FLAG_WAKEUP);
}

/**
 * @brief CAN 发送任务：遍历 3 条 CAN，每轮把每条总线上排队的帧尽量塞进
 *        硬件 FIFO（直到队空或 FIFO 满），一条都发不出去才阻塞等唤醒
 * @note  原先"一轮一条总线只发一帧"在满负荷时发送速率跟不上下发速率，
 *        队列被"覆盖最旧"策略持续丢帧，电机表现为抽搐/不按指令运行
 */
void CAN_TX_Task(void *argument) {
  (void)argument;

  for (;;) {
    uint8_t sent = 0U;

    for (uint8_t i = 0; i < 3U; i++) {
      can_bus_tx_t *bus = &s_can_buses[i];

      /* 排空本总线：硬件 FIFO 有 3 格，一轮最多补 3 帧 */
      while (RingBuffer_Count(bus->queue) != 0U &&
             HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) != 0U) {
        can_tx_msg_t msg;
        uint32_t primask = CAN_EnterCritical();
        uint8_t rd = RingBuffer_Read(bus->queue, &msg, 1U);
        CAN_ExitCritical(primask);
        if (rd != RB_OK)
          break;
        FDCAN_TxHeaderTypeDef hdr;
        hdr.Identifier = msg.id;
        hdr.IdType = FDCAN_STANDARD_ID;
        hdr.TxFrameType = FDCAN_DATA_FRAME;
        hdr.DataLength = FDCAN_DLC_BYTES_8;
        hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        hdr.BitRateSwitch = FDCAN_BRS_OFF;
        hdr.FDFormat = FDCAN_CLASSIC_CAN;
        hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        hdr.MessageMarker = 0;

        if (HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &hdr, msg.data) !=
            HAL_OK)
          break;
        sent = 1U;
      }
    }

    if (sent == 0U) {
      /* 三条 CAN 都没发出去：要么队列全空，要么硬件 FIFO 全满。
         阻塞等唤醒：入队或 TX FIFO 空，都会来通知。 */
      (void)osThreadFlagsWait(CAN_TX_FLAG_WAKEUP, osFlagsWaitAny,
                              osWaitForever);
    }
  }
}

void CAN_Init(void) {
  /* 初始化三条软件发送队列（RING_BUFFER_DEFINE 只分配存储，不填字段） */
  RingBuffer_Init(&s_can1_tx, s_can1_tx_buf, CAN_TX_QUEUE_DEPTH,
                  sizeof(can_tx_msg_t));
  RingBuffer_Init(&s_can2_tx, s_can2_tx_buf, CAN_TX_QUEUE_DEPTH,
                  sizeof(can_tx_msg_t));
  RingBuffer_Init(&s_can3_tx, s_can3_tx_buf, CAN_TX_QUEUE_DEPTH,
                  sizeof(can_tx_msg_t));

  /* 关键：HAL_FDCAN_Init()/HAL_FDCAN_Start() 都不会清 IR，而 IR 是写 1 清零。
     ARA（访问保留地址）一旦置位就会一直保留，且 ARA=1 时 M_CAN 处于
     Restricted Operation Mode —— 只收不发，报文根本发不出去。
     这里在启动前把三个实例的残留标志全部清掉。 */
  hfdcan1.Instance->IR = FDCAN_IR_MASK;
  hfdcan2.Instance->IR = FDCAN_IR_MASK;
  hfdcan3.Instance->IR = FDCAN_IR_MASK;

  CAN_Filter_Init(&hfdcan1);
  CAN_Filter_Init(&hfdcan2);
  CAN_Filter_Init(&hfdcan3);

  /* 启用 TX FIFO 空中断：FIFO 全部发完时唤醒 CAN_TX_Task 往里补 */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_TX_FIFO_EMPTY, 0);

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan3);
}

void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00;
  sFilterConfig.FilterID2 = 0x7FF;

  HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
