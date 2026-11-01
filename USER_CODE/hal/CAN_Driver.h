#ifndef __CAN_Driver__
#define __CAN_Driver__

#include "main.h"

/*===| 发送队列元素：一帧 CAN 报文 |===*/
typedef struct {
  uint16_t id;
  uint8_t data[8];
} can_tx_msg_t;

void CAN_Init(void);

/*===| 非阻塞发送：入队即返回；队列满时覆盖最旧帧 |===*/
void CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *TX_Data);

void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan);

/*===| CAN 发送任务：优先级低于控制任务，遍历 3 条 CAN，
        有数据 + 硬件 FIFO 有位则写，三条都空则阻塞等通知 |===*/
void CAN_TX_Task(void *argument);

#endif
