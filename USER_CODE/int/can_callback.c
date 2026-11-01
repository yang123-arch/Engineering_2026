#include "fdcan.h"
#include "INS.h"
#include "motor_core.h"
#include "RoboControl.h"
#include "Define.h"
#include "stm32g4xx_hal.h"

uint16_t CAN1_Err_Ticker;
uint16_t CAN2_Err_Ticker;
uint16_t CAN3_Err_Ticker;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  (void)RxFifo0ITs;
  FDCAN_RxHeaderTypeDef RxHeader = {0};
  uint8_t CAN_RX_Data1[8] = {0};
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader,
                             CAN_RX_Data1) != HAL_OK) {
    return;
  }

  if (hfdcan == &hfdcan1)
    CAN1_Err_Ticker = 0;
  if (hfdcan == &hfdcan2)
    CAN2_Err_Ticker = 0;
  if (hfdcan == &hfdcan3)
    CAN3_Err_Ticker = 0;

  /* 电机反馈：电机框架按设备表里的 (hfdcan, rx_id) 查表并解包，
     本文件不再关心任何具体电机 ID 与协议细节 */
  motor_receive(hfdcan, (uint16_t)RxHeader.Identifier, CAN_RX_Data1);
  if (hfdcan == &SuperCap_CAN &&
      RxHeader.Identifier == SuperCap_FeedBack_CAN_ID) {
    RoboControl_UpdateSuperCapFeedback(CAN_RX_Data1);
  }
  if (hfdcan == &IMU_CAN && RxHeader.Identifier == IMU_FeedBack_CAN_ID) {
    INS_Get_Data_Call_Back(CAN_RX_Data1);
  }
}
