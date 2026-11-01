#include "Communication_Driver_UART.h"

#include "stm32g4xx_hal.h"

static comm_status_t uart_start_rx_dma(comm_port_t *port) {
  if (port == NULL || port->hardware == NULL || port->rx_dma_buffer == NULL ||
      port->rx_dma_capacity == 0U)
    return COMM_STATUS_INVALID_ARGUMENT;

  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)port->hardware;
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, port->rx_dma_buffer,
                                   port->rx_dma_capacity) != HAL_OK)
    return COMM_STATUS_DRIVER_ERROR;
  if (port->rx_dma != NULL)
    __HAL_DMA_DISABLE_IT((DMA_HandleTypeDef *)port->rx_dma, DMA_IT_HT);
  return COMM_STATUS_OK;
}

static comm_status_t uart_try_start_tx(comm_port_t *port, const uint8_t *data,
                                       uint16_t length) {
  if (port == NULL || port->hardware == NULL || data == NULL || length == 0U)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (port->tx_state.tx_busy)
    return COMM_STATUS_BUSY;

  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
      (UART_HandleTypeDef *)port->hardware, (uint8_t *)data, length);
  if (status == HAL_OK)
    return COMM_STATUS_OK;
  if (status == HAL_BUSY)
    return COMM_STATUS_BUSY;
  return COMM_STATUS_DRIVER_ERROR;
}

static uint8_t uart_is_tx_idle(const comm_port_t *port) {
  return port != NULL && port->tx_state.tx_busy == 0U;
}

static void uart_recover(comm_port_t *port) { (void)uart_start_rx_dma(port); }

static const comm_transport_ops_t s_ops = {
    .start_rx_dma = uart_start_rx_dma,
    .try_start_tx = uart_try_start_tx,
    .is_tx_idle = uart_is_tx_idle,
    .recover = uart_recover,
};

const comm_transport_ops_t *Communication_UART_Driver_Get(void) {
  return &s_ops;
}
