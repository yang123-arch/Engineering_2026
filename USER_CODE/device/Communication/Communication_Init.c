#include "Communication_Init.h"

#include "Communication_Driver_UART.h"
#include "Communication_Driver_USB.h"
#include "Define.h"
#include "USART_Driver.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "usart.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static const comm_port_config_t s_ports[] = {
    {.port_id = COMM_PORT_UART1,
     .transport_id = COMM_TRANSPORT_UART_DMA_IDLE,
     .hardware = &huart1,
     .rx_dma = SBUS_UART_DMA,
     .rx_dma_buffer = SBUS_RX_Data,
     .rx_dma_capacity = sizeof(SBUS_RX_Data)},
    {.port_id = COMM_PORT_UART2,
     .transport_id = COMM_TRANSPORT_UART_DMA_IDLE,
     .hardware = &huart2,
     .rx_dma = RS485_2_UART_DMA,
     .rx_dma_buffer = RS485_2_RX_Data,
     .rx_dma_capacity = sizeof(RS485_2_RX_Data)},
    {.port_id = COMM_PORT_UART3,
     .transport_id = COMM_TRANSPORT_UART_DMA_IDLE,
     .hardware = &huart3,
     .rx_dma = RS485_1_UART_DMA,
     .rx_dma_buffer = RS485_1_RX_Data,
     .rx_dma_capacity = sizeof(RS485_1_RX_Data)},
    {.port_id = COMM_PORT_UART4,
     .transport_id = COMM_TRANSPORT_UART_DMA_IDLE,
     .hardware = &huart4,
     .rx_dma = UART4_DMA,
     .rx_dma_buffer = UART4_RX_Data,
     .rx_dma_capacity = sizeof(UART4_RX_Data)},
    {.port_id = COMM_PORT_UART5,
     .transport_id = COMM_TRANSPORT_UART_DMA_IDLE,
     .hardware = &huart5,
     .rx_dma = UART5_DMA,
     .rx_dma_buffer = UART5_RX_Data,
     .rx_dma_capacity = sizeof(UART5_RX_Data)},
    {.port_id = COMM_PORT_USB,
     .transport_id = COMM_TRANSPORT_USB_CDC,
     .hardware = &hUsbDeviceFS},
};

/* One entry per physical RX stream.  The fixed array makes accidental
 * multiple protocol parsers on one RingBuffer visible at review time. */
static const comm_device_config_t device_RX[COMM_RX_DEVICE_COUNT] = {
    {.device_id = DEVICE_SBUS,
     .driver_id = COMM_DRIVER_SBUS,
     .port_id = COMM_PORT_UART1_RX,
     .endpoint = COMM_ENDPOINT_RX,
     .frame = NULL,
     .enabled = 1U},
    {.device_id = DEVICE_IMAGELINK,
     .driver_id = COMM_DRIVER_IMAGELINK,
     .port_id = COMM_PORT_UART4_RX,
     .endpoint = COMM_ENDPOINT_RX,
     .frame = NULL,
     .enabled = 1U},
    {.device_id = DEVICE_REFEREE,
     .driver_id = COMM_DRIVER_REFEREE,
     .port_id = COMM_PORT_UART5_RX,
     .endpoint = COMM_ENDPOINT_RX,
     .frame = NULL,
     .enabled = 1U},
};

_Static_assert((sizeof(device_RX) / sizeof(device_RX[0])) ==
                   COMM_RX_DEVICE_COUNT,
               "RX device table size must match COMM_RX_DEVICE_COUNT");

/* The compiler derives this length.  Multiple devices may share one TX port;
 * Core keeps one port queue and serializes DMA starts. */
static const comm_device_config_t device_TX[] = {
    {.device_id = DEVICE_IMAGELINK,
     .driver_id = COMM_DRIVER_IMAGELINK,
     .port_id = COMM_PORT_UART4_TX,
     .endpoint = COMM_ENDPOINT_TX,
     .enabled = 1U},
    {.device_id = DEVICE_REFEREE,
     .driver_id = COMM_DRIVER_REFEREE,
     .port_id = COMM_PORT_UART5_TX,
     .endpoint = COMM_ENDPOINT_TX,
     .enabled = 1U},
    {.device_id = DEVICE_UI,
     .driver_id = COMM_DRIVER_RAW,
     .port_id = COMM_PORT_UART5_TX,
     .endpoint = COMM_ENDPOINT_TX,
     .enabled = 1U},
    {.device_id = DEVICE_USB_DEBUG,
     .driver_id = COMM_DRIVER_RAW,
     .port_id = COMM_PORT_USB_TX,
     .endpoint = COMM_ENDPOINT_TX,
     .enabled = 1U},
};

#define COMM_TX_DEVICE_COUNT                                                   \
  ((uint16_t)(sizeof(device_TX) / sizeof(device_TX[0])))

_Static_assert((sizeof(device_TX) / sizeof(device_TX[0])) <=
                   COMM_MAX_TX_DEVICES,
               "TX device table exceeds the Core capacity");

static comm_driver_t *const s_drivers[] = {
    &g_raw_driver,
    &g_sbus_driver,
    &g_referee_driver,
    &g_imagelink_driver,
};

static uint8_t s_initialized;

static comm_status_t register_drivers(void) {
  for (uint16_t i = 0U; i < sizeof(s_drivers) / sizeof(s_drivers[0]); ++i) {
    comm_status_t status = Communication_Core_RegisterDriver(s_drivers[i]);
    if (status != COMM_STATUS_OK)
      return status;
    if (s_drivers[i]->init != NULL) {
      status = s_drivers[i]->init(s_drivers[i]);
      if (status != COMM_STATUS_OK)
        return status;
    }
  }
  return COMM_STATUS_OK;
}

comm_status_t Communication_Init(void) {
  comm_board_config_t board = {
      .ports = s_ports,
      .port_count = (uint16_t)(sizeof(s_ports) / sizeof(s_ports[0])),
      .rx_devices = device_RX,
      .rx_device_count = COMM_RX_DEVICE_COUNT,
      .tx_devices = device_TX,
      .tx_device_count = COMM_TX_DEVICE_COUNT,
  };
  comm_status_t status;

  if (s_initialized)
    return COMM_STATUS_OK;
  status = Communication_Core_RegisterTransport(
      COMM_TRANSPORT_UART_DMA_IDLE, Communication_UART_Driver_Get());
  if (status != COMM_STATUS_OK)
    return status;
  status = Communication_Core_RegisterTransport(COMM_TRANSPORT_USB_CDC,
                                                Communication_USB_Driver_Get());
  if (status != COMM_STATUS_OK)
    return status;
  status = register_drivers();
  if (status != COMM_STATUS_OK)
    return status;
  status = Communication_Core_Configure(&board);
  if (status != COMM_STATUS_OK)
    return status;
  s_initialized = 1U;
  return COMM_STATUS_OK;
}

void Communication_RX_Task(void *argument) {
  (void)argument;
  Communication_Core_RegisterRxTask(xTaskGetCurrentTaskHandle());
  if (Communication_Core_Start() != COMM_STATUS_OK)
    Error_Handler();
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(COMM_POLL_TIMEOUT_MS));
    for (uint16_t i = 0U; i < COMM_RX_DEVICE_COUNT; ++i) {
      comm_status_t status;
      do {
        status = Communication_Core_RxPoll(device_RX[i].device_id);
      } while (status == COMM_STATUS_OK ||
               status == COMM_STATUS_FRAME_INVALID ||
               status == COMM_STATUS_CHECKSUM_ERROR);
    }
  }
}

void Communication_TX_Task(void *argument) {
  (void)argument;
  Communication_Core_RegisterTxTask(xTaskGetCurrentTaskHandle());
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(COMM_POLL_TIMEOUT_MS));
    for (uint16_t i = 0U; i < COMM_TX_DEVICE_COUNT; ++i)
      (void)Communication_Core_TxPoll(device_TX[i].device_id);
  }
}

/* Compatibility entry point retained for generated projects that still refer
 * to the old task name. */
void Communicate_Task(void *argument) { Communication_RX_Task(argument); }
