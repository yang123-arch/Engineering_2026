#ifndef COMMUNICATION_INIT_H
#define COMMUNICATION_INIT_H

#include "Communication_Core.h"
#include "Communication_Protocol_ImageLink.h"
#include "Communication_Protocol_Raw.h"
#include "Communication_Protocol_Referee.h"
#include "Communication_Protocol_SBUS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Board-owned transport and port identifiers.  Core treats all of them as
 * opaque bytes, so a board migration only changes this header and Init.c. */
#define COMM_TRANSPORT_UART_DMA_IDLE 0x01U
#define COMM_TRANSPORT_USB_CDC 0x02U

#define COMM_PORT_UART1 0x11U
#define COMM_PORT_UART2 0x12U
#define COMM_PORT_UART3 0x13U
#define COMM_PORT_UART4 0x14U
#define COMM_PORT_UART5 0x15U
#define COMM_PORT_USB 0x16U

/* RX and TX aliases deliberately identify the same physical port. */
#define COMM_PORT_UART1_RX COMM_PORT_UART1
#define COMM_PORT_UART1_TX COMM_PORT_UART1
#define COMM_PORT_UART2_RX COMM_PORT_UART2
#define COMM_PORT_UART2_TX COMM_PORT_UART2
#define COMM_PORT_UART3_RX COMM_PORT_UART3
#define COMM_PORT_UART3_TX COMM_PORT_UART3
#define COMM_PORT_UART4_RX COMM_PORT_UART4
#define COMM_PORT_UART4_TX COMM_PORT_UART4
#define COMM_PORT_UART5_RX COMM_PORT_UART5
#define COMM_PORT_UART5_TX COMM_PORT_UART5
#define COMM_PORT_USB_RX COMM_PORT_USB
#define COMM_PORT_USB_TX COMM_PORT_USB

/* Board-owned logical device identifiers. */
#define DEVICE_SBUS 0x11U
#define DEVICE_REFEREE 0x12U
#define DEVICE_IMAGELINK 0x13U
#define DEVICE_RS485_1 0x14U
#define DEVICE_RS485_2 0x15U
#define DEVICE_UI 0x16U
#define DEVICE_USB_DEBUG 0x17U

/* Compatibility aliases for existing application code. */
#define COMM_DEVICE_SBUS DEVICE_SBUS
#define COMM_DEVICE_REFEREE DEVICE_REFEREE
#define COMM_DEVICE_IMAGELINK DEVICE_IMAGELINK
#define COMM_DEVICE_RS485_1 DEVICE_RS485_1
#define COMM_DEVICE_RS485_2 DEVICE_RS485_2
#define COMM_DEVICE_UI DEVICE_UI
#define COMM_DEVICE_USB_DEBUG DEVICE_USB_DEBUG
#define COMM_PORT_UART1_ID COMM_PORT_UART1
#define COMM_PORT_UART2_ID COMM_PORT_UART2
#define COMM_PORT_UART3_ID COMM_PORT_UART3
#define COMM_PORT_UART4_ID COMM_PORT_UART4
#define COMM_PORT_UART5_ID COMM_PORT_UART5

#define COMM_RX_DEVICE_COUNT 3U

comm_status_t Communication_Init(void);
void Communication_RX_Task(void *argument);
void Communication_TX_Task(void *argument);
void Communicate_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_INIT_H */
