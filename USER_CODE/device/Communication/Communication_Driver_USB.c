#include "Communication_Driver_USB.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static comm_status_t usb_start_rx(comm_port_t *port) {
  (void)port;
  MX_USB_Device_Init();
  return COMM_STATUS_OK;
}

static comm_status_t usb_try_start_tx(comm_port_t *port, const uint8_t *data,
                                      uint16_t length) {
  (void)port;
  if (data == NULL || length == 0U)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (hUsbDeviceFS.pClassData == NULL)
    return COMM_STATUS_BUSY;

  uint8_t result = CDC_Transmit_FS((uint8_t *)data, length);
  if (result == USBD_OK)
    return COMM_STATUS_OK;
  if (result == USBD_BUSY)
    return COMM_STATUS_BUSY;
  return COMM_STATUS_DRIVER_ERROR;
}

static uint8_t usb_is_tx_idle(const comm_port_t *port) {
  (void)port;
  if (hUsbDeviceFS.pClassData == NULL)
    return 0U;
  return ((USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData)->TxState == 0U;
}

static const comm_transport_ops_t s_ops = {
    .start_rx_dma = usb_start_rx,
    .try_start_tx = usb_try_start_tx,
    .is_tx_idle = usb_is_tx_idle,
    .recover = NULL,
};

const comm_transport_ops_t *Communication_USB_Driver_Get(void) {
  return &s_ops;
}
