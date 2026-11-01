#include "USB_Driver.h"
#include "Aim.h"
#include "Vofa.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"


uint8_t USB_Data[64] = {0};
uint32_t USB_Tick = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief USB发送数据
 * @note  不复制数据：返回 USBD_OK 之后、这一包发完之前，Data 的内容不能改。
 *        下一次调用返回 USBD_OK 时，上一包一定已经发完
 * @return USBD_OK 已开始发送；USBD_BUSY 上一包还没发完；USBD_FAIL USB 还没枚举
 */
uint8_t USB_Send(uint8_t *Data, uint8_t Length) {
  /* 没插电脑、还没枚举时 CDC 句柄为空，CDC_Transmit_FS 会解引用空指针 */
  if (hUsbDeviceFS.pClassData == NULL)
    return USBD_FAIL;
  return CDC_Transmit_FS(Data, Length);
}

/**
 * @brief USB接收数据回调函数
 */
void USB_ReceiveCallback(uint8_t *buf, uint32_t len) {
  memcpy(USB_Data, buf, len);
  USB_Tick++;

  // Aim_Get_Data_CallBack(buf);
}

/**
 * @brief USB_Printf
 */
uint8_t USB_Printf_Buff[128];
void USB_Printf(const char *fmt, ...) {
  static va_list ap;
  uint16_t len = 0;

  va_start(ap, fmt);

  len = vsprintf((char *)USB_Printf_Buff, fmt, ap);

  va_end(ap);

  CDC_Transmit_FS(USB_Printf_Buff, len);
}
