#include "Communication_Protocol_ImageLink.h"

#include "Communication_Protocol_Referee.h"
#include "Referee_Unpack.h"
#include "Remote_Control.h"

static comm_status_t image_init(comm_driver_t *driver) {
  (void)driver;
  return COMM_STATUS_OK;
}

static comm_status_t image_bind(comm_driver_t *driver, comm_device_t *device) {
  (void)driver;
  return device != NULL ? COMM_STATUS_OK : COMM_STATUS_INVALID_ARGUMENT;
}

static comm_status_t image_extract_frame(
    comm_driver_t *driver, comm_device_t *device, const uint8_t *available,
    uint16_t available_length, uint16_t *frame_length, uint16_t *data_offset,
    uint16_t *data_length) {
  (void)driver;
  if (device == NULL || available == NULL || frame_length == NULL ||
      data_offset == NULL || data_length == NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (available_length == 0U)
    return COMM_STATUS_NO_DATA;
  if (available[0] == 0xA5U)
    return Communication_FrameExtract(&device->frame, available, available_length,
                                      frame_length, data_offset, data_length);
  if (available[0] != 0xA9U)
    return COMM_STATUS_FRAME_INVALID;
  if (available_length < 2U)
    return COMM_STATUS_NEED_MORE_DATA;
  if (available[1] != 0x53U)
    return COMM_STATUS_FRAME_INVALID;
  if (available_length < 20U)
    return COMM_STATUS_NEED_MORE_DATA;
  *frame_length = 20U;
  *data_offset = 2U;
  *data_length = 18U;
  return COMM_STATUS_OK;
}

static comm_status_t image_unpack(comm_driver_t *driver, comm_device_t *device,
                                  const uint8_t *data, uint16_t length) {
  (void)driver;
  if (device == NULL || data == NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  /* An A5 command body can also be 18 bytes; use the extracted frame metadata. */
  if (device->rx_data_offset == 2U && device->rx_frame_length == 20U) {
    if (length != 18U)
      return COMM_STATUS_FRAME_INVALID;
    Remote_Rx_CallBack_SBUS_Marked((uint8_t *)data, 0xA9U);
    return COMM_STATUS_OK;
  }
  return ImageLink_Data_UnpackPayload(data, length);
}

static comm_status_t image_pack(comm_driver_t *driver, const void *data,
                                uint16_t length, uint8_t *frame,
                                uint16_t capacity, uint16_t *frame_length) {
  (void)driver;
  return g_raw_pack_compat(data, length, frame, capacity, frame_length);
}

comm_driver_t g_imagelink_driver = {
    .driver_id = COMM_DRIVER_IMAGELINK,
    .init = image_init,
    .bind = image_bind,
    .frame = &g_referee_frame_config,
    .unpack = image_unpack,
    .extract_frame = image_extract_frame,
    .pack = image_pack,
};
