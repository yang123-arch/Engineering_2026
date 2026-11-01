#include "Communication_Protocol_Raw.h"

#include <string.h>

static comm_status_t raw_init(comm_driver_t *driver) {
  (void)driver;
  return COMM_STATUS_OK;
}

static comm_status_t raw_bind(comm_driver_t *driver, comm_device_t *device) {
  (void)driver;
  return device != NULL ? COMM_STATUS_OK : COMM_STATUS_INVALID_ARGUMENT;
}

static comm_status_t raw_unpack(comm_driver_t *driver, comm_device_t *device,
                                const uint8_t *data, uint16_t length) {
  (void)driver;
  (void)device;
  (void)data;
  return length != 0U ? COMM_STATUS_OK : COMM_STATUS_INVALID_ARGUMENT;
}

static comm_status_t raw_pack(comm_driver_t *driver, const void *data,
                              uint16_t length, uint8_t *frame,
                              uint16_t capacity, uint16_t *frame_length) {
  (void)driver;
  if (data == NULL || frame == NULL || frame_length == NULL || length == 0U ||
      length > capacity || length > COMM_MAX_FRAME_SIZE)
    return COMM_STATUS_INVALID_ARGUMENT;
  memcpy(frame, data, length);
  *frame_length = length;
  return COMM_STATUS_OK;
}

comm_driver_t g_raw_driver = {
    .driver_id = COMM_DRIVER_RAW,
    .init = raw_init,
    .bind = raw_bind,
    .frame = NULL,
    .unpack = raw_unpack,
    .extract_frame = NULL,
    .pack = raw_pack,
};
