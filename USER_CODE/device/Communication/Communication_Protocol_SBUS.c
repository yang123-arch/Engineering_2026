#include "Communication_Protocol_SBUS.h"

#include "Remote_Control.h"

static comm_status_t sbus_init(comm_driver_t *driver) {
  (void)driver;
  return COMM_STATUS_OK;
}

static comm_status_t sbus_bind(comm_driver_t *driver, comm_device_t *device) {
  (void)driver;
  return device != NULL ? COMM_STATUS_OK : COMM_STATUS_INVALID_ARGUMENT;
}

static const comm_frame_config_t s_sbus_frame = {
    .length_mode = COMM_LENGTH_FIXED,
    .fixed_data_len = 18U,
    .max_data_len = 18U,
};

static comm_status_t sbus_unpack(comm_driver_t *driver,
                                 comm_device_t *device, const uint8_t *data,
                                 uint16_t length) {
  (void)driver;
  (void)device;
  if (data == NULL || length != 18U)
    return COMM_STATUS_INVALID_ARGUMENT;
  Remote_Rx_CallBack_SBUS((uint8_t *)data);
  return COMM_STATUS_OK;
}

static comm_status_t sbus_pack(comm_driver_t *driver, const void *data,
                               uint16_t length, uint8_t *frame,
                               uint16_t capacity, uint16_t *frame_length) {
  (void)driver;
  return g_raw_pack_compat(data, length, frame, capacity, frame_length);
}

comm_driver_t g_sbus_driver = {
    .driver_id = COMM_DRIVER_SBUS,
    .init = sbus_init,
    .bind = sbus_bind,
    .frame = &s_sbus_frame,
    .unpack = sbus_unpack,
    .extract_frame = NULL,
    .pack = sbus_pack,
};
