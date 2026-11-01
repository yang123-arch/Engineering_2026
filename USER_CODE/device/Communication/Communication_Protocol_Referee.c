#include "Communication_Protocol_Referee.h"

#include "Referee_Unpack.h"

const comm_frame_config_t g_referee_frame_config = {
    .head = {0xA5U},
    .head_len = 1U,
    .length_mode = COMM_LENGTH_FIELD,
    .length_position = COMM_LENGTH_FIELD_AFTER_HEAD,
    .length_value_mode = COMM_LENGTH_VALUE_BODY,
    .length_offset = 1U,
    .length_size = 2U,
    .length_little_endian = 1U,
    .data_offset = 5U,
    .length_body_prefix_len = 2U,
    .max_data_len = 249U,
    .check_count = 2U,
    .checks = {{.type = COMM_CHECK_CRC8,
                .position = COMM_CHECK_AFTER_HEAD,
                .offset = 4U,
                .size = 1U},
               {.type = COMM_CHECK_CRC16,
                .position = COMM_CHECK_BEFORE_TAIL,
                .size = 2U}},
};

static comm_status_t referee_init(comm_driver_t *driver) {
  (void)driver;
  return COMM_STATUS_OK;
}

static comm_status_t referee_bind(comm_driver_t *driver,
                                  comm_device_t *device) {
  (void)driver;
  return device != NULL ? COMM_STATUS_OK : COMM_STATUS_INVALID_ARGUMENT;
}

static comm_status_t referee_unpack(comm_driver_t *driver,
                                    comm_device_t *device,
                                    const uint8_t *data, uint16_t length) {
  (void)driver;
  (void)device;
  return Referee_Data_UnpackPayload(data, length);
}

static comm_status_t referee_pack(comm_driver_t *driver, const void *data,
                                  uint16_t length, uint8_t *frame,
                                  uint16_t capacity,
                                  uint16_t *frame_length) {
  (void)driver;
  return g_raw_pack_compat(data, length, frame, capacity, frame_length);
}

comm_driver_t g_referee_driver = {
    .driver_id = COMM_DRIVER_REFEREE,
    .init = referee_init,
    .bind = referee_bind,
    .frame = &g_referee_frame_config,
    .unpack = referee_unpack,
    .extract_frame = NULL,
    .pack = referee_pack,
};
