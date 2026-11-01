#ifndef COMMUNICATION_PROTOCOL_RAW_H
#define COMMUNICATION_PROTOCOL_RAW_H

#include "Communication_Core.h"

#include <string.h>

#define COMM_DRIVER_RAW 0x01U

extern comm_driver_t g_raw_driver;

static inline comm_status_t g_raw_pack_compat(const void *data, uint16_t length,
                                              uint8_t *frame, uint16_t capacity,
                                              uint16_t *frame_length) {
  if (data == NULL || frame == NULL || frame_length == NULL || length == 0U ||
      length > capacity || length > COMM_MAX_FRAME_SIZE)
    return COMM_STATUS_INVALID_ARGUMENT;
  memcpy(frame, data, length);
  *frame_length = length;
  return COMM_STATUS_OK;
}

#endif
