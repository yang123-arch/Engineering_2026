#include "Communication_Core.h"

#include <string.h>

#include "CRC8_CRC16.h"

static comm_port_t s_ports[COMM_MAX_PORTS];
static comm_device_t s_devices[COMM_MAX_DEVICES];
static uint16_t s_device_count;
static const comm_driver_t *s_drivers[COMM_MAX_DRIVERS];
static uint16_t s_driver_count;
static const comm_transport_ops_t *s_transport_ops[COMM_MAX_TRANSPORTS];
static comm_transport_id_t s_transport_ids[COMM_MAX_TRANSPORTS];
static uint16_t s_transport_count;
static TaskHandle_t s_rx_task;
static TaskHandle_t s_tx_task;
static volatile uint8_t s_configured;
static volatile uint8_t s_started;

static comm_port_t *port_from_id(comm_port_id_t id) {
  for (uint16_t i = 0U; i < COMM_MAX_PORTS; ++i)
    if (s_ports[i].configured && s_ports[i].port_id == id)
      return &s_ports[i];
  return NULL;
}

static comm_device_t *device_from_id(comm_device_id_t id) {
  for (uint16_t i = 0U; i < s_device_count; ++i)
    if (s_devices[i].configured && s_devices[i].device_id == id)
      return &s_devices[i];
  return NULL;
}

static const comm_transport_ops_t *transport_find(comm_transport_id_t id) {
  for (uint16_t i = 0U; i < s_transport_count; ++i)
    if (s_transport_ids[i] == id)
      return s_transport_ops[i];
  return NULL;
}

const comm_driver_t *Communication_Core_FindDriver(comm_driver_id_t id) {
  if (id == COMM_ID_INVALID)
    return NULL;
  for (uint16_t i = 0U; i < s_driver_count; ++i)
    if (s_drivers[i] != NULL && s_drivers[i]->driver_id == id)
      return s_drivers[i];
  return NULL;
}

comm_status_t Communication_Core_RegisterDriver(const comm_driver_t *driver) {
  if (driver == NULL || driver->driver_id == COMM_ID_INVALID)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (Communication_Core_FindDriver(driver->driver_id) != NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (s_driver_count >= COMM_MAX_DRIVERS)
    return COMM_STATUS_NO_SPACE;
  s_drivers[s_driver_count++] = driver;
  return COMM_STATUS_OK;
}

comm_status_t Communication_Core_RegisterTransport(
    comm_transport_id_t id, const comm_transport_ops_t *ops) {
  if (id == COMM_ID_INVALID || ops == NULL ||
      ops->try_start_tx == NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (transport_find(id) != NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (s_transport_count >= COMM_MAX_TRANSPORTS)
    return COMM_STATUS_NO_SPACE;
  s_transport_ids[s_transport_count] = id;
  s_transport_ops[s_transport_count] = ops;
  ++s_transport_count;
  return COMM_STATUS_OK;
}

static uint16_t check_trailer_length(const comm_frame_config_t *config) {
  uint16_t length = config->tail_len;
  if (config->length_mode == COMM_LENGTH_FIELD &&
      config->length_position == COMM_LENGTH_FIELD_BEFORE_TAIL)
    length = (uint16_t)(length + config->length_size);
  for (uint8_t i = 0U; i < config->check_count; ++i)
    if (config->checks[i].position == COMM_CHECK_BEFORE_TAIL)
      length = (uint16_t)(length + config->checks[i].size);
  return length;
}

static uint8_t fields_overlap(uint16_t a, uint16_t a_size,
                              uint16_t b, uint16_t b_size) {
  return a_size != 0U && b_size != 0U &&
         (uint32_t)a < (uint32_t)b + b_size &&
         (uint32_t)b < (uint32_t)a + a_size;
}

comm_status_t Communication_FrameConfigInit(comm_frame_config_t *config) {
  uint32_t prefix_end;
  uint16_t trailer_len;
  uint32_t overhead;

  if (config == NULL || config->head_len > COMM_MAX_HEAD_SIZE ||
      config->tail_len > COMM_MAX_TAIL_SIZE || config->length_size > 4U ||
      (unsigned)config->length_mode > COMM_LENGTH_FIELD ||
      (unsigned)config->length_position > COMM_LENGTH_FIELD_BEFORE_TAIL ||
      (unsigned)config->length_value_mode > COMM_LENGTH_VALUE_FRAME ||
      config->length_little_endian > 1U ||
      config->check_count > COMM_MAX_CHECK_FIELDS)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (config->length_mode == COMM_LENGTH_FIELD &&
      (config->length_size == 0U ||
       (uint32_t)config->length_offset + config->length_size > COMM_MAX_FRAME_SIZE))
    return COMM_STATUS_INVALID_ARGUMENT;
  if (config->length_mode == COMM_LENGTH_FIXED && config->length_size != 0U)
    return COMM_STATUS_INVALID_ARGUMENT;

  prefix_end = config->head_len;
  if (config->length_mode == COMM_LENGTH_FIELD &&
      config->length_position == COMM_LENGTH_FIELD_AFTER_HEAD) {
    if (config->length_offset < config->head_len)
      return COMM_STATUS_INVALID_ARGUMENT;
    prefix_end = (uint32_t)config->length_offset + config->length_size;
  }
  for (uint8_t i = 0U; i < config->check_count; ++i) {
    comm_check_field_t *check = &config->checks[i];
    uint8_t expected_size;
    if ((unsigned)check->type > COMM_CHECK_CRC16 ||
        (unsigned)check->position > COMM_CHECK_BEFORE_TAIL)
      return COMM_STATUS_INVALID_ARGUMENT;
    if (check->type == COMM_CHECK_NONE) {
      check->size = 0U;
      continue;
    }
    expected_size = (check->type == COMM_CHECK_SUM16 ||
                     check->type == COMM_CHECK_CRC16) ? 2U : 1U;
    if (check->size == 0U)
      check->size = expected_size;
    if (check->size != expected_size)
      return COMM_STATUS_INVALID_ARGUMENT;
    if (check->position == COMM_CHECK_AFTER_HEAD) {
      if (check->offset < config->head_len ||
          (uint32_t)check->offset + check->size > COMM_MAX_FRAME_SIZE)
        return COMM_STATUS_INVALID_ARGUMENT;
      if (prefix_end < (uint32_t)check->offset + check->size)
        prefix_end = (uint32_t)check->offset + check->size;
      if (config->length_mode == COMM_LENGTH_FIELD &&
          config->length_position == COMM_LENGTH_FIELD_AFTER_HEAD &&
          fields_overlap(check->offset, check->size,
                         config->length_offset, config->length_size))
        return COMM_STATUS_INVALID_ARGUMENT;
      for (uint8_t j = 0U; j < i; ++j)
        if (config->checks[j].position == COMM_CHECK_AFTER_HEAD &&
            fields_overlap(check->offset, check->size,
                           config->checks[j].offset, config->checks[j].size))
          return COMM_STATUS_INVALID_ARGUMENT;
    }
  }
  if (config->data_offset == 0U)
    config->data_offset = (uint16_t)prefix_end;
  if (config->data_offset < prefix_end)
    return COMM_STATUS_INVALID_ARGUMENT;
  trailer_len = check_trailer_length(config);
  overhead = (uint32_t)config->data_offset + trailer_len;
  if (overhead > COMM_MAX_FRAME_SIZE)
    return COMM_STATUS_FRAME_INVALID;
  config->fixed_overhead_len = (uint16_t)overhead;
  config->fixed_total_len = 0U;

  if (config->length_mode == COMM_LENGTH_FIXED && config->fixed_data_len == 0U &&
      config->tail_len == 0U)
    return COMM_STATUS_UNSUPPORTED;
  if (config->max_data_len == 0U)
    config->max_data_len = (uint16_t)(COMM_MAX_FRAME_SIZE - overhead);
  if (config->max_data_len > COMM_MAX_FRAME_SIZE - overhead ||
      config->length_body_prefix_len > config->max_data_len)
    return COMM_STATUS_FRAME_INVALID;
  if (config->length_mode == COMM_LENGTH_FIELD &&
      config->length_position == COMM_LENGTH_FIELD_BEFORE_TAIL &&
      (config->length_offset < config->data_offset ||
       config->length_offset > (uint32_t)config->data_offset + config->max_data_len))
    return COMM_STATUS_INVALID_ARGUMENT;
  if (config->length_mode == COMM_LENGTH_FIXED && config->fixed_data_len != 0U) {
    uint32_t total = overhead + config->fixed_data_len;
    if (config->fixed_data_len > config->max_data_len || total > COMM_MAX_FRAME_SIZE)
      return COMM_STATUS_FRAME_INVALID;
    config->fixed_total_len = (uint16_t)total;
  }
  return COMM_STATUS_OK;
}

static comm_status_t configure_port(const comm_port_config_t *config) {
  comm_port_t *port = NULL;
  if (config == NULL || config->port_id == COMM_ID_INVALID ||
      config->transport_id == COMM_ID_INVALID)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (port_from_id(config->port_id) != NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  for (uint16_t i = 0U; i < COMM_MAX_PORTS; ++i) {
    if (!s_ports[i].configured) {
      port = &s_ports[i];
      break;
    }
  }
  if (port == NULL)
    return COMM_STATUS_NO_SPACE;
  port->port_id = config->port_id;
  port->transport_id = config->transport_id;
  port->hardware = config->hardware;
  port->rx_dma = config->rx_dma;
  port->rx_dma_buffer = config->rx_dma_buffer;
  port->rx_dma_capacity = config->rx_dma_capacity;
  port->transport = transport_find(config->transport_id);
  if (port->transport == NULL)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  RingBuffer_Init(&port->rx_ring, port->rx_ring_storage,
                  COMM_RX_RING_SIZE, sizeof(uint8_t));
  port->tx_queue = xQueueCreateStatic(
      COMM_TX_QUEUE_DEPTH, sizeof(comm_frame_item_t), port->tx_queue_storage,
      &port->tx_queue_object);
  if (port->tx_queue == NULL)
    return COMM_STATUS_DRIVER_ERROR;
  port->configured = 1U;
  return COMM_STATUS_OK;
}

static comm_status_t configure_device(const comm_device_config_t *config,
                                      comm_endpoint_type_t endpoint) {
  comm_device_t *device = NULL;
  const comm_driver_t *driver;
  comm_port_t *port;
  comm_frame_config_t frame;
  const comm_frame_config_t *frame_source;
  uint8_t is_new = 0U;

  if (config == NULL || !config->enabled || config->device_id == COMM_ID_INVALID ||
      config->driver_id == COMM_ID_INVALID || config->port_id == COMM_ID_INVALID ||
      config->endpoint != endpoint)
    return config != NULL && !config->enabled ? COMM_STATUS_OK
                                               : COMM_STATUS_INVALID_ARGUMENT;
  port = port_from_id(config->port_id);
  driver = Communication_Core_FindDriver(config->driver_id);
  if (port == NULL || driver == NULL)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  if (endpoint == COMM_ENDPOINT_TX && driver->pack == NULL)
    return COMM_STATUS_UNSUPPORTED;
  if (endpoint == COMM_ENDPOINT_RX) {
    frame_source = config->frame != NULL ? config->frame : driver->frame;
    if (driver->extract_frame == NULL && frame_source == NULL)
      return COMM_STATUS_UNSUPPORTED;
    if (frame_source != NULL) {
      comm_status_t status;
      frame = *frame_source;
      status = Communication_FrameConfigInit(&frame);
      if (status != COMM_STATUS_OK)
        return status;
    } else {
      memset(&frame, 0, sizeof(frame));
    }
  }

  device = device_from_id(config->device_id);
  if (device == NULL) {
    if (s_device_count >= COMM_MAX_DEVICES)
      return COMM_STATUS_NO_SPACE;
    device = &s_devices[s_device_count++];
    memset(device, 0, sizeof(*device));
    device->configured = 1U;
    device->device_id = config->device_id;
    device->driver = driver;
    is_new = 1U;
  } else if (device->driver != driver) {
    return COMM_STATUS_INVALID_ARGUMENT;
  }

  if (endpoint == COMM_ENDPOINT_RX) {
    for (uint16_t i = 0U; i < s_device_count; ++i) {
      if (&s_devices[i] != device && s_devices[i].configured &&
          s_devices[i].rx_enabled &&
          s_devices[i].rx_port_id == config->port_id)
        return COMM_STATUS_INVALID_ARGUMENT;
    }
    if (device->rx_enabled)
      return COMM_STATUS_INVALID_ARGUMENT;
    device->rx_enabled = 1U;
    device->rx_port_id = config->port_id;
    device->rx_port = port;
    device->endpoint = COMM_ENDPOINT_RX;
    device->frame = frame;
  } else {
    if (device->tx_enabled)
      return COMM_STATUS_INVALID_ARGUMENT;
    device->tx_enabled = 1U;
    device->tx_port_id = config->port_id;
    device->tx_port = port;
    device->endpoint = COMM_ENDPOINT_TX;
  }

  if (is_new && driver->bind != NULL) {
    comm_status_t status = driver->bind((comm_driver_t *)driver, device);
    if (status != COMM_STATUS_OK)
      return status;
  }
  return COMM_STATUS_OK;
}

comm_status_t Communication_Core_Configure(const comm_board_config_t *config) {
  if (config == NULL || config->ports == NULL ||
      config->port_count > COMM_MAX_PORTS ||
      config->rx_device_count > COMM_MAX_RX_DEVICES ||
      config->tx_device_count > COMM_MAX_TX_DEVICES ||
      (config->rx_device_count != 0U && config->rx_devices == NULL) ||
      (config->tx_device_count != 0U && config->tx_devices == NULL))
    return COMM_STATUS_INVALID_ARGUMENT;
  if (s_configured)
    return COMM_STATUS_OK;

  for (uint16_t i = 0U; i < config->port_count; ++i) {
    comm_status_t status = configure_port(&config->ports[i]);
    if (status != COMM_STATUS_OK)
      return status;
  }
  for (uint16_t i = 0U; i < config->rx_device_count; ++i) {
    comm_status_t status = configure_device(&config->rx_devices[i],
                                            COMM_ENDPOINT_RX);
    if (status != COMM_STATUS_OK)
      return status;
  }
  for (uint16_t i = 0U; i < config->tx_device_count; ++i) {
    comm_status_t status = configure_device(&config->tx_devices[i],
                                            COMM_ENDPOINT_TX);
    if (status != COMM_STATUS_OK)
      return status;
  }
  s_configured = 1U;
  return COMM_STATUS_OK;
}

static void notify_task_from_isr(TaskHandle_t task) {
  if (task != NULL) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void Communication_Core_RegisterRxTask(TaskHandle_t task) { s_rx_task = task; }
void Communication_Core_RegisterTxTask(TaskHandle_t task) { s_tx_task = task; }

comm_status_t Communication_Core_Start(void) {
  if (!s_configured)
    return COMM_STATUS_NOT_INITIALIZED;
  if (s_started)
    return COMM_STATUS_OK;
  for (uint16_t i = 0U; i < COMM_MAX_PORTS; ++i) {
    if (s_ports[i].configured && s_ports[i].transport != NULL &&
        s_ports[i].transport->start_rx_dma != NULL) {
      comm_status_t status = s_ports[i].transport->start_rx_dma(&s_ports[i]);
      if (status != COMM_STATUS_OK)
        return status;
    }
  }
  s_started = 1U;
  return COMM_STATUS_OK;
}

comm_status_t Communication_Send(comm_device_id_t id, const void *data,
                                 uint16_t length) {
  comm_device_t *device;
  comm_port_t *port;
  comm_frame_item_t item = {0};
  comm_status_t status;

  if (!s_configured)
    return COMM_STATUS_NOT_INITIALIZED;
  device = device_from_id(id);
  if (device == NULL || !device->tx_enabled || device->driver == NULL)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  if (data == NULL || length == 0U)
    return COMM_STATUS_INVALID_ARGUMENT;
  port = device->tx_port;
  if (port == NULL || port->tx_queue == NULL)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  if (device->driver->pack == NULL)
    return COMM_STATUS_UNSUPPORTED;
  status = device->driver->pack((comm_driver_t *)device->driver, data, length,
                                item.data, sizeof(item.data), &item.length);
  if (status != COMM_STATUS_OK)
    return status;
  if (xQueueSend(port->tx_queue, &item, 0U) != pdPASS)
    return COMM_STATUS_NO_SPACE;
  if (s_tx_task != NULL)
    xTaskNotifyGive(s_tx_task);
  return COMM_STATUS_OK;
}

comm_status_t Communication_RxPushFromISR(comm_port_id_t port_id,
                                          const uint8_t *data,
                                          uint16_t length) {
  comm_port_t *port = port_from_id(port_id);
  uint8_t result;
  if (!s_configured)
    return COMM_STATUS_NOT_INITIALIZED;
  if (port == NULL || data == NULL || length == 0U)
    return COMM_STATUS_INVALID_ARGUMENT;
  result = RingBuffer_WriteFromISR(&port->rx_ring, data, length);
  if (result == RB_ERR_NOSPC) {
    ++port->rx_overflow_count;
    notify_task_from_isr(s_rx_task);
    return COMM_STATUS_BUFFER_OVERFLOW;
  }
  if (result != RB_OK)
    return COMM_STATUS_INVALID_ARGUMENT;
  notify_task_from_isr(s_rx_task);
  return COMM_STATUS_OK;
}

void Communication_TransportRxEventFromISR(void *hardware, uint16_t size) {
  if (!s_configured || hardware == NULL)
    return;
  for (uint16_t i = 0U; i < COMM_MAX_PORTS; ++i) {
    comm_port_t *port = &s_ports[i];
    if (!port->configured || port->hardware != hardware ||
        port->rx_dma_buffer == NULL)
      continue;
    if (size > port->rx_dma_capacity)
      size = port->rx_dma_capacity;
    if (size != 0U)
      (void)Communication_RxPushFromISR(port->port_id, port->rx_dma_buffer,
                                        size);
    if (port->transport != NULL && port->transport->start_rx_dma != NULL &&
        port->transport->start_rx_dma(port) != COMM_STATUS_OK)
      port->rx_error = 1U;
    return;
  }
}

void Communication_TransportTxCompleteFromISR(void *hardware) {
  if (!s_configured || hardware == NULL)
    return;
  for (uint16_t i = 0U; i < COMM_MAX_PORTS; ++i) {
    comm_port_t *port = &s_ports[i];
    if (port->configured && port->hardware == hardware) {
      port->tx_state.tx_busy = 0U;
      port->tx_state.has_active_frame = 0U;
      notify_task_from_isr(s_tx_task);
      return;
    }
  }
}

void Communication_TransportErrorFromISR(void *hardware) {
  if (!s_configured || hardware == NULL)
    return;
  for (uint16_t i = 0U; i < COMM_MAX_PORTS; ++i) {
    comm_port_t *port = &s_ports[i];
    if (port->configured && port->hardware == hardware) {
      port->rx_error = 1U;
      port->tx_state.tx_busy = 0U;
      notify_task_from_isr(s_rx_task);
      notify_task_from_isr(s_tx_task);
      return;
    }
  }
}

static uint32_t read_length_value(const uint8_t *frame,
                                  const comm_frame_config_t *config) {
  uint32_t value = 0U;
  if (config->length_little_endian) {
    for (uint8_t i = 0U; i < config->length_size; ++i)
      value |= (uint32_t)frame[config->length_offset + i] << (8U * i);
  } else {
    for (uint8_t i = 0U; i < config->length_size; ++i)
      value = (value << 8U) | frame[config->length_offset + i];
  }
  return value;
}

static uint16_t check_field_offset(const comm_frame_config_t *config,
                                   uint16_t frame_length, uint8_t index) {
  const comm_check_field_t *field = &config->checks[index];
  uint16_t offset = field->offset;
  if (field->position == COMM_CHECK_BEFORE_TAIL) {
    offset = (uint16_t)(frame_length - config->tail_len);
    for (uint8_t i = 0U; i <= index; ++i) {
      if (config->checks[i].position == COMM_CHECK_BEFORE_TAIL)
        offset = (uint16_t)(offset - config->checks[i].size);
    }
  }
  return offset;
}

static comm_status_t verify_check(const uint8_t *frame, uint16_t offset,
                                   const comm_check_field_t *field) {
  uint16_t expected;
  switch (field->type) {
  case COMM_CHECK_NONE:
    return COMM_STATUS_OK;
  case COMM_CHECK_SUM8:
  case COMM_CHECK_SUM16: {
    uint32_t sum = 0U;
    for (uint16_t i = 0U; i < offset; ++i)
      sum += frame[i];
    expected = field->size == 1U ? (uint8_t)sum : (uint16_t)sum;
    break;
  }
  case COMM_CHECK_CRC8:
    expected = get_CRC8_check_sum((uint8_t *)frame, offset, 0xFFU);
    break;
  case COMM_CHECK_CRC16:
    expected = get_CRC16_check_sum((uint8_t *)frame, offset, 0xFFFFU);
    break;
  default:
    return COMM_STATUS_FRAME_INVALID;
  }
  if ((uint8_t)expected != frame[offset] ||
      (field->size == 2U && (uint8_t)(expected >> 8U) != frame[offset + 1U]))
    return COMM_STATUS_CHECKSUM_ERROR;
  return COMM_STATUS_OK;
}

comm_status_t Communication_FrameExtract(
    const comm_frame_config_t *config, const uint8_t *frame, uint16_t available,
    uint16_t *frame_length, uint16_t *data_offset, uint16_t *data_length) {
  uint32_t total;
  uint32_t body_length;
  uint16_t trailer_length;
  comm_status_t status;

  if (config == NULL || frame == NULL || frame_length == NULL ||
      data_offset == NULL || data_length == NULL || available > COMM_MAX_FRAME_SIZE)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (available == 0U)
    return COMM_STATUS_NO_DATA;
  /* The configuration must be a copy validated by FrameConfigInit. */
  if (config->head_len > COMM_MAX_HEAD_SIZE || config->tail_len > COMM_MAX_TAIL_SIZE ||
      config->check_count > COMM_MAX_CHECK_FIELDS ||
      config->fixed_overhead_len > COMM_MAX_FRAME_SIZE)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (memcmp(frame, config->head,
             available < config->head_len ? available : config->head_len) != 0)
    return COMM_STATUS_FRAME_INVALID;
  if (available < config->head_len)
    return COMM_STATUS_NEED_MORE_DATA;
  /* Verify the header before trusting a corrupt length and waiting for it. */
  for (uint8_t i = 0U; i < config->check_count; ++i) {
    const comm_check_field_t *field = &config->checks[i];
    if (field->type == COMM_CHECK_NONE || field->position != COMM_CHECK_AFTER_HEAD)
      continue;
    if ((uint32_t)field->offset + field->size > available)
      return COMM_STATUS_NEED_MORE_DATA;
    status = verify_check(frame, field->offset, field);
    if (status != COMM_STATUS_OK)
      return status;
  }
  trailer_length = check_trailer_length(config);
  if (config->length_mode == COMM_LENGTH_FIXED) {
    if (config->fixed_data_len != 0U) {
      total = config->fixed_total_len;
      body_length = config->fixed_data_len;
    } else {
      /* A tail-only format ends at the first complete delimiter. Protocols
       * allowing an unescaped delimiter in data need extract_frame instead. */
      if (config->tail_len == 0U)
        return COMM_STATUS_UNSUPPORTED;
      total = (uint32_t)config->data_offset + trailer_length;
      while (total <= available &&
             total <= (uint32_t)config->fixed_overhead_len + config->max_data_len) {
        if (memcmp(frame + total - config->tail_len,
                   config->tail, config->tail_len) == 0)
          break;
        ++total;
      }
      if (total > available) {
        if (available >= (uint32_t)config->fixed_overhead_len + config->max_data_len)
          return COMM_STATUS_FRAME_INVALID;
        return COMM_STATUS_NEED_MORE_DATA;
      }
      body_length = total - config->fixed_overhead_len;
    }
  } else {
    uint32_t length_value;
    if ((uint32_t)config->length_offset + config->length_size > available)
      return COMM_STATUS_NEED_MORE_DATA;
    length_value = read_length_value(frame, config);
    if (config->length_value_mode == COMM_LENGTH_VALUE_FRAME) {
      if (length_value < config->fixed_overhead_len || length_value > COMM_MAX_FRAME_SIZE)
        return COMM_STATUS_FRAME_INVALID;
      total = length_value;
      body_length = total - config->fixed_overhead_len;
    } else {
      if (length_value > config->max_data_len - config->length_body_prefix_len)
        return COMM_STATUS_FRAME_INVALID;
      body_length = length_value + config->length_body_prefix_len;
      total = config->fixed_overhead_len + body_length;
    }
    /* Footer offsets remain absolute. Moving offsets require extract_frame. */
    if (config->length_position == COMM_LENGTH_FIELD_BEFORE_TAIL &&
        config->length_offset != (uint32_t)config->data_offset + body_length)
      return COMM_STATUS_FRAME_INVALID;
    if (body_length < config->length_body_prefix_len)
      return COMM_STATUS_FRAME_INVALID;
  }
  if (total == 0U || total > COMM_MAX_FRAME_SIZE ||
      total < config->fixed_overhead_len || body_length > config->max_data_len)
    return COMM_STATUS_FRAME_INVALID;
  if (available < total)
    return COMM_STATUS_NEED_MORE_DATA;
  if (config->tail_len != 0U &&
      memcmp(frame + total - config->tail_len, config->tail, config->tail_len) != 0)
    return COMM_STATUS_FRAME_INVALID;
  for (uint8_t i = 0U; i < config->check_count; ++i) {
    const comm_check_field_t *field = &config->checks[i];
    uint16_t offset;
    if (field->type == COMM_CHECK_NONE || field->position != COMM_CHECK_BEFORE_TAIL)
      continue;
    offset = check_field_offset(config, (uint16_t)total, i);
    status = verify_check(frame, offset, field);
    if (status != COMM_STATUS_OK)
      return status;
  }
  *frame_length = (uint16_t)total;
  *data_offset = config->data_offset;
  *data_length = (uint16_t)body_length;
  return COMM_STATUS_OK;
}

comm_status_t Communication_Core_RxPoll(comm_device_id_t id) {
  comm_device_t *device = device_from_id(id);
  comm_port_t *port;
  uint8_t frame[COMM_MAX_FRAME_SIZE];
  uint32_t count;
  uint16_t available;
  uint16_t frame_length = 0U;
  uint16_t data_offset = 0U;
  uint16_t data_length = 0U;
  comm_status_t status = COMM_STATUS_OK;

  if (!s_configured)
    return COMM_STATUS_NOT_INITIALIZED;
  if (device == NULL || !device->rx_enabled || device->driver == NULL)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  port = device->rx_port;
  if (port == NULL)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  if (port->rx_error) {
    port->rx_error = 0U;
    if (port->transport != NULL && port->transport->recover != NULL)
      port->transport->recover(port);
  }
  count = RingBuffer_Count(&port->rx_ring);
  if (count == 0U)
    return COMM_STATUS_NO_DATA;
  available = (uint16_t)(count > COMM_MAX_FRAME_SIZE ? COMM_MAX_FRAME_SIZE : count);
  if (RingBuffer_Peek(&port->rx_ring, frame, available) != RB_OK)
    return COMM_STATUS_NEED_MORE_DATA;

  if (device->driver->extract_frame != NULL) {
    status = device->driver->extract_frame((comm_driver_t *)device->driver,
                                           device, frame, available,
                                           &frame_length, &data_offset,
                                           &data_length);
    if (status != COMM_STATUS_OK) {
      if (status == COMM_STATUS_FRAME_INVALID ||
          status == COMM_STATUS_CHECKSUM_ERROR)
        RingBuffer_Skip(&port->rx_ring, 1U);
      return status;
    }
  } else {
    status = Communication_FrameExtract(&device->frame, frame, available, &frame_length,
                                   &data_offset, &data_length);
    if (status != COMM_STATUS_OK) {
      if (status == COMM_STATUS_FRAME_INVALID ||
          status == COMM_STATUS_CHECKSUM_ERROR)
        RingBuffer_Skip(&port->rx_ring, 1U);
      return status;
    }
  }

  if (frame_length == 0U || frame_length > available ||
      (uint32_t)data_offset + data_length > frame_length ||
      data_length > COMM_MAX_FRAME_SIZE) {
    RingBuffer_Skip(&port->rx_ring, 1U);
    return COMM_STATUS_DRIVER_ERROR;
  }
  device->rx_frame_length = frame_length;
  device->rx_data_offset = data_offset;
  if (device->driver->unpack != NULL)
    status = device->driver->unpack((comm_driver_t *)device->driver, device,
                                    frame + data_offset, data_length);
  RingBuffer_Skip(&port->rx_ring, frame_length);
  return status == COMM_STATUS_OK ? COMM_STATUS_OK : COMM_STATUS_DRIVER_ERROR;
}

static comm_status_t port_tx_poll(comm_port_t *port) {
  comm_status_t status;
  if (port == NULL || port->tx_queue == NULL || port->transport == NULL ||
      port->transport->try_start_tx == NULL)
    return COMM_STATUS_INVALID_ARGUMENT;
  if (port->tx_state.has_active_frame && port->tx_state.tx_busy) {
    if (port->transport->is_tx_idle == NULL ||
        !port->transport->is_tx_idle(port))
      return COMM_STATUS_BUSY;
    port->tx_state.tx_busy = 0U;
    port->tx_state.has_active_frame = 0U;
  }
  if (!port->tx_state.has_active_frame) {
    if (xQueueReceive(port->tx_queue, &port->tx_state.active_frame, 0U) !=
        pdPASS)
      return COMM_STATUS_NO_DATA;
    port->tx_state.has_active_frame = 1U;
  }
  status = port->transport->try_start_tx(
      port, port->tx_state.active_frame.data,
      port->tx_state.active_frame.length);
  if (status == COMM_STATUS_OK) {
    port->tx_state.tx_busy = 1U;
    return status;
  }
  if (status == COMM_STATUS_BUSY)
    return status;
  port->tx_state.has_active_frame = 0U;
  ++port->tx_error_count;
  return status;
}

comm_status_t Communication_Core_TxPoll(comm_device_id_t id) {
  comm_device_t *device = device_from_id(id);
  if (device == NULL || !device->tx_enabled)
    return COMM_STATUS_DEVICE_NOT_FOUND;
  return port_tx_poll(device->tx_port);
}
