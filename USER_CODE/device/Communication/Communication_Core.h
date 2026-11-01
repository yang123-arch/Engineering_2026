#ifndef COMMUNICATION_CORE_H
#define COMMUNICATION_CORE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "RingBuffer.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_ID_INVALID 0xFFU
#define COMM_MAX_FRAME_SIZE 256U
#define COMM_MAX_HEAD_SIZE 4U
#define COMM_MAX_TAIL_SIZE 4U
#define COMM_RX_RING_SIZE 512U
#define COMM_TX_QUEUE_DEPTH 4U
#define COMM_POLL_TIMEOUT_MS 1U
#define COMM_MAX_PORTS 8U
#define COMM_MAX_RX_DEVICES 8U
#define COMM_MAX_TX_DEVICES 32U
#define COMM_MAX_DEVICES (COMM_MAX_RX_DEVICES + COMM_MAX_TX_DEVICES)
#define COMM_MAX_DRIVERS 32U
#define COMM_MAX_TRANSPORTS 8U

typedef uint8_t comm_device_id_t;
typedef uint8_t comm_driver_id_t;
typedef uint8_t comm_port_id_t;
typedef uint8_t comm_transport_id_t;

typedef enum {
  COMM_STATUS_OK = 0,
  COMM_STATUS_INVALID_ARGUMENT,
  COMM_STATUS_NOT_INITIALIZED,
  COMM_STATUS_DEVICE_NOT_FOUND,
  COMM_STATUS_NO_DATA,
  COMM_STATUS_NO_SPACE,
  COMM_STATUS_BUFFER_TOO_SMALL,
  COMM_STATUS_BUSY,
  COMM_STATUS_NEED_MORE_DATA,
  COMM_STATUS_FRAME_INVALID,
  COMM_STATUS_CHECKSUM_ERROR,
  COMM_STATUS_BUFFER_OVERFLOW,
  COMM_STATUS_DRIVER_ERROR,
  COMM_STATUS_UNSUPPORTED
} comm_status_t;

typedef enum {
  COMM_ENDPOINT_RX = 0,
  COMM_ENDPOINT_TX = 1
} comm_endpoint_type_t;

typedef enum { COMM_LENGTH_FIXED = 0, COMM_LENGTH_FIELD } comm_length_mode_t;

typedef enum {
  COMM_LENGTH_FIELD_AFTER_HEAD = 0,
  COMM_LENGTH_FIELD_BEFORE_TAIL = 1
} comm_length_field_position_t;

typedef enum {
  COMM_LENGTH_VALUE_BODY = 0,
  COMM_LENGTH_VALUE_FRAME = 1
} comm_length_value_mode_t;

typedef enum {
  COMM_CHECK_NONE = 0,
  COMM_CHECK_SUM8,
  COMM_CHECK_SUM16,
  COMM_CHECK_CRC8,
  COMM_CHECK_CRC16
} comm_check_type_t;

typedef enum {
  COMM_CHECK_AFTER_HEAD = 0,
  COMM_CHECK_BEFORE_TAIL
} comm_check_position_t;

typedef struct {
  comm_check_type_t type;
  comm_check_position_t position;
  /* AFTER_HEAD: absolute byte offset; BEFORE_TAIL: derived at runtime.
   * Checks cover bytes [0, offset); SUM16 and CRC16 are little endian.
   * For multiple footer checks, checks[0] is nearest the tail. */
  uint16_t offset;
  uint8_t size;
} comm_check_field_t;

#define COMM_MAX_CHECK_FIELDS 2U

typedef struct {
  uint8_t head[COMM_MAX_HEAD_SIZE];
  uint8_t head_len;
  uint8_t tail[COMM_MAX_TAIL_SIZE];
  uint8_t tail_len;
  comm_length_mode_t length_mode;
  comm_length_field_position_t length_position;
  comm_length_value_mode_t length_value_mode;
  /* FIXED with zero data length and a tail means delimiter-terminated data. */
  uint16_t fixed_data_len;
  /* Always absolute. A variable footer whose offset moves needs extract_frame. */
  uint16_t length_offset;
  uint8_t length_size;
  uint8_t length_little_endian;
  uint16_t data_offset;
  /* For BODY lengths, bytes in the body before the length-counted payload
   * (for example a two-byte command ID). */
  uint16_t length_body_prefix_len;
  uint16_t max_data_len;
  uint8_t check_count;
  comm_check_field_t checks[COMM_MAX_CHECK_FIELDS];
  uint16_t fixed_overhead_len;
  uint16_t fixed_total_len;
} comm_frame_config_t;

typedef struct {
  uint16_t length;
  uint8_t data[COMM_MAX_FRAME_SIZE];
} comm_frame_item_t;

struct comm_port_t;
struct comm_device_t;
struct comm_driver_t;

typedef comm_status_t (*comm_pack_fn)(struct comm_driver_t *driver,
                                      const void *object, uint16_t object_len,
                                      uint8_t *frame, uint16_t frame_capacity,
                                      uint16_t *frame_len);

typedef comm_status_t (*comm_unpack_fn)(struct comm_driver_t *driver,
                                        struct comm_device_t *device,
                                        const uint8_t *data, uint16_t length);

typedef comm_status_t (*comm_extract_frame_fn)(
    struct comm_driver_t *driver, struct comm_device_t *device,
    const uint8_t *available, uint16_t available_length,
    uint16_t *frame_length, uint16_t *data_offset, uint16_t *data_length);

/* extract_frame only inspects the supplied snapshot and writes outputs on OK.
 * Core alone consumes the RX ring. unpack runs once in the RX task after a
 * complete frame has been extracted; its data pointer is valid only for the
 * duration of the call. */

typedef struct {
  comm_status_t (*start_rx_dma)(struct comm_port_t *port);
  comm_status_t (*try_start_tx)(struct comm_port_t *port, const uint8_t *data,
                                uint16_t length);
  uint8_t (*is_tx_idle)(const struct comm_port_t *port);
  void (*recover)(struct comm_port_t *port);
} comm_transport_ops_t;

typedef struct comm_driver_t {
  comm_driver_id_t driver_id;
  comm_status_t (*init)(struct comm_driver_t *driver);
  comm_status_t (*bind)(struct comm_driver_t *driver,
                        struct comm_device_t *device);
  const comm_frame_config_t *frame;
  comm_unpack_fn unpack;
  comm_extract_frame_fn extract_frame;
  comm_pack_fn pack;
  void *context;
} comm_driver_t;

typedef struct {
  volatile uint8_t tx_busy;
  uint8_t has_active_frame;
  comm_frame_item_t active_frame;
} comm_tx_state_t;

typedef struct comm_port_t {
  uint8_t configured;
  comm_port_id_t port_id;
  comm_transport_id_t transport_id;
  void *hardware;
  void *rx_dma;
  uint8_t *rx_dma_buffer;
  uint16_t rx_dma_capacity;
  RingBuffer_t rx_ring;
  uint8_t rx_ring_storage[COMM_RX_RING_SIZE];
  QueueHandle_t tx_queue;
  StaticQueue_t tx_queue_object;
  uint8_t tx_queue_storage[COMM_TX_QUEUE_DEPTH * sizeof(comm_frame_item_t)];
  comm_tx_state_t tx_state;
  volatile uint8_t rx_error;
  uint32_t rx_overflow_count;
  uint32_t tx_error_count;
  const comm_transport_ops_t *transport;
} comm_port_t;

typedef struct comm_device_t {
  uint8_t configured;
  uint8_t rx_enabled;
  uint8_t tx_enabled;
  comm_device_id_t device_id;
  comm_port_id_t rx_port_id;
  comm_port_id_t tx_port_id;
  struct comm_port_t *rx_port;
  struct comm_port_t *tx_port;
  comm_endpoint_type_t endpoint;
  comm_frame_config_t frame;
  /* Complete-frame metadata for unpack; distinguishes multi-format payloads
   * even when two formats happen to have the same payload length. */
  uint16_t rx_frame_length;
  uint16_t rx_data_offset;
  const comm_driver_t *driver;
} comm_device_t;

typedef struct {
  comm_port_id_t port_id;
  comm_transport_id_t transport_id;
  void *hardware;
  void *rx_dma;
  uint8_t *rx_dma_buffer;
  uint16_t rx_dma_capacity;
} comm_port_config_t;

typedef struct {
  comm_device_id_t device_id;
  comm_driver_id_t driver_id;
  comm_port_id_t port_id;
  comm_endpoint_type_t endpoint;
  const comm_frame_config_t *frame;
  uint8_t enabled;
} comm_device_config_t;

typedef struct {
  const comm_port_config_t *ports;
  uint16_t port_count;
  const comm_device_config_t *rx_devices;
  uint16_t rx_device_count;
  const comm_device_config_t *tx_devices;
  uint16_t tx_device_count;
} comm_board_config_t;

comm_status_t Communication_Core_RegisterDriver(const comm_driver_t *driver);
const comm_driver_t *Communication_Core_FindDriver(comm_driver_id_t driver_id);
comm_status_t
Communication_Core_RegisterTransport(comm_transport_id_t transport_id,
                                     const comm_transport_ops_t *ops);
comm_status_t Communication_Core_Configure(const comm_board_config_t *config);
comm_status_t Communication_Core_Start(void);
comm_status_t Communication_Core_RxPoll(comm_device_id_t device_id);
comm_status_t Communication_Core_TxPoll(comm_device_id_t device_id);
void Communication_Core_RegisterRxTask(TaskHandle_t task);
void Communication_Core_RegisterTxTask(TaskHandle_t task);

comm_status_t Communication_Send(comm_device_id_t id, const void *data,
                                 uint16_t length);

comm_status_t Communication_RxPushFromISR(comm_port_id_t port_id,
                                          const uint8_t *data, uint16_t length);
void Communication_TransportRxEventFromISR(void *hardware, uint16_t size);
void Communication_TransportTxCompleteFromISR(void *hardware);
void Communication_TransportErrorFromISR(void *hardware);

comm_status_t Communication_FrameConfigInit(comm_frame_config_t *config);
/* Non-consuming parser; config must have passed FrameConfigInit. The three
 * outputs are changed only on OK. Useful in multi-format extract_frame hooks. */
comm_status_t Communication_FrameExtract(
    const comm_frame_config_t *config, const uint8_t *available,
    uint16_t available_length, uint16_t *frame_length, uint16_t *data_offset,
    uint16_t *data_length);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_CORE_H */
