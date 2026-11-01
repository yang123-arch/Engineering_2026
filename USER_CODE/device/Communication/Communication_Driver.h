#ifndef COMMUNICATION_DRIVER_H
#define COMMUNICATION_DRIVER_H

#include "Communication_Core.h"

/* Protocol driver contract:
 * - frame is an immutable default; Init may supply a per-device override.
 * - unpack receives only the body (including command ID, if the protocol has
 *   one). It must not access queues/rings or retain the temporary data pointer.
 * - extract_frame is optional for multi-format/escaped formats. It inspects a
 *   snapshot, returns NEED_MORE_DATA without consuming it, and writes the
 *   three output values only on OK. Core owns resynchronization and delivery.
 * - rx_frame_length/rx_data_offset are set by Core before unpack, so protocols
 *   must use these metadata rather than guessing the format from body length.
 * - pack remains nonblocking and must be safe for concurrent Send callers.
 */

#endif
