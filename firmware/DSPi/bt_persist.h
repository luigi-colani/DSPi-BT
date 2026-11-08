#ifndef BT_PERSIST_H
#define BT_PERSIST_H

#include <stdint.h>
#include <stdbool.h>

/*
 * bt_persist.h — persistence contract for the Bluetooth-enabled flag.
 *
 * Single source of truth, shared SDK-free by:
 *   - bulk_params.c    (the V29 wire-format bt-config section)
 *   - flash_storage.c  (the device-global directory bt_enabled field, DIR V18)
 *   - tools/bt_persist_test  (host verification)
 *
 * The canonical rule: the stored byte is 1 when Bluetooth output is enabled and
 * 0 when disabled, and ANY legacy or zero-filled byte — an old host's payload,
 * a freshly initialised directory, a migrated-from-V17 directory — decodes as
 * DISABLED.  "BT off" is therefore the valid default everywhere, so existing
 * stored configs and old hosts keep working with Bluetooth simply off.
 *
 * Mirrors bt_exclusion.h: pure integer logic in a header, so both the firmware
 * and the host test link one definition and cannot drift.
 */

#define BT_PERSIST_ENABLED_DEFAULT  0u

static inline uint8_t bt_persist_encode_enabled(bool on) {
    return on ? 1u : 0u;
}

static inline bool bt_persist_decode_enabled(uint8_t raw) {
    return raw != 0u;
}

#endif // BT_PERSIST_H
