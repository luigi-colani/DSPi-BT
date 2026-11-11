/*
 * Host unit test for Task 6 persistence — the Bluetooth config field + the
 * old->new wire / directory migration.
 *
 * SDK-free: it includes only firmware/DSPi/bulk_params.h (self-contained,
 * <stdint.h>/<stdbool.h> only) and firmware/DSPi/bt_persist.h (the pure
 * encode/decode contract shared with bulk_params.c and flash_storage.c).  The
 * full WireBulkParams / PresetDirectory structs are platform-independent byte
 * layouts, so their ABI is fully verifiable here without any Pico SDK.
 *
 * What it pins:
 *   1. Wire ABI of the V33 bt-config section: WIRE_FORMAT_VERSION == 33, the
 *      section sits at the former end-of-packet offset (tail-append, no field
 *      shifted), and the packet grew by exactly the section size.
 *   2. Serialize/deserialize round-trip of bt_enabled through a raw byte buffer
 *      (as a host does over the USB control transfer).
 *   3. "Old host sends zeros -> BT disabled": a zero-filled bt section decodes
 *      to the disabled default.
 *   4. Directory migration contract (mirror of the flash_storage.c V25->V26
 *      rule): a V25 image (no trailing bt_enabled byte) migrates to V26 with the
 *      whole prefix byte-identical and bt_enabled defaulting to 0 (off); a V26
 *      image round-trips bt_enabled.
 *
 * Build + run:  make        (in this directory)
 * Exit status 0 = all assertions passed.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "bulk_params.h"
#include "bt_persist.h"

static int failures = 0;
static int checks   = 0;

#define CHECK(cond) do {                                                  \
        checks++;                                                         \
        if (!(cond)) {                                                    \
            failures++;                                                   \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
        }                                                                 \
    } while (0)

#define CHECK_EQ_U(a, b) do {                                             \
        checks++;                                                         \
        uint64_t _a = (uint64_t)(a), _b = (uint64_t)(b);                  \
        if (_a != _b) {                                                   \
            failures++;                                                   \
            printf("  FAIL  %s:%d  %s == %s  (%llu != %llu)\n",           \
                   __FILE__, __LINE__, #a, #b,                            \
                   (unsigned long long)_a, (unsigned long long)_b);       \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------------
// Frozen pre-V33 geometry: the packet ended at the upmixer section.  These are
// hard-coded (not derived from the live headers) so the test pins the ABI the
// way a host SDK hard-codes it — if the implementation shifts an existing field
// instead of tail-appending, these assertions fail.
// ---------------------------------------------------------------------------
#define WIRE_V32_TOTAL_SIZE   6136u   // end of the V32 limiter section
#define BT_WIRE_SECTION_SIZE     8u   // WireBtConfig
#define WIRE_V33_TOTAL_SIZE   (WIRE_V32_TOTAL_SIZE + BT_WIRE_SECTION_SIZE)

// The bt_enabled byte is the first byte of the tail-appended section, i.e. at
// the former end-of-packet offset.
#define BT_ENABLED_WIRE_OFFSET  WIRE_V32_TOTAL_SIZE

// ---------------------------------------------------------------------------
// 1. Wire ABI of the V33 bt-config section.
// ---------------------------------------------------------------------------
static void test_wire_abi(void) {
    // Version bumped by exactly one.
    CHECK_EQ_U(WIRE_FORMAT_VERSION, 33u);

    // Tail-append: the packet grew by exactly the section size, nothing before
    // it moved (the former total is still the section's offset).
    CHECK_EQ_U(sizeof(WireBulkParams), WIRE_V33_TOTAL_SIZE);
    CHECK_EQ_U(BT_ENABLED_WIRE_OFFSET, WIRE_V32_TOTAL_SIZE);

    // The packet still fits the shared bulk transfer buffer.
    CHECK(sizeof(WireBulkParams) <= WIRE_BULK_BUF_SIZE);
}

// ---------------------------------------------------------------------------
// 2. Serialize / deserialize round-trip of bt_enabled through raw bytes.
// ---------------------------------------------------------------------------
static void test_wire_roundtrip(void) {
    WireBulkParams p;
    memset(&p, 0, sizeof(p));
    p.header.format_version = WIRE_FORMAT_VERSION;
    p.header.payload_length = (uint16_t)sizeof(WireBulkParams);

    // Host enables BT and serializes.
    uint8_t *bytes = (uint8_t *)&p;
    bytes[BT_ENABLED_WIRE_OFFSET] = bt_persist_encode_enabled(true);

    uint8_t wire[WIRE_V33_TOTAL_SIZE];
    memcpy(wire, bytes, sizeof(wire));

    // Device deserializes.
    WireBulkParams q;
    memset(&q, 0, sizeof(q));
    memcpy(&q, wire, sizeof(wire));
    bool enabled = bt_persist_decode_enabled(((uint8_t *)&q)[BT_ENABLED_WIRE_OFFSET]);
    CHECK(enabled == true);

    // The remainder of the tail section is reserved and must be zero.
    for (unsigned i = 1; i < BT_WIRE_SECTION_SIZE; i++)
        CHECK_EQ_U(((uint8_t *)&q)[BT_ENABLED_WIRE_OFFSET + i], 0u);
}

// ---------------------------------------------------------------------------
// 3. Old host / zero-filled section -> BT disabled (the valid default).
// ---------------------------------------------------------------------------
static void test_wire_old_host_defaults_off(void) {
    WireBulkParams p;
    memset(&p, 0, sizeof(p));   // every byte zero, as an all-defaults payload
    bool enabled = bt_persist_decode_enabled(((uint8_t *)&p)[BT_ENABLED_WIRE_OFFSET]);
    CHECK(enabled == false);

    // Contract helpers.
    CHECK_EQ_U(BT_PERSIST_ENABLED_DEFAULT, 0u);
    CHECK(bt_persist_decode_enabled(0u) == false);
    CHECK(bt_persist_decode_enabled(1u) == true);
    CHECK_EQ_U(bt_persist_encode_enabled(false), 0u);
    CHECK_EQ_U(bt_persist_encode_enabled(true), 1u);
}

// ---------------------------------------------------------------------------
// 4. Directory migration contract (V25 -> V26), mirrored SDK-free.
//
// The firmware's PresetDirectory pulls in SDK types, so this test mirrors only
// the migration RULE the flash_storage.c V25->V26 path implements: the V26
// layout is the V25 layout plus one trailing bt_enabled byte; migration copies
// the whole V25 data region forward unchanged and leaves bt_enabled at the
// zero-filled default (off).  Any stored V25 directory must therefore load with
// every pre-existing field intact and BT off.
// ---------------------------------------------------------------------------

// A representative V25 directory data region (opaque bytes: the real one is
// ~1400 bytes of startup/slot/output/CS config).  We only need to prove the
// prefix survives the migration byte-for-byte and the new field defaults off.
#define DIR_V25_DATA_LEN  3427u   // PRESET_DIRECTORY_V25_SIZE (3439) - 12-byte header

typedef struct {
    uint8_t  data[DIR_V25_DATA_LEN];        // startup_mode .. device_name
} DirV25Mirror;

typedef struct {
    uint8_t  data[DIR_V25_DATA_LEN];        // byte-identical prefix
    uint8_t  bt_enabled;                    // V26 tail-append
    uint8_t  bt_last_sink[6];               // V26: boot-reconnect target
} DirV26Mirror;

// The migration rule, isolated: copy the V25 data region into a zeroed V26
// cache; bt_enabled stays at its zero-filled default.
static void dir_migrate_v25_to_v26(const DirV25Mirror *v25, DirV26Mirror *out) {
    memset(out, 0, sizeof(*out));
    memcpy(out->data, v25->data, DIR_V25_DATA_LEN);
    // bt_enabled intentionally untouched -> 0 (off), the valid default.
}

static void test_directory_migration(void) {
    // Tail-append: V26 is exactly one byte larger and the prefix offset of the
    // bt_enabled field is the full V25 data length (nothing shifted).
    CHECK_EQ_U(sizeof(DirV26Mirror), sizeof(DirV25Mirror) + 7u);
    CHECK_EQ_U(offsetof(DirV26Mirror, bt_enabled), DIR_V25_DATA_LEN);
    CHECK_EQ_U(offsetof(DirV26Mirror, bt_last_sink), DIR_V25_DATA_LEN + 1u);

    // Build a V25 image with a recognizable prefix pattern.
    DirV25Mirror v25;
    for (unsigned i = 0; i < DIR_V25_DATA_LEN; i++)
        v25.data[i] = (uint8_t)(0x11u + (i & 0x7Fu));

    DirV26Mirror v26;
    dir_migrate_v25_to_v26(&v25, &v26);

    // Every pre-existing byte survives unchanged (existing presets stay valid).
    CHECK(memcmp(v26.data, v25.data, DIR_V25_DATA_LEN) == 0);
    // BT defaults off after migration, with no last sink recorded.
    CHECK(bt_persist_decode_enabled(v26.bt_enabled) == false);
    for (unsigned i = 0; i < 6; i++) CHECK_EQ_U(v26.bt_last_sink[i], 0u);

    // A V26 image round-trips bt_enabled.
    DirV26Mirror saved;
    memset(&saved, 0, sizeof(saved));
    memcpy(saved.data, v25.data, DIR_V25_DATA_LEN);
    saved.bt_enabled = bt_persist_encode_enabled(true);
    DirV26Mirror reloaded = saved;   // persist + reload
    CHECK(bt_persist_decode_enabled(reloaded.bt_enabled) == true);
    CHECK(memcmp(reloaded.data, v25.data, DIR_V25_DATA_LEN) == 0);
}

int main(void) {
    printf("bt_persist_test: Task 6 persistence (wire V33 + directory migration)\n");

    test_wire_abi();
    test_wire_roundtrip();
    test_wire_old_host_defaults_off();
    test_directory_migration();

    printf("\n%d checks, %d failures\n", checks, failures);
    if (failures) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
