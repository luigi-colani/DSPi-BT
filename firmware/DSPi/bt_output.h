#ifndef BT_OUTPUT_H
#define BT_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>
#include "bt_proto.h"
#include "bt_rate.h"

#if DSPI_BT

// Initialise the Bluetooth audio output (Pico 2 W builds with DSPI_BT only).
// Must be called once from Core 0 at boot.  Fast and non-blocking: the slow
// radio/BTstack bring-up is deferred to bt_output_poll().
void bt_output_init(void);

// Service the deferred Bluetooth bring-up.  Call once per main-loop iteration
// on Core 0; it performs the (blocking ~1 s) cyw43 + BTstack init exactly once,
// after USB has enumerated, then returns immediately on every later call.
void bt_output_poll(void);

// True while the Bluetooth output is enabled (A2DP stream playing).  The audio
// pipeline pushes the processed mains into bt_fifo only while this is true.
bool bt_output_is_enabled(void);

// The persisted user-enable intent (radio on/off), independent of whether a
// stream is actually playing.  Read by bulk_params_collect() so a host GET
// round-trips the stored BT-enabled state.  Seeded at boot from the directory
// (preset_get_bt_enabled); default off.
bool bt_output_user_enabled(void);

// Poll for a deferred "persist the BT-enabled state" request.  Returns true and
// writes the value to *on when the enable state changed and must be written to
// the device-global directory; the caller (main loop) performs the flash write
// inside its flash-write bracket.  Returns false when nothing is pending.
bool bt_output_consume_persist_request(bool *on);

// Drive the Pico 2 W onboard LED (wired to the CYW43 radio).  Safe before the
// radio is up (no-op until cyw43_arch_init() has run).  Used for the boot
// heartbeat since GPIO 25 is the radio chip-select on a 2 W.
void bt_output_led(bool on);

// True once cyw43_arch_init() has succeeded and the radio owns its claimed PIO
// state machine + DMA channels + GPIO 23/24/25/29.  While true, the vendor
// handlers refuse to re-enable the wired outputs that were stood down to free
// those resources (the PDM sub, the ADAT bulk output).
bool bt_radio_resources_held(void);

// ---- Vendor-command control surface (Task 5) -------------------------------
// Called from the EP0 / external-transport context: each ONLY records the
// request and returns (a *_pending flag); bt_output_poll() performs the
// BTstack/radio work from the main loop.  Last request wins for each kind.
void bt_request_enable(bool on);
void bt_request_scan(bool start);
void bt_request_connect(const uint8_t addr[6]);
void bt_request_disconnect(void);
void bt_request_forget(const uint8_t addr[6]);
// Snapshot readers — plain RAM reads, no stack calls (safe from EP0).
void bt_get_status(BtStatus *out);
bool bt_get_scan_result(uint8_t index, BtScanResult *out);

// Resource mutual-exclusion (implemented in vendor_commands.c, where the output
// matrix + PDM/ADAT teardown machinery live).  Stand down (on=true) or restore
// (on=false) the PIO1-resident wired outputs that contend with the radio for a
// PIO state machine + DMA, as a logical reservation ahead of cyw43_arch_init().
// Reports the set it changed via *stood_down_mask (BT_STOODDOWN_* bits from
// bt_exclusion.h); returns true iff anything changed.  Must run on Core 0 inside
// a prepare/complete_pipeline_reset() bracket (see bt_output_poll) so the
// still-active S/PDIF mains stay click-free.  NB renamed from bt_output_enable
// (Task 3 review) to avoid confusion with bt_output_is_enabled() — Task 5's
// REQ_BT_* enable/disable caller must use THIS name.
bool bt_radio_standdown(bool on, uint8_t *stood_down_mask);

// Pipeline-reset bracket wrappers (implemented in main.c, where the static
// prepare_pipeline_reset/complete_pipeline_reset + pipeline_reset_ready live).
// bt_output_poll() brackets the stand-down/restore with these so the teardown
// is click-free and the ADAT stream is torn down synchronously (complete ->
// adat_output_resync) before the blocking radio init.
bool bt_pipeline_reset_ready(void);
void bt_pipeline_reset_prepare(void);
void bt_pipeline_reset_complete(void);

// Sink's negotiated SBC sampling rate in Hz (44100 / 48000), or the preferred
// default before a codec is configured.  Plain RAM read, any context.
uint32_t bt_output_sink_rate(void);

#endif // DSPI_BT

// ---- Rate policy (Task 7) --------------------------------------------------
// While BT streams, the pipeline follows the sink's SBC rate and 96 kHz is
// refused.  Both helpers are identity / always-true on non-BT builds, so wired
// rate behaviour is unchanged there.
static inline uint32_t bt_output_apply_rate_policy(uint32_t requested) {
#if DSPI_BT
    return bt_rate_clamp(requested, bt_output_is_enabled(), bt_output_sink_rate());
#else
    return requested;
#endif
}
static inline bool bt_output_rate_allowed(uint32_t requested) {
#if DSPI_BT
    return bt_rate_allowed(requested, bt_output_is_enabled());
#else
    (void)requested;
    return true;
#endif
}

#endif // BT_OUTPUT_H
