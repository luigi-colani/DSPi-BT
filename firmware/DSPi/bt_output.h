#ifndef BT_OUTPUT_H
#define BT_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

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

// Drive the Pico 2 W onboard LED (wired to the CYW43 radio).  Safe before the
// radio is up (no-op until cyw43_arch_init() has run).  Used for the boot
// heartbeat since GPIO 25 is the radio chip-select on a 2 W.
void bt_output_led(bool on);

// True once cyw43_arch_init() has succeeded and the radio owns its claimed PIO
// state machine + DMA channels + GPIO 23/24/25/29.  While true, the vendor
// handlers refuse to re-enable the wired outputs that were stood down to free
// those resources (the PDM sub, the ADAT bulk output).
bool bt_radio_resources_held(void);

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

#endif // DSPI_BT

#endif // BT_OUTPUT_H
