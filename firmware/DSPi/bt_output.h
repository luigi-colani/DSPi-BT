#ifndef BT_OUTPUT_H
#define BT_OUTPUT_H

#include <stdbool.h>

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

#endif // DSPI_BT

#endif // BT_OUTPUT_H
