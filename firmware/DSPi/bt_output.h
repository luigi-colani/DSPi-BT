#ifndef BT_OUTPUT_H
#define BT_OUTPUT_H

#include <stdbool.h>

#if DSPI_BT

// Initialise the Bluetooth audio output (Pico 2 W builds with DSPI_BT only).
// Must be called once from Core 0 at boot.
void bt_output_init(void);

// True while the Bluetooth output is enabled.
bool bt_output_is_enabled(void);

#endif // DSPI_BT

#endif // BT_OUTPUT_H
