#include "bt_output.h"

#if DSPI_BT

void bt_output_init(void) {
    // Task 2 will call cyw43_arch_init() here.  It must run on Core 0, the
    // core that services the radio (BTstack + CYW43 both live on Core 0).
}

bool bt_output_is_enabled(void) {
    return false;
}

#endif // DSPI_BT
