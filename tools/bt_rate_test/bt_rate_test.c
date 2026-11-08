#include <stdio.h>
#include "bt_rate.h"
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
int main(void) {
    // BT inactive: unchanged, 96k allowed
    CHECK(bt_rate_clamp(96000, false, 44100) == 96000);
    CHECK(bt_rate_clamp(44100, false, 48000) == 44100);
    CHECK(bt_rate_clamp(48000, false, 0) == 48000);
    // USB SET_CUR predicate (sink-aware)
    CHECK(bt_rate_allowed(96000, false, 44100));
    CHECK(bt_rate_allowed(44100, false, 48000));
    CHECK(!bt_rate_allowed(96000, true, 48000));
    CHECK(!bt_rate_allowed(96000, true, 44100));
    CHECK(bt_rate_allowed(48000, true, 48000));
    CHECK(bt_rate_allowed(44100, true, 44100));
    CHECK(!bt_rate_allowed(44100, true, 48000));   // would pitch-shift: refuse
    CHECK(!bt_rate_allowed(48000, true, 44100));
    CHECK(bt_rate_allowed(48000, true, 0));        // unknown sink -> 48k
    CHECK(!bt_rate_allowed(44100, true, 0));
    // Pipeline pin (SPDIF/I2S/ADAT inputs): clamp to sink rate
    CHECK(bt_rate_clamp(96000, true, 48000) == 48000);
    CHECK(bt_rate_clamp(96000, true, 44100) == 44100);
    // sink-rate tracking
    CHECK(bt_rate_clamp(48000, true, 44100) == 44100);
    CHECK(bt_rate_clamp(44100, true, 48000) == 48000);
    CHECK(bt_rate_clamp(44100, true, 44100) == 44100);
    // unknown sink rate -> 48k default, never 96k
    CHECK(bt_rate_clamp(96000, true, 0) == 48000);
    CHECK(bt_rate_clamp(96000, true, 96000) == 48000);
    printf(fails ? "FAILED\n" : "bt_rate_test: all passed\n");
    return fails != 0;
}
