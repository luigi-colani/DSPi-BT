#include <stdio.h>
#include "bt_rate.h"
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
#define BOTH  (BT_RATE_CAP_44100 | BT_RATE_CAP_48000)
#define ONLY44 BT_RATE_CAP_44100
#define ONLY48 BT_RATE_CAP_48000
int main(void) {
    // BT inactive: unchanged, 96k allowed
    CHECK(bt_rate_clamp(96000, false, 44100, BOTH) == 96000);
    CHECK(bt_rate_clamp(44100, false, 48000, ONLY48) == 44100);
    CHECK(bt_rate_clamp(48000, false, 0, 0) == 48000);
    CHECK(bt_rate_allowed(96000, false, 44100, ONLY44));
    CHECK(bt_rate_allowed(44100, false, 48000, ONLY48));

    // Capability predicate
    CHECK(bt_rate_sink_supports(44100, BOTH));
    CHECK(bt_rate_sink_supports(48000, BOTH));
    CHECK(!bt_rate_sink_supports(96000, BOTH));
    CHECK(!bt_rate_sink_supports(32000, 0xFF));
    CHECK(!bt_rate_sink_supports(48000, ONLY44));
    CHECK(!bt_rate_sink_supports(44100, ONLY48));
    CHECK(!bt_rate_sink_supports(44100, 0));

    // Dual-rate sink: the link follows the host / input rate
    CHECK(bt_rate_allowed(48000, true, 44100, BOTH));      // host 48k vs link 44.1k: OK
    CHECK(bt_rate_allowed(44100, true, 48000, BOTH));
    CHECK(!bt_rate_allowed(96000, true, 48000, BOTH));     // SBC can't carry 96k
    CHECK(bt_rate_clamp(48000, true, 44100, BOTH) == 48000);
    CHECK(bt_rate_clamp(44100, true, 48000, BOTH) == 44100);
    CHECK(bt_rate_clamp(96000, true, 44100, BOTH) == 44100);   // pinned to link rate

    // Single-rate sink: old pin-to-sink behaviour
    CHECK(bt_rate_allowed(44100, true, 44100, ONLY44));
    CHECK(!bt_rate_allowed(48000, true, 44100, ONLY44));   // would pitch-shift: refuse
    CHECK(!bt_rate_allowed(44100, true, 48000, ONLY48));
    CHECK(bt_rate_clamp(48000, true, 44100, ONLY44) == 44100);
    CHECK(bt_rate_clamp(44100, true, 48000, ONLY48) == 48000);

    // Unknown caps (0): only the negotiated rate counts
    CHECK(bt_rate_allowed(44100, true, 44100, 0));
    CHECK(!bt_rate_allowed(48000, true, 44100, 0));
    CHECK(bt_rate_allowed(48000, true, 0, 0));             // unknown sink -> 48k
    CHECK(!bt_rate_allowed(44100, true, 0, 0));
    CHECK(bt_rate_clamp(44100, true, 48000, 0) == 48000);
    CHECK(bt_rate_clamp(96000, true, 0, 0) == 48000);      // never 96k
    CHECK(bt_rate_clamp(96000, true, 96000, 0) == 48000);

    // Preferred rate offered at connect
    CHECK(bt_rate_preferred(44100) == 44100);
    CHECK(bt_rate_preferred(48000) == 48000);
    CHECK(bt_rate_preferred(96000) == 48000);
    CHECK(bt_rate_preferred(0) == 48000);

    printf(fails ? "FAILED\n" : "bt_rate_test: all passed\n");
    return fails != 0;
}
