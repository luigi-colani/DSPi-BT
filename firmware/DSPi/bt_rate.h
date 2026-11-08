#ifndef BT_RATE_H
#define BT_RATE_H
// SDK-free Bluetooth-active sample-rate policy (host-testable).
// A2DP/SBC carries only 44.1 / 48 kHz and DSPi does not resample, so while the
// BT output is streaming the pipeline rate is pinned to the sink's negotiated
// SBC rate and 96 kHz is refused from every source.  When BT is not active the
// requested rate passes through unchanged.
#include <stdbool.h>
#include <stdint.h>

#define BT_RATE_SBC_DEFAULT 48000u

// Sink rate sanitised to a valid SBC rate (unknown -> 48 kHz).
static inline uint32_t bt_rate_sink_or_default(uint32_t sink_rate) {
    return (sink_rate == 44100u || sink_rate == 48000u) ? sink_rate : BT_RATE_SBC_DEFAULT;
}

// Is `requested` acceptable as a host/source-selected rate?  False only for
// rates SBC cannot carry (anything above 48 kHz) while BT is active.
static inline bool bt_rate_allowed(uint32_t requested, bool bt_active) {
    return !bt_active || requested <= 48000u;
}

// Rate the pipeline may actually run at.
static inline uint32_t bt_rate_clamp(uint32_t requested, bool bt_active, uint32_t sink_rate) {
    if (!bt_active) return requested;
    return bt_rate_sink_or_default(sink_rate);
}
#endif
