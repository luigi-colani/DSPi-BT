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

// Is `requested` acceptable as a USB host-selected rate?  While BT is active
// only the sink's SBC rate is: any other rate (incl. 96 kHz) would either be
// uncarryable or silently pitch-shift host content, so the host is stalled and
// falls back to the sink rate (its mixer resamples).  Non-USB inputs are still
// pinned via bt_rate_clamp (unavoidable pitch shift without a resampler).
// Note: if BT starts while the host already streams at another rate (e.g.
// 96k), the pipeline pins to the sink rate and the host keeps sending until its
// next SET_CUR; the stall only blocks new requests.
static inline bool bt_rate_allowed(uint32_t requested, bool bt_active, uint32_t sink_rate) {
    return !bt_active || requested == bt_rate_sink_or_default(sink_rate);
}

// Rate the pipeline may actually run at.
static inline uint32_t bt_rate_clamp(uint32_t requested, bool bt_active, uint32_t sink_rate) {
    if (!bt_active) return requested;
    return bt_rate_sink_or_default(sink_rate);
}
#endif
