#ifndef BT_RATE_H
#define BT_RATE_H
// SDK-free Bluetooth-active sample-rate policy (host-testable).
// A2DP/SBC carries only 44.1 / 48 kHz and DSPi does not resample.  While the BT
// output is streaming, the Bluetooth link FOLLOWS the pipeline rate whenever
// the sink supports it (bt_output.c reconfigures the SBC stream), so the host
// and every input keep their own 44.1 / 48 kHz rate.  Only a rate the sink
// cannot take (96 kHz always; 44.1 or 48 kHz on a single-rate sink) falls back
// to the old behaviour: the pipeline is pinned to the sink's negotiated rate
// and USB refuses it.  When BT is not active the requested rate passes through.
#include <stdbool.h>
#include <stdint.h>

#define BT_RATE_SBC_DEFAULT 48000u

// Sink SBC sampling-frequency capability bits, as AVDTP reports them
// (avdtp.h: AVDTP_SBC_48000 = 1, AVDTP_SBC_44100 = 2).
#define BT_RATE_CAP_48000   0x01u
#define BT_RATE_CAP_44100   0x02u

// Sink rate sanitised to a valid SBC rate (unknown -> 48 kHz).
static inline uint32_t bt_rate_sink_or_default(uint32_t sink_rate) {
    return (sink_rate == 44100u || sink_rate == 48000u) ? sink_rate : BT_RATE_SBC_DEFAULT;
}

// Can the sink be (re)configured to `rate`?  `sink_caps` = BT_RATE_CAP_* bits;
// 0 = unknown (then only the negotiated rate counts, via the callers).
static inline bool bt_rate_sink_supports(uint32_t rate, uint8_t sink_caps) {
    return (rate == 48000u && (sink_caps & BT_RATE_CAP_48000)) ||
           (rate == 44100u && (sink_caps & BT_RATE_CAP_44100));
}

// Is `requested` acceptable as a USB host-selected rate?  While BT is active:
// the sink's current rate, or any rate the link can be reconfigured to.  Any
// other rate (incl. 96 kHz) would either be uncarryable or silently pitch-shift
// host content, so the host is stalled and falls back (its mixer resamples).
// Note: if BT starts while the host already streams at an unsupported rate
// (e.g. 96k), the pipeline pins to the sink rate and the host keeps sending
// until its next SET_CUR; the stall only blocks new requests.
static inline bool bt_rate_allowed(uint32_t requested, bool bt_active,
                                   uint32_t sink_rate, uint8_t sink_caps) {
    return !bt_active || requested == bt_rate_sink_or_default(sink_rate) ||
           bt_rate_sink_supports(requested, sink_caps);
}

// Rate the pipeline may actually run at.  Non-USB inputs at an unsupported
// rate are pinned to the sink rate (unavoidable pitch shift without a
// resampler).
static inline uint32_t bt_rate_clamp(uint32_t requested, bool bt_active,
                                     uint32_t sink_rate, uint8_t sink_caps) {
    if (!bt_active) return requested;
    if (bt_rate_sink_supports(requested, sink_caps)) return requested;
    return bt_rate_sink_or_default(sink_rate);
}

// SBC rate to prefer when a stream is (re)negotiated: the pipeline's own rate
// when SBC can carry it (no reconfigure needed right after connect), else 48 kHz.
static inline uint32_t bt_rate_preferred(uint32_t pipeline_rate) {
    return (pipeline_rate == 44100u || pipeline_rate == 48000u) ? pipeline_rate
                                                                : BT_RATE_SBC_DEFAULT;
}
#endif
