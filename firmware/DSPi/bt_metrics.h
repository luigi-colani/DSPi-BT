/*
 * bt_metrics.h — instrumentation counters for the Bluetooth (A2DP/SBC source)
 * output path (Pico 2 W, DSPI_BT only).
 *
 * The on-hardware Core-0 budget measurement (spec §8) reads these to confirm
 * that adding BT streaming on top of the full input pipeline does not starve
 * the audio callback or the FIFO, and that SBC encoding stays within budget.
 *
 * Fields:
 *   audio_callback_overruns — output-path audio-callback overruns (the shared
 *       `spdif_overruns` counter: the audio/USB callback could not obtain an
 *       output buffer because the pool was full, i.e. the system fell behind).
 *       The pass criterion is zero of these at steady state.
 *   fifo_overflow_frames    — stereo frames the pipeline producer had to drop
 *       because the BT FIFO was full (the A2DP consumer fell behind).
 *   fifo_underflow_frames   — stereo frames the A2DP media timer needed but the
 *       FIFO could not supply (the producer fell behind / stream outran input).
 *   sbc_encode_us_last      — duration of the most recent SBC encode pass (µs).
 *   sbc_encode_us_max       — largest SBC encode duration observed since boot
 *       (µs); the headroom indicator for the Core-0 budget.
 */

#ifndef BT_METRICS_H
#define BT_METRICS_H

#include <stdint.h>

typedef struct {
    uint32_t audio_callback_overruns;
    uint32_t fifo_overflow_frames;
    uint32_t fifo_underflow_frames;
    uint32_t sbc_encode_us_last;
    uint32_t sbc_encode_us_max;
} BtMetrics;

#if DSPI_BT
// Snapshot the current metrics.  Safe to call from Core 0 (the main loop or the
// BTstack run loop); the counters are plain monotonic words.
BtMetrics bt_output_metrics(void);
#endif

#endif // BT_METRICS_H
