/*
 * bt_fifo.h — SPSC lock-free ring of stereo float frames for the Bluetooth
 * (A2DP/SBC source) output path (Pico 2 W, DSPI_BT only).
 *
 * Producer: the audio pipeline (audio_pipeline.c, process_input_block) on
 *   Core 0, via bt_fifo_push() — the processed main pair (outputs 0-1).
 * Consumer: the BTstack A2DP media timer (bt_output.c) on Core 0, via
 *   bt_fifo_pop() — pops stereo frames and converts float -> interleaved int16
 *   for the SBC encoder.
 *
 * Single-producer / single-consumer, lock-free.  Mirrors the discipline of
 * usb_audio_ring.h: fixed power-of-two slot count, volatile head/tail written
 * by exactly one side each, __dmb() release/acquire barriers around index
 * publication (required on the Cortex-M33 write buffer; harmless elsewhere).
 *
 * Both producer and consumer run on Core 0, so this never crosses cores; the
 * barriers document intent and keep the compiler from reordering the slot
 * writes past the index update.  The ring never blocks: a full ring drops the
 * overflowing frames (overflow_frames++), an empty ring returns short
 * (underflow_frames += shortfall).  The real-time audio callback must never
 * stall, so the producer drops rather than waits.
 *
 * Host-testable: the core ring primitives (bt_fifo_init / bt_fifo_push_n /
 * bt_fifo_pop_n / bt_fifo_available / bt_fifo_free) are header-only and take a
 * bt_fifo_t*, so tools/bt_fifo_test/ exercises them on a plain host compile.
 * The global-instance API (bt_fifo_push / bt_fifo_pop / ...) is declared here
 * and defined in bt_output.c, which owns the one shared instance.
 */

#ifndef BT_FIFO_H
#define BT_FIFO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// On the RP2350 target pull the real __dmb(); on a host build (the FIFO unit
// test) shim it out.  __arm__ is set for the Armv8-M target (AArch32) and not
// for an x86-64 or AArch64 host, so this keys cleanly off the build arch.
#if defined(PICO_ON_DEVICE) || defined(__arm__)
#include "hardware/sync.h"   // __dmb()
#else
#ifndef __dmb
#define __dmb() ((void)0)
#endif
#endif

// ---------------------------------------------------------------------------
// Ring geometry
// ---------------------------------------------------------------------------
// Power of two so head/tail wrap with a mask.  4096 stereo frames is ~85 ms of
// decoupling headroom at 48 kHz / ~93 ms at 44.1 kHz — deep enough to absorb
// the 10 ms A2DP media-timer bursts against the ~1 ms pipeline push cadence,
// at 32 KiB of RAM (4096 * 2ch * 4B).
#define BT_FIFO_FRAMES   4096u
#define BT_FIFO_MASK     (BT_FIFO_FRAMES - 1u)

typedef struct {
    float l;
    float r;
} bt_fifo_frame_t;

typedef struct {
    // 4-byte aligned per the usb_audio_ring discipline (word-aligned float
    // pair access; keeps the compiler free to emit paired loads/stores).
    bt_fifo_frame_t slots[BT_FIFO_FRAMES] __attribute__((aligned(4)));
    volatile uint32_t head;              // producer writes only
    volatile uint32_t tail;              // consumer writes only
    volatile uint32_t overflow_frames;   // frames dropped on a full ring
    volatile uint32_t underflow_frames;  // frames the consumer asked for but
                                         // the ring could not supply
    // Clock-drift state — written by the consumer only (bt_fifo_pop_drift).
    volatile uint32_t drift_skipped_frames;     // frames discarded (fill high)
    volatile uint32_t drift_duplicated_frames;  // frames repeated (fill low)
    volatile uint32_t drift_fill;               // fill seen at the last pop
    volatile uint8_t  drift_mode;               // BT_FIFO_DRIFT_{IDLE,DUP,SKIP}
} bt_fifo_t;

// ---------------------------------------------------------------------------
// Clock-drift policy (consumer side).  The producer (DSPi sample clock) and
// the consumer (BTstack media timer / sink clock) run at slightly different
// rates, so fill slowly walks toward empty or full.  The consumer keeps it
// bounded by repeating (fill low) or discarding (fill high) ONE whole stereo
// frame per pop block — at the PCM frame boundary, before SBC encoding, so
// the SBC stream is never touched.  Max correction is 1 frame per block
// (~7800 ppm for a 128-frame block), far above crystal-class drift.
//
// Hysteresis: a correction mode engages past LOW/HIGH and only releases once
// fill is back inside the LOW_EXIT/HIGH_EXIT band, so it cannot thrash near a
// threshold.  The band sits around TARGET (~21 ms @ 48 kHz); the media timer
// should prefill to TARGET before the first pop.
// ---------------------------------------------------------------------------
#define BT_FIFO_DRIFT_TARGET     1024u
#define BT_FIFO_DRIFT_LOW         512u   // enter DUP below this
#define BT_FIFO_DRIFT_LOW_EXIT    768u   // leave DUP at/above this
#define BT_FIFO_DRIFT_HIGH       2048u   // enter SKIP above this
#define BT_FIFO_DRIFT_HIGH_EXIT  1536u   // leave SKIP at/below this

enum { BT_FIFO_DRIFT_IDLE = 0, BT_FIFO_DRIFT_DUP = 1, BT_FIFO_DRIFT_SKIP = 2 };

// ---------------------------------------------------------------------------
// Core ring primitives (header-only, host-testable)
// ---------------------------------------------------------------------------

static inline void bt_fifo_init(bt_fifo_t *f) {
    f->head = 0;
    f->tail = 0;
    f->overflow_frames = 0;
    f->underflow_frames = 0;
    f->drift_skipped_frames = 0;
    f->drift_duplicated_frames = 0;
    f->drift_fill = 0;
    f->drift_mode = BT_FIFO_DRIFT_IDLE;
}

// Reset drift state only (ring and counters untouched).
static inline void bt_fifo_drift_init(bt_fifo_t *f) {
    f->drift_mode = BT_FIFO_DRIFT_IDLE;
}

// Frames currently available to pop.
static inline uint32_t bt_fifo_available(const bt_fifo_t *f) {
    return (f->head - f->tail) & BT_FIFO_MASK;
}

// Frames that can still be pushed (one slot is reserved to tell full from
// empty, so the usable capacity is BT_FIFO_FRAMES - 1).
static inline uint32_t bt_fifo_free(const bt_fifo_t *f) {
    return (BT_FIFO_FRAMES - 1u) - bt_fifo_available(f);
}

// Flush all pending frames (consumer side; used on stream start/stop).
static inline void bt_fifo_reset_ring(bt_fifo_t *f) {
    __dmb();
    f->tail = f->head;
}

// Full-scale float [-1,1] -> clamped signed 16-bit.  Values past the rails
// saturate rather than wrap; in-range values truncate toward zero (as the C
// float->int cast does), matching the SBC encoder's int16 input domain.
static inline int16_t bt_fifo_f32_to_s16(float v) {
    float s = v * 32767.0f;
    if (s >= 32767.0f)  return (int16_t)32767;
    if (s <= -32768.0f) return (int16_t)-32768;
    return (int16_t)s;
}

// Producer: push up to n stereo frames from separate L/R float planes.
// Returns the number of frames accepted; any shortfall is counted in
// overflow_frames and those frames are dropped (never blocks).
static inline uint32_t bt_fifo_push_n(bt_fifo_t *f,
                                      const float *l, const float *r,
                                      uint32_t n) {
    uint32_t h = f->head;
    const uint32_t t = f->tail;                      // snapshot consumer index
    uint32_t avail = (h - t) & BT_FIFO_MASK;
    uint32_t space = (BT_FIFO_FRAMES - 1u) - avail;  // leave one slot free
    uint32_t count = (n <= space) ? n : space;

    for (uint32_t i = 0; i < count; i++) {
        f->slots[h].l = l[i];
        f->slots[h].r = r[i];
        h = (h + 1) & BT_FIFO_MASK;
    }
    // Release barrier: slot writes visible before the head advance publishes
    // them to the consumer.
    __dmb();
    f->head = h;

    if (count < n) {
        f->overflow_frames += (n - count);
    }
    return count;
}

// Consumer: pop up to n stereo frames, converting each float sample to a
// clamped interleaved int16 (L,R,L,R,...).  Returns the number of frames
// popped; any shortfall is counted in underflow_frames (the caller fills the
// remainder with silence).
static inline uint32_t bt_fifo_pop_n(bt_fifo_t *f,
                                     int16_t *interleaved,
                                     uint32_t n) {
    const uint32_t h = f->head;                      // snapshot producer index
    uint32_t t = f->tail;
    uint32_t avail = (h - t) & BT_FIFO_MASK;
    uint32_t count = (n <= avail) ? n : avail;

    // Acquire barrier: pair with the producer's release so slot data written
    // before head advanced is observed here.
    __dmb();
    for (uint32_t i = 0; i < count; i++) {
        interleaved[i * 2]     = bt_fifo_f32_to_s16(f->slots[t].l);
        interleaved[i * 2 + 1] = bt_fifo_f32_to_s16(f->slots[t].r);
        t = (t + 1) & BT_FIFO_MASK;
    }
    // Release barrier: slot reads complete before tail frees them.
    __dmb();
    f->tail = t;

    if (count < n) {
        f->underflow_frames += (n - count);
    }
    return count;
}

// Consumer: like bt_fifo_pop_n, but applies the clock-drift policy above.
// Always delivers the same number of output frames as a plain pop would
// (n, unless the ring genuinely underflows), so the caller's framing is
// unchanged.  Non-blocking, O(n), consumer-context only.
//   DUP : pops n-1 frames and repeats the last one to fill the block.
//   SKIP: discards the oldest pending frame, then pops n.
static inline uint32_t bt_fifo_pop_drift(bt_fifo_t *f,
                                         int16_t *interleaved,
                                         uint32_t n) {
    uint32_t avail = bt_fifo_available(f);
    f->drift_fill = avail;

    uint8_t mode = f->drift_mode;
    if (mode == BT_FIFO_DRIFT_DUP) {
        if (avail >= BT_FIFO_DRIFT_LOW_EXIT) mode = BT_FIFO_DRIFT_IDLE;
    } else if (mode == BT_FIFO_DRIFT_SKIP) {
        if (avail <= BT_FIFO_DRIFT_HIGH_EXIT) mode = BT_FIFO_DRIFT_IDLE;
    }
    if (mode == BT_FIFO_DRIFT_IDLE) {
        if (avail < BT_FIFO_DRIFT_LOW)       mode = BT_FIFO_DRIFT_DUP;
        else if (avail > BT_FIFO_DRIFT_HIGH) mode = BT_FIFO_DRIFT_SKIP;
    }
    f->drift_mode = mode;

    if (mode == BT_FIFO_DRIFT_SKIP && avail > n) {
        // Drop the oldest frame unread (tail is consumer-owned).
        __dmb();
        f->tail = (f->tail + 1) & BT_FIFO_MASK;
        f->drift_skipped_frames++;
    } else if (mode == BT_FIFO_DRIFT_DUP && n >= 2 && avail >= 1) {
        uint32_t got = bt_fifo_pop_n(f, interleaved, n - 1);
        if (got == n - 1) {
            interleaved[(n - 1) * 2]     = interleaved[(n - 2) * 2];
            interleaved[(n - 1) * 2 + 1] = interleaved[(n - 2) * 2 + 1];
            f->drift_duplicated_frames++;
            return n;
        }
        return got;   // genuine underflow; caller pads silence
    }
    return bt_fifo_pop_n(f, interleaved, n);
}

// ---------------------------------------------------------------------------
// Global-instance API (defined in bt_output.c; see file header)
// ---------------------------------------------------------------------------
#if DSPI_BT
void     bt_fifo_push(const float *l, const float *r, uint32_t n);
uint32_t bt_fifo_pop(int16_t *interleaved, uint32_t n_frames);  // applies drift policy
void     bt_fifo_reset(void);
#endif

#endif // BT_FIFO_H
