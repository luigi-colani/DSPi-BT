/*
 * Host unit test for firmware/DSPi/bt_fifo.h
 *
 * Exercises the SPSC ring primitives (bt_fifo_push_n / bt_fifo_pop_n /
 * bt_fifo_available / bt_fifo_free) that back the Bluetooth output FIFO:
 * fill, float->int16 conversion + clamping, wrap-around across the ring
 * boundary, overflow (push into a full ring) and underflow (pop from an
 * empty ring) accounting.
 *
 * Build + run:  make        (in this directory)
 * Exit status 0 = all assertions passed.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "bt_fifo.h"

static int failures = 0;
static int checks   = 0;

#define CHECK(cond) do {                                                  \
        checks++;                                                         \
        if (!(cond)) {                                                    \
            failures++;                                                   \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
        }                                                                 \
    } while (0)

#define CHECK_EQ_U(a, b) do {                                             \
        checks++;                                                         \
        uint32_t _a = (uint32_t)(a), _b = (uint32_t)(b);                  \
        if (_a != _b) {                                                   \
            failures++;                                                   \
            printf("  FAIL  %s:%d  %s == %s  (%u != %u)\n",               \
                   __FILE__, __LINE__, #a, #b, _a, _b);                   \
        }                                                                 \
    } while (0)

#define CHECK_EQ_I(a, b) do {                                             \
        checks++;                                                         \
        int _a = (int)(a), _b = (int)(b);                                 \
        if (_a != _b) {                                                   \
            failures++;                                                   \
            printf("  FAIL  %s:%d  %s == %s  (%d != %d)\n",               \
                   __FILE__, __LINE__, #a, #b, _a, _b);                   \
        }                                                                 \
    } while (0)

// A single ring is large (32 KiB); keep one static instance for all tests.
static bt_fifo_t fifo;

// ---------------------------------------------------------------------------

static void test_init_empty(void) {
    printf("test_init_empty\n");
    bt_fifo_init(&fifo);
    CHECK_EQ_U(bt_fifo_available(&fifo), 0);
    CHECK_EQ_U(bt_fifo_free(&fifo), BT_FIFO_FRAMES - 1u);
    CHECK_EQ_U(fifo.overflow_frames, 0);
    CHECK_EQ_U(fifo.underflow_frames, 0);

    // Popping an empty ring yields nothing and counts the full shortfall.
    int16_t out[8];
    uint32_t got = bt_fifo_pop_n(&fifo, out, 4);
    CHECK_EQ_U(got, 0);
    CHECK_EQ_U(fifo.underflow_frames, 4);
}

static void test_push_pop_values(void) {
    printf("test_push_pop_values\n");
    bt_fifo_init(&fifo);

    // Known L/R planes, including values that must clamp at the int16 rails.
    //   0.0   -> 0
    //   0.5   -> 16383 (0.5 * 32767 = 16383.5, truncates toward zero)
    //   1.0   -> 32767
    //   2.0   -> 32767 (clamped)
    //  -1.0   -> -32767
    //  -2.0   -> -32768 (clamped to the negative rail)
    const float l[6] = { 0.0f, 0.5f, 1.0f,  2.0f, -1.0f, -2.0f };
    const float r[6] = { 0.0f, -0.5f, -1.0f, -2.0f, 1.0f,  2.0f };

    uint32_t pushed = bt_fifo_push_n(&fifo, l, r, 6);
    CHECK_EQ_U(pushed, 6);
    CHECK_EQ_U(bt_fifo_available(&fifo), 6);
    CHECK_EQ_U(bt_fifo_free(&fifo), BT_FIFO_FRAMES - 1u - 6u);

    int16_t out[12];
    memset(out, 0x7F, sizeof(out));
    uint32_t got = bt_fifo_pop_n(&fifo, out, 6);
    CHECK_EQ_U(got, 6);
    CHECK_EQ_U(bt_fifo_available(&fifo), 0);
    CHECK_EQ_U(fifo.underflow_frames, 0);

    // Interleaved L,R,L,R,...
    CHECK_EQ_I(out[0],  0);       CHECK_EQ_I(out[1],  0);
    CHECK_EQ_I(out[2],  16383);   CHECK_EQ_I(out[3],  -16383);
    CHECK_EQ_I(out[4],  32767);   CHECK_EQ_I(out[5],  -32767);
    CHECK_EQ_I(out[6],  32767);   CHECK_EQ_I(out[7],  -32768);  // +2 clamp, -2 clamp
    CHECK_EQ_I(out[8],  -32767);  CHECK_EQ_I(out[9],  32767);
    CHECK_EQ_I(out[10], -32768);  CHECK_EQ_I(out[11], 32767);
}

static void test_partial_pop(void) {
    printf("test_partial_pop\n");
    bt_fifo_init(&fifo);

    float l[10], r[10];
    for (int i = 0; i < 10; i++) { l[i] = 0.0f; r[i] = 0.0f; }
    CHECK_EQ_U(bt_fifo_push_n(&fifo, l, r, 10), 10);

    // Ask for more than available: get what's there, count the shortfall.
    int16_t out[32];
    uint32_t got = bt_fifo_pop_n(&fifo, out, 16);
    CHECK_EQ_U(got, 10);
    CHECK_EQ_U(bt_fifo_available(&fifo), 0);
    CHECK_EQ_U(fifo.underflow_frames, 6);  // 16 asked - 10 supplied
}

static void test_overflow(void) {
    printf("test_overflow\n");
    bt_fifo_init(&fifo);

    uint32_t cap = bt_fifo_free(&fifo);      // usable capacity
    CHECK_EQ_U(cap, BT_FIFO_FRAMES - 1u);

    // Fill to the brim.
    static float l[BT_FIFO_FRAMES];
    static float r[BT_FIFO_FRAMES];
    for (uint32_t i = 0; i < BT_FIFO_FRAMES; i++) { l[i] = 0.1f; r[i] = -0.1f; }

    uint32_t pushed = bt_fifo_push_n(&fifo, l, r, cap);
    CHECK_EQ_U(pushed, cap);
    CHECK_EQ_U(bt_fifo_free(&fifo), 0);
    CHECK_EQ_U(bt_fifo_available(&fifo), cap);
    CHECK_EQ_U(fifo.overflow_frames, 0);

    // One more full block: nothing accepted, the whole lot counts as overflow.
    uint32_t extra = 100;
    uint32_t more = bt_fifo_push_n(&fifo, l, r, extra);
    CHECK_EQ_U(more, 0);
    CHECK_EQ_U(fifo.overflow_frames, extra);
    CHECK_EQ_U(bt_fifo_available(&fifo), cap);  // ring contents intact
}

static void test_partial_push_on_near_full(void) {
    printf("test_partial_push_on_near_full\n");
    bt_fifo_init(&fifo);

    // Leave room for only 5 frames, then try to push 20 -> 5 accepted, 15 drop.
    static float l[BT_FIFO_FRAMES];
    static float r[BT_FIFO_FRAMES];
    for (uint32_t i = 0; i < BT_FIFO_FRAMES; i++) { l[i] = 0.0f; r[i] = 0.0f; }

    uint32_t prefill = (BT_FIFO_FRAMES - 1u) - 5u;
    CHECK_EQ_U(bt_fifo_push_n(&fifo, l, r, prefill), prefill);
    CHECK_EQ_U(bt_fifo_free(&fifo), 5);

    uint32_t pushed = bt_fifo_push_n(&fifo, l, r, 20);
    CHECK_EQ_U(pushed, 5);
    CHECK_EQ_U(bt_fifo_free(&fifo), 0);
    CHECK_EQ_U(fifo.overflow_frames, 15);
}

static void test_wraparound(void) {
    printf("test_wraparound\n");
    bt_fifo_init(&fifo);

    // Push/pop a prime-sized block many times so head/tail sweep past the
    // BT_FIFO_FRAMES boundary repeatedly; verify data integrity each round.
    const uint32_t BLK = 317;            // not a divisor of BT_FIFO_FRAMES
    const uint32_t ROUNDS = 100;         // 31700 frames, ~7.7 wraps
    float l[BLK], r[BLK];
    int16_t out[BLK * 2];

    uint64_t seq = 0;
    uint64_t check_seq = 0;
    for (uint32_t round = 0; round < ROUNDS; round++) {
        for (uint32_t i = 0; i < BLK; i++) {
            // Encode a slowly increasing ramp in [-1,1] tied to the sequence
            // number so we can verify ordering survives the wrap.  Signed
            // intermediate: (seq % 2001) is unsigned, so subtract 1000 only
            // after casting to a signed type.
            float v = (float)((int32_t)(seq % 2001) - 1000) / 1000.0f;  // [-1,1]
            l[i] = v;
            r[i] = -v;
            seq++;
        }
        CHECK_EQ_U(bt_fifo_push_n(&fifo, l, r, BLK), BLK);
        CHECK_EQ_U(bt_fifo_pop_n(&fifo, out, BLK), BLK);

        for (uint32_t i = 0; i < BLK; i++) {
            float v = (float)((int32_t)(check_seq % 2001) - 1000) / 1000.0f;
            int16_t exp_l = (int16_t)(v * 32767.0f);
            int16_t exp_r = (int16_t)(-v * 32767.0f);
            CHECK_EQ_I(out[i * 2],     exp_l);
            CHECK_EQ_I(out[i * 2 + 1], exp_r);
            check_seq++;
        }
    }
    CHECK_EQ_U(bt_fifo_available(&fifo), 0);
    CHECK_EQ_U(fifo.overflow_frames, 0);
    CHECK_EQ_U(fifo.underflow_frames, 0);
}

int main(void) {
    printf("=== bt_fifo host test ===\n");
    test_init_empty();
    test_push_pop_values();
    test_partial_pop();
    test_overflow();
    test_partial_push_on_near_full();
    test_wraparound();

    printf("\n%d checks, %d failures\n", checks, failures);
    if (failures) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
