/*
 * Host unit test for firmware/DSPi/bt_exclusion.h
 *
 * Exercises the two pure decision helpers that back Task 3's radio
 * mutual-exclusion:
 *
 *   gpio_reserved_for_radio(gpio)       — the CYW43 radio owns GPIO
 *                                         23/24/25/29 on the Pico 2 W; no DSPi
 *                                         output/clock pin may land on them.
 *
 *   bt_output_standdown_resolve(...)    — when Bluetooth is enabled, which of
 *                                         the PIO1-resident wired outputs (PDM
 *                                         sub, ADAT bulk) must stand down so the
 *                                         radio can claim a free PIO state
 *                                         machine + DMA pair.
 *
 * These are platform-independent integer/bit logic, so they are fully
 * verifiable on the host without any Pico SDK or hardware.  The resolution
 * *table* (which stand-down bits come back for each active-output combination)
 * is the contract the firmware's bt_radio_standdown() applies, and the pin
 * predicate is the contract is_valid_gpio_pin() enforces.
 *
 * Build + run:  make        (in this directory)
 * Exit status 0 = all assertions passed.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "bt_exclusion.h"

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
            printf("  FAIL  %s:%d  %s == %s  (0x%x != 0x%x)\n",           \
                   __FILE__, __LINE__, #a, #b, _a, _b);                   \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------------
// gpio_reserved_for_radio()
// ---------------------------------------------------------------------------

static void test_radio_pins_reserved(void) {
    printf("test_radio_pins_reserved\n");
    // The four CYW43 pins: WL_ON=23, WL_DATA=24, WL_CS=25, WL_CLK=29.
    CHECK(gpio_reserved_for_radio(23));
    CHECK(gpio_reserved_for_radio(24));
    CHECK(gpio_reserved_for_radio(25));
    CHECK(gpio_reserved_for_radio(29));
}

static void test_non_radio_pins_free(void) {
    printf("test_non_radio_pins_free\n");
    // A representative sweep of DSPi's real output/clock pins and the gaps
    // around the reserved block.  None of these belong to the radio.
    const uint8_t free_pins[] = {
        0, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 26, 27, 28
    };
    for (unsigned i = 0; i < sizeof(free_pins); i++) {
        CHECK(!gpio_reserved_for_radio(free_pins[i]));
    }
    // 26/27/28 are valid RP2350 GPIOs that are NOT radio pins — they must stay
    // assignable even on the Bluetooth build (only 29 joins the reserved set).
    CHECK(!gpio_reserved_for_radio(28));
    // Out-of-range pin numbers are never radio pins.
    CHECK(!gpio_reserved_for_radio(30));
    CHECK(!gpio_reserved_for_radio(255));
}

static void test_radio_pins_are_exactly_four(void) {
    printf("test_radio_pins_are_exactly_four\n");
    int count = 0;
    for (int g = 0; g < 256; g++)
        if (gpio_reserved_for_radio((uint8_t)g)) count++;
    CHECK_EQ_U(count, 4);
}

// ---------------------------------------------------------------------------
// gpio_pin_assignable() — the exact rule is_valid_gpio_pin() delegates to.
// Exercises the reserved-pin logic across the build axes so a regression in it
// (e.g. dropping the +29 radio reservation, or widening the upper bound) is
// caught here rather than only on hardware.
// ---------------------------------------------------------------------------

static void test_pin_assignable_radio_reservation(void) {
    printf("test_pin_assignable_radio_reservation\n");
    // RP2350 axis (Pico 2 / Pico 2 W).
    // GPIO 29 (WL_CLK): REJECTED on the BT build, ACCEPTED on non-BT RP2350.
    CHECK(!gpio_pin_assignable(29, /*radio_build=*/true,  /*rp2350=*/true));
    CHECK( gpio_pin_assignable(29, /*radio_build=*/false, /*rp2350=*/true));

    // GPIO 23/24/25: these three radio pins fall inside the power/LED block
    // (23-25), so they are rejected on BOTH builds anyway — 29 is the ONLY pin
    // whose rejection is radio-specific (asserted above).
    for (uint8_t g = 23; g <= 25; g++) {
        CHECK(!gpio_pin_assignable(g, true,  true));
        CHECK(!gpio_pin_assignable(g, false, true));
    }

    // GPIO 26/27/28: valid RP2350 pins, NOT radio pins — accepted on BOTH.
    for (uint8_t g = 26; g <= 28; g++) {
        CHECK(gpio_pin_assignable(g, true,  true));
        CHECK(gpio_pin_assignable(g, false, true));
    }

    // A representative real output/clock pin is always assignable.
    CHECK(gpio_pin_assignable(6,  true,  true));   // S/PDIF 1
    CHECK(gpio_pin_assignable(10, true,  true));   // PDM sub
    CHECK(gpio_pin_assignable(12, true,  true));   // ADAT
}

static void test_pin_assignable_upper_bound(void) {
    printf("test_pin_assignable_upper_bound\n");
    // RP2350 tops out at 29, RP2040 at 28.
    CHECK( gpio_pin_assignable(29, false, /*rp2350=*/true));
    CHECK(!gpio_pin_assignable(29, false, /*rp2350=*/false));  // RP2040: no GPIO 29
    CHECK( gpio_pin_assignable(28, false, false));             // RP2040: 28 ok
    CHECK(!gpio_pin_assignable(30, true,  true));              // out of range
    CHECK(!gpio_pin_assignable(30, false, true));
}

// ---------------------------------------------------------------------------
// bt_output_standdown_resolve()
// ---------------------------------------------------------------------------

static void test_standdown_resolution_table(void) {
    printf("test_standdown_resolution_table\n");

    // Neither PIO1 wired output active → nothing to stand down.  The radio
    // claims its free PIO1 SM + DMA with no output disturbed.
    CHECK_EQ_U(bt_output_standdown_resolve(false, false), 0);

    // PDM sub active only → stand down PDM (frees PIO1 SM0 + its DMA channel).
    CHECK_EQ_U(bt_output_standdown_resolve(true, false), BT_STOODDOWN_PDM);

    // ADAT bulk active only → stand down ADAT (frees PIO1 SM1 + DMA 13/14).
    CHECK_EQ_U(bt_output_standdown_resolve(false, true), BT_STOODDOWN_ADAT);

    // Both active → both stand down; PIO1 is fully vacated for the radio.
    CHECK_EQ_U(bt_output_standdown_resolve(true, true),
               (uint8_t)(BT_STOODDOWN_PDM | BT_STOODDOWN_ADAT));
}

static void test_standdown_bits_distinct(void) {
    printf("test_standdown_bits_distinct\n");
    // The two stand-down category bits must not overlap, so a restore can tell
    // exactly which outputs it disabled and re-enable only those.
    CHECK(BT_STOODDOWN_PDM != 0);
    CHECK(BT_STOODDOWN_ADAT != 0);
    CHECK_EQ_U(BT_STOODDOWN_PDM & BT_STOODDOWN_ADAT, 0);
    // The whole mask fits in the uint8_t the firmware API reports it through.
    CHECK_EQ_U((BT_STOODDOWN_PDM | BT_STOODDOWN_ADAT) & ~0xFFu, 0);
}

// ---------------------------------------------------------------------------

int main(void) {
    printf("=== bt_exclusion_test ===\n");
    test_radio_pins_reserved();
    test_non_radio_pins_free();
    test_radio_pins_are_exactly_four();
    test_pin_assignable_radio_reservation();
    test_pin_assignable_upper_bound();
    test_standdown_resolution_table();
    test_standdown_bits_distinct();

    printf("---\n%d checks, %d failures\n", checks, failures);
    if (failures) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
