#ifndef BT_EXCLUSION_H
#define BT_EXCLUSION_H

// ----------------------------------------------------------------------------
// Bluetooth radio mutual-exclusion — pure decision helpers (Task 3)
// ----------------------------------------------------------------------------
//
// Platform-independent integer/bit logic, kept in a self-contained header so it
// can be unit-tested on the host (tools/bt_exclusion_test) with no Pico SDK:
//
//   1. gpio_reserved_for_radio() / gpio_pin_assignable() — the four GPIOs the
//      CYW43439 radio owns on a Pico 2 W (WL_ON=23, WL_DATA=24, WL_CS=25,
//      WL_CLK=29), and the full pin-assignability rule is_valid_gpio_pin()
//      enforces.  No DSPi output / clock / RX pin-config may land on a radio pad.
//
//   2. bt_output_standdown_resolve() — which wired outputs are stood down when
//      Bluetooth is enabled, as a logical reservation ahead of the radio's
//      cyw43_arch_init().
//
// Grounding against the PIO/DMA map (RP2350 / Pico 2 W), and its LIMITS:
//   - cyw43_spi_init() claims ONE free PIO SM (pio_claim_free_sm_and_add_program
//     _for_gpio_range) + TWO DMA channels (dma_claim_unused_channel x2).  On the
//     Pico 2 W (RP2350A, 30 GPIOs, PICO_PIO_USE_GPIO_BASE=0) the GPIO base is a
//     no-op — any PIO reaches GPIO 24..29 — so the claim reduces to "a spare SM
//     + 2 spare DMA".
//   - PIO0 is full (4x S/PDIF TX, plus I2S TX) and carries the mains that FEED
//     the BT stream, so it never stands down.  PIO2 is claimed only while a
//     digital input runs.  PIO1 is the block the radio would fall back to; its
//     wired outputs are the PDM sub (SM0 + 1 DMA) and the ADAT bulk output
//     (SM1 + DMA 13/14).  Standing those down is the lever that most directly
//     vacates a PIO1 SM + a contiguous DMA pair for the radio.
//   - IMPORTANT — this is a LOGICAL reservation + GPIO guard, not a physical
//     release.  DSPi claims its PIO SMs / DMA for the life of the program and
//     the disable paths used here (pdm_set_enabled flag; the deferred ADAT
//     config teardown) do NOT pio_sm_unclaim / dma_channel_unclaim.  Physical
//     unclaim (so the freed SM/DMA re-enter the radio's claim pool) is a pending
//     follow-up.  Moreover, under USB input a free PIO SM (PIO2) + >=2 free DMA
//     already exist, so whether ANY stand-down is actually required for
//     cyw43_arch_init() to succeed is to be confirmed on hardware (the bring-up
//     prints `[bt] cyw43 init OK` / `FAILED`).  The stand-down + GPIO guard are
//     the correct mutual-exclusion regardless of that outcome.
//
// The stand-down respects the existing PDM <-> EQ-worker Core-1 interlock, and
// the firmware's bt_radio_standdown() applies it inside a prepare/complete_
// pipeline_reset() bracket so the still-active S/PDIF mains never click.

#include <stdint.h>
#include <stdbool.h>

// True iff `gpio` is one of the CYW43 radio's reserved pins on a Pico 2 W.
// Pure predicate (radio-pin membership); the firmware gates its USE behind
// DSPI_BT, so non-W / RP2040 / non-BT builds are unaffected.
static inline bool gpio_reserved_for_radio(uint8_t gpio) {
    return gpio == 23u || gpio == 24u || gpio == 25u || gpio == 29u;
}

// The exact rule is_valid_gpio_pin() enforces, factored out so the host test
// exercises the real logic.  GPIO 23-25 (power/LED) are never assignable on any
// build.  On the Bluetooth build the radio ALSO reserves 29 (via
// gpio_reserved_for_radio).  The upper bound is 29 on RP2350, 28 on RP2040.
//   radio_build : DSPI_BT is set (a Pico 2 W build).
//   rp2350      : PICO_RP2350 is set.
static inline bool gpio_pin_assignable(uint8_t gpio, bool radio_build,
                                       bool rp2350) {
    if (gpio >= 23u && gpio <= 25u) return false;               // power/LED, all builds
    if (radio_build && gpio_reserved_for_radio(gpio)) return false;  // + WL_CLK=29 on BT
    return rp2350 ? (gpio <= 29u) : (gpio <= 28u);
}

// Stand-down category bits reported by bt_radio_standdown() through its
// uint8_t *stood_down_mask.  One bit per PIO1-resident wired output, so a
// later restore re-enables exactly what was disabled.
#define BT_STOODDOWN_PDM   (1u << 0)   // PDM subwoofer  (PIO1 SM0 + 1 DMA)
#define BT_STOODDOWN_ADAT  (1u << 1)   // ADAT bulk out  (PIO1 SM1 + DMA 13/14)

// Resolve which PIO1 wired outputs are stood down for the radio.
//   pdm_active  : the PDM sub is currently enabled/running.
//   adat_active : the ADAT bulk output is currently config-enabled/running.
// Returns the OR of the BT_STOODDOWN_* bits for those that are active.
static inline uint8_t bt_output_standdown_resolve(bool pdm_active,
                                                  bool adat_active) {
    uint8_t mask = 0;
    if (pdm_active)  mask |= BT_STOODDOWN_PDM;
    if (adat_active) mask |= BT_STOODDOWN_ADAT;
    return mask;
}

#endif // BT_EXCLUSION_H
