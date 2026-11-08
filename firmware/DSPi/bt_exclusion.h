#ifndef BT_EXCLUSION_H
#define BT_EXCLUSION_H

// ----------------------------------------------------------------------------
// Bluetooth radio mutual-exclusion — pure decision helpers (Task 3)
// ----------------------------------------------------------------------------
//
// Two platform-independent contracts, kept in a self-contained header so they
// can be unit-tested on the host (tools/bt_exclusion_test) with no Pico SDK:
//
//   1. gpio_reserved_for_radio() — the four GPIOs the CYW43439 radio owns on a
//      Pico 2 W (WL_ON=23, WL_DATA=24, WL_CS=25, WL_CLK=29).  is_valid_gpio_pin()
//      rejects these on the DSPI_BT build so no output / clock / RX pin-config
//      can ever drive a pad the radio is using.
//
//   2. bt_output_standdown_resolve() — which wired outputs must stand down when
//      Bluetooth is enabled so the radio's cyw43_arch_init() can claim a free
//      PIO state machine + 2 DMA channels.
//
// Why these outputs, grounded in the actual PIO/DMA map (RP2350 / Pico 2 W):
//   - The radio's cyw43_spi_init() claims ONE free PIO SM (via
//     pio_claim_free_sm_and_add_program_for_gpio_range) + TWO DMA channels
//     (dma_claim_unused_channel x2).  On the Pico 2 W (RP2350A, 30 GPIOs,
//     PICO_PIO_USE_GPIO_BASE=0) the GPIO base imposes no constraint — any PIO
//     can reach GPIO 24..29 — so the claim is purely "a spare SM + 2 spare DMA".
//   - PIO0 is permanently full: all 4 S/PDIF TX engines (and any I2S TX output)
//     live there, and they carry the processed mains that FEED the BT stream,
//     so they can never stand down.
//   - PIO2 is claimed only while a digital input source (S/PDIF-RX / I2S-RX) is
//     active, so it cannot be relied on as free.
//   - PIO1 is therefore the block the radio must be able to fall back to.  Its
//     wired outputs are the PDM sub (SM0 + 1 DMA channel) and the ADAT bulk
//     output (SM1 + DMA channels 13/14 — the contiguous pair that mirrors the
//     radio's dma_out/dma_in claim).  ADAT RX, when present, sits on SM2.
//     Standing the PDM sub and ADAT bulk output down vacates PIO1 SM0/SM1 and
//     their DMA channels, guaranteeing the radio a free SM + >=2 free DMA
//     regardless of which input source is active.
//
// The stand-down respects the existing PDM <-> EQ-worker Core-1 interlock:
// disabling the PDM sub returns Core 1 to EQ-worker mode through the normal
// derive_core1_mode() path, so the still-active S/PDIF mains are undisturbed.
//
// NB this header expresses only the DECISION (which outputs).  The firmware's
// bt_output_enable() applies it inside the mute/settle discipline so the other
// active outputs never click.

#include <stdint.h>
#include <stdbool.h>

// True iff `gpio` is one of the CYW43 radio's reserved pins on a Pico 2 W.
// Pure predicate (radio-pin membership); the firmware gates its USE behind
// DSPI_BT, so non-W / RP2040 / non-BT builds are unaffected.
static inline bool gpio_reserved_for_radio(uint8_t gpio) {
    return gpio == 23u || gpio == 24u || gpio == 25u || gpio == 29u;
}

// Stand-down category bits reported by bt_output_enable() through its
// uint8_t *stood_down_mask.  One bit per PIO1-resident wired output, so a
// later restore re-enables exactly what was disabled.
#define BT_STOODDOWN_PDM   (1u << 0)   // PDM subwoofer  (PIO1 SM0 + 1 DMA)
#define BT_STOODDOWN_ADAT  (1u << 1)   // ADAT bulk out  (PIO1 SM1 + DMA 13/14)

// Resolve which PIO1 wired outputs must stand down for the radio.
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
