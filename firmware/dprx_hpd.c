// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Hot-plug detect: drive the HPD pin for native DP, or in Type-C mode hand the
 * request to PD as a DP Attention.
 *
 * Three requests exist: HPD low (hpd_low, which first raises the video-off
 * event 4), HPD high (hpd_high) and an HPD IRQ pulse (hpd_irq). With
 * typec_mode = 0 they drive the pin through hpd_pin(); with typec_mode = 1
 * they post hpd_request_post(2 / 1 / 3) for the PD side, which sends the HPD
 * state in a DP Status/Attention VDM.
 *
 * The pin: C4:39 bit 4 is the HPD level, bit 3 a write-1-then-0 IRQ strobe.
 * rx_plug_detect reports whether a source is plugged: input E6:33 bit 3
 * natively, or in Type-C mode DP Configure done (dp_alt_status bit 5) with
 * hpd_raised.
 */
#include "fw.h"

/* Drive the native HPD pin: high = 0 low, high = 1 with irq = 0
 * high, high = 1 with irq = 1 an IRQ pulse. */
void hpd_pin(uint8_t high, uint8_t irq)
{
    R_C4_3C = 0x7D;
    if ((R_E1_02 & 0x0F) == 0)
        R_E3_22 = 0x10;
    if (high == 1) {
        if (irq == 1)
            REG_PULSE(R_C4_39, 0x08);
        else
            R_C4_39 |= 0x10;
    } else {
        R_C4_39 &= ~0x10;
    }
}

/* HPD low, after the video-off event 4. */
void hpd_low(void)
{
    rx_video_event(4);
    if (typec_mode == 0)
        hpd_pin(0, 0);
    else
        hpd_request_post(2);
}

/* Type-C HPD state: DP Configure done and HPD raised. */
uint8_t hpd_gate(void)
{
    if ((dp_alt_status & 0x20) && hpd_raised == 1)
        return 1;
    return 0;
}

/* 1 if a source is plugged into the current input (pin sense or
 * Type-C state). */
uint8_t rx_plug_detect(void)
{
    uint8_t mode = typec_mode;

    if (mode == 0)
        return rx_plug_sense();
    if (mode == 1)
        return hpd_gate();
    return 0;
}

/* HPD IRQ pulse. */
void hpd_irq(void)
{
    if (typec_mode == 0)
        hpd_pin(1, 1);
    else
        hpd_request_post(3);
}

/* HPD high. */
void hpd_high(void)
{
    if (typec_mode == 0)
        hpd_pin(1, 0);
    else
        hpd_request_post(1);
}

/* Input E6:33 bit 3 as 0/1 (rx_plug_detect native plug sense). */
uint8_t rx_plug_sense(void)
{
    return (R_E6_33 & 0x08) ? 1 : 0;
}
