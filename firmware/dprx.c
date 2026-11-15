// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* DisplayPort receiver: the RX state machine, plug and unplug, the events
 * that restart it, and the link setup it drives (PHY, equaliser, link
 * training checks, pixel-clock recovery).
 *
 * rx_poll() runs rx_plug(), rx_events() and rx_state_machine() once per
 * main-loop pass. The state lives in rx_state and every change goes through
 * set_rx_state(), which mirrors it to the host (HOST_RX_STATE, E0:AE; the
 * numbering is a host contract, enum rx_state_id in fw.h) and sets
 * rx_restart_req so the new state runs its entry work once.
 *
 *   1 INIT       defaults for the link parameters (rx_ref_clock_sel..rx_tps4)    -> 2
 *   2 WAIT_PLUG  wait for rx_plug_gate (rx_plug); link and
 *                PHY setup from those parameters                      -> 3
 *   3            wait for rx_setup_gate == 1                                 -> 4
 *   4 SETUP      wait 16 000 000 ticks, then clear the IRAM variables,
 *                load the EDID into the AUX EDID RAM, set up DPCD/AUX
 *                and interrupt sources, hpd_high (HPD path)            -> 5
 *   5            wait until rx_lanes_ready() reports every lane (C3:C9)      -> 6
 *   6 EQ         wait for C3:E8 per lane: done -> 7; else pulse
 *                C3:9C or, 16 000 000 ticks on, give up               -> 5
 *   7 DESKEW     wait for lane de-skew (rx_deskew_ok)                      -> 8
 *   8                                                                 -> 9
 *   9 ECC        every 2 000 000 ticks: until C0:EF reads 0           -> A
 *   A MSA        wait until rx_timing_changed() sees no MSA change, then rx_color_changed -> B
 *   B DSC        stream setup, with the DSC path (PPS) when C0:F6..F9
 *                all have bit 6 set; 50 ms                            -> C
 *   C PCR        set up pixel-clock recovery (rx_pixel_clock), wait for
 *                C1:E5 bit 7 on 50 consecutive 1 ms polls (no timeout) -> D
 *   D STABLE     video is up; rx_video_event(3) on entry, (4) on exit.
 *
 * Going back: unplug (rx_plug) -> 2; from 6 on, a retrain request from INT0
 * (rx_retrain_req) -> 5; from B on, a colour space or MSA change -> A; in D, a lost
 * pixel clock -> A (rx_events). INT0 (isr.c) also calls rx_eq_enable, rx_reset_assert,
 * rx_fec_enable, rx_eq_retry_set and rx_lane_count_read.
 *
 * Registers: the DP RX core and DPCD/AUX side (C0..C4, CB, CD), the PHY and
 * clocks (E2, E3, E6, E8), the interrupt masks and acknowledges (E5).
 */
#include <string.h>

#include "fw.h"

/* ---- RX state machine ---- */

/* First pass in the current state: test and clear rx_restart_req. */
static bool rx_entry(void)
{
    if (rx_restart_req != 1)
        return false;
    rx_restart_req = 0;
    return true;
}

static void rx_stamp_now(void)
{
    timer_stamp(&rx_stamp);
}

static bool rx_elapsed(uint32_t ticks)
{
    return timer_elapsed(&rx_stamp, ticks) == 1;
}

/* One step of the RX state machine. Each state either waits for
 * its condition (and returns) or does its work and moves on through
 * set_rx_state(). rx_restart_req is 1 on the first pass after every state
 * change, so the states use it to run their entry work once. */
void rx_state_machine(void)
{
    switch (rx_state) {
    case RX_STATE_INIT:
        rx_init_regs_clear();
        rx_phy_lane_config(4, 4);
        rx_config_default();
        set_rx_state(RX_STATE_WAIT_PLUG);
        break;

    case RX_STATE_WAIT_PLUG:
        if (rx_plug_gate != 1)
            return;
        rx_link_setup(rx_ref_clock_sel, ((uint16_t)rx_max_link_rate_h << 8) | rx_max_link_rate_l, rx_link_rate_cfg_on, rx_phy_e2_71_bit2);
        rx_c3_be_setup();
        rx_pcr_init();
        rx_pclk_pll_band();
        set_rx_state(RX_STATE_3);
        break;

    case RX_STATE_3:
        if (rx_setup_gate != 1)
            return;
        set_rx_state(RX_STATE_SETUP);
        break;

    case RX_STATE_SETUP:
        if (rx_entry())
            rx_stamp_now();
        if (!rx_elapsed(16000000))
            return;
        iram_vars_clear();
        dpcd_addr_latch_a(0x0102);
        dpcd_addr_latch_b(0x0120);
        rx_link_reset_pulse();
        edid_ram_upload(edid_buf);
        rx_lane_routing_setup();
        dpcd_init_default();
        rx_c3_setup_ed_pattern(rx_c3_ed_pattern_sel);
        rx_eq_init();
        rx_eq_enable(0);
        rx_irqs_enable();
        irq_enable(0x12, 1);
        irq_ack(0x12);
        aux_controller_mode(3);
        rx_e6_5f_onehot_set(2);
        aux_c2_regs_setup();
        aux_c2_0c_bit7_clear();
        hpd_high();
        set_rx_state(RX_STATE_5);
        break;

    case RX_STATE_5:
        if (rx_entry())
            rx_retrain_armed = 0;
        if (rx_lanes_ready() != 1) {
            rx_restart_req = 1;
            return;
        }
        rx_eq_enable(1);
        rx_retrain_armed = 1;
        rx_rate_count();
        set_rx_state(RX_STATE_EQ);
        break;

    case RX_STATE_EQ:
        if (rx_lanes_eq_finished() != 1)
            return;
        if (rx_lanes_eq_passed() == 1) {
            rx_training_info();
            rx_eq_enable(0);
            rx_reset_assert(0);
            set_rx_state(RX_STATE_DESKEW);
            break;
        }
        if (rx_eq_retry_get() == 0) {
            rx_eq_fail1 = 1;
            rx_eq_retry_set(1);
            delay_ms(1);
            rx_eq_restart_pulse();
            return;
        }
        if (rx_eq_fail1 == 1) {
            rx_stamp_now();
            rx_eq_fail1 = 0;
        }
        if (!rx_elapsed(16000000))
            return;
        rx_link_reset_pulse();
        hpd_irq();
        set_rx_state(RX_STATE_5);
        break;

    case RX_STATE_DESKEW:
        rx_entry();
        if (rx_deskew_ok() == 0)
            return;
        set_rx_state(RX_STATE_8);
        break;

    case RX_STATE_8:
        set_rx_state(RX_STATE_ECC);
        break;

    case RX_STATE_ECC:
        if (rx_entry()) {
            rx_ecc_poll_count = 0;
            rx_soft_reset_pulse(8);
            rx_soft_reset_pulse(7);
            rx_stamp_now();
        }
        if (!rx_elapsed(2000000))
            return;
        if (rx_ecc_poll_count++ > 3 && fec_ready == 1) {
            rx_ecc_recover_pulse();
            rx_ecc_poll_count = 0;
        }
        if (rx_ecc_ok() == 0) {
            rx_stamp_now();
            return;
        }
        set_rx_state(RX_STATE_MSA);
        break;

    case RX_STATE_MSA:
        if (rx_entry()) {
            irq_enable(6, 1);
            irq_ack(6);
            rx_color_snap_format = 0xFF;
            rx_color_snap_colorimetry = 0xFF;
            rx_color_snap_depth = 0xFF;
            msa_snap_htotal_h = 0;
            msa_snap_htotal_l = 0;
            msa_snap_hact_h = 0;
            msa_snap_hact_l = 0;
        }
        if (rx_timing_changed() != 0)
            return;
        rx_color_changed();
        set_rx_state(RX_STATE_DSC);
        break;

    case RX_STATE_DSC:
        rx_c1_81_from_c0_f6();
        rx_dsc_path_config();
        if (rx_dsc_detect() == 1) {
            rx_dsc_mode_config(1);
            dsc_info_clear();
            dsc_bypass_info();
            dsc_pps();
            rx_video_event(0x10);
        } else {
            rx_dsc_mode_config(0);
        }
        rx_pixel_clock_reset();
        delay_ms(50);
        set_rx_state(RX_STATE_PCR);
        break;

    case RX_STATE_PCR:
        if (rx_entry())
            rx_pixel_clock();
        if (rx_pcr_stable() != 1)
            return;
        rx_pclk_e297_bit7_clear();
        set_rx_state(RX_STATE_STABLE);
        break;

    default:
        return;
    }
}

/* In the later states, watch for events that send the machine
 * back: a retrain request from the INT0 handler (rx_retrain_req, to state 5), FEC
 * being switched on (enables it, no state change), a colour space or MSA
 * change (to state 0x0A), a lost pixel clock (to state 0x0A). The lowest
 * target wins. Only while plugged (rx_plug_gate). */
void rx_events(void)
{
    uint8_t target = 0xFF;

    if (!rx_plug_gate)
        return;
    if (rx_state >= RX_STATE_EQ && rx_retrain_req == 1) {
        rx_retrain_req = 0;
        target = RX_STATE_5;
    }
    if (rx_state >= RX_STATE_MSA && target >= RX_STATE_MSA) {
        if (rx_fec_check() == 1)
            rx_fec_enable(1);
    }
    if (rx_state >= RX_STATE_DSC && target >= RX_STATE_DSC) {
        if (rx_color_changed() == 1)
            target = RX_STATE_MSA;
        if (rx_timing_changed() == 1)
            target = RX_STATE_MSA;
    }
    if (rx_state >= RX_STATE_STABLE && target >= RX_STATE_STABLE) {
        if (!rx_pcr_locked())
            target = RX_STATE_MSA;
        test_sink_crc_start();
    }
    if (rx_state >= RX_STATE_STABLE && irq_pending(4) == 1) {
        irq_ack(4);
        rx_video_event(0x0B);
    }
    if (target != 0xFF)
        set_rx_state(target);
}

/* Enter RX state s (no-op if already there): request the new
 * state's entry work, mirror it to the host, and post the video events
 * for reaching (3) and leaving (4) the stable state. */
void set_rx_state(uint8_t s)
{
    uint8_t old = rx_state;

    if (old == s)
        return;
    rx_state = s;
    rx_restart_req = 1;
    HOST_RX_STATE = s;
    if (rx_state == RX_STATE_STABLE)
        rx_video_event(3);
    if (old == RX_STATE_STABLE)
        rx_video_event(4);
}

/* Plug and unplug, from state 2 on. Unplug (rx_plug_detect() == 0 while
 * plugged) drops back to state 2 and posts event 2; plug (rx_plug_detect() == 1)
 * sets rx_plug_gate and posts event 1. */
void rx_plug(void)
{
    if (rx_state < RX_STATE_WAIT_PLUG)
        return;
    if (rx_plug_gate == 1) {
        if (rx_plug_detect() != 0)
            return;
        set_rx_state(RX_STATE_WAIT_PLUG);
        hpd_low();
        rx_plug_gate = 0;
        rx_video_event(2);
    } else {
        if (rx_plug_detect() != 1)
            return;
        rx_plug_gate = 1;
        rx_video_event(1);
    }
}

/* ---- Link, PHY and clock setup ---- */

/* Pixel-clock bands for rx_pixel_clock(): a measured clock strictly inside
 * (lo, hi) kHz selects the divider settings written to E2:97, E2:98 and
 * E2:9A bit 2. */
struct a_pclk_band {
    uint32_t lo, hi;
    uint8_t e297, e298, e29a_b2;
};

static const __code struct a_pclk_band a_pclk_bands[] = {
    {   7000,  13000, 0x33, 0x0C, 0 },
    {  15000,  22000, 0x30, 0x0C, 1 },
    {  22500,  28000, 0x40, 0x0A, 0 },
    {  33000,  41000, 0x30, 0x0A, 1 },
    {  47000,  53000, 0x40, 0x08, 0 },
    {  72000,  78000, 0x30, 0x08, 1 },
    {  97000, 103000, 0x40, 0x06, 0 },
    { 147000, 153000, 0x30, 0x06, 1 },
    { 247000, 253000, 0x50, 0x04, 0 },
    { 297000, 303000, 0x30, 0x04, 1 },
    { 447000, 453000, 0x48, 0x02, 0 },
    { 597000, 603000, 0x30, 0x02, 1 },
};

/* Pixel-clock recovery setup. Reads the measured stream clock in kHz
 * (C1:FB bits 3..0, C1:FC, C1:FD) and, when it falls in a
 * known band with C1:FA bits 2..0 clear (or 747..753 MHz with C1:F2 = 0 or
 * 0x7F), programs the E2:97/98/9A dividers, C1:81 and C1:B0..B2, sets
 * C1:9E bits 5..4 and toggles rx_reset_assert. 6..7 MHz only gets a fixed E2:95..9A
 * setup. Any other clock changes nothing. */
void rx_pixel_clock(void) __reentrant
{
    const __code struct a_pclk_band *b;
    uint32_t khz;
    uint8_t e297, e298, e29a_b2, v;
    uint8_t i;

    khz = (uint32_t)(R_C1_FB & 0x0F) << 16;
    khz += (uint16_t)R_C1_FC << 8;
    khz += R_C1_FD;

    for (i = 0, b = a_pclk_bands; i < sizeof a_pclk_bands / sizeof a_pclk_bands[0]; i++, b++) {
        if (khz > b->lo && khz < b->hi && !(R_C1_FA & 0x07)) {
            e297 = b->e297;
            e298 = b->e298;
            e29a_b2 = b->e29a_b2;
            goto found;
        }
    }
    if (khz > 747000 && khz < 753000) {
        v = R_C1_F2;
        if (v == 0 || v == 0x7F) {
            e297 = 0x3C;
            e298 = 0;
            e29a_b2 = 0;
            goto found;
        }
    }
    if (khz > 6000 && khz < 7000) {
        R_E2_95 = 0x30;
        R_E2_97 = 0xC0;
        R_E2_98 = 0x1C;
        R_E2_98 = 0x1D;
        v = R_E2_9A | 0x08;
        R_E2_9A = v;
        R_E2_9A = v & 0xFB;
    }
    return;

found:
    R_E2_97 = e297;
    R_E2_97 = e297 | 0x80;
    R_E2_98 = e298;
    R_E2_98 = e298 | 0x10;
    if (e29a_b2)
        R_E2_9A |= 0x04;
    else
        R_E2_9A &= 0xFB;
    R_E2_9A |= 0x08;
    v = R_C1_81 & 0x80;
    R_C1_81 = v;
    R_C1_81 = v | e297;
    R_C1_B0 = 0;
    R_C1_B1 = 0;
    R_C1_B2 = 0;
    R_C1_9E |= 0x30;
    rx_reset_assert(1);
    rx_reset_assert(0);
}

/* Initial per-lane values for E2:7F in rx_eq_band(). */
static const __code uint8_t a_eq_init[4] = { 0x40, 0x40, 0x80, 0x80 };

/* Equaliser band search, one pass per lane 0..3. Each lane selects
 * itself in E8:7A bits 7..6, loads its E2:7F value (rx_eq_lane_value[lane], raised by
 * 0x10 on every retry) into E2:72 bit 0 (lanes 0, 1) or bit 1 (lanes 2, 3),
 * then pulses CB:1D/E1:1B/E8:70 up to 3 times until E8:91 bits 2..1 read
 * 0b10. A lane retries (up to 5 passes) while E8:99 bits 5..0 read all 0 or
 * all 1. Its result E8:98 bits 1..0 and E8:99 is stored in E8:88+2*lane and
 * E8:89+2*lane. mode 0 also sets E8:79 bits 1..0 to 01. */
void rx_eq_band(uint8_t mode)
{
    uint8_t lane, tries, n, r99, v;

    R_E8_70 = 0x02;
    R_E8_70 = 0x02;
    R_E8_72 = 0x0C;
    R_E8_74 = 0x40;
    R_E8_7B &= 0xEF;
    for (n = 0; n < 4; n++)
        rx_eq_lane_value[n] = a_eq_init[n];

    for (lane = 0; lane < 4; lane++) {
        R_E8_7A &= 0x3F;
        R_E2_7E |= 0x01;
        R_E2_72 &= 0xFC;
        tries = 0;
        do {
            R_E8_7A |= (uint8_t)(lane << 6);
            if (tries)
                rx_eq_lane_value[lane] += 0x10;
            R_E2_7F = rx_eq_lane_value[lane];
            R_E2_72 |= lane < 2 ? 0x01 : 0x02;

            for (n = 3; n; n--) {
                v = R_CB_1D & 0xFB;
                R_CB_1D = v;
                R_CB_1D = v & 0xF7;
                R_E1_1B &= 0xFB;
                R_E8_70 &= 0xEF;
                delay_ms(1);
                v = R_CB_1D | 0x04;
                R_CB_1D = v;
                R_CB_1D = v | 0x08;
                R_E1_1B |= 0x04;
                R_E8_70 |= 0x10;
                delay_ms(2);
                if ((R_E8_91 & 0x06) == 0x04)
                    break;
            }
            tries++;
            r99 = R_E8_99;
        } while (((r99 & 0x3F) == 0 || (r99 & 0x3F) == 0x3F) && tries < 5);

        /* Lane 0 stores the E8:99 value already read; the others re-read. */
        if (lane == 0) {
            R_E8_88 = R_E8_98 & 0x03;
            R_E8_89 = r99;
        } else if (lane == 1) {
            R_E8_8A = R_E8_98 & 0x03;
            R_E8_8B = R_E8_99;
        } else if (lane == 2) {
            R_E8_8C = R_E8_98 & 0x03;
            R_E8_8D = R_E8_99;
        } else {
            R_E8_8E = R_E8_98 & 0x03;
            R_E8_8F = R_E8_99;
        }
    }

    if (mode == 0) {
        v = R_E8_79 & 0xFC;
        R_E8_79 = v;
        R_E8_79 = v | 0x01;
    }
    R_E2_7E &= 0xFE;
    R_E8_7B |= 0x10;
    (void)R_E8_88;
    (void)R_E8_89;
    (void)R_E8_8A;
    (void)R_E8_8B;
    (void)R_E8_8C;
    (void)R_E8_8D;
    (void)R_E8_8E;
    (void)R_E8_8F;
}

/* Common tail of rx_phy_lane_config(): set E2:4C bit 1 and E2:4D bits 6/4/2, then E2:4C
 * bit 0 to set0, then E2:4D bits 5, 3 and 1 one at a time to their values in
 * bits4d, and write k to E2:83 and E2:84. */
static void a_e24x(uint8_t set0, uint8_t bits4d, uint8_t k)
{
    uint8_t c, d;

    c = R_E2_4C | 0x02;
    R_E2_4C = c;
    d = R_E2_4D | 0x54;
    R_E2_4D = d;
    R_E2_4C = set0 ? c | 0x01 : c & 0xFE;
    d = (d & 0xDF) | (bits4d & 0x20);
    R_E2_4D = d;
    d = (d & 0xF7) | (bits4d & 0x08);
    R_E2_4D = d;
    d = (d & 0xFD) | (bits4d & 0x02);
    R_E2_4D = d;
    R_E2_83 = k;
    R_E2_84 = k;
}

/* E2:48/4C/4D/83/84 setup selected by sel (0..4) and mode (1, 2;
 * 4 = none). Leaving sel 4 with mode 4 first resets E2:48 bits 3..0, E2:4C
 * bits 1..0, E2:4D bits 6..1 and E2:83/84. sel 4 always clears E2:48 bits
 * 3..0. */
void rx_phy_lane_config(uint8_t sel, uint8_t mode)
{
    if (mode == 4 && sel != 4) {
        R_E2_48 |= 0x0F;
        R_E2_4C &= 0xFC;
        R_E2_4D &= 0x81;
        R_E2_83 = 0;
        R_E2_84 = 0;
    }
    switch (sel) {
    case 0:
    case 1:
        if (mode == 4)
            break;
        if (mode == 2)
            goto sel2_mode2;
        if (mode == 1) {
            R_E2_48 |= 0x08;
            a_e24x(1, 0x00, 0x11);
        }
        break;
    case 2:
        if (mode == 4)
            break;
        if (mode == 2) {
sel2_mode2:
            R_E2_48 |= 0x0C;
            a_e24x(1, 0x20, 0x13);
        } else if (mode == 1) {
            R_E2_48 |= 0x04;
            a_e24x(0, 0x20, 0x12);
        }
        break;
    case 3:
        if (mode == 4)
            break;
        if (mode == 2) {
            R_E2_48 |= 0x03;
            a_e24x(0, 0x0A, 0x1C);
        } else if (mode == 1) {
            R_E2_48 |= 0x01;
            a_e24x(0, 0x02, 0x18);
        }
        break;
    case 4:
        R_E2_48 &= 0xF0;
        a_e24x(0, 0x00, 0x10);
        break;
    }
}

/* Read the training EQ results for lanes 0..3 (C3:F3..FA and
 * C3:90..97, 16-bit pairs). The reads are kept; nothing else happens. */
void rx_training_info(void)
{
    (void)R_C3_F3;
    (void)R_C3_F4;
    (void)R_C3_90;
    (void)R_C3_91;
    (void)R_C3_F5;
    (void)R_C3_F6;
    (void)R_C3_92;
    (void)R_C3_93;
    (void)R_C3_F7;
    (void)R_C3_F8;
    (void)R_C3_94;
    (void)R_C3_95;
    (void)R_C3_F9;
    (void)R_C3_FA;
    (void)R_C3_96;
    (void)R_C3_97;
}

/* VCO and divider setup. on = 0 sets E8:79 bits 3..0 and returns.
 * Otherwise E8:79 bits 3..2 are set, VcoClk = (rate / 2) * 270 and the
 * divider (written to E2:7A with bit 7 set) is VcoClk / 27 (E2:0B bit 0
 * set) or / 25, truncated to 8 bits, divided by 5 when rate >= 12 (E2:7B
 * bit 5 set) or by 1 (bit 5 clear). */
void rx_vco_setup(uint8_t on, uint16_t rate)
{
    uint16_t vco;
    uint8_t q, div, v;

    R_E8_79 &= 0xF0;
    if (!on) {
        R_E8_79 |= 0x0F;
        return;
    }
    R_E8_79 |= 0x0C;
    vco = (rate >> 1) * 270;
    if (R_E2_0B & 0x01)
        q = vco / 27;
    else
        q = vco / 25;
    v = R_E2_7B | 0x40;
    if (rate >= 12) {
        div = 5;
        R_E2_7B = v;
        R_E2_7B = v | 0x20;
    } else {
        div = 1;
        R_E2_7B = v;
        R_E2_7B = v & 0xDF;
    }
    R_E2_7A = (q / div) | 0x80;
}

/* One read, then two writes: (v & keep), then (v & keep) | set. */
#define B_AND_THEN_OR(reg, keep, set)          \
    do {                                       \
        uint8_t v_ = (reg) & (keep);           \
        (reg) = v_;                            \
        (reg) = v_ | (set);                    \
    } while (0)

/* Interrupt sources irq_enable enables and irq_ack acknowledges, by source
 * number. en: the register and bit that enable the source; en_low: the
 * source is enabled when that bit is clear (a mask bit). clr: the
 * register and bit pulsed (write 1 then 0) to acknowledge it. Source 1 (the
 * whole INT0 group) is handled in the functions. */
struct b_irq_src {
    uint8_t src;
    volatile __xdata uint8_t *en;
    uint8_t en_bit;
    uint8_t en_low;
    volatile __xdata uint8_t *clr;
    uint8_t clr_bit;
};

static const __code struct b_irq_src b_irq_srcs[] = {
    { 0x04, &R_C0_16, 0x08, 0, &R_C0_18, 0x08 },
    { 0x06, &R_C0_17, 0x04, 0, &R_C0_19, 0x04 },
    { 0x07, &R_C0_16, 0x40, 0, &R_C0_18, 0x40 },
    { 0x08, &R_C0_16, 0x80, 0, &R_C0_18, 0x80 },
    { 0x0A, &INT1_MASK, 0x08, 1, &INT1_CLEAR, 0x08 },
    { 0x0B, &INT1_MASK, 0x04, 1, &INT1_CLEAR, 0x04 },
    { 0x11, &INT1_MASK, 0x20, 1, &INT1_CLEAR, 0x20 },
    { 0x12, &R_E5_05, 0x10, 1, &R_E5_1C, 0x10 },
    { 0x13, 0, 0, 0, &INT1_CLEAR, 0x80 },
    { 0x17, &R_CD_12, 0x10, 1, &R_CD_12, 0x20 },
    { 0x18, &R_CD_60, 0x08, 0, &R_CD_60, 0x04 },
    { 0x23, 0, 0, 0, &INT0_CLEAR2, 0x40 },
};

static const __code struct b_irq_src *b_irq_find(uint8_t src)
{
    const __code struct b_irq_src *s;

    for (s = b_irq_srcs; s != b_irq_srcs + sizeof b_irq_srcs / sizeof *s; s++)
        if (s->src == src)
            return s;
    return 0;
}

/* Enable (on != 0) or disable interrupt source src. Source 1
 * writes the whole INT0 mask INT0_MASK (0x00 = all enabled); sources without
 * an enable bit (0x13, 0x23) and unknown ones do nothing. */
void irq_enable(uint8_t src, uint8_t on)
{
    const __code struct b_irq_src *s;

    if (src == 0x01) {
        INT0_MASK = on ? 0x00 : 0xFF;
        return;
    }
    s = b_irq_find(src);
    if (!s || !s->en)
        return;
    if ((on != 0) != s->en_low)
        *s->en |= s->en_bit;
    else
        *s->en &= (uint8_t)~s->en_bit;
}

/* Acknowledge interrupt source src. Source 1 writes 0xFF then 0x00
 * to INT0_CLEAR (every INT0 bit, no read). */
void irq_ack(uint8_t src)
{
    const __code struct b_irq_src *s;

    if (src == 0x01) {
        INT0_CLEAR = 0xFF;
        INT0_CLEAR = 0x00;
        return;
    }
    s = b_irq_find(src);
    if (s)
        REG_PULSE(*s->clr, s->clr_bit);
}

/* Capture the RX link parameters rx_ref_clock_sel..typec_mode: the defaults
 * rx_config_get fills in, then the Type-C overrides (typec_mode = 1 and rx_max_lane_count =
 * typec_dp_lane_count on a USB-C input), then the values input_config_get supplies when it
 * reports some. Called at the end of rx_config_default. */
void rx_config_load(void)
{
    uint8_t dflt[0x22];
    struct input_config info;

    memset(&info, 0, sizeof info);
    memset(dflt, 0, sizeof dflt);
    if (rx_config_get(dflt)) {
        rx_ref_clock_sel = dflt[0];
        rx_phy_e2_71_bit2 = dflt[2];
        rx_max_link_rate_h = dflt[3];
        rx_max_link_rate_l = dflt[4];
        rx_max_lane_count = dflt[5];
        typec_mode = dflt[7];
    }
    if (input_typec == 1) {
        typec_mode = 1;
        rx_max_lane_count = typec_dp_lane_count;
    }
    if (input_config_get(&info)) {
        typec_mode = info.typec;
        rx_max_lane_count = info.dp_lane_count;
    }
}

/* Configure the RX front end for the input: native DP, or on
 * Type-C by the pin assignment class rx_typec_routing_class reports (1, 2 or 3; nothing
 * for other values). */
void rx_lane_routing_setup(void)
{
    uint8_t mode = rx_typec_routing_class();
    uint8_t c187_arg, sel, b45c_arg;

    (void)dp_pin_assignment_get();
    if (typec_mode == 0) {
        c187_arg = 0;
        sel = 0;
        b45c_arg = 0;
    } else if (mode == 1) {
        c187_arg = 0;
        sel = 1;
        b45c_arg = 0;
    } else if (mode == 3) {
        c187_arg = 0;
        sel = 3;
        b45c_arg = 3;
    } else if (mode == 2) {
        c187_arg = 1;
        sel = 2;
        b45c_arg = 1;
    } else {
        return;
    }
    rx_e3_1f_bit0_set(c187_arg);
    rx_lane_map_set(sel, rx_link_rate_cfg_on);
    rx_phy_lane_config(sel, rx_max_lane_count);
    rx_phy_lane_select(b45c_arg);
}

/* Up to three attempts: drop and raise the enables CB:1E bit 3,
 * CB:1D bit 0, E1:1B bit 6 and E8:E0 bit 5 (1 ms apart), then wait 2 ms and
 * stop once E8:E6 bits 2:1 read 2. Then copy E8:EE to E2:99 and close the
 * E2:97/98 window. */
void rx_pclk_pll_band(void)
{
    uint8_t tries = 3;

    R_E2_97 = 0xC0;
    while (tries--) {
        R_CB_1E &= 0xF7;
        R_CB_1D &= 0xFE;
        R_E1_1B &= 0xBF;
        R_E8_E0 &= 0xDF;
        delay_ms(1);
        R_CB_1E |= 0x08;
        R_CB_1D |= 0x01;
        R_E1_1B |= 0x40;
        R_E8_E0 |= 0x20;
        delay_ms(2);
        if ((R_E8_E6 & 0x06) == 0x04)
            break;
    }
    R_E2_99 = R_E8_EE;
    R_E2_97 = 0x00;
    R_E2_98 &= 0xFE;
}

/* Fixed setup of the C1 clock-recovery block and E8:E0..E5. */
void rx_pcr_init(void)
{
    B_AND_THEN_OR(R_C1_86, 0xBF, 0x01);
    R_C1_D6 = 0x04;
    R_C1_CD = 0x0B;
    R_C1_CE = 0x00;
    R_C1_CF = 0x04;
    R_C1_D0 = 0x58;
    R_C1_D7 = 0x08;
    R_C1_8C = 0x03;
    R_C1_8D = 0x00;
    R_C1_9C = 0xBE;
    R_C1_B3 |= 0x01;
    R_C1_AD |= 0x80;
    R_C1_E2 = 0x1C;
    R_E2_9A = 0x02;
    R_E8_E0 = 0x28;
    R_E8_E2 = 0x08;
    R_E8_E4 = 0x40;
    R_E8_E5 = 0x04;
}

/* One of two C1 settings sets: mode 1, or any other value. */
void rx_dsc_mode_config(uint8_t mode)
{
    uint8_t v = R_C1_86 & 0xBF;

    R_C1_86 = v;
    if (mode == 1) {
        R_C1_86 = v & 0xFE;
        R_C1_CF = 0x04;
        R_C1_D0 = 0xB0;
        R_C1_C7 = 0xF0;
        R_C1_C8 = 0x00;
        R_C1_CA = 0x0E;
        R_C1_BD = 0x0A;
    } else {
        R_C1_86 = v | 0x01;
        R_C1_CF = 0x04;
        R_C1_D0 = 0x58;
        R_C1_C7 = 0x00;
        R_C1_C8 = 0x80;
        R_C1_CA = 0x20;
        R_C1_BD = 0x20;
    }
}

/* C2:00 bit 0 on, C2:02 bit 4 off, then by mode (0..3): C2:08
 * bit 0 = mode & 1; modes 0/1 set C2:21 bit 7, modes 2/3 clear it and zero
 * C2:23/3C/3F. Other modes stop after the first two writes. */
void aux_controller_mode(uint8_t mode)
{
    R_C2_00 |= 0x01;
    R_C2_02 &= 0xEF;
    if (mode > 3)
        return;
    if (mode & 1)
        R_C2_08 |= 0x01;
    else
        R_C2_08 &= 0xFE;
    if (mode < 2) {
        R_C2_21 |= 0x80;
    } else {
        R_C2_21 &= 0x7F;
        R_C2_23 = 0x00;
        R_C2_3C = 0x00;
        R_C2_3F = 0x00;
    }
}

/* C3 setup; C3:ED = 0x00, 0x55, 0xAA or 0xFF for sel 0, 1, 2 or
 * any other value. */
void rx_c3_setup_ed_pattern(uint8_t sel)
{
    uint8_t v;

    R_C3_5D |= 0x80;
    R_C3_D4 &= 0xFD;
    R_C3_D7 |= 0x80;
    R_C3_D8 |= 0x01;
    v = R_C3_D9 | 0x01;
    R_C3_D9 = v;
    R_C3_D9 = v | 0x40;
    R_C3_EC |= 0x04;
    if (sel == 0)
        R_C3_ED = 0x00;
    else if (sel == 1)
        R_C3_ED = 0x55;
    else if (sel == 2)
        R_C3_ED = 0xAA;
    else
        R_C3_ED = 0xFF;
}

/* RX termination calibration: up to ten 2 ms tries, each
 * restarting E8:D0 bit 7, until E8:D9 bits 7:6 read 2. E8:DA (the result)
 * is read once at the end. */
void rx_rterm(void)
{
    uint8_t tries = 10;
    uint8_t v;

    R_E2_61 |= 0x04;
    while (tries--) {
        v = R_E8_D0 & 0x7F;
        R_E8_D0 = v;
        R_E8_D0 = v | 0x80;
        delay_ms(2);
        if ((R_E8_D9 & 0xC0) == 0x80)
            break;
    }
    (void)R_E8_DA;
}

/* Set C3:AB and E4:00/01 for sel 0/1, 2 or 3; other values do
 * nothing. arg is unused. */
void rx_lane_map_set(uint8_t sel, uint8_t arg)
{
    (void)arg;
    switch (sel) {
    case 0:
    case 1:
        R_C3_AB = 0xE4;
        R_E4_00 &= 0x0F;
        R_E4_01 = 0x1B;
        break;
    case 2:
        R_C3_AB = 0xE1;
        R_E4_00 |= 0xA0;
        R_E4_01 = 0x4B;
        break;
    case 3:
        R_C3_AB = 0x1B;
        R_E4_00 |= 0xA0;
        R_E4_01 = 0xE1;
        break;
    }
}

/* C3 setup; mode 0 first writes C3:21/25/39. */
void rx_c3_regs_init(uint8_t mode)
{
    uint8_t v;

    if (mode == 0) {
        R_C3_21 = 0x17;
        R_C3_25 = 0x22;
        R_C3_39 = 0x08;
    }
    R_C3_0C = 0x3C;
    R_C3_23 = 0x08;
    R_C3_1B = 0x0E;
    v = R_C3_6A | 0x80;
    R_C3_6A = v;
    R_C3_6A = v & 0xBF;
    R_C3_8A |= 0x04;
    R_C3_D4 |= 0x05;
    R_C3_15 = 0x01;
    R_C3_D6 = 0x90;
}

/* The PHY setup sequence for one link configuration. */
void rx_phy_setup(uint8_t a, uint16_t w, uint8_t c, uint8_t d)
{
    rx_phy_e6_e2_setup();
    rx_ref_clock_select(a);
    rx_phy_e2_71_bit2_set(d);
    rx_phy_e2_6c_bit6_set(a);
    rx_vco_setup(a, w);
    rx_eq_band(a);
    rx_phy_e2_6c_bit2_set(c);
}

/* C1 settings that depend on rx_dsc_detect (C0:F6..F9 bit 6 all set). */
void rx_dsc_path_config(void)
{
    uint8_t all = rx_dsc_detect();
    uint8_t v = R_C1_05 & 0xFB;

    R_C1_05 = v;
    R_C1_05 = v & 0xEF;
    if (all == 1) {
        R_C1_3F |= 0x70;
        R_C1_41 |= 0x80;
        R_C1_62 |= 0x10;
    } else {
        R_C1_3F &= 0xCF;
        R_C1_41 &= 0x7F;
        R_C1_62 &= 0xEF;
    }
    R_C1_9E = 0xCF;
}

/* Read the trained link configuration C3:CA (bits 1:0 link-rate
 * index, bits 7:3 lane count); the value is unused. */
void rx_rate_count(void)
{
    (void)RX_LINK_CFG;
}

/* Fixed E2:47 / E8:54..65 setup. Called at the end of rx_phy_analog_init. */
void rx_phy_e8_defaults(void)
{
    R_E2_47 |= 0x80;
    R_E8_5E = 0x67;
    R_E8_5F = 0x67;
    R_E8_60 = 0x55;
    R_E8_61 = 0x55;
    R_E8_64 = 0x77;
    R_E8_65 = 0xFF;
    R_E8_54 = 0x34;
    R_E8_55 = 0x22;
    R_E8_5A = 0x33;
    R_E8_5B = 0x77;
}

/* 1 when, for the lane count in C3:CA bits 6:3 (1, 2 or 4), every lane's
 * bits of r are set; lane 0 owns bits m1, each next lane the bits one
 * higher. 0 for other lane counts (r is then not read). */
static uint8_t b_lanes_set(volatile __xdata uint8_t *r, uint8_t m1)
{
    uint8_t m;

    switch ((RX_LINK_CFG >> 3) & 0x0F) {
    case 1:
        m = m1;
        break;
    case 2:
        m = m1 * 3;
        break;
    case 4:
        m = m1 * 15;
        break;
    default:
        return 0;
    }
    return (*r & m) == m;
}

/* C3:C9 bits 0 and 4 set for every active lane. */
uint8_t rx_lanes_ready(void)
{
    return b_lanes_set(&R_C3_C9, 0x11);
}

/* C3:E8 bit 0 set for every active lane. */
uint8_t rx_lanes_eq_finished(void)
{
    return b_lanes_set(&R_C3_E8, 0x01);
}

/* C3:E8 bits 0 and 4 set for every active lane. */
uint8_t rx_lanes_eq_passed(void)
{
    return b_lanes_set(&R_C3_E8, 0x11);
}

/* C3:BC/BD = w (high, low) with C3:03 strobes 0x40 then 0x80. */
void dpcd_addr_latch_a(uint16_t w)
{
    uint8_t v;

    R_C3_D9 |= 0x40;
    B_AND_THEN_OR(R_C3_C5, 0xF0, 0x01);
    v = DPCD_ADDR_H & 0x3F;
    DPCD_ADDR_H = v;
    R_C3_BB &= 0xF0;
    R_C3_BC = w >> 8;
    R_C3_BD = (uint8_t)w;
    v |= 0x40;
    DPCD_ADDR_H = v;
    DPCD_ADDR_H = v | 0x80;
}

/* E2:0B bit 0 set only when on, E2:0F = 5 and (E7:40 = 6 or
 * E7:4B = 1); E2:70 bits 1:0 cleared, bit 3 set. */
void rx_ref_clock_select(uint8_t on)
{
    R_E2_0B &= 0xFE;
    R_E2_70 &= 0xFC;
    if (on && (R_E7_40 == 0x06 || R_E7_4B == 0x01) && R_E2_0F == 0x05)
        R_E2_0B |= 0x01;
    R_E2_70 |= 0x08;
}

/* E2:95..9A and C1:81/9E reset values, then rx_reset_assert(1), rx_reset_assert(0). */
void rx_pixel_clock_reset(void)
{
    uint8_t v;

    R_E2_95 = 0x10;
    v = R_E2_98 & 0xFE;
    R_E2_98 = v;
    R_C1_81 = 0x00;
    R_C1_9E &= 0xCF;
    R_E2_97 &= 0x7F;
    R_E2_98 = v & 0xEF;
    R_E2_9A &= 0xF7;
    rx_reset_assert(1);
    rx_reset_assert(0);
}

/* C3:BC/BD = w (high, low) with C3:03 strobes 0x20 then 0x10. */
void dpcd_addr_latch_b(uint16_t w)
{
    uint8_t v;

    R_C3_D9 |= 0x40;
    R_C3_C5 &= 0xF0;
    v = DPCD_ADDR_H & 0xCF;
    DPCD_ADDR_H = v;
    R_C3_BB &= 0xF0;
    R_C3_BC = w >> 8;
    R_C3_BD = (uint8_t)w;
    v |= 0x20;
    DPCD_ADDR_H = v;
    DPCD_ADDR_H = v | 0x10;
}

/* Clear then set one C2:B3/B5/B9 bit, chosen by sel (3, 4, 7, 8). */
void rx_soft_reset_pulse(uint8_t sel)
{
    switch (sel) {
    case 3:
        B_AND_THEN_OR(R_C2_B5, 0xBF, 0x40);
        break;
    case 4:
        B_AND_THEN_OR(R_C2_B5, 0xEF, 0x10);
        break;
    case 7:
        B_AND_THEN_OR(R_C2_B3, 0xFE, 0x01);
        break;
    case 8:
        B_AND_THEN_OR(R_C2_B9, 0xEF, 0x10);
        break;
    }
}

/* ---- Small helpers ---- */

/* FEC request check. 1: requested (C3:E1 bit 5) but not enabled
 * (C3:60 bit 6, C3:B8 bit 1 clear); 0: enabled but no longer requested;
 * 2: neither. */
uint8_t rx_fec_check(void)
{
    uint8_t v = R_C3_60;

    if (!(v & 0x40) && !(R_C3_B8 & 0x02) && (R_C3_E1 & 0x20))
        return 1;
    if ((v & 0x40) && (R_C3_B8 & 0x02) && !(R_C3_E1 & 0x20))
        return 0;
    return 2;
}

/* Default link parameters, then rx_config_load fills in the rest. */
void rx_config_default(void)
{
    rx_phy_e2_71_bit2 = 0;
    typec_mode = 0;
    rx_ref_clock_sel = 0;
    rx_link_rate_cfg_on = 1;
    rx_max_link_rate_h = 0;
    rx_max_link_rate_l = 0x14;
    rx_max_lane_count = 4;
    rx_c3_ed_pattern_sel = 1;
    rx_downspread = 1;
    rx_tps3 = 1;
    rx_tps4 = 1;
    rx_config_load();
}

/* Lane de-skew check: strobe C3:C2 bit 4, wait 10 ms; 0 when
 * C3:FD bit 4 is set and C3:CA bits 7..3 are not 1, else 1. */
uint8_t rx_deskew_ok(void)
{
    REG_PULSE(R_C3_C2, 0x10);
    delay_ms(10);
    if ((R_C3_FD & 0x10) && ((RX_LINK_CFG >> 3) & 0x1F) != 1)
        return 0;
    return 1;
}

/* Fill an 8-byte link-parameter record with defaults (bytes 1 and
 * 6 untouched); returns 1. */
uint8_t rx_config_get(__xdata uint8_t *p)
{
    if (p) {
        p[2] = 0;
        p[0] = 0;
        p[3] = 0x00; /* 16-bit 0x0014, high byte first */
        p[4] = 0x14;
        p[5] = 4;
        p[7] = 1;
    }
    return 1;
}

/* Pixel clock stable: 1 once rx_pcr_locked() has read 1 on 50
 * consecutive calls (1 ms apart); pcr_lock_count counts them. */
uint8_t rx_pcr_stable(void)
{
    if (rx_pcr_locked() == 1) {
        pcr_lock_count++;
        delay_ms(1);
    } else {
        pcr_lock_count = 0;
    }
    if (pcr_lock_count >= 50) {
        pcr_lock_count = 0;
        return 1;
    }
    return 0;
}

/* CB:14 bits 7..6 = 1 -> 01, 2 -> 10, 3 -> 11, else 00. */
void rx_phy_lane_select(uint8_t v)
{
    R_CB_14 &= 0x3F;
    switch (v) {
    case 1: R_CB_14 |= 0x40; break;
    case 2: R_CB_14 |= 0x80; break;
    case 3: R_CB_14 |= 0xC0; break;
    }
}

/* E6:5F bits 2..0 = 0 -> 100, 1 -> 010, 2 -> 001, else 000. */
void rx_e6_5f_onehot_set(uint8_t v)
{
    R_E6_5F &= 0xF8;
    if (v == 0)
        R_E6_5F |= 0x04;
    if (v == 1)
        R_E6_5F |= 0x02;
    if (v == 2)
        R_E6_5F |= 0x01;
}

/* On (0) / off (else): E2:6C bit 4 set and bit 2 set/cleared in
 * two writes, C3:1F bit 1 cleared/set. */
void rx_phy_e2_6c_bit2_set(uint8_t off)
{
    uint8_t v = R_E2_6C | 0x10;

    if (off == 0) {
        R_E2_6C = v;
        R_E2_6C = v | 0x04;
        R_C3_1F &= ~0x02;
    } else {
        R_E2_6C = v;
        R_E2_6C = v & ~0x04;
        R_C3_1F |= 0x02;
    }
}

void rx_c3_be_setup(void)
{
    uint8_t v;

    R_C3_B8 &= ~0x02;
    v = R_C3_BE | 0x80;
    R_C3_BE = v;
    v &= ~0x02;
    R_C3_BE = v;
    R_C3_BE = v & ~0x01;
    R_C3_C3 |= 0x80;
    R_C2_B6 &= ~0x10;
}

/* 1 when C0:F6..F9 all have bit 6 set (the DSC case). */
uint8_t rx_dsc_detect(void)
{
    return (R_C0_F6 & 0x40) && (R_C0_F7 & 0x40) && (R_C0_F8 & 0x40) &&
           (R_C0_F9 & 0x40);
}

/* DPCD 0x00270 == 1 -> C1:4A = 0x31. */
void test_sink_crc_start(void)
{
    DPCD_ADDR_H &= 0xF0;
    DPCD_ADDR_M = 0x02;
    DPCD_ADDR_L = 0x70;
    if (DPCD_RDATA == 1)
        R_C1_4A = 0x31;
}

void rx_init_regs_clear(void)
{
    R_CB_25 &= 0xF8;
    R_CB_26 &= 0xFE;
    R_C4_42 = 0;
    R_CD_6D = 0;
    R_CD_6E = 0;
    R_CD_6F = 0;
}

void rx_phy_e6_e2_setup(void)
{
    R_E6_38 |= 0x0C;
    R_E2_71 |= 0x40;
    if (sys_e1_02_low_read() == 0)
        R_E2_71 &= ~0x40;
}

/* On (1) / off: C3:5D bit 3 and C3:60 bit 4. Also called from
 * INT0. */
void rx_eq_enable(uint8_t on) __reentrant
{
    if (on == 1) {
        R_C3_5D |= 0x08;
        R_C3_60 |= 0x10;
    } else {
        R_C3_5D &= ~0x08;
        R_C3_60 &= ~0x10;
    }
}

/* Link and PHY setup for state 2 (arguments: the link parameters
 * rx_ref_clock_sel, rx_max_link_rate_h:23, rx_link_rate_cfg_on, rx_phy_e2_71_bit2). */
void rx_link_setup(uint8_t a, uint16_t b, uint8_t c, uint8_t d)
{
    rx_phy_setup(a, b, c, d);
    rx_phy_analog_init();
    aux_controller_init();
    rx_c3_regs_init(c);
    rx_e6_38_low_bits_set();
}

/* 1 clears, else sets, bit 2 of C2:BB and C2:B9. Also called from
 * INT0. */
void rx_reset_assert(uint8_t v) __reentrant
{
    if (v == 1) {
        R_C2_BB &= ~0x04;
        R_C2_B9 &= ~0x04;
    } else {
        R_C2_BB |= 0x04;
        R_C2_B9 |= 0x04;
    }
}

/* FEC on (1) / off: C3:60 bit 6 and C3:B8 bit 1. Also called from
 * INT0. */
void rx_fec_enable(uint8_t on) __reentrant
{
    if (on == 1) {
        R_C3_60 |= 0x40;
        R_C3_B8 |= 0x02;
    } else {
        R_C3_60 &= ~0x40;
        R_C3_B8 &= ~0x02;
    }
}

/* Clear the DSC/PPS info and dsc_bypass_timing. */
void dsc_info_clear(void)
{
    dsc_color_format = 0;
    dsc_bpc_half = 0;
    dsc_number = 0;
    dsc_slice_count = 0;
    dsc_ratio = 0;
    dsc_bpp_h = 0;
    dsc_bpp_l = 0;
    dsc_slice_height_h = 0;
    dsc_slice_height_l = 0;
    dsc_slice_width_h = 0;
    dsc_slice_width_l = 0;
    dsc_chunk_size_h = 0;
    dsc_chunk_size_l = 0;
    memset(&dsc_bypass_timing, 0, sizeof dsc_bypass_timing);
}

void rx_irqs_enable(void)
{
    irq_enable(0x11, 1);
    irq_ack(0x11);
    hpd_irq_event_enable();
    irq_enable(1, 1);
    irq_ack(1);
    irq_enable(4, 1);
    irq_ack(4);
}

void aux_c2_regs_setup(void)
{
    REG_PULSE(R_C2_04, 0x04);
    R_C2_07 &= ~0x04;
    R_C2_13 = 0;
    R_C2_14 = 0x0E;
}

void aux_controller_init(void)
{
    R_C2_03 = 0x44;
    R_C2_04 &= 0xFC;
    R_C2_08 = 0x09;
    R_C2_09 = 0x13;
    R_C2_09 = 0x13 | 0x80;
}

void rx_eq_init(void)
{
    uint8_t v = R_C3_EA & 0xFE;

    R_C3_EA = v;
    R_C3_9C &= ~0x20;
    v &= 0x03;
    R_C3_EA = v;
    R_C3_EA = v | 0xC8;
}

/* 0 sets E2:6C bit 6; else clears it and E2:75 bit 3. */
void rx_phy_e2_6c_bit6_set(uint8_t v)
{
    if (v == 0) {
        R_E2_6C |= 0x40;
    } else {
        R_E2_6C &= ~0x40;
        R_E2_75 &= ~0x08;
    }
}

/* 0 sets, 1 clears E2:71 bit 2. */
void rx_phy_e2_71_bit2_set(uint8_t v)
{
    if (v == 0)
        R_E2_71 |= 0x04;
    else if (v == 1)
        R_E2_71 &= ~0x04;
}

/* ECC check: 1 when C0:EF reads 0; else strobes C0:10 bit 6 and
 * returns 0. */
uint8_t rx_ecc_ok(void)
{
    if (R_C0_EF) {
        REG_PULSE(R_C0_10, 0x40);
        return 0;
    }
    return 1;
}

/* C3:2A bit 6 high for 2 ms. */
void rx_ecc_recover_pulse(void)
{
    R_C3_2A |= 0x40;
    delay_ms(2);
    R_C3_2A &= ~0x40;
}

/* C3:9C bit 6 high for 1 ms. */
void rx_eq_restart_pulse(void)
{
    R_C3_9C |= 0x40;
    delay_ms(1);
    R_C3_9C &= ~0x40;
}

/* 1 when typec_receptacle is 0, else 2 when pd0_cc_line is set, else 3. */
uint8_t rx_typec_routing_class(void)
{
    if (typec_receptacle == 0)
        return 1;
    return pd0_cc_line ? 2 : 3;
}

/* C1:81 bit 7 = C0:F6 bit 2. */
void rx_c1_81_from_c0_f6(void)
{
    if (R_C0_F6 & 0x04)
        R_C1_81 |= 0x80;
    else
        R_C1_81 &= 0x7F;
}

void rx_phy_e3_e6_setup(void)
{
    R_E3_21 = 0;
    R_E3_1E = 0x1A;
    R_E6_3A |= 0xC0;
}

/* 1 sets, else clears E3:1F bit 0. */
void rx_e3_1f_bit0_set(uint8_t v)
{
    if (v == 1)
        R_E3_1F |= 0x01;
    else
        R_E3_1F &= ~0x01;
}

/* 1 sets, else clears C3:EA bit 0 (read back by rx_eq_retry_get). Also
 * called from INT0. */
void rx_eq_retry_set(uint8_t v) __reentrant
{
    if (v == 1)
        R_C3_EA |= 0x01;
    else
        R_C3_EA &= ~0x01;
}

/* Zero the IRAM variables (not the AUX request: aux_hdr..aux_reply_i). */
void iram_vars_clear(void)
{
    rx_retrain_armed = 0;
    rx_retrain_req = 0;
    rx_link_rate_index = 0;
    aux_t2_stage = 0;
    fec_config = 0;
    fec_ready = 0;
    aux_t2_count = 0;
    rx_int0_bit7_flag = 0;
    memset(aux_addr, 0, sizeof aux_addr);
    memset(aux_data, 0, sizeof aux_data);
}

/* Pixel clock recovery locked (C1:E5 bit 7). */
uint8_t rx_pcr_locked(void)
{
    return (R_C1_E5 & 0x80) ? 1 : 0;
}

/* C3:EA bit 0. */
uint8_t rx_eq_retry_get(void)
{
    return (R_C3_EA & 0x01) ? 1 : 0;
}

/* C3:CA bits 6..3. Called from INT0. */
uint8_t rx_lane_count_read(void)
{
    return (RX_LINK_CFG >> 3) & 0x0F;
}

/* Strobe C3:6A bits 2..0. */
void rx_link_reset_pulse(void)
{
    REG_PULSE(R_C3_6A, 0x07);
}

void rx_phy_analog_init(void)
{
    rx_phy_e3_e6_setup();
    rx_rterm();
    rx_phy_e8_defaults();
}

void aux_c2_0c_bit7_clear(void)
{
    R_C2_0C &= 0x7F;
}

void rx_e6_38_low_bits_set(void)
{
    R_E6_38 |= 0x03;
}

void hpd_irq_event_enable(void)
{
    R_C4_3E |= 0x20;
}

void rx_pclk_e297_bit7_clear(void)
{
    R_E2_97 &= 0x7F;
}

uint8_t dp_pin_assignment_get(void)
{
    return dp_pin_assignment;
}
