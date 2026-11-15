// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Boot-time setup: clocks, interrupts, Timer 2 and the state resets.
 *
 * sys_init runs once from main: clock registers E2:05..0F (clock_setup), the
 * free-running counter and every interrupt source (irq_init), pin setup,
 * oscillator registers E5:45..57 and the oscillator check (osc_init), the
 * host version registers, a few E4:04 / C4:2E / C2:A8 / E3:1A..1B bits
 * whose meaning is unknown, the SystemEvent reset, and finally the host
 * IRQ pin (GPIO 0x0B) as an output driven high (idle; the IRQ is an
 * active-low pulse). state_reset then puts the RX, TX and audio state
 * machines in their first state (1) and initialises HDCP.
 *
 * Interrupts: INT0 (DP RX, AUX) level-triggered at high priority, INT1
 * (USB-C PD, DP) level-triggered, the extra interrupt at vector 0x43
 * (SFR_E8 bit 0), and Timer 2 as a stopped one-shot that isr.c arms.
 */
#include <string.h>

#include "fw.h"

/* Clock registers E2:05..0F. mode 1 programs them (E2:0F = 5);
 * mode 2 clears E2:0A, E2:0B and E2:0F; other modes do nothing. */
void clock_setup(uint8_t mode)
{
    if (mode == 1) {
        R_E2_05 = 0x51;
        R_E2_06 = 0x88;
        R_E2_08 = 0x1E;
        R_E2_09 = 0x0F;
        R_E2_0A = 0x03;
        R_E2_0B = 0x10;
        R_E2_0F = 0x05;
    } else if (mode == 2) {
        R_E2_0A = 0;
        R_E2_0B = 0;
        R_E2_0F = 0;
    }
}

/* In mode 3 load 0x009C40 into E5:45..47 and E5:55..57, then read
 * the oscillator state: the internal oscillator counts as enabled when
 * E2:0F = 5 and either E7:40 = 6 (measurement in E7:3C..3E) or E7:4B = 1
 * (measurement in E7:49..4B). The result is not used; only the reads are made. */
void osc_init(uint8_t mode)
{
    if (mode == 3) {
        R_E5_45 = 0x00;
        R_E5_46 = 0x9C;
        R_E5_47 = 0x40;
        R_E5_55 = 0x00;
        R_E5_56 = 0x9C;
        R_E5_57 = 0x40;
    }
    if ((R_E7_40 == 6 || R_E7_4B == 1) && R_E2_0F == 5) {
        if (R_E7_40 == 6) {
            (void)R_E7_3C;
            (void)R_E7_3D;
            (void)R_E7_3E;
        } else if (R_E7_4B == 1) {
            (void)R_E7_49;
            (void)R_E7_4A;
        }
    }
}

/* E3:1A bit 4 and E3:1B bits 7..5 by mode. 0 and 3: clear E3:1A
 * bit 4, set E3:1B bit 7. 1, 2, 4: set E3:1A bit 4, then write E3:1B three
 * times: bit 7 cleared; bit 6 set (1, 2) or cleared (4); bit 5 set (2) or
 * cleared (1, 4). Other modes do nothing. */
void pin_e3_1a_1b_mode(uint8_t mode)
{
    uint8_t v;

    if (mode == 0 || mode == 3) {
        R_E3_1A &= (uint8_t)~0x10;
        R_E3_1B |= 0x80;
    } else if (mode == 1 || mode == 2 || mode == 4) {
        R_E3_1A |= 0x10;
        v = R_E3_1B & 0x7F;
        R_E3_1B = v;
        if (mode == 4)
            v &= (uint8_t)~0x40;
        else
            v |= 0x40;
        R_E3_1B = v;
        if (mode == 2)
            v |= 0x20;
        else
            v &= (uint8_t)~0x20;
        R_E3_1B = v;
    }
}

/* Mode 0 sets C4:2E bit 4 and clears E4:04 bit 2; mode 5 does the
 * reverse; other modes do nothing. */
void sys_c4_2e_e4_04_mode(uint8_t mode)
{
    if (mode == 0) {
        R_C4_2E |= 0x10;
        R_E4_04 &= (uint8_t)~0x04;
    } else if (mode == 5) {
        R_C4_2E &= (uint8_t)~0x10;
        R_E4_04 |= 0x04;
    }
}

/* E4:04 bit 1: cleared in mode 3, set in mode 4. */
void sys_e4_04_bit1_mode(uint8_t mode)
{
    if (mode == 3)
        R_E4_04 &= (uint8_t)~0x02;
    else if (mode == 4)
        R_E4_04 |= 0x02;
}

/* E4:04 bit 0: cleared in mode 5, set in mode 4. */
void sys_e4_04_bit0_mode(uint8_t mode)
{
    if (mode == 5)
        R_E4_04 &= (uint8_t)~0x01;
    else if (mode == 4)
        R_E4_04 |= 0x01;
}

/* Clear C2:A8 bit 0 in modes 0 and 5. */
void sys_c2_a8_bit0_clear(uint8_t mode)
{
    if (mode == 0 || mode == 5)
        R_C2_A8 &= (uint8_t)~0x01;
}

/* Pin 0x05 to function 0x0C. */
void pin_05_func_0c_set(void)
{
    pin_config(0x05, 0x0C, 0);
}

/* INT0 level-triggered, high priority, enabled. */
void int0_enable(void)
{
    IT0 = 0;
    EX0 = 1;
    PX0 = 1;
    EA = 1;
}

/* INT1 level-triggered, enabled. */
void int1_enable(void)
{
    IT1 = 0;
    EX1 = 1;
    EA = 1;
}

/* SFR_F8 = 0, SFR_E8 = 0x01 (the interrupt at vector 0x43). */
void ext2_enable(void)
{
    SFR_F8 = 0;
    SFR_E8 = 0x01;
    EA = 1;
}

/* Timer 2 stopped at count 0 (T2CON = 0x01), its interrupt enabled. */
void t2_init(void)
{
    T2CON = 0x01;
    TL2 = 0;
    TH2 = 0;
    TR2 = 0;
    ET2 = 1;
    EA = 1;
}

/* Counter and interrupts. */
void irq_init(void)
{
    now_reset();
    counter_init();
    int0_enable();
    int1_enable();
    ext2_enable();
    t2_init();
}

/* No SystemEvent pending, current SystemEvent 4. */
void event_reset(void)
{
    sys_event_pending = 0;
    sys_event = 4;
}

/* Boot-time hardware setup, ending with the host IRQ pin driven
 * high. */
void sys_init(void)
{
    clock_setup(2);
    irq_init();
    pin_05_func_0c_set();
    osc_init(3);
    host_version_init();
    sys_c4_2e_e4_04_mode(0);
    sys_c2_a8_bit0_clear(0);
    sys_e4_04_bit1_mode(4);
    sys_e4_04_bit0_mode(4);
    event_reset();
    pin_e3_1a_1b_mode(0);
    pin_config(0x0B, 0x04, 0);
    gpio_write(0x0B, 1);
}

/* dpcd_ext_caps = 1 when E3:35 bits 1:0 are both set, else 0. */
void dpcd_ext_caps_detect(void)
{
    dpcd_ext_caps = 0;
    dpcd_ext_caps = (R_E3_35 & 0x03) == 0x03;
}

void dpcd_ext_caps_init(void)
{
    dpcd_ext_caps_detect();
}

/* E1:02 bits 3:0, read once at boot. */
uint8_t sys_e1_02_low_read(void)
{
    return R_E1_02 & 0x0F;
}

/* RX state cleared, RxState 1. */
void rx_state_reset(void)
{
    rx_ref_clock_sel = 0;
    rx_link_rate_cfg_on = 0;
    rx_phy_e2_71_bit2 = 0;
    rx_max_link_rate_h = 0;
    rx_max_link_rate_l = 0;
    rx_max_lane_count = 0;
    rx_lane_count = 0;
    typec_mode = 0;
    rx_c3_ed_pattern_sel = 0;
    rx_downspread = 0;
    rx_tps3 = 0;
    rx_tps4 = 0;
    rx_plug_gate = 0;
    rx_restart_req = 0;
    rx_setup_gate = 0;
    rx_color_snap_from_vsc = 0;
    rx_color_snap_format = 0;
    rx_color_snap_colorimetry = 0;
    rx_color_snap_depth = 0;
    rx_color_snap_range = 0;
    msa_snap_htotal_h = 0;
    msa_snap_htotal_l = 0;
    msa_snap_hact_h = 0;
    msa_snap_hact_l = 0;
    msa_snap_vtotal_h = 0;
    msa_snap_vtotal_l = 0;
    msa_snap_vact_h = 0;
    msa_snap_vact_l = 0;
    rx_eq_fail1 = 0;
    rx_state = 1;
}

/* TX state and the VidChk timing cleared, TxState 1. */
void tx_state_reset(void)
{
    mipi_cphy = 0;
    mipi_out_csi = 0;
    mipi_port_num = 0;
    mipi_lane_num = 0;
    dphy_clk_burst = 0;
    dsi_mode = 0;
    mipi_fmt = 0;
    mipi_port_mode = 0;
    mipi_bpp = 0;
    tx_restart_req = 0;
    tx_video_detected = 0;
    tx_rx_stable = 0;
    mipi_csi_c8a0_bit6 = 0;
    mipi_pll_src = 0;
    tx_dsc_detected = 0;
    mipi_in_clk = 0;
    mipi_phy_clk = 0;
    lmtx_read_clk = 0;
    mipi_host_gate_en = 0;
    mipi_out_enabled = 0;
    vidchk_clear();
    tx_state = 1;
}

/* Audio state cleared, audio stage 1. */
void audio_state_reset(void)
{
    audio_rx_stable = 0;
    audio_in_sel = 0;
    audio_out_sel = 0;
    audio_info_valid = 0;
    audio_channels = 0;
    audio_word_length = 0;
    audio_rate_h = 0;
    audio_rate_l = 0;
    audio_host_gate_en = 0;
    audio_gate_latch = 0;
    audio_stage = 1;
}

/* RX, TX and audio state machines to their first state, then
 * HDCP. */
void state_reset(void)
{
    rx_state_reset();
    tx_state_reset();
    audio_state_reset();
    hdcp_init();
}
