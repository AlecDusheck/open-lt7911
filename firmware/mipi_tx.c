// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

#include <string.h>
#include "fw.h"

/* MIPI CSI-2/DSI transmitter: the TxState machine, the output gate, the
 * port/lane/data-type setup, the D-PHY analog setup and the DSI panel init.
 * Clock measurement, the TX PLL and the timing maths are in mipi_clk.c.
 *
 * TxState (tx_state, mirrored to the host in HOST_TX_STATE by set_tx_state)
 * advances one step per main-loop pass (tx_state_machine):
 *   1 -> 2  at once (tx_state_reset sets 1).
 *   2 -> 3  output off on a restart request, then wait for the RX side's
 *           video flag (tx_video_detected) and load the configuration
 *           defaults (CSI-2, 1 port, 4 lanes).
 *   3 -> 4  RX stable (tx_rx_stable) and a settled input clock (in_clk_settled).
 *   4 -> 5  lane rate and TX PLL set up, PLL band calibration locked.
 *   5 -> 6  timing, D-PHY, lanes, data type, Rdly programmed; always.
 *   6 -> 7  once mipi_out_enabled: output on (CSI-2 or DSI), DSI panel init.
 * Every state change raises tx_restart_req; entering 6 posts system event 5.
 * tx_output_gate runs after it in each pass: from 3 back to 2 when the
 * video flag drops, from 4 back to 3 when RX stability drops, and from 6 on
 * it follows the host's MIPI gate (HOST_MIPI_GATE, only when
 * mipi_host_gate_en): closing turns the D-PHY clock and the output off,
 * opening turns them on and returns to 6 (mipi_out_enabled set, so 6 -> 7
 * runs again).
 *
 * Registers: page C8 is the MIPI TX controller (C8:89 output type and clock
 * lane, C8:86/88 data type, C8:87 ports/lanes, C8:98 DSI packet FIFO,
 * C8:AB..AE D-PHY control); pages E2/E6/C9 hold the D-PHY analog and its
 * calibration.
 */

/* TxState values (tx_state, mirrored to HOST_TX_STATE). */
enum {
    TX_RESET = 1,       /* after tx_state_reset */
    TX_WAIT_VIDEO = 2,  /* output off; wait for the RX video flag */
    TX_WAIT_STABLE = 3, /* wait for RX stable and an input clock */
    TX_PLL = 4,         /* clocks and TX PLL; wait for the PLL band */
    TX_CONFIG = 5,      /* timing, lanes, data type, measurements */
    TX_WAIT_OUTPUT = 6, /* configured; wait for the output gate */
    TX_OUTPUT_ON = 7,   /* MIPI output running */
};

/* ---- TxState machine ---- */

/* Enter TxState s. A change raises the restart request and mirrors
 * the state to the host; entering 6 also runs tx_event_notify(3). */
void set_tx_state(uint8_t s)
{
    if (tx_state == s)
        return;
    tx_restart_req = 1;
    tx_state = s;
    HOST_TX_STATE = s;
    if (s == TX_WAIT_OUTPUT)
        tx_event_notify(3);
}

/* One step of the TxState machine (called from the main loop). */
void tx_state_machine(void)
{
    switch (tx_state) {
    case TX_RESET:
        set_tx_state(TX_WAIT_VIDEO);
        break;
    case TX_WAIT_VIDEO:
        if (tx_restart_req) {
            tx_restart_req = 0;
            mipi_output_enable(mipi_out_csi, 0);
            tx_event_notify(1);
        }
        if (tx_video_detected) {
            mipi_config_defaults();
            set_tx_state(TX_WAIT_STABLE);
        }
        break;
    case TX_WAIT_STABLE:
        if (tx_rx_stable && in_clk_settled())
            set_tx_state(TX_PLL);
        break;
    case TX_PLL:
        mipi_phy_clk_calc();
        mipi_tx_pll_setup();
        if (lmtx_pll_band())
            set_tx_state(TX_CONFIG);
        break;
    case TX_CONFIG:
        mipi_phy_timing_setup();
        tx_timing_check();
        mipi_tx_config_apply();
        mipi_phy_analog_setup();
        tx_c8_d5_bits45_enable(1);
        mipi_rdly_update();
        lmtx_read_clk_measure();
        set_tx_state(TX_WAIT_OUTPUT);
        break;
    case TX_WAIT_OUTPUT:
        if (mipi_out_enabled == 1) {
            dphy_ctl_ab_ae_pulse_if_dphy();
            tx_c8_05_bit0_clear(1);
            mipi_output_enable(mipi_out_csi, 1);
            dsi_panel_init_if_dsi();
            tx_c8_05_bit0_clear(0);
            set_tx_state(TX_OUTPUT_ON);
        }
        break;
    }
}

/* Fall back when the RX side loses video or stability, and from
 * TxState 6 on follow the host's output gate: switch the D-PHY clock and
 * the output off when it closes, back on (and to TxState 6) when it opens. */
void tx_output_gate(void)
{
    uint8_t next = 0xFF;

    if (tx_state >= TX_WAIT_STABLE && !tx_video_detected)
        set_tx_state(TX_WAIT_VIDEO);
    if (tx_state >= TX_PLL && !tx_rx_stable)
        set_tx_state(TX_WAIT_STABLE);
    if (tx_state >= TX_WAIT_OUTPUT) {
        if (!host_mipi_gate(mipi_host_gate_en, HOST_MIPI_GATE)) {
            if (mipi_out_enabled == 1) {
                mipi_out_enabled = 0;
                dphy_clk_enable(0, dphy_clk_burst);
                mipi_output_enable(mipi_out_csi, 0);
            }
        } else if (!mipi_out_enabled) {
            next = TX_WAIT_OUTPUT;
            dphy_clk_enable(1, dphy_clk_burst);
            mipi_out_enabled = 1;
        }
    }
    if (next != 0xFF)
        set_tx_state(next);
}

/* TX events towards the rest of the firmware: 1 (restart) and
 * 2 set rx_setup_gate through rx_setup_gate_set with 3 and 4; 3 (TxState 6
 * reached) posts system event 5. */
void tx_event_notify(uint8_t ev)
{
    uint8_t v;

    if (ev == 1) {
        v = 3;
    } else if (ev == 2) {
        v = 4;
    } else {
        if (ev == 3)
            sys_event_post(5);
        return;
    }
    rx_setup_gate_set(v);
}

/* rx_setup_gate (the RxState 3 -> 4 gate) = 1 for 3, 0 for 4. */
void rx_setup_gate_set(uint8_t v)
{
    if (v == 4)
        rx_setup_gate = 0;
    else if (v == 3)
        rx_setup_gate = 1;
}

/* ---- Output and D-PHY clock control ---- */

/* MIPI output off (enable 0), or on as CSI-2 (csi 1, C8:89 bit 5)
 * or DSI (csi 0, bit 4). */
void mipi_output_enable(uint8_t csi, uint8_t enable)
{
    uint8_t v;

    if (!enable) {
        R_C8_89 &= 0xCF;
    } else if (!csi) {
        v = R_C8_89 & 0xDF;
        R_C8_89 = v;
        R_C8_89 = v | 0x10;
    } else {
        v = R_C8_89 & 0xEF;
        R_C8_89 = v;
        R_C8_89 = v | 0x20;
    }
}

/* D-PHY clock lane on (C8:89 bit 7), in burst mode (burst != 0,
 * C8:AB bit 7 clear) or continuous mode (C8:AB bit 7 set). */
void dphy_clk_mode(uint8_t burst)
{
    R_C8_89 |= 0x80;
    if (burst)
        R_C8_AB &= 0x7F;
    else
        R_C8_AB |= 0x80;
}

/* D-PHY clock lane off (on 0: C8:AB bit 7 then C8:89 bit 7
 * cleared), or on as dphy_clk_mode(burst). */
void dphy_clk_enable(uint8_t on, uint8_t burst)
{
    if (!on) {
        R_C8_AB &= 0x7F;
        R_C8_89 &= 0x7F;
    } else {
        dphy_clk_mode(burst);
    }
}

/* C8:D5 bits 4 and 5: set one after the other (on), or clear bit 4
 * then bit 5 (off). */
void tx_c8_d5_bits45_enable(uint8_t on)
{
    uint8_t v;

    if (on) {
        v = R_C8_D5 & 0xCF;
        R_C8_D5 = v;
        v |= 0x10;
        R_C8_D5 = v;
        R_C8_D5 = v | 0x20;
    } else {
        v = R_C8_D5 & 0xEF;
        R_C8_D5 = v;
        R_C8_D5 = v & 0xDF;
    }
}

/* C8:05 bit 0 = (v == 0). */
void tx_c8_05_bit0_clear(uint8_t v)
{
    if (v)
        R_C8_05 &= 0xFE;
    else
        R_C8_05 |= 0x01;
}

/* ---- MIPI configuration ---- */

/* MIPI configuration defaults (CSI-2 out, 1 port, 4 lanes, host
 * gate ignored), then mipi_config_finalize. */
void mipi_config_defaults(void)
{
    mipi_cphy = 0;
    mipi_out_csi = 1;
    mipi_port_num = 1;
    mipi_lane_num = 4;
    mipi_fmt = 0x0B;
    mipi_pll_src = 0;
    dphy_clk_burst = 1;
    mipi_port_mode = 0;
    dsi_mode = 3;
    mipi_csi_c8a0_bit6 = 0;
    mipi_host_gate_en = 0;
    mipi_config_finalize();
}

/* Apply the override hook's configuration when it offers one, then
 * clamp: mipi_port_mode becomes 1 when it is 0 and there is more than one port, and
 * with mipi_cphy == 1 at most 3 lanes. */
void mipi_config_finalize(void)
{
    __xdata uint8_t cfg[31];

    memset(cfg, 0, sizeof cfg);
    if (mipi_config_get(cfg)) {
        mipi_cphy = cfg[0x00];
        mipi_out_csi = cfg[0x01];
        mipi_port_num = cfg[0x02];
        mipi_lane_num = cfg[0x03];
        dphy_clk_burst = cfg[0x04];
        mipi_port_mode = cfg[0x07];
        mipi_fmt = cfg[0x06];
        dsi_mode = cfg[0x05];
        mipi_pll_src = cfg[0x0E];
        mipi_csi_c8a0_bit6 = cfg[0x0D];
        mipi_host_gate_en = cfg[0x1D];
    }
    if (mipi_port_mode == 0 && mipi_port_num > 1)
        mipi_port_mode = 1;
    if (mipi_cphy == 1 && mipi_lane_num >= 3)
        mipi_lane_num = 3;
}

/* Override hook for the MIPI configuration: fills `cfg` (31 bytes, at the
 * offsets mipi_config_finalize reads) with a configuration when cfg is not
 * NULL, and always returns 0, so the caller never applies it. */
uint8_t mipi_config_get(__xdata uint8_t *cfg)
{
    /* {offset, value}, in write order */
    static const __code uint8_t fill[][2] = {
        {0x00, 0}, {0x01, 0}, {0x02, 1}, {0x03, 4}, {0x04, 0}, {0x07, 0},
        {0x06, 2}, {0x05, 3}, {0x0E, 0}, {0x0D, 0}, {0x0F, 0}, {0x1D, 0},
    };
    uint8_t i;

    if (cfg) {
        for (i = 0; i < sizeof fill / sizeof fill[0]; i++)
            cfg[fill[i][0]] = fill[i][1];
    }
    return 0;
}

/* ---- Port, lane and data-type setup (TxState 5) ---- */

/* Port/lane setup and data type; then, for two ports, vidchk_hactive (H
 * active) not a multiple of 4, mipi_fmt 2 or 10 and tx_dsc_detected clear: C8:7A
 * bits 6/7 on and the per-port byte count (bpp * vidchk_hactive / 8 / ports) into
 * C8:30..33 (the software word count). */
void mipi_tx_config_apply(void)
{
    uint16_t wc;

    mipi_ports_lanes_setup();
    mipi_phy_type_setup();
    mipi_out_type_setup();
    mipi_fmt_set(mipi_fmt);
    tx_c8_71_enable(0);
    tx_c8_7a_bit6_set(0);
    tx_c8_7a_bit7_set(0);
    if (tx_dsc_detected || mipi_port_num != 2 || !(vidchk_hactive & 0x03))
        return;
    if (mipi_fmt != 0x02 && mipi_fmt != 0x0A)
        return;
    wc = (uint16_t)(mipi_bpp * vidchk_hactive) >> 3;
    wc /= mipi_port_num;
    tx_c8_7a_bit7_set(1);
    tx_c8_7a_bit6_set(1);
    mipi_word_count_set(wc, wc);
}

/* Look fmt up in mipi_fmt_table (15 rows of 5 bytes) and program
 * its columns 3 and 2 through mipi_data_type_set; 0x3E, 0x21 when absent. */
void mipi_fmt_set(uint8_t fmt)
{
    const __xdata uint8_t *row = mipi_fmt_table;
    uint8_t n = 15, a = 0x3E, b = 0x21;

    do {
        if (row[0] == fmt) {
            a = row[3];
            b = row[2];
            break;
        }
        row += 5;
    } while (--n);
    mipi_data_type_set(a, b);
}

/* C8:86[5:0] = a, C8:88[6:0] = b. */
void mipi_data_type_set(uint8_t a, uint8_t b)
{
    R_C8_86 = (R_C8_86 & 0xC0) | (a & 0x3F);
    R_C8_88 = (R_C8_88 & 0x80) | (b & 0x7F);
}

/* Lane map, port count and lane count. */
void mipi_ports_lanes_setup(void)
{
    mipi_lane_map_set();
    tx_set_c8_0d(mipi_port_num == 2 ? 1 : 3);
    mipi_port_count_set(mipi_port_num);
    mipi_lane_count_set(mipi_lane_num);
    mipi_port_mode_set(mipi_port_mode);
}

/* mipi_cphy == 1: C8:0F bit 5 and C8:89 bit 6 on; otherwise both off,
 * C8:AA = 0xAA, D-PHY clock mode dphy_clk_burst and C8:A0 bit 7 on. */
void mipi_phy_type_setup(void)
{
    if (mipi_cphy == 1) {
        tx_c8_0f_bit5_set(1);
        mipi_cphy_select(1);
        return;
    }
    tx_c8_0f_bit5_set(0);
    mipi_cphy_select(0);
    tx_c8_aa_write_aa();
    dphy_clk_mode(dphy_clk_burst);
    tx_c8_a0_bit7_on(1);
}

/* DSI: C8:97[1:0] = dsi_mode, C8:70 bit 3 off; CSI-2: C8:A0 bit 6 =
 * mipi_csi_c8a0_bit6 (0 when mipi_cphy is set), C8:70 bit 3 on. */
void mipi_out_type_setup(void)
{
    if (!mipi_out_csi) {
        dsi_mode_set(dsi_mode);
        tx_c8_70_bit3_set(0);
    } else {
        tx_c8_a0_bit6_set(mipi_cphy ? 0 : mipi_csi_c8a0_bit6);
        tx_c8_70_bit3_set(1);
    }
}

/* C8:30..33 = a (high, low), b (high, low). */
void mipi_word_count_set(uint16_t a, uint16_t b)
{
    R_C8_30 = a >> 8;
    R_C8_31 = a;
    R_C8_32 = b >> 8;
    R_C8_33 = b;
}

/* C8:08..0C = 10 32 04 21 43. */
void mipi_lane_map_set(void)
{
    R_C8_08 = 0x10;
    R_C8_09 = 0x32;
    R_C8_0A = 0x04;
    R_C8_0B = 0x21;
    R_C8_0C = 0x43;
}

/* C8:87[2:0] = 2 for two or more ports, else 1. */
void mipi_port_count_set(uint8_t ports)
{
    uint8_t v = R_C8_87 & 0xF8;

    if (ports < 2)
        v |= 0x01;
    else
        v |= 0x02;
    R_C8_87 = v;
}

/* C8:0D = v. */
void tx_set_c8_0d(uint8_t v)
{
    R_C8_0D = v;
}

/* C8:89[1:0] = lane count. */
void mipi_lane_count_set(uint8_t lanes)
{
    R_C8_89 = (R_C8_89 & 0xFC) | (lanes & 0x03);
}

/* C8:87[6:4] = v. */
void mipi_port_mode_set(uint8_t v)
{
    R_C8_87 = (R_C8_87 & 0x8F) | ((v << 4) & 0x70);
}

/* C8:97[1:0] = v. */
void dsi_mode_set(uint8_t v)
{
    R_C8_97 = (R_C8_97 & 0xFC) | (v & 0x03);
}

/* C8:AA = 0xAA. */
void tx_c8_aa_write_aa(void)
{
    R_C8_AA = 0xAA;
}

/* v == 1 sets C8:A0 bit 7 (nothing otherwise). */
void tx_c8_a0_bit7_on(uint8_t v)
{
    if (v == 1)
        R_C8_A0 |= 0x80;
}

/* C8:71 = 0x13 (on) or 0. */
void tx_c8_71_enable(uint8_t on)
{
    R_C8_71 = on ? 0x13 : 0x00;
}

/* One register bit set when v == 1, cleared otherwise. */

/* C8:0F bit 5. */
void tx_c8_0f_bit5_set(uint8_t v)
{
    if (v == 1)
        R_C8_0F |= 0x20;
    else
        R_C8_0F &= 0xDF;
}

/* C8:89 bit 6. */
void mipi_cphy_select(uint8_t v)
{
    if (v == 1)
        R_C8_89 |= 0x40;
    else
        R_C8_89 &= 0xBF;
}

/* C8:A0 bit 6. */
void tx_c8_a0_bit6_set(uint8_t v)
{
    if (v == 1)
        R_C8_A0 |= 0x40;
    else
        R_C8_A0 &= 0xBF;
}

/* C8:70 bit 3. */
void tx_c8_70_bit3_set(uint8_t v)
{
    if (v == 1)
        R_C8_70 |= 0x08;
    else
        R_C8_70 &= 0xF7;
}

/* C8:7A bit 7. */
void tx_c8_7a_bit7_set(uint8_t v)
{
    if (v == 1)
        R_C8_7A |= 0x80;
    else
        R_C8_7A &= 0x7F;
}

/* C8:7A bit 6. */
void tx_c8_7a_bit6_set(uint8_t v)
{
    if (v == 1)
        R_C8_7A |= 0x40;
    else
        R_C8_7A &= 0xBF;
}

/* ---- D-PHY analog setup and calibration (TxStates 5 and 6) ----
 *
 * mipi_phy_analog_setup: E2:9B..E3 analog setup for the two TX ports
 * (P0 E2:9D..A5, P1 E2:AF..B7), in one of two modes chosen by mipi_cphy, then
 * rprd_cal runs each port's RP/RD calibration (C9:20 start, C9:25/C9:28 done
 * bit 6 and result, copied to E2:A4/A5 and E2:B6/B7).
 */

/* dphy_ctl_ab_ae_pulse when mipi_cphy is 0. */
void dphy_ctl_ab_ae_pulse_if_dphy(void)
{
    if (!mipi_cphy)
        dphy_ctl_ab_ae_pulse();
}

/* C8:AB..AE sequence with a 5 ms hold. */
void dphy_ctl_ab_ae_pulse(void)
{
    uint8_t v = R_C8_AB | 0x0F;

    R_C8_AB = v;
    R_C8_AC = 0x18;
    R_C8_AD = 0;
    R_C8_AE = 0x10;
    R_C8_AB = v | 0x10;
    delay_ms(5);
    R_C8_AB &= 0xEF;
    R_C8_AE = 0;
}

/* D-PHY analog setup and calibration of both TX ports, in the
 * mode mipi_cphy selects (1 or not). */
void mipi_phy_analog_setup(void)
{
    mipi_phy_analog_type_set(mipi_cphy);
    if (mipi_cphy == 1) {
        cphy_ports_analog_setup(1, 1);
        cphy_ports_ab_bd_bits76_set(1, 1);
    } else {
        dphy_e2_c7c8_e2e3_set();
        dphy_ports_analog_setup(1, 1);
        dphy_ports_ab_bd_bits54_set(1, 1);
    }
    rprd_cal(1, 1);
    mipi_phy_analog_enable(1);
}

/* E2:9B bit 5 = (x == 1). */
void mipi_phy_analog_type_set(uint8_t x)
{
    if (x == 1)
        R_E2_9B |= 0x20;
    else
        R_E2_9B &= 0xDF;
}

/* Ports P0 (en0) and P1 (en1) analog setup, mode 1. */
void cphy_ports_analog_setup(uint8_t en0, uint8_t en1)
{
    mipi_port0_analog_setup(1, en0);
    mipi_port1_analog_setup(1, en1);
}

/* Ports P0 (en0) and P1 (en1) analog setup, mode 0. */
void dphy_ports_analog_setup(uint8_t en0, uint8_t en1)
{
    mipi_port0_analog_setup(0, en0);
    mipi_port1_analog_setup(0, en1);
}

/* Port P0 analog setup (E2:9D..AA, E6:3B bit 6); en == 1 selects
 * the enabled values. */
void mipi_port0_analog_setup(uint8_t mode, uint8_t en)
{
    if (!mode) {
        if (en == 1) {
            R_E2_9D = 0x54;
            R_E2_A0 = 0x88;
            R_E2_A2 = 0x41;
            R_E2_A3 = 0x03;
            R_E2_9E = 0x88;
            R_E2_9F = 0x88;
            R_E6_3B |= 0x40;
        } else {
            R_E2_9D = 0x10;
            R_E2_A0 = 0x00;
            R_E2_A2 = 0x40;
            R_E2_A3 = 0x00;
            R_E2_9E = 0x00;
            R_E2_9F = 0x00;
            R_E6_3B &= 0xBF;
        }
        return;
    }
    if (en == 1) {
        R_E2_AA = 0x07;
        R_E2_9D = 0x50;
        R_E2_A0 = 0x08;
        R_E2_A9 = 0x31;
        R_E6_3B |= 0x40;
    } else {
        R_E2_AA = 0x00;
        R_E2_9D = 0x10;
        R_E2_A0 = 0x00;
        R_E2_A9 = 0x11;
        R_E6_3B &= 0xBF;
    }
    R_E2_A3 = 0;
}

/* Port P1 analog setup (E2:AF..BC, E6:3B bit 5), as mipi_port0_analog_setup. */
void mipi_port1_analog_setup(uint8_t mode, uint8_t en)
{
    if (!mode) {
        if (en == 1) {
            R_E2_AF = 0x54;
            R_E2_B2 = 0x88;
            R_E2_B4 = 0x41;
            R_E2_B5 = 0x03;
            R_E2_B0 = 0x88;
            R_E2_B1 = 0x88;
            R_E6_3B |= 0x20;
        } else {
            R_E2_AF = 0x10;
            R_E2_B2 = 0x00;
            R_E2_B4 = 0x40;
            R_E2_B5 = 0x00;
            R_E2_B0 = 0x00;
            R_E2_B1 = 0x00;
            R_E6_3B &= 0xDF;
        }
        return;
    }
    if (en == 1) {
        R_E2_BC = 0x07;
        R_E2_AF = 0x50;
        R_E2_B2 = 0x08;
        R_E2_BB = 0x31;
        R_E6_3B |= 0x20;
    } else {
        R_E2_BC = 0x00;
        R_E2_AF = 0x10;
        R_E2_B2 = 0x00;
        R_E2_BB = 0x11;
        R_E6_3B &= 0xDF;
    }
    R_E2_B5 = 0;
}

/* E2:AB / E2:BD bits 5..4 = 1 for the ports selected. */
void dphy_ports_ab_bd_bits54_set(uint8_t p0, uint8_t p1)
{
    if (p0)
        R_E2_AB = (R_E2_AB & 0xCF) | 0x10;
    if (p1)
        R_E2_BD = (R_E2_BD & 0xCF) | 0x10;
}

/* E2:AB / E2:BD bits 7..6 = 1 for the ports selected. */
void cphy_ports_ab_bd_bits76_set(uint8_t p0, uint8_t p1)
{
    if (p0)
        R_E2_AB = (R_E2_AB & 0x3F) | 0x40;
    if (p1)
        R_E2_BD = (R_E2_BD & 0x3F) | 0x40;
}

/* E2:C7/C8 and E2:E2/E3 for mode 0. */
void dphy_e2_c7c8_e2e3_set(void)
{
    R_E2_C7 = 0x01;
    R_E2_C8 = 0xAA;
    R_E2_E2 = 0x01;
    R_E2_E3 = 0xAA;
}

/* RP/RD calibration of port P0 (p0 != 0) then P1 (p1 != 0):
 * start it (C9:20 = 2 / 1), poll the done bit up to 10 ms, copy the
 * results into E2:A4/A5 (P0) or E2:B6/B7 (P1). C9:20 ends 0. */
void rprd_cal(uint8_t p0, uint8_t p1)
{
    uint8_t n, v, rp;

    if (p0) {
        R_C9_20 = 0;
        R_C9_20 = 2;
        for (n = 0;;) {
            delay_ms(1);
            if (n++ >= 10)
                break;
            v = R_C9_25;
            if (v & 0x40) {
                rp = R_C9_26 & 0x1F;
                R_E2_A4 = rp;
                R_E2_A5 = v & 0x1F;
                break;
            }
        }
    }
    if (p1) {
        R_C9_20 = 0;
        R_C9_20 = 1;
        for (n = 0;;) {
            delay_ms(1);
            if (n++ >= 10)
                break;
            v = R_C9_28;
            if (v & 0x40) {
                rp = R_C9_29 & 0x1F;
                R_E2_B6 = rp;
                R_E2_B7 = v & 0x1F;
                break;
            }
        }
    }
    R_C9_20 = 0;
}

/* E2:9B bits 3/4 and E6:3A bit 0 on (x != 0) or off. */
void mipi_phy_analog_enable(uint8_t x)
{
    uint8_t v;

    if (x) {
        v = R_E2_9B | 0x08;
        R_E2_9B = v;
        R_E6_3A |= 0x01;
        v &= 0xEF;
        R_E2_9B = v;
        R_E2_9B = v | 0x10;
    } else {
        v = R_E2_9B & 0xEF;
        R_E2_9B = v;
        R_E2_9B = v & 0xF7;
        R_E6_3A &= 0xFE;
    }
}

/* ---- DSI panel init (TxState 6 -> 7, DSI output only) ----
 *
 * dsi_panel_init sends a fixed command list through the C8:98 packet FIFO
 * (dsi_send_packet builds long and short DSI packets), then exit-sleep and
 * display-on.
 */

/* DSI data types the packet sender distinguishes (MIPI DSI spec). */
#define DSI_DCS_SHORT_WRITE   0x05
#define DSI_DCS_SHORT_WRITE_1 0x15
#define DSI_GENERIC_LONG      0x29
#define DSI_DCS_LONG_WRITE    0x39

/* Panel init commands, sent in order by dsi_panel_init. Each record is
 * {payload length, DSI data type, payload...}; a zero length ends the list.
 * The payloads are Himax HX8399 commands (B9 FF 83 99 = SETEXTC unlock). */
static const __code uint8_t panel_init_cmds[] = {
    4, 0x39, 0xb9, 0xff, 0x83, 0x99,
    3, 0x39, 0xba, 0x63, 0x23,
    2, 0x15, 0xd2, 0x11,
    12, 0x39, 0xb1, 0x00, 0x04, 0x68, 0x88, 0x01, 0x32, 0x33, 0x11, 0x11, 0x4d, 0x57,
    16, 0x39, 0xb2, 0x00, 0x80, 0x80, 0x45, 0x05, 0x07, 0xbc, 0x11, 0x00, 0x00, 0x10, 0x1f, 0x70, 0x03, 0xc4,
    46, 0x39, 0xb4, 0x00, 0xff, 0x02, 0xa7, 0x08, 0x00, 0x08, 0x00, 0x06, 0x00, 0x08, 0x0a, 0x00, 0x30, 0x05, 0x0f, 0x0a, 0x65, 0x03, 0x03, 0x00, 0x14, 0xff, 0x03, 0x02, 0xa7, 0x08, 0x00, 0x08, 0x00, 0x06, 0x00, 0x08, 0x0a, 0x00, 0x3b, 0x05, 0x0f, 0x0a, 0x03, 0x00, 0x14, 0xff, 0x02, 0x00,
    34, 0x39, 0xd3, 0x00, 0x0c, 0x03, 0x03, 0x00, 0x00, 0x14, 0x04, 0x32, 0x10, 0x09, 0x00, 0x09, 0x32, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x02, 0x02, 0x13, 0x00, 0x00, 0x00, 0x0a, 0x40,
    33, 0x39, 0xd5, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x21, 0x20, 0x19, 0x19, 0x18, 0x18, 0x18, 0x18, 0x03, 0x02, 0x01, 0x00, 0x18, 0x18, 0x18, 0x18, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x18, 0x18, 0x18, 0x18,
    33, 0x39, 0xd6, 0x18, 0x18, 0x18, 0x18, 0x40, 0x40, 0x20, 0x21, 0x18, 0x18, 0x19, 0x19, 0x18, 0x18, 0x00, 0x01, 0x02, 0x03, 0x18, 0x18, 0x40, 0x40, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x18, 0x18, 0x40, 0x40,
    17, 0x39, 0xd8, 0xaa, 0xae, 0xaa, 0xaa, 0xaa, 0xae, 0xaa, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0x15, 0xbd, 0x01,
    17, 0x39, 0xd8, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0x15, 0xbd, 0x02,
    9, 0x39, 0xd8, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xfb,
    2, 0x15, 0xbd, 0x00,
    3, 0x39, 0xc7, 0x00, 0x88,
    3, 0x39, 0xc6, 0xff, 0xf9,
    55, 0x39, 0xe0, 0x08, 0x23, 0x27, 0x22, 0x4a, 0x51, 0x5d, 0x58, 0x61, 0x6c, 0x74, 0x7b, 0x7f, 0x88, 0x8f, 0x92, 0x97, 0xa0, 0xa2, 0xaa, 0x9e, 0xac, 0xb1, 0x5c, 0x58, 0x65, 0x73, 0x08, 0x23, 0x27, 0x22, 0x4a, 0x51, 0x5d, 0x58, 0x61, 0x6c, 0x74, 0x7b, 0x7f, 0x88, 0x8f, 0x92, 0x97, 0xa0, 0xa2, 0xaa, 0x9e, 0xac, 0xb1, 0x5c, 0x58, 0x65, 0x73,
    2, 0x15, 0xcc, 0x08,
    2, 0x15, 0xcb, 0x0a,
    5, 0x39, 0xcf, 0x00, 0x14, 0x00, 0x80,
    0
};

/* DCS exit_sleep_mode and set_display_on (second byte: unused parameter). */
static const __code uint8_t dcs_exit_sleep[] = { 0x11, 0x00 };
static const __code uint8_t dcs_display_on[] = { 0x29, 0x00 };

/* Send the panel init commands when the output is DSI. */
void dsi_panel_init_if_dsi(void)
{
    if (!mipi_out_csi)
        dsi_panel_init();
}

/* DSI panel init: packet engine setup, the command list, then
 * exit sleep and display on. */
void dsi_panel_init(void)
{
    const __code uint8_t *p;

    dsi_packet_setup();
    tx_c8_84_bit3_set(0);
    tx_set_c8_0f_low5(1);
    tx_set_c8_0e(1);
    dsi_packet_nowait_set(0);
    for (p = panel_init_cmds; *p; p += *p + 2)
        dsi_send_packet_wait(p[1], p[0], p + 2);
    dsi_send_packet_wait(DSI_DCS_SHORT_WRITE, 2, dcs_exit_sleep);
    delay_ms(100);
    dsi_send_packet_wait(DSI_DCS_SHORT_WRITE, 2, dcs_display_on);
    delay_ms(50);
    dsi_packet_finish();
}

/* DSI packet engine setup before the panel commands. */
void dsi_packet_setup(void)
{
    R_C8_B6 = 0x10;
    R_C8_A3 = 0x02;
    R_C8_9A = 0xC0;
    R_C8_9C = 0x10;
}

/* Send one DSI packet; wait unless C8:AB bit 6 is set. */
void dsi_send_packet_wait(uint8_t type, uint8_t len, const __code uint8_t *payload)
{
    dsi_send_packet(type, len, payload);
    if (!(R_C8_AB & 0x40))
        delay_short(250);
}

/* Write one DSI packet into the C8:98 FIFO. Long writes (0x29,
 * 0x39): type, word count (len, 0), payload; C8:99 = len + 6 (header and
 * CRC). Anything else is a short packet: type and two payload bytes. */
void dsi_send_packet(uint8_t type, uint8_t len, const __code uint8_t *payload)
{
    uint8_t i;

    R_C8_D5 &= 0x7F;
    R_C8_A0 &= 0xEF;
    if (type == DSI_GENERIC_LONG || type == DSI_DCS_LONG_WRITE) {
        R_C8_9A = 0xE0;
        R_C8_99 = len + 6;
        R_C8_98 = type;
        R_C8_98 = len;
        R_C8_98 = 0;
        for (i = 0; i < len; i++)
            R_C8_98 = *payload++;
        delay_short(250);
    } else {
        R_C8_9A = 0xC0;
        R_C8_99 = 4;
        R_C8_98 = type;
        R_C8_98 = payload[0];
        R_C8_98 = payload[1];
    }
}

/* C8:84 bit 3. */
void tx_c8_84_bit3_set(uint8_t on)
{
    if (on)
        R_C8_84 |= 0x08;
    else
        R_C8_84 &= 0xF7;
}

/* C8:AB bit 6 (set: no wait after each DSI packet). */
void dsi_packet_nowait_set(uint8_t on)
{
    if (on)
        R_C8_AB |= 0x40;
    else
        R_C8_AB &= 0xBF;
}

/* C8:0F[4:0] = x. */
void tx_set_c8_0f_low5(uint8_t x)
{
    R_C8_0F = (R_C8_0F & 0xE0) | (x & 0x1F);
}

/* C8:0E = x. */
void tx_set_c8_0e(uint8_t x)
{
    R_C8_0E = x;
}

/* End of the panel init: C8:9A = 0, C8:AB bit 6 on. */
void dsi_packet_finish(void)
{
    R_C8_9A = 0;
    R_C8_AB |= 0x40;
}
