// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* DP AUX: the request handler, the DPCD register window and the DPCD
 * initial values.
 *
 * The AUX controller (page C2) answers most requests in hardware from the
 * DPCD register file (page C3) and the EDID RAM. The firmware sees each
 * request in INT0 (status bit 0 native AUX, status2 bit 4 I2C-over-AUX) and
 * aux_request() pops its header from AUX_FIFO into IRAM: aux_hdr (command |
 * address 19:16), aux_cmd, aux_addr[3], aux_len and up to 16 data bytes.
 * Those IRAM bytes stay valid after the call (code that runs after the
 * handler in INT0 may read them). Then:
 *   - native read (0x90) of DPCD 0x00100..0x00103 or 0x00120: the reply
 *     bytes for 0x00102/0x00103/0x00120 are patched (C3:9E index, C3:9F
 *     value) with the values read back through the DPCD window;
 *   - native write (0x80) to the same registers: the written link training
 *     values are read back; TRAINING_PATTERN_SET 0x00102 bits 3:0 go to
 *     aux_t2_stage, and a pattern of 2 or more on a write starting at
 *     0x00102/0x00103 restarts the Timer 2 AUX work (aux_t2_work);
 *     FEC_CONFIGURATION 0x00120 is kept in fec_config/fec_ready;
 *   - I2C-over-AUX MOT read (0x50) of 7-bit addresses 0x52, 0x58, 0x5C:
 *     fixed 3-byte replies; any other I2C command than a MOT read: an
 *     empty reply. EDID (0x50) is served by the hardware.
 *
 * The DPCD window (C3:03..05 address, C3:53 write data, C3:54 read data)
 * reaches the whole DPCD; dpcd_page() selects bits 15:8 (bits 19:16 = 0)
 * and dpcd_write() writes one register of that page. dpcd_init() fills the
 * receiver capabilities (DPCD 0x00000.. and the extended copy 0x02200..),
 * the DSC and FEC capabilities, the sink OUI and the sink count.
 *
 * The other INT0 helpers here drive the DP RX PHY (pages E1, E2, E4, E8,
 * CB) from RX_LINK_CFG bits 1:0 and DPCD TEST_SINK 0x00270.
 */
#include <stdbool.h>

#include "fw.h"

/* dpcd_init()'s capability bytes, shared with dpcd_write_rx_caps(). */
static uint8_t init_caps;       /* dpcd_init() argument */
static uint8_t init_lane_flags; /* MAX_LANE_COUNT bits 7:6 */
static uint8_t init_spread;     /* MAX_DOWNSPREAD bit 0 */
static uint8_t init_tps4;       /* MAX_DOWNSPREAD bit 7 */

/* DSC capabilities, DPCD 0x00060..0x0006F. */
static const __code uint8_t dsc_caps[16] = {
    0x01, 0x21, 0x00, 0x14, 0x2B, 0x02, 0x01, 0x80,
    0x00, 0x1F, 0x06, 0x11, 0x08, 0x00, 0x00, 0x04,
};

/* Sink OUI and device identification, DPCD 0x00400..0x0040B. */
static const __code uint8_t sink_id[12] = {
    0xC8, 0xA9, 0x13, 'S', 'Y', 'N', 'A', 'S', ' ', 0x00, 0x00, 0x00,
};

/* Write n consecutive registers of the current DPCD page from a table. */
static void dpcd_write_run(uint8_t reg, const __code uint8_t *v, uint8_t n)
{
    do
        dpcd_write(reg++, *v++);
    while (--n);
}

/* Receiver capabilities 0x01..0x0E of the current page (0x0000 or the
 * extended copy 0x2200). ext_known: the caller already saw dpcd_ext_caps_on() != 0,
 * so it is not asked again. */
static void dpcd_write_rx_caps(uint8_t rate, bool ext_known)
{
    dpcd_write(0x01, rate);
    dpcd_write_max_lane_count(init_lane_flags);
    dpcd_write(0x03, (ext_known || dpcd_ext_caps_on()) ? (init_spread | init_tps4) : init_spread);
    dpcd_write(0x04, 0x01);
    dpcd_write(0x05, 0x00);
    dpcd_write(0x06, 0x01);
    dpcd_write(0x07, (init_caps & 0x01) ? 0xC0 : 0x80);
    dpcd_write(0x08, (init_caps & 0x02) ? 0x2A : 0x02);
    dpcd_write(0x09, 0x0F);
    dpcd_write(0x0A, 0x04);
    dpcd_write(0x0B, 0x00);
    dpcd_write(0x0C, 0x00);
    dpcd_write(0x0D, 0x00);
    dpcd_write(0x0E, (ext_known || dpcd_ext_caps_on()) ? 0x81 : 0x01);
}

/* Write the DPCD initial values. caps bit 0: MSA timing ignored
 * (0x00007 bit 6, 0x02214 bit 0); bit 1: 0x00008 = 0x2A instead of 0x02;
 * bit 2: DSC capabilities; bit 3: FEC capabilities. Link rate, lane count,
 * downspread and TPS3/TPS4 come from rx_max_link_rate_h..rx_tps4; dpcd_ext_caps_on() != 0
 * enables the extended receiver capabilities at 0x02200 and TPS4. */
void dpcd_init(uint8_t caps)
{
    uint8_t i;

    init_caps = caps;
    init_spread = rx_downspread ? 0x01 : 0x00;
    init_lane_flags = rx_tps3 ? 0xC0 : 0x80;
    init_tps4 = rx_tps4 ? 0x80 : 0x00;

    for (i = 0; i < 2; i++) {
        if (i == 0) {
            /* DPCD_REV 1.2; MAX_LINK_RATE 8.1 Gb/s is reported as 5.4. */
            dpcd_page(0x00);
            dpcd_write(0x00, 0x12);
            dpcd_write_rx_caps((rx_max_link_rate_h == 0 && rx_max_link_rate_l == 0x1E) ? 0x14 : rx_max_link_rate_l, false);
        } else {
            /* Extended copy: DPCD_REV 1.4 and the real link rate. */
            dpcd_page(0x22);
            dpcd_write(0x00, 0x14);
            dpcd_write_rx_caps(rx_max_link_rate_l, false);
        }
    }

    i = dpcd_ext_caps_on();
    dpcd_page(0x22);
    if (i == 0) {
        dpcd_write(0x00, 0x00);
        dpcd_write(0x01, 0x00);
        dpcd_write(0x02, 0x00);
        dpcd_write(0x06, 0x00);
        dpcd_write(0x07, 0x00);
        dpcd_write(0x0E, 0x00);
    } else {
        dpcd_write(0x00, 0x14);
        dpcd_write_rx_caps(rx_max_link_rate_l, true);
        dpcd_write(0x10, 0x08);
    }

    dpcd_page(0x00);
    if (caps & 0x04)
        dpcd_write_run(0x60, dsc_caps, sizeof(dsc_caps));
    else
        dpcd_write(0x60, 0x00);
    dpcd_write(0x90, (caps & 0x08) ? 0xBF : 0x00);
    dpcd_write(0x21, 0x00);
    dpcd_write(0x23, 0x00);
    dpcd_write(0x80, 0x00);

    dpcd_page(0x04);
    dpcd_write_run(0x00, sink_id, sizeof(sink_id));

    dpcd_page(0x05);
    dpcd_write(0x10, 0xBE);
    dpcd_write(0x11, 0x05);

    dpcd_page(0x01);
    dpcd_write(0x0A, 0x00);

    dpcd_page(0x02);
    dpcd_write(0x00, 0x01); /* SINK_COUNT */

    dpcd_page(0x20);
    dpcd_write(0x02, 0x01); /* SINK_COUNT_ESI */

    i = dpcd_ext_caps_on();
    dpcd_page(0x22);
    dpcd_write(0x10, i ? 0x08 : 0x00);
    dpcd_write(0x14, (caps & 0x01) ? 0x01 : 0x00);
}

/* Patch reply byte aux_reply_i with the DPCD register at 0x001xx. */
static void aux_reply_dpcd(uint8_t reg)
{
    dpcd_page(0x01);
    DPCD_ADDR_L = reg;
    R_C3_9E = aux_reply_i;
    R_C3_9F = DPCD_RDATA;
}

/* Native read: clear the 16 reply patch slots, note a read of 0x0303C in
 * host_marker, then patch the link training registers 0x00102/0x00103 and
 * 0x00120 when the request covers them. */
static void aux_native_read(void)
{
    for (aux_i = 0; aux_i < 16; aux_i++) {
        R_C3_9E = aux_i;
        R_C3_9F = 0x00;
    }
    if (aux_addr[0] == 0x00 && aux_addr[1] == 0x30 && aux_addr[2] == 0x3C)
        host_marker |= HOST_MARK_DPCD_0303C;
    if (aux_addr[0] != 0x00 || aux_addr[1] != 0x01)
        return;
    aux_reply_i = 0;
    switch (aux_addr[2]) {
    case 0x00:
        if (++aux_reply_i == aux_len)
            return;
        /* fall through */
    case 0x01:
        if (++aux_reply_i == aux_len)
            return;
        /* fall through */
    case 0x02:
        aux_reply_dpcd(0x02);
        if (++aux_reply_i == aux_len)
            return;
        /* fall through */
    case 0x03:
        aux_reply_dpcd(0x03);
        return;
    case 0x20:
        aux_reply_dpcd(0x20);
        return;
    }
}

/* Native write: read back the link training registers the source wrote. */
static void aux_native_write(void)
{
    if (aux_addr[0] != 0x00 || aux_addr[1] != 0x01)
        return;
    aux_reply_i = 0;
    switch (aux_addr[2]) {
    case 0x00:
        if (++aux_reply_i == aux_len)
            return;
        /* fall through */
    case 0x01:
        if (++aux_reply_i == aux_len)
            return;
        /* fall through */
    case 0x02:
        dpcd_page(0x01);
        DPCD_ADDR_L = 0x02;
        aux_t2_stage = DPCD_RDATA & 0x0F;
        if (++aux_reply_i == aux_len)
            return;
        /* fall through */
    case 0x03:
        dpcd_page(0x01);
        DPCD_ADDR_L = 0x03;
        (void)DPCD_RDATA; /* read, the value unused */
        if (rx_int0_bit7_flag == 1 && (DPCD_RDATA & 0x03) <= 1) {
            R_C3_EC &= 0xFD;
            dpcd_page(0x01);
            DPCD_ADDR_L = 0x02;
            DPCD_WDATA = 0x20;
            DPCD_WDATA = 0x21;
            delay_short(100);
            R_C3_EC |= 0x02;
        }
        if (aux_t2_stage < 2 || aux_reply_i != 0)
            return;
        aux_t2_count = 0;
        t2_arm_short();
        return;
    case 0x20:
        dpcd_page(0x01);
        DPCD_ADDR_L = 0x20;
        fec_config = DPCD_RDATA;
        fec_ready = fec_config & 0x01;
        return;
    }
}

/* Load a 3-byte I2C-over-AUX reply: 0x00, a, b. */
static void aux_i2c_reply(uint8_t a, uint8_t b)
{
    R_C2_11 = 0x00;
    R_C2_11 = a;
    R_C2_11 = b;
    R_C2_12 = 0x00;
}

/* I2C-over-AUX: note a request to 0x52 in host_marker, then reply. */
static void aux_i2c_request(void)
{
    if (aux_addr[2] == 0x52)
        host_marker |= HOST_MARK_DISPLAYID;
    R_C2_08 |= 0x02;
    if (aux_cmd == 0x50) {
        if (aux_addr[0] == 0 && aux_addr[1] == 0 && aux_addr[2] == 0x52)
            aux_i2c_reply(0x11, 0x22);
        if (aux_addr[0] == 0 && aux_addr[1] == 0 && aux_addr[2] == 0x58)
            aux_i2c_reply(0x33, 0x44);
        if (aux_addr[0] == 0 && aux_addr[1] == 0 && aux_addr[2] == 0x5C)
            aux_i2c_reply(0x55, 0x66);
    } else {
        R_C2_11 = 0x00;
        R_C2_12 = 0x00;
    }
    R_C2_08 &= 0xFD;
}

/* Handle one AUX request (i2c 0: native, 1: I2C-over-AUX). Parses
 * the request header into aux_hdr/aux_cmd/aux_addr/aux_len/aux_data; a
 * length over 16 drops the request. */
void aux_request(uint8_t i2c)
{
    aux_reply_i = 0;
    aux_fifo_count = AUX_FIFO_COUNT & 0x1F;
    aux_hdr = AUX_FIFO;
    aux_cmd = aux_hdr & 0xF0;
    aux_addr[0] = aux_hdr & 0x0F;
    aux_addr[1] = AUX_FIFO;
    aux_addr[2] = AUX_FIFO;
    if (aux_fifo_count >= 4) {
        aux_len = AUX_FIFO + 1;
        /* SECURITY: the source sets the length; this bound keeps its bytes inside aux_data. */
        if (aux_len > 16)
            return;
    } else {
        aux_len = 0;
    }
    if (aux_fifo_count > 4)
        for (aux_i = 0; aux_i < aux_len; aux_i++)
            aux_data[aux_i] = AUX_FIFO;

    if (i2c == 0) {
        if (aux_cmd == 0x90)
            aux_native_read();
        else if (aux_cmd == 0x80)
            aux_native_write();
    } else if (i2c == 1) {
        aux_i2c_request();
    }
}

/* INT0 bit 1. Program E8:0F/21/22 from the PHY test lane enable
 * (TEST_SINK 0x00270 bit 7), R_C3_D9 bit 2 and RX_LINK_CFG bits 1:0. */
void rx_phy_e8_rate_regs_set(void)
{
    dpcd_page(0x02);
    DPCD_ADDR_L = 0x70;
    if (DPCD_RDATA & 0x80) {
        switch (RX_LINK_CFG & 0x03) {
        case 0:
            R_E8_0F = 0x7B;
            R_E8_21 = 0x05;
            break;
        case 1:
            R_E8_0F = 0x7A;
            R_E8_21 = 0x05;
            break;
        case 2:
            R_E8_0F = 0x77;
            R_E8_21 = 0x08;
            break;
        default:
            R_E8_0F = 0x77;
            R_E8_21 = 0x0A;
            break;
        }
        R_E8_22 = 0x00;
    } else if (R_C3_D9 & 0x04) {
        switch (RX_LINK_CFG & 0x03) {
        case 0:
            R_E8_0F = 0x7A;
            R_E8_21 = 0x04;
            R_E8_22 = 0x82;
            break;
        case 1:
            R_E8_0F = 0x79;
            R_E8_21 = 0x05;
            R_E8_22 = 0xA2;
            break;
        case 2:
            R_E8_0F = 0x77;
            R_E8_21 = 0x05;
            R_E8_22 = 0xA2;
            break;
        default:
            R_E8_0F = 0x77;
            R_E8_21 = 0x09;
            R_E8_22 = 0x03;
            break;
        }
    } else {
        (void)RX_LINK_CFG; /* every case is the same */
        R_E8_0F = 0x57;
        R_E8_21 = 0x00;
        R_E8_22 = 0x00;
    }
}

/* INT0 bit 5. TEST_SINK 0x00270: bit 0 clear sets C1:4A = 0x30;
 * with PHY_SINK_TEST_LANE_EN (bit 7), the lane chosen by bits 5:4 is
 * looked up in E4:01 (2 bits per lane, lane 0 in bits 7:6) and written to
 * CB:14 bits 7:6, bit 7 first. */
void test_sink_update(void)
{
    uint8_t test, lane = 0, v;

    dpcd_page(0x02);
    DPCD_ADDR_L = 0x70;
    test = DPCD_RDATA;
    if (!(test & 0x01))
        R_C1_4A = 0x30;
    if (!(test & 0x80))
        return;
    R_C3_C3 |= 0x40;
    R_C3_1B = 0x03;
    R_C3_BF &= 0x7F;
    R_E8_40 = 0x55;
    R_E8_41 = 0x55;
    switch (test & 0xF0) {
    case 0x80:
        lane = (R_E4_01 >> 6) & 0x03;
        break;
    case 0x90:
        lane = (R_E4_01 >> 4) & 0x03;
        break;
    case 0xA0:
        lane = (R_E4_01 >> 2) & 0x03;
        break;
    case 0xB0:
        lane = R_E4_01 & 0x03;
        break;
    }
    v = R_CB_14;
    v = (lane & 0x02) ? (v | 0x80) : (v & 0x7F);
    R_CB_14 = v;
    R_CB_14 = (lane & 0x01) ? (v | 0x40) : (v & 0xBF);
}

/* INT0 bit 4 (with rx_link_rate_cfg_on). */
void rx_phy_link_rate_set(uint8_t on)
{
    uint8_t v;

    R_E8_6E = 0xD4;
    if (on == 0) {
        R_E8_68 = 0x44;
        R_E8_69 = 0x56;
        R_E8_0F = 0x02;
        R_E8_11 &= 0xF7;
        return;
    }
    R_E8_11 |= 0x08;
    R_E2_72 &= 0xFC;
    if ((RX_LINK_CFG & 0x03) < 2) {
        R_E2_72 |= 0x01;
        R_E4_00 |= 0x01;
        R_E8_14 |= 0x80;
        v = R_CB_14 & 0xDF;
        R_CB_14 = v;
        R_CB_14 = v | 0x10;
    } else {
        R_E2_72 |= 0x02;
        R_E4_00 &= 0xFE;
        R_E8_14 &= 0x7F;
        v = R_CB_14 & 0xDF;
        R_CB_14 = v;
        R_CB_14 = v & 0xEF;
    }
}

/* INT0 bits 6/7 and the RX state machine: set (on == 1) or clear
 * bits of E1:18..1B and CB:23 bits 3:0, then E8:12 = 0. */
void rx_e1_18_1b_cb_23_set(uint8_t on)
{
    if (on == 1) {
        R_E1_18 |= 0x2A;
        R_E1_19 |= 0xAA;
        R_E1_1A |= 0x80;
        R_E1_1B |= 0x10;
        R_CB_23 |= 0x0F;
    } else {
        R_E1_18 &= 0xD5;
        R_E1_19 &= 0x55;
        R_E1_1A &= 0x7F;
        R_E1_1B &= 0xEF;
        R_CB_23 &= 0xF0;
    }
    R_E8_12 = 0x00;
}

/* INT0 bit 4. E2:7E bit 0 set, E2:7F = rx_eq_lane_value[i] (i > 3: [3]). */
void rx_eq_band_apply(uint8_t i)
{
    R_E2_7E |= 0x01;
    R_E2_7F = rx_eq_lane_value[i > 3 ? 3 : i];
}

/* MAX_LANE_COUNT (register 0x02 of the current DPCD page) =
 * lane count rx_max_lane_count | flags (enhanced framing, TPS3). */
void dpcd_write_max_lane_count(uint8_t flags)
{
    dpcd_write(0x02, rx_max_lane_count | flags);
}

/* Write DPCD register reg of the page set by dpcd_page(). */
void dpcd_write(uint8_t reg, uint8_t v)
{
    DPCD_ADDR_L = reg;
    DPCD_WDATA = v;
}

/* Select DPCD address bits 15:8 (bits 19:16 = 0). */
void dpcd_page(uint8_t page) __reentrant
{
    DPCD_ADDR_H &= 0xF0;
    DPCD_ADDR_M = page;
}

/* dpcd_ext_caps, nonzero to advertise the extended receiver capabilities. */
uint8_t dpcd_ext_caps_on(void)
{
    return dpcd_ext_caps;
}

void dpcd_init_default(void)
{
    dpcd_init(0);
}
