// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

#include <string.h>
#include "fw.h"

/* MIPI TX clocks, PLL and output timing: the work behind TxStates 3..5
 * (the state machine itself is in mipi_tx.c).
 *
 * TxState 3 waits for a settled input clock: in_clk_settled measures it with
 * clock meter 1 into mipi_in_clk and returns 1 after 20 matching readings.
 * TxState 4 derives the per-lane rate mipi_phy_clk from it (mipi_phy_clk_calc: bpp, lanes,
 * ports, float rounding to 25000 steps), programs the TX PLL dividers on
 * page E3 (mipi_tx_pll_setup -> mipi_pll_configure -> mipi_pll) and runs the PLL band
 * calibration (lmtx_pll_band, C8:11 / C9:96 / C9:55) until it locks.
 * TxState 5 copies the measured RX timing into the TX timing generator
 * (tx_timing_check, page C8, float maths for the free-run timing), sets the
 * D-PHY timing registers from the lane rate (mipi_phy_timing_setup, tx_c8_d4_dd_fixed_set), computes the
 * read delay Rdly (mipi_rdly_update) and measures the TX read clock (lmtx_read_clk_measure).
 *
 * Clock meters: page E5 has two with the same register layout, unit 0 at
 * E5:40 and unit 1 at E5:50: offset 0x0 is the control register (the source
 * in the low bits, bit 7 latches the count while it is read), 0xA bit 5 is
 * "measurement done", 0xB[3:0]:0xC:0xD is the 20-bit count.
 *
 * All the image's float use is here (mipi_phy_clk_calc, ceil_f, tx_freerun_hact,
 * tx_timing_check).
 */

/* ---- Clock measurement (TxStates 3 and 5) ---- */

/* Latch and read the 20-bit count of clock meter `unit` (0 or 1);
 * any other unit returns 0 without touching the hardware. */
uint32_t clk_meter_read(uint8_t unit)
{
    uint8_t hi, mid, lo;

    if (unit == 0) {
        CLK_METER0_CTL |= 0x80;
        hi = CLK_METER0_CNT_H & 0x0F;
        mid = CLK_METER0_CNT_M;
        lo = CLK_METER0_CNT_L;
        CLK_METER0_CTL &= 0x7F;
    } else if (unit == 1) {
        CLK_METER1_CTL |= 0x80;
        hi = CLK_METER1_CNT_H & 0x0F;
        mid = CLK_METER1_CNT_M;
        lo = CLK_METER1_CNT_L;
        CLK_METER1_CTL &= 0x7F;
    } else {
        return 0;
    }
    return ((uint32_t)hi << 16) | ((uint16_t)mid << 8) | lo;
}

/* 1 when clock meter `unit` has finished (status bit 5); 1 for any
 * other unit. */
uint8_t clk_meter_done(uint8_t unit)
{
    if (unit == 0)
        return (CLK_METER0_STATUS & 0x20) ? 1 : 0;
    if (unit == 1)
        return (CLK_METER1_STATUS & 0x20) ? 1 : 0;
    return 1;
}

/* Write `ctl` (the source to measure) to the control register of
 * clock meter `unit`; other units are ignored. */
void clk_meter_select(uint8_t unit, uint8_t ctl)
{
    if (unit == 0)
        CLK_METER0_CTL = ctl;
    else if (unit == 1)
        CLK_METER1_CTL = ctl;
}
/* Absolute value. */
int16_t abs16(int16_t v)
{
    if (v < 0)
        v = (int16_t)(0u - (uint16_t)v);
    return v;
}

/* C2:A8 bit 0 (selects the source in_clk_settled measures). */
uint8_t in_clk_meter_src_bit(void)
{
    return R_C2_A8 & 0x01;
}

/* Measure the input clock with meter 1 (source 0x1B when C2:A8
 * bit 0 is set, else 0x15; up to 10 polls 1 ms apart). A reading of 0 or
 * one more than 40 away from mipi_in_clk (16-bit difference, signed compare)
 * becomes the new mipi_in_clk and restarts the count in in_clk_settle_count;
 * otherwise the count goes up. Returns 1, and restarts the count, once 20
 * matching readings have been counted. */
uint8_t in_clk_settled(void)
{
    uint32_t clk;
    uint8_t n = 0;

    clk_meter_select(1, in_clk_meter_src_bit() ? 0x1B : 0x15);
    do {
        if (clk_meter_done(1) == 1)
            break;
        delay_ms(1);
    } while (++n < 10);
    clk = clk_meter_read(1);
    if (clk == 0 || abs16((int16_t)((uint16_t)clk - (uint16_t)mipi_in_clk)) > 40) {
        mipi_in_clk = clk;
        in_clk_settle_count = 0;
    } else {
        in_clk_settle_count++;
    }
    if (in_clk_settle_count >= 20) {
        in_clk_settle_count = 0;
        return 1;
    }
    return 0;
}

/* Measure the MIPI TX read clock with meter 1 (source 0x0E; up to
 * 10 polls 2 ms apart), read it twice 1 ms apart and, when the two readings
 * are less than 40 apart (16-bit difference, signed compare), store the
 * first in lmtx_read_clk and return 1; else return 0. */
uint8_t lmtx_read_clk_measure(void)
{
    uint32_t a;
    uint16_t d;
    uint8_t n = 0;

    clk_meter_select(1, 0x0E);
    do {
        delay_ms(2);
        if (clk_meter_done(1) == 1)
            break;
    } while (++n < 10);
    a = clk_meter_read(1);
    delay_ms(1);
    d = (uint16_t)clk_meter_read(1);
    d = (uint16_t)a - d;
    if (abs16((int16_t)d) < 40) {
        lmtx_read_clk = a;
        return 1;
    }
    return 0;
}

/* ---- Lane rate (TxState 4) ---- */

/* MIPI bpp from the format mipi_fmt, then the per-lane rate mipi_phy_clk
 * from the input clock; unless mipi_pll_src, round it up to a 25000 step, add
 * seven steps and clamp to 400000..2500000. */
void mipi_phy_clk_calc(void)
{
    uint32_t rate;
    uint16_t div;

    mipi_bpp = mipi_fmt_bpp(mipi_fmt);
    rate = mipi_in_clk * 2 * mipi_bpp;
    if (mipi_cphy == 0) {
        div = mipi_lane_num * mipi_port_num; /* 8x8 -> 16 bit */
    } else {
        rate *= 7;
        div = (uint16_t)(mipi_port_num * 16) * mipi_lane_num;
    }
    rate = udiv32(rate, div);
    if (mipi_pll_src == 0) {
        rate = (int32_t)((7.0f + ceil_f((float)rate / 25000.0f)) * 25000.0f);
        if (rate < 400000UL)
            rate = 400000UL;
        else if (rate > 2500000UL)
            rate = 2500000UL;
    }
    mipi_phy_clk = rate;
}

/* Bits per pixel of MIPI format fmt (mipi_fmt_table column 2);
 * 24 when fmt is not in the table. */
uint8_t mipi_fmt_bpp(uint8_t fmt)
{
    const __xdata uint8_t *row = mipi_fmt_table;
    uint8_t i;

    for (i = 0; i < 15; i++, row += 5)
        if (row[0] == fmt)
            return row[1];
    return 0x18;
}

/* ceil(x). x is returned unchanged when it is already integral
 * by magnitude (|x| >= 2^23: biased exponent >= 150), infinite or NaN. */
float ceil_f(float x)
{
    union {
        float f;
        uint16_t w[2];
    } v;
    float t;

    v.f = x;
    if ((v.w[1] & 0x7F80) >= (150u << 7))
        return x;
    t = (float)(int32_t)x;
    if (t < x)
        t += 1.0f;
    return t;
}

/* ---- TX PLL (TxState 4) ----
 *
 * From the input pixel clock, the PHY clock mipi_phy_clk and a reference of
 * 25000 (same unit as the clocks), pick a pre-divider (E3:02[3:0]), a
 * post-divider that brings the rate into 1 200 000..2 000 000 (E3:02[6:4],
 * E3:02 bit 7 when it was halved once more) and the loop divider (E3:05).
 * lmtx_pll_band then restarts the PLL until its band calibration reports
 * done (C9:96[1:0] == 2) and returns the lock bit C9:55 bit 3.
 */

/* Set up the MIPI TX PLL for the PHY clock mipi_phy_clk from the fixed
 * reference clock 25000 (the clocks' unit), mode mipi_pll_src. */
void mipi_tx_pll_setup(void)
{
    struct mipi_pll_clocks clk;

    clk.in_clk = mipi_in_clk;
    clk.data_rate = mipi_phy_clk;
    clk.ref_clk = 25000;
    mipi_pll_configure(mipi_pll_src, &clk);
}

/* Select the PLL mode bit (E3:03) and program the PLL. */
void mipi_pll_configure(uint8_t sel, const __xdata struct mipi_pll_clocks *clk)
{
    mipi_pll_src_set(sel);
    mipi_pll(sel, clk);
}

/* Program the MIPI TX PLL dividers for clk->data_rate.
 * sel 0: the PLL runs from ref_clk (no pre-divider); otherwise from in_clk,
 * pre-divided by in_clk / ref_clk. The loop divider is rounded up. */
void mipi_pll(uint8_t sel, const __xdata struct mipi_pll_clocks *clk)
{
    uint8_t prediv = 0, postdiv, div, v;
    uint32_t rate = clk->data_rate, num;
    /* volatile: re-read from XRAM for each division instead of being
     * kept in IRAM spill space (SDCC) */
    volatile uint32_t den = clk->ref_clk;

    R_E3_02 &= 0x7F;
    if (sel)
        prediv = udiv32(clk->in_clk, den);
    v = R_E3_02 & 0xF0;
    R_E3_02 = v;
    R_E3_02 = v | (prediv & 0x0F);

    for (postdiv = 0; postdiv != 4 && rate < 1200000UL; postdiv++)
        rate <<= 1;
    if (rate > 2000000UL) {
        rate >>= 1;
        R_E3_02 |= 0x80;
    }
    v = R_E3_02 & 0x8F;
    R_E3_02 = v;
    R_E3_02 = v | (postdiv << 4);

    num = rate;
    if (sel) {
        num *= prediv + 1u;
        den = clk->in_clk;
    }
    div = 0;
    /* For a zero divisor num's low byte counts as the remainder. */
    if (den ? num % den : (uint8_t)num)
        div = 1;
    div += udiv32(num, den);
    R_E3_05 = div;
    R_E3_05 = div | 0x80;
}

/* E3:03 bit 7 on (x != 0), else clear bits 7..6. */
void mipi_pll_src_set(uint8_t x)
{
    if (x)
        R_E3_03 |= 0x80;
    else
        R_E3_03 &= 0x3F;
}

/* Restart the TX PLL band calibration until it reports done
 * (up to 3 tries), up to 3 rounds until locked. Returns 1 when locked. */
uint8_t lmtx_pll_band(void)
{
    uint8_t i, round = 0, v;

    R_E6_3B &= 0xEF;
    do {
        i = 0;
        do {
            v = R_C8_11 & 0xFB;
            R_C8_11 = v;
            R_C8_11 = v & 0xF7;
            R_E1_1B &= 0xFE;
            R_C9_90 &= 0xDF;
            delay_ms(1);
            v = R_C8_11 | 0x04;
            R_C8_11 = v;
            R_C8_11 = v | 0x08;
            R_E1_1B |= 0x01;
            R_C9_90 |= 0x20;
            delay_ms(2);
        } while (++i < 3 && (R_C9_96 & 0x03) != 2);
        delay_ms(2);
        (void)R_C9_9D;
        (void)R_C9_9E;
    } while (++round < 3 && !(R_C9_55 & 0x08));
    return (R_C9_55 & 0x08) ? 1 : 0;
}

/* ---- Output timing (TxState 5) ----
 *
 * tx_timing_check reads the DP RX timing measurement (rx_timing_read,
 * E4:7A..A3) into the VidChk words vidchk_* (H total/active/front
 * porch/sync/back porch, the same for V, and the two sync polarities) and
 * copies them into the TX timing generator on page C8. It then programs
 * the horizontal sync start (C8:84/85) and, when mipi_pll_src is 1, the
 * free-run horizontal active and front porch, which are derived from the
 * lane rate mipi_phy_clk (mipi_phy_clk_calc) and the input pixel clock with float math.
 */

/* Write a 16-bit value to C8:7E (high), C8:7F (low): H active. */
void tx_set_hactive(uint16_t v)
{
    R_C8_7E = v >> 8;
    R_C8_7F = (uint8_t)v;
}

/* Write a 16-bit value to C8:7C/7D: V active. */
void tx_set_vactive(uint16_t v)
{
    R_C8_7C = v >> 8;
    R_C8_7D = (uint8_t)v;
}

/* Write a 16-bit value to C8:80/81: V sync + V back porch. */
void tx_set_vsw_vbp(uint16_t v)
{
    R_C8_80 = v >> 8;
    R_C8_81 = (uint8_t)v;
}

/* Write a 16-bit value to C8:95/96: V sync. */
void tx_set_vsw(uint16_t v)
{
    R_C8_95 = v >> 8;
    R_C8_96 = (uint8_t)v;
}

/* V front porch: bits 14..13 to C8:5E[6:5], bits 12..8 to
 * C8:5C[4:0], low byte to C8:5D. */
void tx_set_vfp(uint16_t v)
{
    uint8_t hi = v >> 8;

    R_C8_5E = (R_C8_5E & 0x9F) | (hi & 0x60);
    R_C8_5C = (R_C8_5C & 0xE0) | (hi & 0x1F);
    R_C8_5D = (uint8_t)v;
}

/* Bits 11..8 to C8:7A[3:0], low byte to C8:7B (H active / 4). */
void tx_set_hactive_quarter(uint16_t v)
{
    R_C8_7A = (R_C8_7A & 0xF0) | ((v >> 8) & 0x0F);
    R_C8_7B = (uint8_t)v;
}

/* Free-run H front porch: bits 12..8 to C8:5E[4:0], low byte to
 * C8:5F. */
void tx_set_freerun_hfp(uint16_t v)
{
    R_C8_5E = (R_C8_5E & 0xE0) | ((v >> 8) & 0x1F);
    R_C8_5F = (uint8_t)v;
}

/* H sync polarity: C8:70 bit 1 set when pol is 0, cleared otherwise. */
void tx_set_hspol(uint8_t pol)
{
    if (pol)
        R_C8_70 &= 0xFD;
    else
        R_C8_70 |= 0x02;
}

/* V sync polarity: C8:70 bit 0 set when pol is 0, cleared otherwise. */
void tx_set_vspol(uint8_t pol)
{
    if (pol)
        R_C8_70 &= 0xFE;
    else
        R_C8_70 |= 0x01;
}

/* C8:87 bit 7 = on (brackets the free-run H front porch write). */
void tx_freerun_enable(uint8_t on)
{
    if (on)
        R_C8_87 |= 0x80;
    else
        R_C8_87 &= 0x7F;
}

/* H sync start, clamped to 10 bits: C8:84[1:0] and C8:85. C8:84 is
 * read before the clamp. */
void tx_set_hss_reg(uint16_t v)
{
    uint8_t r = R_C8_84;

    if (v > 0x3FF)
        v = 0x3FF;
    R_C8_84 = (r & 0xFC) | (uint8_t)(v >> 8);
    R_C8_85 = (uint8_t)v;
}

/* Same as tx_set_hss_reg. */
void tx_set_hss(uint16_t v)
{
    tx_set_hss_reg(v);
}

/* Per-line overhead bytes added in tx_freerun_hact, by lane count 1..3. */
static const __code uint8_t csi_line_extra[3] = { 0x12, 0x22, 0x32 };
static const __code uint8_t dsi_line_extra[3] = { 0x14, 0x1A, 0x20 };

#pragma save
#pragma nogcse
/* Free-run H active: H active (vidchk_hactive) in bytes per port, plus a
 * per-line overhead, divided over the lanes and rounded up. With mipi_cphy
 * set, 0 for a lane count other than 1..3. */
uint16_t tx_freerun_hact(void) __reentrant
{
    uint32_t bytes = udiv32((uint32_t)vidchk_hactive * mipi_bpp / 8, mipi_port_num);
    uint8_t lanes = mipi_lane_num;
    float f;

    if (mipi_cphy == 0) {
        f = (float)(bytes + 6) / (float)(uint16_t)(lanes * 2);
    } else {
        if (lanes == 0 || lanes > 3)
            return 0;
        bytes += (mipi_out_csi == 1 ? csi_line_extra : dsi_line_extra)[lanes - 1];
        f = (float)(bytes / (uint8_t)(lanes * 2));
    }
    return (uint16_t)(int32_t)ceil_f(f);
}
#pragma restore

/* Clear the VidChk timing words. */
void vidchk_clear(void)
{
    vidchk_htotal = 0;
    vidchk_hactive = 0;
    vidchk_hfp = 0;
    vidchk_hsw = 0;
    vidchk_hbp = 0;
    vidchk_vactive = 0;
    vidchk_vfp = 0;
    vidchk_vsw = 0;
    vidchk_vbp = 0;
    vidchk_hspol = 0;
    vidchk_vspol = 0;
}

#pragma save
#pragma nogcse
/* Read the RX timing into the VidChk words and program the TX
 * timing generator; see the comment above. */
void tx_timing_check(void)
{
    uint16_t hss = 0x20;
    uint16_t hact;
    float f;

    vidchk_clear();
    vidchk_htotal = rx_timing_read(0);
    vidchk_hactive = rx_timing_read(1);
    vidchk_hfp = rx_timing_read(2);
    vidchk_hsw = rx_timing_read(3);
    vidchk_hbp = rx_timing_read(4);
    (void)rx_timing_read(5); /* V total, unused */
    vidchk_vactive = rx_timing_read(6);
    vidchk_vfp = rx_timing_read(7);
    vidchk_vsw = rx_timing_read(8);
    vidchk_vbp = rx_timing_read(9);
    vidchk_vspol = rx_timing_read(0x0B); /* V sync polarity */
    vidchk_hspol = rx_timing_read(0x0A); /* H sync polarity */

    tx_set_hactive(vidchk_hactive);
    tx_set_vactive(vidchk_vactive);
    tx_set_vsw_vbp(vidchk_vbp + vidchk_vsw);
    tx_set_vfp(vidchk_vfp);
    tx_set_vsw(vidchk_vsw);
    tx_set_hactive_quarter(vidchk_hactive >> 2);
    tx_set_hspol((uint8_t)vidchk_hspol);
    tx_set_vspol((uint8_t)vidchk_vspol);
    if (mipi_cphy == 0 && mipi_out_csi == 1)
        hss = (uint16_t)((mipi_phy_clk >> 3) / 1000 * 0x70 / 0xF0);
    tx_set_hss(hss);
    tx_freerun_enable(0);
    if (mipi_pll_src != 1)
        return;

    hact = tx_freerun_hact(); /* free-run H active */
    f = (float)mipi_phy_clk / 8.0f;
    f = f / (float)mipi_in_clk;
    f = (float)(uint16_t)(vidchk_htotal >> 1) * f;
    hss <<= 1;
    f = f - (float)hss;
    hact <<= 1;
    f = f - (float)hact;
    /* free-run H front porch; 0x7FFF for a zero input clock, where f is
     * infinite or NaN */
    tx_set_freerun_hfp(mipi_in_clk ? (uint16_t)(int32_t)f >> 1 : 0x7FFF);
    tx_freerun_enable(1);
}
#pragma restore

/* ---- D-PHY timing from the lane rate (TxState 5) ---- */

/* C8:A6/A9 from mipi_phy_clk (mipi_cphy == 0: r = mipi_phy_clk / 1000 / 8 * 7 /
 * 100, A6 = r + 4, A9 = r + 7, in 8 bits; else A9 = 0x1E), then the fixed
 * C8:D4..DD values of tx_c8_d4_dd_fixed_set. */
void mipi_phy_timing_setup(void)
{
    uint8_t r;

    if (mipi_cphy == 0) {
        r = (uint8_t)(mipi_phy_clk / 1000 / 8 * 7 / 100);
        tx_set_c8_a6(r + 4);
        tx_set_c8_a9(r + 7);
    } else {
        tx_set_c8_a9_cphy(0x1E);
    }
    tx_c8_d4_dd_fixed_set();
}

/* Fixed values for C8:D4..DD (C8:D8/DA/DC only when mipi_cphy == 0,
 * C8:D8 with another value otherwise). */
void tx_c8_d4_dd_fixed_set(void)
{
    tx_set_c8_d4(0x0640);
    tx_set_c8_d6(0x3D);
    tx_set_c8_d7(0x33);
    if (mipi_cphy == 0) {
        tx_set_c8_d8(0xC8);
        tx_set_c8_da(0xD2 - 0x33);
        tx_set_c8_dc(3);
    } else {
        tx_set_c8_d8_reg(0x78);
    }
}

/* C8:EC bit 7 : C8:D8 = v, saturated to 9 bits. */
void tx_set_c8_d8_reg(uint16_t v)
{
    if (v > 0x1FF)
        v = 0x1FF;
    if (v > 0xFF)
        R_C8_EC |= 0x80;
    else
        R_C8_EC &= 0x7F;
    R_C8_D8 = (uint8_t)v;
}

/* Same as tx_set_c8_d8_reg. */
void tx_set_c8_d8(uint16_t v)
{
    tx_set_c8_d8_reg(v);
}

/* C8:DD bit 7 : C8:DA = v, saturated to 9 bits. */
void tx_set_c8_da(uint16_t v)
{
    if (v > 0x1FF)
        v = 0x1FF;
    if (v > 0xFF)
        R_C8_DD |= 0x80;
    else
        R_C8_DD &= 0x7F;
    R_C8_DA = (uint8_t)v;
}

/* C8:D5[3:0] : C8:D4 = v (12 bits). */
void tx_set_c8_d4(uint16_t v)
{
    R_C8_D5 = (R_C8_D5 & 0xF0) | ((v >> 8) & 0x0F);
    R_C8_D4 = (uint8_t)v;
}

void tx_set_c8_d6(uint8_t v)
{
    R_C8_D6 = v;
}

void tx_set_c8_d7(uint8_t v)
{
    R_C8_D7 = v;
}

void tx_set_c8_dc(uint8_t v)
{
    R_C8_DC = v;
}

void tx_set_c8_a6(uint8_t v)
{
    R_C8_A6 = v;
}

void tx_set_c8_a9(uint8_t v)
{
    R_C8_A9 = v;
}

/* Same register as tx_set_c8_a9. */
void tx_set_c8_a9_cphy(uint8_t v)
{
    R_C8_A9 = v;
}

/* ---- Rdly: the TX read delay ----
 *
 * mipi_rdly_update bounds it from below (two candidates,
 * the larger wins) and from above (up to three candidates, the smallest
 * wins), each candidate being a span of the measured input timing (the
 * VidChk words vidchk_htotal..vidchk_hbp) converted from input-pixel clocks to TX
 * clocks (x mipi_phy_clk/1000/8, or /7, halved in one mode; / (2 x mipi_in_clk /
 * 1000)) minus fixed offsets read back from C8. The result, the midpoint of
 * the window (pulled towards the upper bound for short active lines in
 * mode mipi_port_mode = 4), goes to C8:82..83.
 *
 * Arithmetic is 32-bit and wrapping; rdly_div does the signed divisions. */

/* C8:C1[5:0]:C8:C2 as a 14-bit value. */
uint16_t tx_get_c8_c1(void)
{
    uint8_t hi = R_C8_C1 & 0x3F;
    return ((uint16_t)hi << 8) | R_C8_C2;
}

/* C8:84[1:0]:C8:85 as a 10-bit value. */
uint16_t tx_get_hss_reg(void)
{
    uint8_t hi = R_C8_84 & 0x03;
    return ((uint16_t)hi << 8) | R_C8_85;
}

/* C8:7A[3:0]:C8:7B as a 12-bit value. */
uint16_t tx_get_hactive_quarter(void)
{
    uint8_t hi = R_C8_7A & 0x0F;
    return ((uint16_t)hi << 8) | R_C8_7B;
}

/* Write Rdly to C8:82 (high) and C8:83 (low). */
void mipi_rdly_write(uint16_t rdly)
{
    MIPI_RDLY_H = rdly >> 8;
    MIPI_RDLY_L = (uint8_t)rdly;
}

/* mipi_rdly_update's working set: the clock conversion, the offsets subtracted from
 * each candidate and the window found so far. */
static __xdata uint32_t rdly_rate, rdly_clk_div, rdly_lane2, rdly_min, rdly_max;
static __xdata uint16_t rdly_off;

/* Signed a / b: udiv32 on the magnitudes (so by 0 too), the quotient
 * negated when the signs differ. */
static uint32_t rdly_div(uint32_t a, uint32_t b)
{
    uint8_t neg = 0;

    if ((int32_t)a < 0) {
        a = -a;
        neg = 1;
    }
    if ((int32_t)b < 0) {
        b = -b;
        neg ^= 1;
    }
    a = udiv32(a, b);
    return neg ? -a : a;
}

/* Input-pixel clocks to TX clocks. */
static uint32_t rdly_scale(uint32_t ticks)
{
    return rdly_div(ticks * rdly_rate, rdly_clk_div);
}

/* Subtract a span's per-lane share and the C8:84..85 offset. */
static uint32_t rdly_less(uint32_t v, uint32_t span)
{
    return v - rdly_div(span, rdly_lane2) - rdly_off;
}

/* A lower-bound candidate (negative counts as 1); the larger one wins. */
static void rdly_lower(uint32_t v)
{
    if ((int32_t)v < 0)
        v = 1;
    if (v > rdly_min)
        rdly_min = v;
}

/* An upper-bound candidate (negative or 0xFFFF counts as none); the
 * smallest one wins. */
static void rdly_upper(uint32_t v)
{
    if ((int32_t)v >= 0 && v < rdly_max)
        rdly_max = v;
}

/* nogcse (also on tx_freerun_hact and tx_timing_check): global CSE would keep
 * their 32-bit and float temporaries in IRAM spill bytes (DSEG). */
#pragma save
#pragma nogcse
/* Compute the Rdly window and write Rdly (mipi_rdly_write). */
void mipi_rdly_update(void)
{
    uint32_t span;
    uint16_t span_c1, off_7a, sub_a, w30, w32, w34, w36, w38;

    rdly_clk_div = mipi_in_clk * 2 / 1000;
    if (mipi_cphy == 0)
        rdly_rate = mipi_phy_clk / 1000 / 8;
    else
        rdly_rate = mipi_phy_clk / 1000 / 7;
    if (mipi_cphy == 0 && mipi_csi_c8a0_bit6 != 1)
        rdly_rate /= 2;

    rdly_off = tx_get_hss_reg();
    span_c1 = (uint16_t)((uint16_t)(tx_get_c8_c1() * mipi_lane_num) << 1);
    off_7a = tx_get_hactive_quarter();

    if (dsi_mode != 3 && mipi_out_csi == 0)
        sub_a = 0;
    else
        sub_a = vidchk_hfp;

    w36 = 0;
    w38 = 0x3D40;
    w30 = w32 = w34 = 0;
    if (mipi_port_num == 2) {
        w30 = (uint16_t)(off_7a << 1);
        w32 = w34 = vidchk_hactive >> 1;
        w38 = 0x2580;
        w36 = 0x17C0;
    } else if (mipi_port_num == 1) {
        w36 = 0x3D40;
    }
    rdly_lane2 = (uint32_t)mipi_lane_num * 2;
    rdly_min = 0;
    rdly_max = 0xFFFF;

    rdly_lower(rdly_less(rdly_scale((uint32_t)vidchk_htotal - sub_a - w30), span_c1));

    span = (uint32_t)span_c1 - w36;
    if ((int32_t)span >= 0)
        rdly_upper(rdly_less(rdly_scale((uint32_t)vidchk_htotal - vidchk_hfp - w30), span));

    rdly_lower(rdly_scale((uint32_t)vidchk_hbp + vidchk_hsw + w32 - w30) - rdly_off);

    rdly_upper(rdly_scale(vidchk_htotal));

    if (mipi_port_num == 2) {
        span = (uint32_t)span_c1 * 2 - w38;
        if ((int32_t)span >= 0)
            rdly_upper(rdly_less(rdly_scale((uint32_t)vidchk_htotal + vidchk_hsw + vidchk_hbp
                                            - (uint32_t)off_7a * 2 + w34), span));
    }

    if (rdly_max == 0xFFFF)
        rdly_max = 1000;
    if (rdly_min > rdly_max) {
        span = 100;
    } else {
        span = (rdly_max + rdly_min) / 2;
        if (mipi_port_mode == 4 && vidchk_hactive <= 720)
            span = (rdly_max + span) / 2;
    }
    mipi_rdly_write((uint16_t)span);
}
#pragma restore
