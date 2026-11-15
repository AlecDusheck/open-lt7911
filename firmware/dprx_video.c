// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Incoming video format: MSA timing, colour format from the VSC SDP or MSA
 * MISC0, the colour-space conversion (CSC) block, DSC PPS info, and the
 * video events that drive the MIPI TX state flags.
 *
 * Change detection (polled by the RX state machine): rx_color_changed snapshots the
 * colour format into rx_color_snap_from_vsc..rx_color_snap_range and rx_timing_changed the MSA totals/actives into
 * msa_snap_htotal_h..msa_snap_vact_l; each returns 1 when the value changed (rx_timing_changed returns 2
 * while irq_pending(6) says there is no video).
 *
 * Colour format codes (rx_color_read, also rx_color_format and the DSC dsc_color_format):
 * COLOR_RGB 0, COLOR_YCBCR422 1, COLOR_YCBCR444 2, COLOR_YCBCR420 3; MSA
 * MISC0's reserved value 3 becomes 4, and DSC "simple 4:2:2" is 5.
 *
 * Video events, rx_video_event(ev):
 *   1 / 2   set / clear tx_video_detected
 *   3       RX stable: tx_rx_stable = 1, audio_rx_stable = 1, then the CSC setup
 *   4       video off: tx_rx_stable = 0, tx_dsc_detected = 0, audio_rx_stable = 0, then
 *           sys_event_post(6)
 *   0x10    tx_dsc_detected = 1
 * tx_video_event and audio_video_event also take 0x0B / 0x0C: MIPI output and audio on / off.
 *
 * CSC setup (csc_setup): csc_defaults sets the defaults (csc_enable = 1 enabled,
 * csc_range_cfg = 0, csc_conv_sel = 2) and offers them to a hook that keeps them;
 * rx_video_info() records the input format (rx_color_format) and dynamic-range bit
 * (rx_color_range); csc_disable clears bit 0 of C5:0B..C5:12; then, if enabled,
 * csc_select() picks the output (csc_out_format: 0 RGB, 1 YCbCr 4:2:2) from the
 * configured MIPI format mipi_fmt, csc_range_mode_select the range mode csc_range_mode, and csc_program
 * sets the C5:0C / C5:0D / C5:10 / C5:11 stages the in/out pair needs.
 *
 * DSC (dsc_pps): the PPS is read four bytes at a time: C0:05 = 0x20 | group
 * selects PPS bytes 4g..4g+3 in C0:35, C0:34, C0:33, C0:32.
 */
#include <string.h>

#include "fw.h"

#define COLOR_RGB 0
#define COLOR_YCBCR422 1
#define COLOR_YCBCR444 2
#define COLOR_YCBCR420 3

/* Colour format of the incoming stream (rx_color_read). */
struct rx_color {
    uint8_t from_vsc;    /* 1: from the VSC SDP, 0: from MSA MISC0 */
    uint8_t range;       /* VSC DB17 bit 7 / MISC0 bit 3: 1 = CTA (limited) range */
    uint8_t format;      /* COLOR_* */
    uint8_t colorimetry; /* VSC DB16 bits 3:0 / MISC0 bits 4:3 */
    uint8_t depth;       /* VSC DB17 bits 2:0 / MISC0 bits 7:5 */
};

/* Select PPS group g (C0:05 = g) and let it settle. */
static void pps_select(uint8_t g)
{
    R_C0_05 = g;
    delay_ms(1);
}

/* The selected group's first two bytes, C0:35 then C0:34, big-endian. */
static uint16_t pps_word0(void)
{
    uint8_t hi = R_C0_35;

    return ((uint16_t)hi << 8) | R_C0_34;
}

/* The selected group's last two bytes, C0:33 then C0:32, big-endian. */
static uint16_t pps_word1(void)
{
    uint8_t hi = R_C0_33;

    return ((uint16_t)hi << 8) | R_C0_32;
}

/* Read the DSC PPS: version check, then colour format dsc_color_format,
 * bits per component / 2 dsc_bpc_half, bits per pixel (1/16 bpp) dsc_bpp_h:dsc_bpp_l,
 * compression ratio x 10 dsc_ratio, slice count dsc_slice_count (= dsc_number), slice height
 * dsc_slice_height_h:dsc_slice_height_l, slice width dsc_slice_width_h:dsc_slice_width_l and chunk size dsc_chunk_size_h:dsc_chunk_size_l
 * (big-endian byte pairs); C1:62 bit 3 set for native 4:2:0. */
void dsc_pps(void)
{
    uint8_t i, v, pps4;
    uint16_t bpp, bits, w;

    delay_ms(10);
    for (i = 0; i < 0x20; i++) {
        pps_select(i | 0x20);
        (void)R_C0_35;
        (void)R_C0_34;
        (void)R_C0_33;
        (void)R_C0_32;
    }

    pps_select(0x20);
    v = R_C0_35;                            /* PPS0: DSC version */
    if (v < 0x11 || v > 0x13)
        return;
    dsc_bpc_half = (R_C0_32 >> 5) & 0x07;         /* PPS3 bits_per_component / 2 */

    pps_select(0x21);
    pps4 = R_C0_35;                         /* PPS4: convert_rgb, simple_422 */
    pps_select(0x36);
    v = R_C0_35;                            /* PPS88: native_420, native_422 */
    if (v & 0x01)
        dsc_color_format = COLOR_YCBCR422;
    else if (v & 0x02)
        dsc_color_format = COLOR_YCBCR420;
    else if (pps4 & 0x08)
        dsc_color_format = 5;
    else if (pps4 & 0x10)
        dsc_color_format = COLOR_RGB;
    else
        dsc_color_format = COLOR_YCBCR444;

    pps_select(0x21);
    v = R_C0_35 & 0x03;                     /* PPS4/5: bits_per_pixel */
    bpp = ((uint16_t)v << 8) | R_C0_34;
    dsc_bpp_h = bpp >> 8;
    dsc_bpp_l = (uint8_t)bpp;
    pps_select(0x20);
    bits = (R_C0_32 >> 4) * 3;              /* bits per pixel, uncompressed */
    if (dsc_color_format == COLOR_YCBCR422 || dsc_color_format == COLOR_YCBCR420)
        bpp >>= 1;
    dsc_ratio = (uint8_t)udiv16((uint16_t)(bits * 160), bpp);

    pps_select(0x22);
    w = pps_word0();                        /* PPS8/9: pic_width */
    pps_select(0x23);
    dsc_slice_count = (uint8_t)udiv16(w, pps_word0()); /* PPS12/13: slice_width */

    pps_select(0x22);
    bits = pps_word1();                     /* PPS10/11: slice_height */
    pps_select(0x23);
    w = pps_word0();                        /* slice_width */
    if (dsc_color_format == COLOR_YCBCR422 || dsc_color_format == COLOR_YCBCR420)
        w >>= 1;
    if (dsc_color_format == COLOR_YCBCR420)
        bits >>= 1;
    dsc_slice_height_h = bits >> 8;
    dsc_slice_height_l = (uint8_t)bits;
    dsc_slice_width_h = w >> 8;
    dsc_slice_width_l = (uint8_t)w;

    pps_select(0x23);
    w = pps_word1();                        /* PPS14/15: chunk_size */
    dsc_chunk_size_h = w >> 8;
    dsc_chunk_size_l = (uint8_t)w;
    dsc_number = dsc_slice_count;
    if (dsc_color_format == COLOR_YCBCR420)
        R_C1_62 |= 0x08;
    else
        R_C1_62 &= ~0x08;
}

/* Read the MSA (C0:DB..EA) into *t. */
void msa_timing_read(__xdata struct rx_timing *t) __reentrant
{
    uint8_t hi;
    uint16_t start;

    hi = R_C0_E7;
    t->hact = ((uint16_t)hi << 8) | R_C0_E8;
    hi = R_C0_DF & 0x7F;
    t->hsync = ((uint16_t)hi << 8) | R_C0_E0;
    hi = R_C0_E1;
    start = ((uint16_t)hi << 8) | R_C0_E2;
    t->hbp = start - t->hsync;
    hi = R_C0_DB;
    t->htotal = ((uint16_t)hi << 8) | R_C0_DC;
    t->hfp = t->htotal - t->hact - start;

    hi = R_C0_E9;
    t->vact = ((uint16_t)hi << 8) | R_C0_EA;
    hi = R_C0_E5 & 0x7F;
    t->vsync = ((uint16_t)hi << 8) | R_C0_E6;
    hi = R_C0_E3;
    start = ((uint16_t)hi << 8) | R_C0_E4;
    t->vbp = start - t->vsync;
    hi = R_C0_DD;
    t->vtotal = ((uint16_t)hi << 8) | R_C0_DE;
    t->vfp = t->vtotal - t->vact - start;

    t->hsync_neg = R_C0_DF >> 7;
    t->vsync_neg = R_C0_E5 >> 7;
}

/* Read the MSA timing into dsc_bypass_timing (DSC bypass info). */
void dsc_bypass_info(void)
{
    msa_timing_read(&dsc_bypass_timing);
}

/* The stream's colour format, from the VSC SDP (C0:A8..BF) when one
 * with pixel encoding/colorimetry (HB1 7, HB2 5, HB3 0x13, encoding <= 3)
 * is valid (C0:EC bit 6), else from MSA MISC0 (C0:EB). */
void rx_color_read(__xdata struct rx_color *c)
{
    uint8_t vsc = 0;
    uint8_t v, w;

    if (R_C0_A9 == 0x07 && R_C0_AA == 0x05 && R_C0_AB == 0x13 && (R_C0_BC & 0xF0) <= 0x30)
        vsc++;
    if ((R_C0_EC & 0x40) && vsc == 1) {
        c->from_vsc = 1;
        v = R_C0_BC;                        /* DB16 */
        c->format = v >> 4;
        w = R_C0_BD;                        /* DB17 */
        c->range = w >> 7;
        if (c->format == 2)                 /* VSC: 1 = 4:4:4, 2 = 4:2:2 */
            c->format = COLOR_YCBCR422;
        else if (c->format == 1)
            c->format = COLOR_YCBCR444;
        c->colorimetry = v & 0x0F;
        c->depth = w & 0x07;
    } else {
        c->from_vsc = 0;
        v = R_C0_EB;
        c->range = (v >> 3) & 0x01;
        c->format = (v >> 1) & 0x03;
        if (c->format == 3)
            c->format = 4;
        w = R_C0_EB;
        c->colorimetry = (v >> 3) & 0x03;
        c->depth = (w >> 5) & 0x07;
    }
}

/* Whether interrupt source id is pending, by the source numbers of
 * irq_enable/irq_ack; 0 for an unknown id. */
uint8_t irq_pending(uint8_t id)
{
    switch (id) {
    case 0x04:
        return (R_C0_ED & 0x08) ? 1 : 0;
    case 0x06:
        return (R_C0_EE & 0x04) ? 1 : 0;
    case 0x07:
        return (R_C0_ED & 0x40) ? 1 : 0;
    case 0x08:
        return (R_C0_ED & 0x80) ? 1 : 0;
    case 0x13:
        return (INT1_STATUS & 0x80) ? 1 : 0;
    case 0x17:
        return (R_CD_F2 & 0x20) ? 1 : 0;
    case 0x18:
        return (R_CD_F4 & 0x04) ? 1 : 0;
    case 0x23:
        return (INT0_STATUS2 & 0x40) ? 1 : 0;
    default:
        return 0;
    }
}

/* Compare the MSA htotal/hact with the snapshot msa_snap_htotal_h..msa_snap_hact_l
 * (big-endian pairs): 0 unchanged; else store htotal, hact, vtotal, vact in
 * msa_snap_htotal_h..msa_snap_vact_l and return 1. 2 while irq_pending(6) is 0. */
uint8_t rx_timing_changed(void)
{
    __xdata struct rx_timing t;

    if (!irq_pending(6))
        return 2;
    msa_timing_read(&t);
    if (msa_snap_htotal_h == (uint8_t)(t.htotal >> 8) && msa_snap_htotal_l == (uint8_t)t.htotal &&
        msa_snap_hact_h == (uint8_t)(t.hact >> 8) && msa_snap_hact_l == (uint8_t)t.hact)
        return 0;
    msa_snap_htotal_h = t.htotal >> 8;
    msa_snap_htotal_l = (uint8_t)t.htotal;
    msa_snap_hact_h = t.hact >> 8;
    msa_snap_hact_l = (uint8_t)t.hact;
    msa_snap_vtotal_h = t.vtotal >> 8;
    msa_snap_vtotal_l = (uint8_t)t.vtotal;
    msa_snap_vact_h = t.vact >> 8;
    msa_snap_vact_l = (uint8_t)t.vact;
    return 1;
}

/* CSC output csc_out_format from the configured MIPI format (mipi_fmt, via
 * mipi_tx_info_get): 0 RGB for 0..3 and 8..10, 1 YCbCr 4:2:2 for 4..6 and 11..12,
 * else the format code itself. */
void csc_select(void)
{
    __xdata struct mipi_tx_info cfg;

    memset(&cfg, 0, sizeof cfg);
    mipi_tx_info_get(&cfg);
    csc_out_format = cfg.fmt;
    switch (cfg.fmt) {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x08:
    case 0x09:
    case 0x0A:
        csc_out_format = 0;
        break;
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x0B:
    case 0x0C:
        csc_out_format = 1;
        break;
    }
}

/* Record the input colour format (rx_color_format) and range bit (rx_color_range). */
void rx_video_info(void)
{
    __xdata struct rx_color c;

    rx_color_read(&c);
    rx_color_format = c.format;
    rx_color_range = c.range;
}

/* Snapshot the colour format into rx_color_snap_from_vsc..rx_color_snap_range; 1 if format,
 * colorimetry or depth changed, else 0. */
uint8_t rx_color_changed(void)
{
    __xdata struct rx_color c;
    uint8_t changed = 1;

    rx_color_read(&c);
    if (rx_color_snap_format == c.format && rx_color_snap_colorimetry == c.colorimetry && rx_color_snap_depth == c.depth)
        changed = 0;
    rx_color_snap_from_vsc = c.from_vsc;
    rx_color_snap_format = c.format;
    rx_color_snap_colorimetry = c.colorimetry;
    rx_color_snap_depth = c.depth;
    rx_color_snap_range = c.range;
    return changed;
}

/* Enable the CSC stages for input format in (COLOR_*) and output
 * out (0 RGB, 1 or 2 YCbCr 4:2:2); sel and mode pick the C5:0C / C5:11
 * coefficient fields (csc_conv_field). */
void csc_program(uint8_t in, uint8_t out, uint8_t sel, uint8_t mode)
{
    if (out == 0) {
        if (in == COLOR_YCBCR422) {
            csc_422_to_444_enable();
            csc_ycbcr_to_rgb_enable(sel, mode);
        } else if (in == COLOR_YCBCR444) {
            csc_ycbcr_to_rgb_enable(sel, mode);
        }
    } else if (out == 1) {
        if (in == COLOR_YCBCR444) {
            csc_444_to_422_enable();
        } else if (in == COLOR_RGB) {
            csc_rgb_to_ycbcr_enable(sel, mode);
            csc_444_to_422_enable();
        }
    } else if (out == 2) {
        if (in == COLOR_YCBCR422)
            csc_422_to_444_enable();
        else if (in == COLOR_RGB)
            csc_rgb_to_ycbcr_enable(sel, mode);
    }
}

/* TX flags for a video event; 0x0B / 0x0C turn the MIPI output on /
 * off. */
void tx_video_event(uint8_t ev)
{
    switch (ev) {
    case 0x01:
        tx_video_detected = 1;
        break;
    case 0x02:
        tx_video_detected = 0;
        break;
    case 0x05:
        tx_rx_stable = 1;
        break;
    case 0x06:
        tx_rx_stable = 0;
        tx_dsc_detected = 0;
        break;
    case 0x07:
        tx_dsc_detected = 1;
        break;
    case 0x0B:
        mipi_output_enable(mipi_out_csi, 1);
        break;
    case 0x0C:
        mipi_output_enable(mipi_out_csi, 0);
        break;
    }
}

/* Handle a video event (see the file comment). */
void rx_video_event(uint8_t ev)
{
    switch (ev) {
    case 0x01:
        tx_video_event(1);
        break;
    case 0x02:
        tx_video_event(2);
        break;
    case 0x03:
        tx_video_event(5);
        audio_video_event(5);
        csc_setup();
        break;
    case 0x04:
        tx_video_event(6);
        audio_video_event(6);
        sys_event_post(6);
        break;
    case 0x10:
        tx_video_event(7);
        break;
    }
}

/* Range mode csc_range_mode from the range bit rx_color_range and csc_range_cfg:
 * (1,1) 0, (1,0) 1, (0,1) 2, (0,0) 3, else 0. */
void csc_range_mode_select(void)
{
    if (rx_color_range == 1 && csc_range_cfg == 1)
        csc_range_mode = 0;
    else if (rx_color_range == 1 && csc_range_cfg == 0)
        csc_range_mode = 1;
    else if (rx_color_range == 0 && csc_range_cfg == 1)
        csc_range_mode = 2;
    else if (rx_color_range == 0 && csc_range_cfg == 0)
        csc_range_mode = 3;
    else
        csc_range_mode = 0;
}

/* CSC override hook: csc_config_get may replace csc_enable, csc_range_cfg and
 * csc_conv_sel (it fills its buffer but returns 0, so nothing changes). */
void csc_config_load(void)
{
    __xdata uint8_t cfg[7];

    memset(cfg, 0, sizeof cfg);
    if (csc_config_get(cfg)) {
        csc_enable = cfg[0];
        csc_range_cfg = cfg[2];
        csc_conv_sel = cfg[6];
    }
}

/* Audio and audio_rx_stable for a video event; 0x0B / 0x0C turn audio
 * on / off. */
void audio_video_event(uint8_t ev)
{
    switch (ev) {
    case 0x05:
        audio_rx_stable = 1;
        break;
    case 0x06:
        audio_rx_stable = 0;
        break;
    case 0x0B:
        audio_on_off(audio_out_sel, 1);
        break;
    case 0x0C:
        audio_on_off(audio_out_sel, 0);
        break;
    }
}

/* Set up the CSC for the current input (see the file comment). */
void csc_setup(void)
{
    csc_defaults();
    rx_video_info();
    csc_disable();
    if (csc_enable == 1) {
        csc_select();
        csc_range_mode_select();
        csc_program(rx_color_format, csc_out_format, csc_conv_sel, csc_range_mode);
    }
}

/* Field for sel 0: mode 1 0x10, 2 0x60, 3 0x70, else 0. */
uint8_t csc_conv_field_sel0(uint8_t mode)
{
    switch (mode) {
    case 1:
        return 0x10;
    case 2:
        return 0x60;
    case 3:
        return 0x70;
    default:
        return 0;
    }
}

/* The bits 7:4 field for C5:0C / C5:11: by sel 0, 2 or 4 from
 * csc_conv_field_sel0, csc_conv_field_sel2 or csc_conv_field_sel4; else 0. */
uint8_t csc_conv_field(uint8_t sel, uint8_t mode)
{
    switch (sel) {
    case 0:
        return csc_conv_field_sel0(mode);
    case 2:
        return csc_conv_field_sel2(mode);
    case 4:
        return csc_conv_field_sel4(mode);
    default:
        return 0;
    }
}

/* Clear bit 0 (stage enable) of C5:0B..C5:12. */
void csc_disable(void)
{
    R_C5_0B &= ~0x01;
    R_C5_0C &= ~0x01;
    R_C5_0D &= ~0x01;
    R_C5_10 &= ~0x01;
    R_C5_11 &= ~0x01;
    R_C5_12 &= ~0x01;
}

/* Field for sel 2: mode 0 0x20, 1 0x30, 2 0x80, 3 0x90, else 0. */
uint8_t csc_conv_field_sel2(uint8_t mode)
{
    switch (mode) {
    case 0:
        return 0x20;
    case 1:
        return 0x30;
    case 2:
        return 0x80;
    case 3:
        return 0x90;
    default:
        return 0;
    }
}

/* CSC defaults: the CSC state cleared, csc_enable = 1, csc_conv_sel = 2,
 * then the override hook. */
void csc_defaults(void)
{
    rx_color_range = 0;
    rx_color_format = 0;
    csc_out_format = 0;
    csc_range_mode = 0;
    csc_enable = 1;
    csc_range_cfg = 0;
    csc_conv_sel = 2;
    csc_config_load();
}

/* CSC override source: fills cfg (enable 1, csc_range_cfg 1, csc_conv_sel 2) but
 * returns 0, so the caller ignores it. */
uint8_t csc_config_get(__xdata uint8_t *cfg)
{
    if (cfg) {
        cfg[0] = 1;
        cfg[2] = 1;
        cfg[6] = 2;
    }
    return 0;
}

/* Field for sel 4: mode 1 0x40, 3 0xA0, else 0. */
uint8_t csc_conv_field_sel4(uint8_t mode)
{
    if (mode == 3)
        return 0xA0;
    if (mode == 1)
        return 0x40;
    return 0;
}

/* Enable stage C5:11 with field csc_conv_field(sel, mode) in bits 7:4. */
void csc_ycbcr_to_rgb_enable(uint8_t sel, uint8_t mode)
{
    uint8_t v = R_C5_11;

    R_C5_11 = (v & 0x0F) | csc_conv_field(sel, mode) | 0x01;
}

/* Enable stage C5:0C with field csc_conv_field(sel, mode) in bits 7:4. */
void csc_rgb_to_ycbcr_enable(uint8_t sel, uint8_t mode)
{
    uint8_t v = R_C5_0C;

    R_C5_0C = (v & 0x0F) | csc_conv_field(sel, mode) | 0x01;
}

/* Enable stage C5:10. */
void csc_422_to_444_enable(void)
{
    R_C5_10 |= 0x01;
}

/* Enable stage C5:0D. */
void csc_444_to_422_enable(void)
{
    R_C5_0D |= 0x01;
}
