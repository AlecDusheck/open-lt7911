// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* HDCP: key check, repeater and decoder setup, and the HDCP state the main
 * loop polls.
 *
 * Setup (hdcp_init, once): hdcp_config_load loads the configuration through the hook
 * hdcp_config_get into hdcp_mode (mode) and hdcp_versions (enabled versions: 0x10
 * HDCP 1.x, 0x20 HDCP 2.x). The hook returns mode 0x20 (decoder) with versions
 * 0x30, so hdcp_decoder(0x30) runs; mode 0x10 selects hdcp_repeater. Both
 * first check the key (C4:BA bit 2) and publish it on HOST_HDCP bit 7; with
 * no valid key, or no version enabled, HDCP is turned off (C3:C4 bit 1 set).
 *
 * State (hdcp_state_poll, every main-loop pass once RxState >= 8): hdcp_active_version
 * reads which version is active (C4:F0 bit 0 HDCP 1.x, else
 * C4:B3 bit 0 HDCP 2.x) into hdcp_state.
 */
#include <string.h>

#include "fw.h"

#define HDCP_1X 0x10
#define HDCP_2X 0x20

/* HDCP configuration hook: if cfg is given, cfg[0] = mode 0x20
 * (decoder), cfg[1] = versions 0x30. Returns 1 (use it). */
uint8_t hdcp_config_get(__xdata uint8_t *cfg)
{
    if (cfg) {
        cfg[1] = HDCP_1X | HDCP_2X;
        cfg[0] = 0x20;
    }
    return 1;
}

/* HDCP repeater setup for the enabled versions. */
void hdcp_repeater(uint8_t versions)
{
    const uint8_t count = 1;
    uint8_t i, v;

    if (!(R_C4_BA & 0x04)) {
        hdcp_repeater_disable(1);
        HOST_HDCP &= 0x7F;
        return;
    }
    HOST_HDCP |= 0x80;
    if (versions == 0) {
        R_C4_14 = 0;
        R_C4_15 = 0;
        R_C4_16 = 0;
        R_C4_36 &= ~0x01;
        hdcp_repeater_disable(1);
        return;
    }
    hdcp_repeater_disable(0);
    if (versions & HDCP_1X) {
        v = R_C4_2E & ~0x40;
        R_C4_2E = v;
        R_C4_2E = v | 0x40;
        for (i = 0; i < count; i++) {
            R_C4_B7 = 0xD8;
            delay_short(1);
            R_C4_B7 = 0x2C;
            delay_short(1);
            R_C4_B7 = 0x9A;
            delay_short(1);
            R_C4_B7 = 0x71;
            delay_short(1);
            R_C4_B7 = 0x67;
        }
        R_C4_2E &= ~0x40;
    }
    if (versions & HDCP_1X) {
        R_C4_14 = 0;
        R_C4_15 = 0;
        R_C4_16 = 0;
        R_C4_31 |= 0x01;
        v = R_C4_36 | 0x01;
        R_C4_36 = v;
        R_C4_36 = v | 0x02;
        R_C4_37 = count;
        R_C4_38 = 1;
        v = R_C4_39 | 0x40;
        R_C4_39 = v;
        v |= 0x02;
        R_C4_39 = v;
        R_C4_39 = v | 0x01;
        R_C4_3E |= 0x02;
    }
    if (versions & HDCP_2X) {
        v = R_C4_12 | 0x04;
        R_C4_12 = v;
        R_C4_12 = v | 0x08;
        R_C4_13 |= 0x08;
        R_C4_14 = 3;
        R_C4_15 = 0;
        R_C4_16 = 2;
        R_C4_21 = count << 4;
        R_C4_22 = 2;
        R_C4_31 |= 0x01;
        R_C4_3E |= 0x01;
    }
}

/* HDCP decoder (sink) setup for the enabled versions. */
void hdcp_decoder(uint8_t versions)
{
    uint8_t v;

    if (!(R_C4_BA & 0x04)) {
        hdcp_decoder_disable(1);
        HOST_HDCP &= 0x7F;
        return;
    }
    HOST_HDCP |= 0x80;
    if (versions == 0) {
        R_C4_14 = 0;
        R_C4_15 = 0;
        R_C4_16 = 0;
        R_C4_36 &= ~0x01;
        hdcp_decoder_disable(1);
        return;
    }
    hdcp_decoder_disable(0);
    if (versions & HDCP_1X) {
        v = R_C4_36 & ~0x02;
        R_C4_36 = v;
        R_C4_39 |= 0x02;
        R_C4_36 = v | 0x01;
        R_C4_14 = 0;
        R_C4_15 = 0;
        R_C4_16 = 0;
    }
    if (versions & HDCP_2X) {
        v = R_C4_12 & ~0x08;
        R_C4_12 = v;
        R_C4_12 = v | 0x04;
        R_C4_14 = 2;
        R_C4_15 = 0;
        R_C4_16 = 2;
    }
}

/* Once RxState >= 8, track the authenticated HDCP version in
 * hdcp_state. */
void hdcp_state_poll(void)
{
    uint8_t state;

    if (rx_state_get() < 8)
        return;
    state = hdcp_active_version();
    if (hdcp_state != state)
        hdcp_state = state;
}

/* Load the HDCP configuration and set up decoder or repeater. */
void hdcp_init(void)
{
    hdcp_versions = 0;
    hdcp_mode = 0x20;
    hdcp_config_load();
    if (hdcp_mode == 0x20)
        hdcp_decoder(hdcp_versions);
    else if (hdcp_mode == 0x10)
        hdcp_repeater(hdcp_versions);
}

/* Read the HDCP configuration hook into hdcp_mode (mode) and hdcp_versions
 * (versions). */
void hdcp_config_load(void)
{
    uint8_t cfg[3];

    memset(cfg, 0, sizeof cfg);
    if (hdcp_config_get(cfg)) {
        hdcp_mode = cfg[0];
        hdcp_versions = cfg[1];
    }
}

/* The HDCP version in use: HDCP_1X, HDCP_2X or 0. */
uint8_t hdcp_active_version(void)
{
    if (R_C4_F0 & 0x01)
        return HDCP_1X;
    if (R_C4_B3 & 0x01)
        return HDCP_2X;
    return 0;
}

/* C3:C4 bit 1 set when off = 1 (no key or no version), else clear; decoder path. */
void hdcp_decoder_disable(uint8_t off)
{
    if (off == 1)
        R_C3_C4 |= 0x02;
    else
        R_C3_C4 &= ~0x02;
}

/* As hdcp_decoder_disable, for the repeater path. */
void hdcp_repeater_disable(uint8_t off)
{
    if (off == 1)
        R_C3_C4 |= 0x02;
    else
        R_C3_C4 &= ~0x02;
}

/* RxState. */
uint8_t rx_state_get(void)
{
    return rx_state;
}
