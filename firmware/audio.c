// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Audio: input and output path setup, sample-rate tracking, on/off and the
 * host's audio gate.
 *
 * The audio stage (audio_stage, changed only by set_audio_stage) advances in
 * audio_stage_step, called from the main loop:
 *   1  load the audio config (types and host-gate enable)          -> 2
 *   2  set up the input path (audio_in_sel) and output path (audio_out_sel)     -> 3
 *   3  wait for audio info (audio_info_valid = 1, set by audio_task)         -> 4
 *   4  entering it posts SYS_EVENT_AUDIO_READY; with the gate latch
 *      set, turn the output pins on                                -> 5
 * audio_task (stage >= 3) samples the audio info once the TX state
 * reaches 6: a valid sample rate sets audio_info_valid and records the info the host
 * reads at E0:AA/AB/AD; a lost one turns the output off, posts
 * SYS_EVENT_AUDIO_OFF and drops back to stage 3. audio_gate (stage >= 4)
 * applies the host's E0:B1 gate through audio_gate_latch and, when it
 * opens, restarts stage 4.
 *
 * Registers: the CA page audio block, the C0 page audio status (C0:15,
 * 2C, F1, F5), the per-pin fields E6:4D..5F and E1:18 / E6:3B.
 */
#include <string.h>

#include "fw.h"

enum audio_in { AUDIO_IN_DP = 0, AUDIO_IN_HDMI = 1 };
enum audio_out { AUDIO_OUT_I2S = 0, AUDIO_OUT_TDM = 1, AUDIO_OUT_SPDIF = 2 };

/* Pin numbers (pin_config's) of the audio outputs. */
#define SPDIF_PIN 0x0A
#define I2S_PIN_FIRST 0x0F
#define I2S_PIN_LAST 0x12

/* Set up the audio input path: 0 DP, 1 HDMI; other values do
 * nothing. */
void audio_in_type(uint8_t type)
{
    uint8_t v;

    if (type > AUDIO_IN_HDMI)
        return;
    v = R_CA_05 & 0xFE;
    R_CA_05 = v;
    v &= 0xF9;
    R_CA_05 = v;
    R_CA_05 = v | 0x04;
    R_CA_08 |= 0x01;
    R_CA_11 = 0x24;
    R_CA_17 |= 0x10;
    v = R_CA_25 & 0xF0;
    R_CA_25 = v;
    R_CA_25 = v | 0x02;
    if (type == AUDIO_IN_DP)
        R_CA_26 &= 0x7F;
    else
        R_CA_26 |= 0x80;
    R_CA_28 |= 0x10;
    R_CA_2A = 0x18;
    R_CA_2B = 0x6A;
    R_CA_2C = 0;
    R_CA_2D = 1;
    v = R_CA_2F & 0xFC;
    R_CA_2F = v;
    if (type == AUDIO_IN_HDMI)
        R_CA_2F = v | 0x01;
    R_CA_4A |= 0x20;
    if (type == AUDIO_IN_DP) {
        R_CA_4B = 0x80;
        v = R_CA_50 & 0x3F;
        R_CA_50 = v;
        R_CA_50 = v | 0x40;
        R_CA_60 = 0x02;
        R_CA_61 = 0;
    } else {
        R_CA_4B = 0x35;
        R_CA_50 &= 0x3F;
        R_CA_60 = 0;
        R_CA_61 = 0x64;
    }
}

/* Per pin (index into E6:4D..5F, bit 7: high nibble); 0xFF: no field. */
#define PF(off, hi) ((off) | ((hi) ? 0x80 : 0))
static const __code uint8_t pin_field[0x2B] = {
    PF(0x0C, 1), PF(0x0D, 1), PF(0x0D, 0), PF(0x0C, 0), /* 0x00 */
    PF(0x02, 0), PF(0x03, 1), PF(0x00, 1), PF(0x02, 1),
    PF(0x01, 1), PF(0x01, 0), PF(0x03, 0), PF(0x08, 1), /* 0x08 */
    PF(0x06, 1), PF(0x05, 0), PF(0x05, 1), PF(0x04, 0),
    PF(0x06, 0), PF(0x07, 1), PF(0x07, 0), PF(0x04, 1), /* 0x10 */
    PF(0x0E, 1), PF(0x0E, 0), PF(0x0F, 1), PF(0x0F, 0),
    PF(0x12, 1), 0xFF, 0xFF, 0xFF,                       /* 0x18 */
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, PF(0x08, 0), 0xFF,                       /* 0x20 */
    0xFF, 0xFF, 0xFF, PF(0x0A, 1),
    PF(0x0A, 0), PF(0x0B, 1), PF(0x0B, 0),               /* 0x28 */
};

/* OR val into the pin's 4-bit field in E6:4D..5F (one read, one
 * write); pins without a field are ignored. */
void pin_field_or(uint8_t pin, uint8_t val)
{
    uint8_t f;

    if (pin >= sizeof pin_field)
        return;
    f = pin_field[pin];
    if (f == 0xFF)
        return;
    if (f & 0x80)
        R_E6_4D[f & 0x1F] |= (uint8_t)(val << 4);
    else
        R_E6_4D[f] |= val;
}

/* Set up the audio output path: 0 I2S, 1 TDM, 2 S/PDIF; other
 * values do nothing. */
void audio_out_type(uint8_t type)
{
    uint8_t v, b;

    if (type > AUDIO_OUT_SPDIF)
        return;
    R_CA_26 &= 0xF7;
    v = R_CA_0D & 0x3F;
    R_CA_0D = v;
    R_CA_0D = v | 0x40;
    if (type == AUDIO_OUT_SPDIF) {
        R_CA_08 |= 0xF0;
        R_CA_1A &= 0x01;
        R_CA_53 &= 0xFD;
        R_CA_4B = 0x40;
        return;
    }
    if (type == AUDIO_OUT_I2S)
        b = R_CA_0B & 0x7F;
    else
        b = R_CA_0B | 0x80;
    R_CA_0B = b;
    R_CA_0A |= 0x0F;
    v = R_CA_1A & 0xF1;
    R_CA_1A = v;
    R_CA_1A = v | 0xF0;
    if (type == AUDIO_OUT_I2S) {
        v = R_CA_53 & 0xFD;
        R_CA_53 = v;
        R_CA_53 = v & 0xFE;
        R_CA_0B = b | 0x01;
        R_CA_0C |= 0x08;
    } else {
        R_CA_0E &= 0x1F;
        R_CA_53 &= 0xFD;
        R_CA_4B = 0x40;
        R_CA_42 = 0;
        R_CA_0B = b | 0x02;
        R_CA_0B = b | 0x03;
    }
}

/* Track the audio info (stage >= 3, TX state >= 6, audio_rx_stable == 1),
 * then (stage >= 4) set CA:2F bit 2 from C0:F1 bit 4. */
void audio_task(void)
{
    uint16_t rate;
    uint8_t a, b, c;

    if (audio_stage >= 3) {
        if (audio_rx_stable == 1 && tx_state_get() >= 6) {
            a = audio_channel_count_read();
            b = audio_word_length_read();
            rate = audio_rate_read();
            c = audio_ca_8d_bit1_get();
            if (audio_fs() == 1)
                audio_event_post(3);
            if (rate == 0x00FF || rate == 0 || c == 1) {
                if (audio_info_valid == 1) {
                    audio_info_valid = 0;
                    audio_on_off(audio_out_sel, 0);
                    audio_event_post(2);
                    set_audio_stage(3);
                }
            } else if (!c && !audio_info_valid) {
                audio_info_valid = 1;
                audio_channels = a;
                audio_word_length = b;
                audio_rate_h = rate >> 8;
                audio_rate_l = (uint8_t)rate;
            }
        } else if (audio_info_valid == 1) {
            audio_info_valid = 0;
            audio_on_off(audio_out_sel, 0);
            set_audio_stage(3);
        }
    }
    if (audio_stage >= 4)
        audio_ca_2f_bit2_set(audio_c0_f1_bit4_get() == 1 ? 1 : 0);
}

/* The sample rate in kHz from CA:7C, snapped to 192, 176, 96, 88,
 * 48, 44 or 32 when within 1; 0 for 0, 0xFF for anything else. */
uint16_t audio_rate_read(void)
{
    static const __code uint8_t rates[] = { 192, 176, 96, 88, 48, 44, 32 };
    uint8_t r = R_CA_7C;
    uint8_t i;

    for (i = 0; i < sizeof rates; i++)
        if ((uint8_t)(r - (rates[i] - 1)) <= 2)
            return rates[i];
    return r ? 0xFF : 0;
}

/* Turn the audio output pins on (on == 1) or off for output type
 * out: the S/PDIF pin, or the four I2S/TDM pins. */
void audio_on_off(uint8_t out, uint8_t on)
{
    uint8_t mode = on == 1 ? 0x0C : 0;
    uint8_t pin;

    if (out == AUDIO_OUT_SPDIF) {
        if (on == 1)
            pin_field_or(SPDIF_PIN, 1);
        pin_config(SPDIF_PIN, mode, 0);
        return;
    }
    for (pin = I2S_PIN_FIRST; pin <= I2S_PIN_LAST; pin++)
        pin_config(pin, mode, 0);
}

/* Apply the host's audio gate (stage >= 4): closing it turns the
 * output off; opening it latches and restarts stage 4. */
void audio_gate(void)
{
    if (audio_stage < 4)
        return;
    if (!host_audio_gate(audio_host_gate_en, HOST_AUDIO_GATE)) {
        if (audio_gate_latch == 1) {
            audio_gate_latch = 0;
            audio_on_off(audio_out_sel, 0);
        }
    } else if (!audio_gate_latch) {
        audio_gate_latch = 1;
        set_audio_stage(4);
    }
}

/* Advance the audio stage (see the file comment). */
void audio_stage_step(void)
{
    switch (audio_stage) {
    case 1:
        audio_config_default();
        set_audio_stage(2);
        break;
    case 2:
        audio_in_type(audio_in_sel);
        audio_out_type(audio_out_sel);
        audio_path_setup_finish();
        set_audio_stage(3);
        break;
    case 3:
        if (audio_info_valid == 1)
            set_audio_stage(4);
        break;
    case 4:
        if (audio_gate_latch == 1) {
            audio_on_off(audio_out_sel, 1);
            set_audio_stage(5);
        }
        break;
    }
}

/* A count from C0:F5 bits 2..0 (+1), or 8 / 2 by C0:F1 bit 3 when
 * those are 0; also mirrors C0:F1 bit 3 into C0:15 bit 0 and CA:26 bit 3. */
uint8_t audio_channel_count_read(void)
{
    uint8_t n = R_C0_F5 & 0x07;

    if (n)
        n++;
    else
        n = (R_C0_F1 & 0x08) ? 8 : 2;
    if (R_C0_F1 & 0x08) {
        R_C0_15 |= 0x01;
        R_CA_26 |= 0x08;
    } else {
        R_C0_15 &= 0xFE;
        R_CA_26 &= 0xF7;
    }
    return n;
}

/* Load the audio config; the getter reports no config, so the
 * values set by audio_config_default stay. */
void audio_config_load(void)
{
    __xdata struct audio_config cfg;

    memset(&cfg, 0, sizeof cfg);
    if (audio_config_get(&cfg)) {
        audio_in_sel = cfg.in_type;
        audio_out_sel = cfg.out_type;
        audio_host_gate_en = cfg.host_gate_en;
    }
}

/* When CA:81 bit 0 and CA:97 bit 2 are set, strobe CA:3A bit 2 and
 * return 1; else 0. */
uint8_t audio_fs(void)
{
    if (!(R_CA_81 & 0x01) || !(R_CA_97 & 0x04))
        return 0;
    REG_PULSE(R_CA_3A, 0x04);
    (void)R_CA_7C; /* read, value unused */
    return 1;
}

/* C0:2C bits 3..0 mapped 2 -> 16, 3 and 10 -> 20, 11 -> 24; other
 * codes are returned as read. */
uint8_t audio_word_length_read(void)
{
    uint8_t v = R_C0_2C & 0x0F;

    if (v == 0x02)
        return 16;
    if (v == 0x03 || v == 0x0A)
        return 20;
    if (v == 0x0B)
        return 24;
    return v;
}

/* Set the audio stage; entering stage 4 posts audio ready. */
void set_audio_stage(uint8_t stage)
{
    if (audio_stage == stage)
        return;
    if (stage == 4)
        audio_event_post(1);
    audio_stage = stage;
}

/* Fill an audio config with in DP, out I2S, host gate off; returns
 * 0 (no config), so callers keep their own values. */
uint8_t audio_config_get(__xdata struct audio_config *cfg)
{
    if (cfg) {
        cfg->in_type = AUDIO_IN_DP;
        cfg->out_type = AUDIO_OUT_I2S;
        cfg->host_gate_en = 0;
    }
    return 0;
}

/* Final audio path enables after the type setup. */
void audio_path_setup_finish(void)
{
    uint8_t v;

    R_E6_3B &= 0xFD;
    R_CA_5A |= 0x40;
    v = R_E1_18 & 0xBF;
    R_E1_18 = v;
    R_E1_18 = v | 0x40;
}

/* Post the audio SystemEvent: 1 ready, 2 off; others nothing. */
void audio_event_post(uint8_t what)
{
    if (what == 2)
        sys_event_post(SYS_EVENT_AUDIO_OFF);
    else if (what == 1)
        sys_event_post(SYS_EVENT_AUDIO_READY);
}

/* CA:2F bit 2 on (on == 1) or off. */
void audio_ca_2f_bit2_set(uint8_t on)
{
    if (on == 1)
        R_CA_2F |= 0x04;
    else
        R_CA_2F &= 0xFB;
}

/* Default audio config (DP in, I2S out, host gate ignored), then
 * the config load. */
void audio_config_default(void)
{
    audio_in_sel = AUDIO_IN_DP;
    audio_out_sel = AUDIO_OUT_I2S;
    audio_host_gate_en = 0;
    audio_config_load();
}

/* CA:8D bit 1. */
uint8_t audio_ca_8d_bit1_get(void)
{
    return (R_CA_8D & 0x02) ? 1 : 0;
}

/* C0:F1 bit 4. */
uint8_t audio_c0_f1_bit4_get(void)
{
    return (R_C0_F1 & 0x10) ? 1 : 0;
}

/* The TX state. */
uint8_t tx_state_get(void)
{
    return tx_state;
}
