// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* The host mailbox E0:80..B1, the firmware's only external contract.
 *
 * The host (lt7911_manage on the SoC) reads E0:80..AF over I2C and may write
 * only E0:B0/B1:
 *   E0:81..83  firmware version (host_version_init, once at boot)
 *   E0:84      interrupt type: 0 video off, 1 video ready, 2 audio off,
 *              3 audio ready; the host reads it on the IRQ pin's falling edge
 *   E0:85..8F  pixel clock (kHz, 24-bit) and htotal, vtotal, width, height
 *              (16-bit), all big-endian; the host derives the frame rate
 *   E0:90..A4  further timing, MIPI and colour fields no host driver reads
 *   E0:A8      host marker (host_marker_publish)
 *   E0:AA, AB, AD  audio info; E0:AD is the sample rate in kHz
 *   E0:B0/B1   host -> MCU gates for MIPI output and audio, honoured only
 *              when mipi_host_gate_en / audio_host_gate_en is 1
 *
 * Events: the video and audio code post a SystemEvent (sys_event_post). The
 * main loop's host_event_poll takes a pending event when no IRQ pulse is in
 * progress, publishes the matching info block (host_publish_info) and starts
 * a pulse; irq_pulse drives the IRQ pin (GPIO 0x0B) low, releases it after
 * IRQ_PULSE_TICKS of the free-running counter and then delivers the one
 * event that may have been queued meanwhile:
 *
 *   idle --post--> pending --poll, no pulse--> pulse (pin low) --timeout--> idle
 *                     \--post--> queued (one slot, last post wins)
 *
 * The published values come from host_* snapshots (vars.h), each filled
 * from the subsystem's state through a small info getter.
 */
#include <string.h>

#include "fw.h"

/* Reported at E0:81..83: FW_VERSION_0..2, the parts of build.env FW_VERSION a.b.c. */
#if !defined(FW_VERSION_0) || !defined(FW_VERSION_1) || !defined(FW_VERSION_2)
#error "FW_VERSION is required (build.env)"
#endif

#define IRQ_PIN 0x0B
#define IRQ_PULSE_TICKS 800000UL

/* E0:84 values, as the host driver names them. */
enum host_int_type {
    HOST_INT_VIDEO_OFF = 0,
    HOST_INT_VIDEO_READY = 1,
    HOST_INT_AUDIO_OFF = 2,
    HOST_INT_AUDIO_READY = 3,
};

/* rx_timing_read indices published at E0:88..9B, two bytes each, in order:
 * htotal, vtotal, width, height, then six fields no host driver reads. */
static const __code uint8_t published_timing[10] = { 0, 5, 1, 6, 2, 3, 4, 7, 8, 9 };

/* Publish the info block for SystemEvent ev and set E0:84; other
 * events change nothing. Refreshes every snapshot first. */
void host_publish_info(uint8_t ev)
{
    volatile __xdata uint8_t *p;
    uint32_t v;
    uint8_t i, idx, cs;

    mipi_info_snapshot();
    video_info_snapshot();
    audio_info_snapshot();

    switch (ev) {
    case SYS_EVENT_VIDEO_READY:
        HOST_INT_TYPE = HOST_INT_VIDEO_READY;
        v = pclk_khz << 1;
        HOST_PCLK_H = (uint8_t)(v >> 16) & 0x0F;
        HOST_PCLK_M = (uint8_t)(v >> 8);
        HOST_PCLK_L = (uint8_t)v;
        /* rx_timing_read runs twice per field: the RX measurement
         * registers are read once per published byte. */
        p = &HOST_HTOTAL_H;
        for (i = 0; i < sizeof published_timing; i++) {
            idx = published_timing[i];
            *p++ = rx_timing_read(idx) >> 8;
            *p++ = (uint8_t)rx_timing_read(idx);
        }
        R_E0_9C = host_color_format;
        R_E0_9D = (uint8_t)(host_color_range << 4) + host_color_depth;
        v = host_lmtx_read_clk;
        R_E0_A0 = (uint8_t)(v >> 16) & 0x0F;
        R_E0_A1 = (uint8_t)(v >> 8);
        R_E0_A2 = (uint8_t)(v >> 3);
        R_E0_A3 = (uint8_t)(host_mipi_port_num << 4) + host_mipi_lane_num;
        cs = host_mipi_fmt;
        HOST_COLOR_SPACE = cs == 0x0C ? 1 : cs == 0x0A ? 2 : 0;
        break;
    case SYS_EVENT_VIDEO_OFF:
        for (p = &HOST_INT_TYPE; p <= &R_E0_9D; p++)
            *p = 0;
        HOST_HDCP &= 0x80;
        for (p = &R_E0_A0; p <= &R_E0_A6; p++)
            *p = 0;
        break;
    case SYS_EVENT_AUDIO_READY:
        HOST_INT_TYPE = HOST_INT_AUDIO_READY;
        R_E0_AA = host_audio_channels;
        R_E0_AB = host_audio_word_length;
        HOST_AUDIO_FS = (uint8_t)host_audio_rate;
        break;
    case SYS_EVENT_AUDIO_OFF:
        HOST_INT_TYPE = HOST_INT_AUDIO_OFF;
        R_E0_AA = 0;
        R_E0_AB = 0;
        HOST_AUDIO_FS = 0;
        break;
    }
}

/* One RX timing measurement by index. 0..4 are horizontal fields
 * the hardware counts in pixel pairs (doubled here): 0 htotal, 1 width;
 * 5..9 vertical: 5 vtotal, 6 height; 10/11 are E4:A3 bits 4/5; others 0. */
uint16_t rx_timing_read(uint8_t idx)
{
    /* Offset of each measurement's high byte from E4:7A. */
    static const __code uint8_t timing_reg[10] = {
        0x0E, 0x12, 0x0A, 0x02, 0x08, 0x0C, 0x10, 0x06, 0x00, 0x04,
    };
    uint8_t o, h;
    uint16_t v;

    if (idx < 10) {
        o = timing_reg[idx];
        h = R_E4_7A_8D[o];
        v = (h << 8) | R_E4_7A_8D[o + 1];
        return idx < 5 ? v << 1 : v;
    }
    if (idx == 10)
        return (R_E4_A3 >> 4) & 1;
    if (idx == 11)
        return (R_E4_A3 >> 5) & 1;
    return 0;
}

/* Run the IRQ pulse. While active: pull the pin low when it reads
 * high, release it once it reads low and IRQ_PULSE_TICKS have passed, then
 * post the queued SystemEvent, if any. */
void irq_pulse(void)
{
    if (!irq_pulse_active)
        return;
    if (irq_pin_level() == 1) {
        gpio_write(IRQ_PIN, 0);
        timer_stamp(&irq_pulse_stamp);
    }
    if (!irq_pulse_active || irq_pin_level() != 0)
        return;
    if (timer_elapsed(&irq_pulse_stamp, IRQ_PULSE_TICKS) != 1)
        return;
    irq_pulse_active = 0;
    gpio_write(IRQ_PIN, 1);
    if (sys_event_queued) {
        sys_event_post(sys_event_queued);
        sys_event_queued = 0;
    }
}

/* Publish the firmware version at E0:81..83, clear E0:80. */
void host_version_init(void)
{
    HOST_80 = 0;
    HOST_FW_VER_0 = FW_VERSION_0;
    HOST_FW_VER_1 = FW_VERSION_1;
    HOST_FW_VER_2 = FW_VERSION_2;
    /* Read back; the values are unused. */
    (void)HOST_FW_VER_0;
    (void)HOST_FW_VER_1;
    (void)HOST_FW_VER_2;
}

/* Snapshot the MIPI TX info (port and lane count, the mipi_fmt code,
 * input and read clocks) for the video info block. */
void mipi_info_snapshot(void)
{
    struct mipi_tx_info info;

    memset(&info, 0, sizeof info);
    mipi_tx_info_get(&info);
    host_mipi_port_num = info.port_num;
    host_mipi_lane_num = info.lane_num;
    host_mipi_fmt = info.fmt;
    pclk_khz = info.in_clk;
    host_lmtx_read_clk = info.read_clk;
}

/* Main-loop host task: publish a pending SystemEvent and start its
 * IRQ pulse when none is in progress, then run the pulse. */
void host_event_poll(void)
{
    uint8_t ev;

    if (sys_event_is_pending() == 1 && !irq_pulse_active) {
        ev = sys_event_get();
        sys_event_clear();
        switch (ev) {
        case SYS_EVENT_VIDEO_READY:
        case SYS_EVENT_VIDEO_OFF:
        case SYS_EVENT_AUDIO_READY:
        case SYS_EVENT_AUDIO_OFF:
            host_publish_info(ev);
            irq_pulse_active = 1;
        }
    }
    irq_pulse();
}

/* Fill a MIPI TX info block (nothing for a null pointer). */
void mipi_tx_info_get(__xdata struct mipi_tx_info *info)
{
    if (!info)
        return;
    info->port_num = mipi_port_num;
    info->lane_num = mipi_lane_num;
    info->fmt = mipi_fmt;
    info->in_clk = mipi_in_clk;
    info->read_clk = lmtx_read_clk;
}

/* Snapshot the audio info for the audio info block. */
void audio_info_snapshot(void)
{
    struct audio_info info;

    memset(&info, 0, sizeof info);
    audio_info_get(&info);
    host_audio_channels = info.channels;
    host_audio_word_length = info.word_length;
    host_audio_rate = info.rate;
}

/* Snapshot the video_info_get info (E0:9C/9D). */
void video_info_snapshot(void)
{
    struct video_info info;

    memset(&info, 0, sizeof info);
    video_info_get(&info);
    host_color_format = info.color_format;
    host_color_depth = info.color_depth;
    host_color_range = info.color_range;
}

/* Post a SystemEvent; while one is pending it goes to the single
 * queued slot instead. */
void sys_event_post(uint8_t ev)
{
    if (sys_event_pending == 1) {
        sys_event_queued = ev;
        return;
    }
    sys_event_pending = 1;
    sys_event = ev;
}

/* Fill an audio info block. */
void audio_info_get(__xdata struct audio_info *info)
{
    info->channels = audio_channels;
    info->word_length = audio_word_length;
    info->rate = (uint16_t)(audio_rate_h << 8) | audio_rate_l;
}

/* Fill a video_info block. */
void video_info_get(__xdata struct video_info *info)
{
    info->color_format = rx_color_snap_format;
    info->color_depth = rx_color_snap_depth;
    info->color_range = rx_color_snap_range;
}

/* Audio allowed? 1 unless the host gate is honoured (en == 1) and
 * the host wrote 0 to E0:B1. */
uint8_t host_audio_gate(uint8_t en, uint8_t gate)
{
    if (en == 1)
        return gate ? 1 : 0;
    return 1;
}

/* MIPI output allowed? Same rule for E0:B0. */
uint8_t host_mipi_gate(uint8_t en, uint8_t gate)
{
    if (en == 1)
        return gate ? 1 : 0;
    return 1;
}

/* SystemEvent pending flag. */
uint8_t sys_event_is_pending(void)
{
    return sys_event_pending;
}

/* Clear the SystemEvent pending flag. */
void sys_event_clear(void)
{
    sys_event_pending = 0;
}

/* The current SystemEvent. */
uint8_t sys_event_get(void)
{
    return sys_event;
}

/* The host marker latch (fw.h), set from INT0 by the AUX request handlers
 * and published here. */
volatile __xdata uint8_t host_marker;

/* Main loop, after the video poll: publish the marker on E0:A8 (bit 7: the
 * firmware runs). RX_STATE_WAIT_PLUG (unplugged, HPD low) clears it, so
 * each plug starts afresh. */
void host_marker_publish(void)
{
    if (rx_state == RX_STATE_WAIT_PLUG)
        host_marker = 0;
    HOST_MARKER = host_marker | HOST_MARK_ALIVE;
}
