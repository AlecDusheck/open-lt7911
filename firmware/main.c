// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Startup, the boot sequence and the main loop.
 *
 * Everything runs from one loop: the PD task, then the video poll (DP RX
 * state machine, MIPI TX state machine, audio, HDCP state, host events),
 * then the host marker.
 * Interrupts (isr.c) do the time-critical parts: AUX replies, PD receive,
 * PD timers.
 */
#include "fw.h"

void main(void)
{
    if (sys_e1_02_low_read() == 0)
        pin_config(0x22, 4, 0);
    sys_init();
    dpcd_ext_caps_init();
    state_reset();
    input_select();
    edid_load();
    for (;;) {
        pd_main_task();
        video_poll();
        host_marker_publish();
    }
}

void video_poll(void)
{
    rx_poll();
    tx_poll();
    audio_poll();
    hdcp_state_poll();
    host_event_poll();
}

void rx_poll(void)
{
    rx_plug();
    rx_events();
    rx_state_machine();
}

void tx_poll(void)
{
    tx_state_machine();
    tx_output_gate();
}

void audio_poll(void)
{
    audio_task();
    audio_stage_step();
    audio_gate();
}
