// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* USB Type-C attach and roles on the data port (Try.SRC, UFP/DFP, Rp/Rd), PD
 * state reset, and VBUS detection on the power port.
 *
 * Both ports have the same CC hardware on their own registers:
 *   data port   E2:2D bit 2 + C7:3E bit 0 (pull select, pd0_cc_pull_set),
 *               C7:30 (CC levels: bits 7:6 and 5:4 one per CC line),
 *               C7:2F bit 1 (CC line select), E2:2E bits 3:2 (pd0_rp_level_set)
 *   power port  E2:39 bit 2 + C7:BE bit 0 (pd1_cc_pull_set), C7:B0, C7:AF bit 1,
 *               E2:3A bits 3:2 (pd1_rp_level_set); C7:AF bit 4 is power-port VBUS.
 * pd0_cc_pull_set(2) sets the pull bits and pd0_cc_pull_set(1) clears them. With the bits
 * set, CC seen on C7:30 & 0x50 makes the data port attach as DFP
 * (source); with them clear, any C7:30 & 0xF0 makes it attach as UFP
 * (sink).
 *
 * Attach state machine (attach_timer_task, main loop; state attach_state):
 *   0  stamp attach_stamp and load attach_timeout for the current pull setting
 *      attach_pull_toggle (1: 2 800 000 ticks, 0: 1 200 000), go to 1.
 *   1  wait for power-port VBUS (vbus_check(1)) with pd0_caps_exchanged, pr_swap_awaiting != 2 and
 *      pd0_power_role_state clear; then attach as DFP (pd0_cc_wait_dfp) or UFP (pd0_cc_wait_ufp) when the
 *      pull setting pd0_cc_pull_flag agrees, and go to 2; otherwise, when the timeout
 *      expires, flip attach_pull_toggle and re-apply the pulls (attach_pull_apply), restarting the
 *      timer. Any failed precondition (or pd1_adapter_state set) returns to 0.
 *   2  attached: nothing until something resets attach_state.
 * typec_attach() resets the PD state (pd_state_reset), records the role in
 * pd0_power_role/pd0_data_role/pd0_vconn_source and selects the CC line (pd0_cc_line_select). A UFP
 * attach with Try.SRC attempts left (try_src_count) instead spends one: pulls up
 * (Rp side), pulses E2:26 bit 5, and leaves the attach to the next pass.
 */
#include <string.h>

#include "fw.h"

/* An empty message header, where the PD state reset points the direct-send
 * templates. */
static const __code uint8_t tpl_empty[2] = { 0x00, 0x00 };

/* Power-port Request (PD 2.0 header 0x1042, one RDO 0x1301685A). */
static const __code uint8_t tpl_pd1_request_5v[6] = { 0x42, 0x10, 0x5A, 0x68, 0x01, 0x13 };

/* Clear the data port's policy state, but not the input configuration
 * (input_typec, typec_receptacle, typec_dp_lane_count, dp_pin_assignment)
 * or try_src_count. */
static void pd0_policy_clear(void)
{
    pd0_power_role = 0;
    pd0_data_role = 0;
    pd0_power_settled = 0;
    dp_multi_function_pref = 0;
    pd_timer_port = 0;
    typec_attach_phase = 0;
    pd0_wait_resend = 0;
    pd0_tx_first_try = 0;
    pd0_cc_line = 0;
    pd0_cc_pull_flag = 0;
    pd0_recovery_req = 0;
    hpd_raised = 0;
    pd0_goodcrc_wait = 0;
    pr_swap_first = 0;
    pr_swap_awaiting = 0;
    pd0_power_role_state = 0;
    dr_swap_req = 0;
    fr_swap_state = 0;
    pd0_caps_exchanged = 0;
    pd0_swap_flags = 0;
    pd0_retry_count = 0;
    pd0_ams_state = 0;
    dp_alt_status = 0;
    src_caps_send_count = 0;
    pd0_retry_limit = 0;
    src_caps_delayed = 0;
    pr_swap_src_t2_pending = 0;
    pd0_vconn_source = 0;
    cable_5a = 0;
    cable_id_state = 0;
    hpd_request = 0;
    pd0_soft_reset_req = 0;
}

/* Reset the data-port PD state: clear its policy state (keeping the input
 * configuration), post HPD request 0, clear pd0_port and the messages built
 * for it, set the PD revision defaults, clear the swap stages, and stop
 * Timer 2. */
void pd_state_reset(void)
{
    pd0_policy_clear();
    hpd_request_post(0);
    cable_id_reset();
    pd0_tx_first_try = 1;
    typec_attach_phase = 1;
    memset(&pd0_port, 0, sizeof pd0_port);
    memset(pd1_request_msg, 0, sizeof pd1_request_msg);
    memset(src_caps_msg, 0, sizeof src_caps_msg);
    src_caps_max_watts = 0;
    pd0_port.request_pdo_offset = 2;
    pd0_port.tx_settled = 1;
    pd0_port.rev = 2;
    pd0_port.tx_direct = (uint8_t *)tpl_empty;
    /* Never sent from here: the reset clears both "parked" flags (pd0_wait_resend,
     * pd0_port.deferred), and every write that sets one also writes
     * pd0_port.deferred_msg. */
    pd0_port.deferred_msg = (uint8_t *)tpl_empty;
    pr_swap_stage = 0;
    vconn_swap_state = 0;
    pd0_bist_test_mode = 0;
    pd0_vconn_off();
    TR2 = 0;
}

/* attach_timer_task's period for the pull setting attach_pull_toggle. */
static void attach_timeout_load(void)
{
    if (attach_pull_toggle == 1)
        attach_timeout = 2800000;
    else
        attach_timeout = 1200000;
}

/* Type-C attach state machine (file comment): DRP toggle between
 * the two pull settings until the data port attaches. */
void attach_timer_task(void)
{
    if (pd1_adapter_state != 0)
        goto idle;
    if (attach_state == 0) {
        timer_stamp(&attach_stamp);
        attach_timeout_load();
        attach_state++;
        return;
    }
    if (attach_state != 1)
        goto idle;
    if (pd0_caps_exchanged != 0 || vbus_check(1) != 1 || pr_swap_awaiting == 2 || pd0_power_role_state != 0)
        goto idle;
    if (pd0_cc_wait_dfp() == 1) {
        if (pd0_cc_pull_flag != 1)
            return;
        typec_attach(2);
    } else if (pd0_cc_wait_ufp() == 1) {
        if (pd0_cc_pull_flag != 0)
            return;
        typec_attach(1);
    } else {
        if (timer_elapsed(&attach_stamp, attach_timeout) != 1)
            return;
        attach_pull_toggle = !attach_pull_toggle;
        attach_pull_apply(attach_pull_toggle);
        attach_timeout_load();
        timer_stamp(&attach_stamp);
        return;
    }
    attach_state++;
    return;
idle:
    attach_state = 0;
}

/* Attach the data port as UFP (role 1) or DFP (role 2); see the
 * file comment for the Try.SRC detour. Posts HPD request 0 at the end. */
void typec_attach(uint8_t role)
{
    uint8_t pulls = pd0_cc_pull_flag;

    pd_state_reset();
    pd0_cc_pull_flag = !pulls;
    typec_attach_phase = 2;
    gpio_write(0x0E, 0);
    if (role == 1) {
        if (try_src_count == 0) {
            pd0_vconn_source = 0;
            pd0_power_role = 0;
            pd0_data_role = 0;
            pd0_cc_line_select();
        } else {
            try_src_count--;
            pd0_cc_enable(0);
            pd0_cc_pull_set(2);
            delay_ms(100);
            pd0_cc_enable(1);
            delay_ms(50);
        }
    } else if (role == 2) {
        pd0_vconn_source = 1;
        pd0_power_role = 1;
        pd0_data_role = 1;
        pd0_cc_line_select();
    }
    hpd_request_post(0);
}

/* Clear pd0_cc_line and, when typec_receptacle is 1, poll the data-port CC
 * levels up to ten times 10 apart for the line in use: line 0 (C7:30 bits
 * 7:6) or line 1 (bits 5:4). The line found goes to pd0_cc_line and C7:2F
 * bit 1, and typec_orient_e2_02_set(3 or 2 by line) runs unless
 * typec_dp_lane_count > 2 (then 0). */
void pd0_cc_line_select(void)
{
    uint8_t pull;
    uint8_t cc;

    pd0_cc_line = 0;
    if (typec_receptacle != 1) {
        pd0_cc_line = 0;
        return;
    }
    cc_poll_count = 0;
    do {
        pull = R_E2_2D;
        if ((!(pull & 0x04) && (R_C7_30 & 0xC0)) || ((cc = R_C7_30) & 0x40)) {
            pd0_cc_line = 0;
            R_C7_2F &= ~0x02;
            cc = 3;
            goto found;
        }
        if ((!(pull & 0x04) && (cc & 0x30)) || (R_C7_30 & 0x10)) {
            pd0_cc_line = 1;
            R_C7_2F |= 0x02;
            cc = 2;
            goto found;
        }
        delay_short(10);
    } while (++cc_poll_count < 10);
    return;
found:
    typec_orient_e2_02_set(typec_dp_lane_count > 2 ? 0 : cc);
}

/* Turn the data-port CC off and send the power-port
 * Request (tpl_pd1_request_5v), waiting up to ~300 ms for pd1_ams_state to leave 4. */
void pd0_error_recovery(void)
{
    uint16_t n;

    pd0_vconn_off();
    delay_ms(1);
    pd0_cc_enable(0);
    delay_ms(26);
    gpio_write(0x0E, 0);
    gpio_write(0x04, 1);
    pd0_c754_bit2_pulse();
    pd0_cc_pull_set(1);
    pd_state_reset();
    switch_docked = 0;
    pd1_port.tx_msg = (uint8_t *)tpl_pd1_request_5v;
    pd1_tx();
    pd1_ams_state = 4;
    delay_ms(100);
    for (n = 0; pd1_ams_state == 4 && n < 300; n++)
        delay_ms(1);
    gpio_write(0x04, 0);
    pd0_cc_enable(1);
}

/* pd1_vbus_drop_calibrate's sweep value, in XRAM (sweep_up reads it with MOVX). */
static __xdata uint8_t sweep_v;

/* pd1_vbus_drop_calibrate's sweep: while sweep_v < 0xFE and C7:AF bit 5 is clear, raise
 * sweep_v by 2 into E2:38 and wait delay_short(5). In assembly, so each
 * step's dwell is fixed. It saves the registers
 * it uses, as SDCC expects of a naked function. */
static void sweep_up(void) __naked
{
    __asm
        push ar4
        push ar6
        push ar7
    00001$:
        mov  dptr, #_sweep_v
        movx a, @dptr
        setb c
        subb a, #0xfd
        jnc  00002$
        mov  dptr, #_R_C7_AF
        movx a, @dptr
        jb   acc.5, 00002$
        mov  dptr, #_sweep_v
        movx a, @dptr
        add  a, #0x02
        movx @dptr, a
        movx a, @dptr
        mov  dptr, #_R_E2_38
        movx @dptr, a
        mov  r7, #0x05
        mov  r6, #0x00
        lcall _delay_short_r7r6
        sjmp 00001$
    00002$:
        pop  ar7
        pop  ar6
        pop  ar4
        ret
    __endasm;
}

/* Power-port threshold sweep: raise E2:38 in steps of 2 until C7:AF
 * bit 5 sets (or past 0xFD), then set E2:38 to 7/8 (below 0x8D) or 15/16 of
 * the trip value, with E2:39-side bit pd1_vbus_drop_event_set held during the sweep. */
void pd1_vbus_drop_calibrate(void)
{
    uint8_t v;

    sweep_v = 0;
    pd1_vbus_drop_event_set(1);
    R_E2_38 = 0;
    sweep_up();
    v = sweep_v;
    if (v >= 3) {
        if (v < 0x8D)
            R_E2_38 = (uint8_t)(((uint16_t)v * 7) >> 3);
        else
            R_E2_38 = (uint8_t)(((uint16_t)v * 15) >> 4);
    }
    delay_ms(1);
    pd1_vbus_drop_event_set(0);
}

/* Data-port CC line tracking: while the pulls are clear and pr_swap_awaiting
 * is not 2, follow the CC level to pd0_cc_line and C7:2F bit 1; then, with CC
 * present but no power-port VBUS and fr_swap_state clear, drive GPIO 0x0E high. */
void pd0_cc_line_track(void)
{
    uint8_t cc;

    if (!(R_E2_2D & 0x04) && pr_swap_awaiting != 2) {
        cc = R_C7_30;
        if (cc & 0xC0) {
            R_C7_2F &= ~0x02;
            pd0_cc_line = 0;
        } else if (cc & 0x30) {
            R_C7_2F |= 0x02;
            pd0_cc_line = 1;
        }
    }
    if ((R_C7_30 & 0xF0) && vbus_check(1) != 1 && fr_swap_state == 0)
        gpio_write(0x0E, 1);
}

/* 1 if pd0_cc_open holds now and on 100 further checks 3 apart. */
uint8_t pd0_detached(void)
{
    uint8_t ok = 0;
    uint8_t i;

    if (pd0_cc_open() == 1) {
        delay_short(3);
        for (i = 0; i < 100; i++) {
            if (pd0_cc_open() != 1)
                return 0;
            ok = 1;
            delay_short(3);
        }
    }
    return ok;
}

/* Power-port VBUS (C7:AF bit 4): 1 if present now or within 100
 * polls 4 apart. Only when bit 0 of flags is set and the power-port pulls
 * (E2:39 bit 2) are clear; else 0. */
uint8_t vbus_check(uint8_t flags)
{
    uint8_t i;

    if (!(flags & 0x01) || (R_E2_39 & 0x04))
        return 0;
    if (R_C7_AF & 0x10)
        return 1;
    for (i = 0; i < 100; i++) {
        delay_short(4);
        if (R_C7_AF & 0x10)
            return 1;
    }
    return 0;
}

/* Re-apply the data-port pulls after a DRP toggle: reset the PD
 * state, pd0_cc_pull_set(2) and pd0_cc_pull_flag = 1 when on, else pd0_cc_pull_set(1), pd0_cc_pull_flag = 0. */
void attach_pull_apply(uint8_t on)
{
    pd_state_reset();
    gpio_write(0x0E, 0);
    typec_attach_phase = 1;
    if (on) {
        pd0_cc_pull_set(2);
        pd0_cc_pull_flag = 1;
    } else {
        pd0_cc_pull_set(1);
        pd0_cc_pull_flag = 0;
    }
}

/* With the data-port pulls set, wait up to 150 polls 10 apart for
 * C7:30 & 0x50: 1 seen, 0 timeout or pulls cleared. */
uint8_t pd0_cc_wait_dfp(void)
{
    uint8_t i;

    for (i = 0; i < 150; i++) {
        if (!(R_E2_2D & 0x04))
            return 0;
        if (R_C7_30 & 0x50)
            return 1;
        delay_short(10);
    }
    return 0;
}

/* With the data-port pulls clear, wait up to 150 polls 10 apart for
 * C7:30 & 0xF0: 1 seen, 0 timeout or pulls set. */
uint8_t pd0_cc_wait_ufp(void)
{
    uint8_t i;

    for (i = 0; i < 150; i++) {
        if (R_E2_2D & 0x04)
            return 0;
        if (R_C7_30 & 0xF0)
            return 1;
        delay_short(10);
    }
    return 0;
}

/* Clear the role and attach flags, GPIO 0x0E low, toggle pd0_cc_pull_flag. */
void pd0_source_detach(void)
{
    pd0_power_role = 0;
    pd0_data_role = 0;
    pd0_power_role_state = 0;
    src_caps_send_count = 0;
    typec_attach_phase = 1;
    pd0_caps_exchanged = 0;
    dr_swap_req = 0;
    gpio_write(0x0E, 0);
    pd0_cc_pull_flag = !pd0_cc_pull_flag;
}

/* E2:02 bit 6 clear (mode 3) or set (mode 2), then E2:02 bits 5:3
 * and E6:3A bits 4:1 on; any other mode turns both off. */
void typec_orient_e2_02_set(uint8_t mode) __reentrant
{
    uint8_t v;

    if (mode == 3) {
        v = R_E2_02 & ~0x40;
    } else if (mode == 2) {
        v = R_E2_02 | 0x40;
    } else {
        R_E2_02 &= 0xC7;
        R_E6_3A &= 0xE1;
        return;
    }
    R_E2_02 = v;
    R_E2_02 = v | 0x38;
    R_E6_3A |= 0x1E;
}

/* 2 unless pd0_cc_wait_dfp sees CC; then C7:30 & 0xA0: 0x20 -> 1,
 * 0x80 -> 0, else 2. */
uint8_t pd0_vconn_cc_line(void)
{
    uint8_t r = 2;
    uint8_t cc;

    if (pd0_cc_wait_dfp() == 1) {
        cc = R_C7_30 & 0xA0;
        if (cc == 0x20)
            r = 1;
        else if (cc == 0x80)
            r = 0;
    }
    return r;
}

/* 1 when the data port may start a message: PD revision below 3.0
 * (pd0_port.rev < 2), pulls set, or a CC line at level 3 (C7:30 bits 7:6 or 5:4
 * both set: Rp 3.0 A, the PD 3.0 SinkTxOk). */
uint8_t pd0_tx_allowed(void)
{
    uint8_t cc;

    if (pd0_port.rev < 2 || (R_E2_2D & 0x04))
        return 1;
    cc = R_C7_30;
    if ((cc & 0xC0) == 0xC0 || (cc & 0x30) == 0x30)
        return 1;
    return 0;
}

/* Power-port CC line tracking (as pd0_cc_line_track for the data port), only
 * with power-port VBUS. */
void pd1_cc_line_track(void)
{
    uint8_t cc;

    if (vbus_check(1) != 1)
        return;
    cc = R_C7_B0;
    if (cc & 0xC0)
        R_C7_AF &= ~0x02;
    else if (cc & 0x30)
        R_C7_AF |= 0x02;
}

/* Data-port pulls: 1 clears E2:2D bit 2 and C7:3E bit 0, 2 sets
 * them; other values do nothing. */
void pd0_cc_pull_set(uint8_t mode) __reentrant
{
    if (mode == 1) {
        R_E2_2D &= ~0x04;
        R_C7_3E &= ~0x01;
    } else if (mode == 2) {
        R_E2_2D |= 0x04;
        R_C7_3E |= 0x01;
    }
}

/* Power-port pulls, as pd0_cc_pull_set on E2:39 bit 2 and C7:BE bit 0. */
void pd1_cc_pull_set(uint8_t mode)
{
    if (mode == 1) {
        R_E2_39 &= ~0x04;
        R_C7_BE &= ~0x01;
    } else if (mode == 2) {
        R_E2_39 |= 0x04;
        R_C7_BE |= 0x01;
    }
}

/* 1 when C7:30 & 0xF0 is 0, or is 0x80/0x20 with the data-port
 * pulls set; else 0. */
uint8_t pd0_cc_open(void)
{
    uint8_t cc = R_C7_30 & 0xF0;

    if (cc == 0)
        return 1;
    if ((cc == 0x80 || cc == 0x20) && (R_E2_2D & 0x04))
        return 1;
    return 0;
}

/* E2:26 bit 5 on (1) or off (anything else). */
void pd0_cc_enable(uint8_t on)
{
    if (on == 1)
        R_E2_26 |= 0x20;
    else
        R_E2_26 &= ~0x20;
}

/* Data-port E2:2E bits 3:2 = v. */
void pd0_rp_level_set(uint8_t v) __reentrant
{
    R_E2_2E = (R_E2_2E & 0xF3) | (uint8_t)(v << 2);
}

/* Power-port E2:3A bits 3:2 = v. */
void pd1_rp_level_set(uint8_t v)
{
    R_E2_3A = (R_E2_3A & 0xF3) | (uint8_t)(v << 2);
}

/* C7:2F bit 5 (0 or 0x20). */
uint8_t pd0_c72f_bit5_get(void)
{
    return R_C7_2F & 0x20;
}
