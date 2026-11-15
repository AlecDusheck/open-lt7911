// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* USB PD policy on the data port (PHY block C700): the received-message
 * handler, the PD task the main loop runs, power-role (PR_Swap) and
 * data-role (DR_Swap) logic, and the data-port PD timers Timer 2 runs.
 *
 * Who runs what
 *   pd0_rx (INT1, pd_phy.c) reads a message from PD0_FIFO into the data
 *   port's state (pd0_port: rx_extended extended flag, rx_count objects,
 *   rx_type type, rx_header0/1, rx_data) and calls
 *   pd_message_handler, which answers it at once through pd0_tx.
 *   pd_main_task (main loop, only while input_typec) re-inits PD on
 *   pd_init_req, then runs the timed tasks below, each a small step machine
 *   with its own step byte and timer_stamp. pd0_tx_timeout and pd1_tx_timeout run from
 *   Timer 2 unless pd_timer_port is 1 (then pd0_t2_ps_rdy).
 *
 * Sending
 *   pd0_port.tx_msg points at the message pd0_tx sends: a code template (tpl_*,
 *   PD header then data objects) or an XRAM buffer (Source_Capabilities in
 *   src_caps_msg from src_caps_prepare, the Request pd_build_request builds in
 *   pd0_port.request, the Source_Capabilities_Extended message
 *   src_caps_ext_msg). After a send that expects a
 *   reply, pd0_port.pending_key holds PD_KEY(kind, type) of what was sent (0xFF: none);
 *   Accept, Reject, Wait and PS_RDY are interpreted against it.
 *   A send the CC line does not allow yet (pd0_tx_allowed) is parked (pd0_port.deferred,
 *   message pd0_port.deferred_msg, key pd0_port.deferred_key) and pd0_tx_deferred_send sends it later. pd0_tx_timeout
 *   retransmits pd0_port.tx_direct on a timeout while pd0_retry_count < pd0_retry_limit, then asks
 *   pd0_soft_reset for a Soft_Reset (pd0_soft_reset_req).
 *   Timer 2 belongs to these retries while pd_timer_port is 0; with 1 it
 *   runs pd0_t2_ps_rdy, which ends a power transition by sending PS_RDY here.
 *
 * Policy state (there is no single state byte; these bytes carry it)
 *   pd0_power_role  0 sink, 1 source; Source_Capabilities received -> 0,
 *                  Request received or our PR_Swap accepted -> 1.
 *   pd0_ams_state         1 idle / refused, 4 contract (Source_Cap or Request
 *                  handled), 5 VCONN_Swap accepted, 6 FR_Swap accepted;
 *                  PS_RDY returns 4/5 to 1. pd0_sink_task and dr_swap_task
 *                  act only while it is < 2.
 *   pr_swap_stage         our PR_Swap: 1 sent or Wait received, 2 accepted,
 *                  3 Source_Capabilities after it, 4 done or refused.
 *   pd0_power_role_state         PR_Swap progress: 1 sent (pr_swap), 2 accepted,
 *                  3 settled, 5 -> 6 in pd0_t2_ps_rdy (now sink), which the
 *                  next PS_RDY settles. pr_swap_awaiting 1/2 tracks the same swap.
 *   pr_swap_first  1: a PR_Swap is wanted (set on PS_RDY as sink with
 *                  power-port VBUS), 2: a PR_Swap was sent, or the
 *                  partner's own was refused.
 *   vconn_swap_state         VCONN_Swap: 2 accepted, 3 waiting, 4 done.
 *   pd0_recovery_req         a request for the main-loop tasks: 1 pd0_error_recovery (pd0_recovery_task),
 *                  2 and 4 (pd0_hard_reset_task), 3 (switch_dock_restart_task).
 *   pd0_bist_test_mode         1 after a BIST message other than Carrier Mode: the
 *                  sink-side tasks stop.
 *   Incoming PR_Swap is always refused (Reject, Not_Supported every 11th);
 *   incoming DR_Swap is accepted only while pd0_data_role is set and no DP Alt Mode
 *   step (dp_alt_status bits 7:3) has started.
 *
 * Registers: data-port PHY C7:17/19/1A (BIST carrier), C7:3D/44
 * (pd0_fr_swap_signal), C7:AF bit 5 (pd1_vbus_drop_handler); pins via gpio_write.
 */
#include <string.h>

#include "fw.h"

/* Message templates. */
const __code uint8_t tpl_sink_cap_pd2[6] = { 0x44, 0x12, 0x2C, 0x91, 0x01, 0x06 }; /* Sink_Capabilities, 5 V 3 A */
static const __code uint8_t tpl_sink_cap_pd3[6] = { 0x44, 0x12, 0x2C, 0x91, 0x81, 0x27 }; /* Sink_Capabilities, PD 3.x flags */
static const __code uint8_t tpl_pr_swap[2] = { 0x4A, 0x0A };
const __code uint8_t tpl_ps_rdy[2] = { 0x46, 0x0F };
const __code uint8_t tpl_accept[2] = { 0x43, 0x02 };
const __code uint8_t tpl_reject[2] = { 0x44, 0x02 };
static const __code uint8_t tpl_dr_swap[2] = { 0x49, 0x03 };
/* Not_Supported, then Soft_Reset (TPL_NOT_SUPPORTED, TPL_SOFT_RESET).
 * pd0_tx_timeout recognises a failed Soft_Reset by its template's address,
 * and pd0_tx leaves pd0_port.tx_msg just past the message it sent, so a
 * Not_Supported that runs out of retries matches too: one array, in this
 * order. */
const __code uint8_t tpl_not_supported_soft_reset[4] = { 0x90, 0x0F, 0x4D, 0x01 };

/* Clear the power port's policy state, pd1_port, the swap stages and the
 * BIST test mode, and
 * set their initial flags. */
void pd_init_clear(void)
{
    pd1_power_role = 0;
    pd1_data_role = 0;
    pd1_vbus_drop_armed = 0;
    pd1_adapter_state = 0;
    pd1_src_caps_valid = 0;
    pd1_tx_first_try = 0;
    pd1_goodcrc_wait = 0;
    pd1_retry_count = 0;
    pd1_ams_state = 0;
    pd1_retry_limit = 0;
    pd1_adapter_state = 1;
    pd1_tx_first_try = 1;
    memset(&pd1_port, 0, sizeof pd1_port);
    pd1_port.rev = 1;
    pd1_port.tx_settled = 1;
    pd1_port.request_pdo_offset = 2;
    pr_swap_stage = 0;
    vconn_swap_state = 0;
    pd0_bist_test_mode = 0;
}

/* Queue a code template on the data port and send it. */
static void pd0_send(const __code uint8_t *tpl)
{
    pd0_port.tx_msg = PD_MSG(tpl);
    pd0_tx();
}

/* Handle one message received on the data port (pd0_rx stored its
 * type in pd0_port.rx_type, its object count in pd0_port.rx_count and the extended flag in
 * pd0_port.rx_extended). pd0_port.pending_key holds PD_KEY(kind, type) of the message the reply is
 * matched against (0xFF: none). */
void pd_message_handler(void)
{
    if (pd0_port.rx_extended) {
        switch (pd0_port.rx_type) {
        case PD_EXT_GET_BATTERY_CAP:
        case PD_EXT_GET_BATTERY_STATUS:
        case PD_EXT_GET_MANUFACTURER_INFO:
        case PD_EXT_SECURITY_REQUEST:
        case PD_EXT_FW_UPDATE_REQUEST:
            pd0_send(TPL_NOT_SUPPORTED);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_NOT_SUPPORTED);
        }
        return;
    }

    if (pd0_port.rx_count) {
        switch (pd0_port.rx_type) {
        case PD_DATA_SOURCE_CAP:
            if (pr_swap_stage == 2)
                pr_swap_stage = 3;
            pd0_cc_line_select();
            typec_attach_phase = 0;
            pd0_ams_state = 4;
            pd0_caps_exchanged = 1;
            if (pd0_port.rev >= 2)
                pd0_port.rev = pd0_port.rx_header0 >> 6;
            pd_save_rx_msg(&pd0_port);
            pd_build_request(&pd0_port, 1);
            pd0_power_role = 0;
            try_src_count = 2;
            pd0_port.tx_msg = pd0_port.request; /* the Request pd_build_request built */
            pd0_tx();
            pd0_port.pending_key = PD_KEY(1, PD_DATA_REQUEST);
            break;
        case PD_DATA_REQUEST:
            delay_ms(3);
            pd1_vbus_drop_armed = 0;
            pd1_vbus_drop_event_set(1);
            pd1_vbus_drop_enable(0);
            pd0_ams_state = 4;
            if (pd0_port.rev >= 2)
                pd0_port.rev = pd0_port.rx_header0 >> 6;
            pd0_rp_level_set(pd0_port.rev >= 2 ? 3 : 2);
            pd0_power_role = 1;
            pd1_request_build();
            pd0_send(tpl_accept);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_ACCEPT);
            break;
        case PD_DATA_BIST:
            if (pd0_port.rx_data[3] == 0x50) {
                /* BIST Carrier Mode: send the carrier for 40 ms. */
                pd0_bist_test_mode = 0;
                R_C7_17 |= 0x80;
                PD0_TX_DATA = 0x43;
                R_C7_19 = 0x09;
                delay_ms(40);
                R_C7_17 &= 0x7F;
            } else {
                pd0_bist_test_mode = 1;
            }
            break;
        case PD_DATA_ALERT:
        case PD_DATA_GET_COUNTRY_INFO:
        case PD_DATA_EPR_MODE:
            pd0_send(TPL_NOT_SUPPORTED);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_NOT_SUPPORTED);
            break;
        case PD_DATA_ENTER_USB:
        case PD_DATA_EPR_REQUEST:
            pd0_send(tpl_reject);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_REJECT);
            break;
        case PD_DATA_VENDOR_DEFINED:
            pd0_port.tx_vdm = 1;
            vdm_rx(pd0_port.rx_type);
            break;
        }
        return;
    }

    switch (pd0_port.rx_type) {
    case PD_CTRL_ACCEPT:
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_PS_RDY) && pd0_ams_state == 1)
            pd0_send(TPL_SOFT_RESET);
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_DR_SWAP)) {
            pd0_swap_flags = (pd0_swap_flags & 0x0F) | 0x10;
            pd0_data_role = 0;
        }
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_PR_SWAP)) {
            pr_swap_stage = 2;
            if (pr_swap_awaiting == 1)
                pr_swap_awaiting = 2;
            if (pd0_power_role_state == 1)
                pd0_power_role_state = 2;
            pd0_power_role = 1;
        }
        break;

    case PD_CTRL_REJECT:
    case PD_CTRL_NOT_SUPPORTED:
        if (pd0_port.rx_type == PD_CTRL_REJECT) {
            pd0_ams_state = 1;
            if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_PR_SWAP)
                && !(dp_alt_status & DP_ALT_MODES))
                pd0_recovery_req = 3;
        }
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_DR_SWAP)) {
            pd0_swap_flags |= 0x20;
            pd0_port.pending_key = 0xFF;
        }
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_PR_SWAP)) {
            pr_swap_stage = 4;
            gpio_write(0x0D, 0);
            pd0_power_settled = 1;
            pr_swap_awaiting = 0;
            pd0_power_role_state = 3;
            if (!hpd_raised)
                hpd_raised = 1;
        }
        break;

    case PD_CTRL_PS_RDY:
        pr_swap_reject_count = 0;
        if (pd0_ams_state == 4 || pd0_ams_state == 5)
            pd0_ams_state = 1;
        pr_swap_src_t2_pending = 0;
        if (pr_swap_stage == 3)
            pr_swap_stage = 4;
        if (pr_swap_awaiting && (pd0_power_role_state == 2 || pd0_power_role_state == 3)) {
            /* Our PR_Swap completed: become source. */
            pd0_power_role_state = 3;
            pd0_rp_level_set(pd0_port.rev >= 2 ? 3 : 2);
            delay_ms(10);
            gpio_write(0x0E, 1);
            gpio_write(0x0D, 1);
            pd0_cc_pull_set(2);
            pd0_cc_pull_flag = 1;
            pd0_power_role = 1;
            pr_swap_src_t2_pending = 1;
            pd_timer_port = 1;
            delay_ms(50);
            t2_arm_c000();
        }
        if (pd0_power_role_state == 6) {
            pd0_power_settled = 1;
            pr_swap_awaiting = 0;
            pd0_power_role = 0;
            pd0_power_role_state = 3;
            if (!hpd_raised)
                hpd_raised = 1;
        }
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_VCONN_SWAP)) {
            vconn_swap_state = 4;
            pd0_vconn_source = 0;
            pd0_vconn_off();
        }
        dr_swap_want();
        if (vbus_check(1) == 1 && !pr_swap_first && !pd0_power_role && !fr_swap_state)
            pr_swap_first = 1;
        break;

    case PD_CTRL_GET_SOURCE_CAP:
        if (vbus_check(1) == 1) {
            src_caps_prepare();
            if (pd0_cc_wait_dfp() == 1)
                pd0_ams_state = 4;
        } else {
            pd0_port.tx_msg = PD_MSG(tpl_reject);
        }
        pd0_tx();
        break;

    case PD_CTRL_GET_SINK_CAP:
        pd0_ams_state = 1;
        pd0_send(pd0_port.rev >= 2 ? tpl_sink_cap_pd3 : tpl_sink_cap_pd2);
        break;

    case PD_CTRL_DR_SWAP:
        if (dp_alt_status & 0xF8) {
            pd0_recovery_req = 2;
            break;
        }
        pd0_wait_resend = 0;
        if (!pd0_data_role) {
            dr_swap_req = 0;
            pd0_send(tpl_reject);
        } else {
            dr_swap_req = 2;
            pd0_send(tpl_accept);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_DR_SWAP);
        }
        pd0_swap_flags = (pd0_swap_flags & 0x0F) | 0x10;
        break;

    case PD_CTRL_PR_SWAP:
        /* Always refused: Reject, and Not_Supported every 11th time. */
        pr_swap_awaiting = 0;
        pd0_power_role_state = 3;
        pr_swap_first = 2;
        if (pr_swap_reject_count >= 10) {
            pr_swap_reject_count = 0;
            pd0_port.tx_msg = PD_MSG(TPL_NOT_SUPPORTED);
        } else {
            pr_swap_reject_count++;
            pd0_port.tx_msg = PD_MSG(tpl_reject);
        }
        pd0_ams_state = 1;
        pd0_tx();
        pd0_port.pending_key = 0xFF;
        pr_swap_stage = 4;
        break;

    case PD_CTRL_VCONN_SWAP:
        dr_swap_want();
        pd0_wait_resend = 0;
        if (typec_receptacle == 1) {
            if (pd0_vconn_cc_line() == 1)
                gpio_write(0x13, 1);
            else if (pd0_vconn_cc_line() == 0)
                gpio_write(0x18, 1);
        }
        pd0_swap_flags |= 0x01;
        if (vbus_check(1) == 1 && pd0_vconn_source == 1) {
            pd0_ams_state = 5;
            pd0_port.tx_msg = PD_MSG(tpl_accept);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_VCONN_SWAP);
            vconn_swap_state = 2;
        } else {
            pd0_ams_state = 1;
            pd0_port.tx_msg = PD_MSG(tpl_reject);
            pd0_port.pending_key = 0xFF;
            vconn_swap_state = 4;
        }
        pd0_tx();
        break;

    case PD_CTRL_WAIT:
        if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_PR_SWAP)) {
            pr_swap_stage = 1;
            pr_swap_awaiting = 0;
        }
        pd0_wait_resend = 1;
        pd0_port.deferred_msg = pd0_port.tx_direct;
        break;

    case PD_CTRL_SOFT_RESET:
        pd0_port.tx_msg_id = 0;
        if (pd0_power_role == 1)
            src_caps_delayed = 1;
        pd0_send(tpl_accept);
        pd0_port.pending_key = 0xFF;
        break;

    case PD_CTRL_GET_SOURCE_CAP_EXT:
        if (vbus_check(1) == 1) {
            src_caps_ext_msg[0x1B] = src_caps_max_watts;
            pd0_port.tx_msg = src_caps_ext_msg;
        } else {
            pd0_port.tx_msg = PD_MSG(TPL_NOT_SUPPORTED);
        }
        pd0_tx();
        pd0_port.pending_key = 0xFF;
        break;

    case PD_CTRL_FR_SWAP:
        if (!fr_swap_state) {
            pd0_ams_state = 1;
            pd0_recovery_req = 2;
        } else if (pd0_power_role) {
            pd0_ams_state = 6;
            pd0_send(tpl_accept);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_FR_SWAP);
        } else {
            pd0_ams_state = 1;
            fr_swap_state = 0;
            pd0_send(tpl_reject);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_REJECT);
        }
        break;

    case PD_CTRL_GET_STATUS:
    case PD_CTRL_GET_PPS_STATUS:
    case PD_CTRL_GET_COUNTRY_CODES:
        pd0_send(TPL_NOT_SUPPORTED);
        pd0_port.pending_key = PD_KEY(0, PD_CTRL_NOT_SUPPORTED);
        break;

    case PD_CTRL_GOODCRC:
    case PD_CTRL_GOTOMIN:
    case PD_CTRL_PING:
        break;

    default:
        /* pd0_port.pending_key is set before the send here. */
        if (pd0_port.rev >= 2) {
            pd0_port.tx_msg = PD_MSG(TPL_NOT_SUPPORTED);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_NOT_SUPPORTED);
        } else {
            pd0_port.tx_msg = PD_MSG(tpl_reject);
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_REJECT);
        }
        pd0_tx();
        break;
    }
}

/* The PD task, run from the main loop while the input is USB-C. */
void pd_main_task(void)
{
    if (!input_typec)
        return;
    if (pd_init_req == 1) {
        pd_init();
        pd_init_req = 0;
    }
    if (!pd0_power_settled) {
        pd0_cc_line_track();
        if (!pd0_bist_test_mode) {
            pr_swap_timeout_task();
        }
        pd1_cc_line_track();
        pd1_adapter_task();
        attach_timer_task();
        if (!pd0_bist_test_mode) {
            src_caps_send_task();
            pd0_sink_task();
        }
        if (pr_swap_awaiting != 2 && pd0_caps_exchanged && fr_swap_state != 3 && pd0_detached() == 1)
            pd0_error_recovery();
    } else if (pd0_detached() == 1) {
        pd0_error_recovery();
    }
    if (!pd0_bist_test_mode) {
        dr_swap_task();
        vconn_swap_task();
    }
    src_caps_delayed_task();
    pd0_recovery_task();
    pd0_hard_reset_task();
    switch_dock_restart_task();
    hpd_attention_poll();
    fr_swap_task();
    pd0_tx_deferred_send();
    pd0_soft_reset();
}

/* Up to 300 reads of C7:AF until bit 5 sets, in assembly so the wait's
 * length is fixed. It saves the registers it uses, as SDCC expects of a
 * naked function. */
static void c7af_bit5_wait(void) __naked
{
    __asm
        push ar6
        push ar7
        clr  a
        mov  r7, a
        mov  r6, a
    00001$:
        mov  dptr, #_R_C7_AF
        movx a, @dptr
        jb   acc.5, 00002$
        inc  r7
        cjne r7, #0x00, 00003$
        inc  r6
    00003$:
        cjne r6, #0x01, 00001$
        cjne r7, #0x2c, 00001$
    00002$:
        pop  ar7
        pop  ar6
        ret
    __endasm;
}

/* Wait up to 300 reads for C7:AF bit 5 (power port); if it is set
 * while pd1_vbus_drop_armed and pd0_power_settled are 1, run the pd1_adapter_lost_reset reset with fr_swap_state = 1. */
void pd1_vbus_drop_handler(void)
{
    c7af_bit5_wait();
    if ((R_C7_AF & 0x20) && pd1_vbus_drop_armed == 1 && pd0_power_settled == 1) {
        pd0_fr_swap_signal(1);
        fr_swap_state = 1;
        pd1_adapter_lost_reset();
    }
}

/* Drive GPIO 0x0D low, clear the swap state, then pd0_fr_swap_signal(0). */
void pd1_adapter_lost_reset(void)
{
    gpio_write(0x0D, 0);
    pd1_vbus_drop_armed = 0;
    pd1_src_caps_valid = 0;
    pd0_power_settled = 0;
    pd0_data_role = 0;
    pr_swap_first = 0;
    pr_swap_awaiting = 0;
    pd0_power_role_state = 0;
    pd0_ams_state = 0;
    pr_swap_stage = 0;
    src_caps_15v_class = 0;
    pd0_fr_swap_signal(0);
}

/* Like pd1_vbus_drop_handler's tail without the wait: fr_swap_state = 1 (with
 * pd0_fr_swap_signal(1)) when pd1_vbus_drop_armed and pd0_power_settled are 1, else pd1_adapter_state = 1; then pd1_adapter_lost_reset. */
void pd1_detach_handler(void)
{
    if (pd1_vbus_drop_armed == 1 && pd0_power_settled == 1) {
        pd0_fr_swap_signal(1);
        fr_swap_state = 1;
    } else {
        pd1_adapter_state = 1;
    }
    pd1_adapter_lost_reset();
}

/* Set dr_swap_req when pd0_swap_flags bits 5:4 are clear, pd0_data_role is 1 and DP
 * Alt Mode status bit 3 is clear. */
void dr_swap_want(void)
{
    if (!(pd0_swap_flags & 0x30) && pd0_data_role == 1 && !(dp_alt_status & 0x08))
        dr_swap_req = 1;
}

/* cable_id_state = 0; cable_5a = 1 unless typec_receptacle is 1. */
void cable_id_reset(void)
{
    cable_id_state = 0;
    cable_5a = typec_receptacle == 1 ? 0 : 1;
}

/* on = 1 sets C7:3D bit 6 and strobes C7:44 (writes 1, 0); else
 * clears C7:3D bit 6. */
void pd0_fr_swap_signal(uint8_t on)
{
    if (on == 1) {
        R_C7_3D |= 0x40;
        R_C7_44 = 1;
        R_C7_44 = 0;
    } else {
        R_C7_3D &= 0xBF;
    }
}

/* When typec_receptacle is 1, drive GPIOs 0x18 and 0x13 low. */
void pd0_vconn_off(void)
{
    if (typec_receptacle == 1) {
        gpio_write(0x18, 0);
        gpio_write(0x13, 0);
    }
}

/* Run pd0_error_recovery once when pd0_recovery_req is 1. */
void pd0_recovery_task(void)
{
    if (pd0_recovery_req == 1) {
        pd0_error_recovery();
        pd0_recovery_req = 0;
    }
}

/* PD init: both PHYs and the PD state. */
void pd_init(void)
{
    pd_pins_init();
    pd_state_reset();
    pd0_phy_init();
    pd_init_clear();
    pd1_phy_init();
}

/* As source, (re)send Source_Capabilities. src_caps_step steps: 0 stamp,
 * 1 wait 6 000 000 ticks, 2 every 4 000 000 ticks (at most 50 times, counted
 * in src_caps_send_count) try up to four times while CC is attached (pd0_cc_wait_dfp); src_caps_first_pass set
 * means the first pass only re-drives the pins and restarts the wait. */
void src_caps_send_task(void)
{
    uint8_t i;

    if (pd0_caps_exchanged || typec_attach_phase == 1)
        return;
    if (typec_attach_phase == 2) {
        typec_attach_phase = 0;
        src_caps_step = 0;
    }
    switch (src_caps_step) {
    case 1:
        if (timer_elapsed(&src_caps_stamp, 6000000UL) != 1)
            return;
        /* fall through */
    case 0:
        timer_stamp(&src_caps_stamp);
        src_caps_step++;
        src_caps_first_pass = 1;
        return;
    case 2:
        if (pd0_caps_exchanged || pd0_power_role != 1 || pd0_data_role != 1)
            break;
        if (timer_elapsed(&src_caps_stamp, 4000000UL) != 1) {
            if (pd0_cc_wait_dfp() == 0) {
                pd0_source_detach();
                break;
            }
            return;
        }
        if (src_caps_send_count >= 0x32) {
            src_caps_send_count = 0x32;
        } else {
            src_caps_send_count++;
            for (i = 0; i < 4; i++) {
                if (pd0_cc_wait_dfp() == 0) {
                    pd0_source_detach();
                    src_caps_step = 0;
                    break;
                }
                if (src_caps_first_pass == 1) {
                    if (typec_receptacle == 1) {
                        pd0_cable_probe();
                        delay_ms(0x14);
                        pd0_cable_rx_stop();
                    }
                    src_caps_first_pass = 0;
                    gpio_write(0x0D, 1);
                    gpio_write(0x0E, 1);
                    timer_stamp(&src_caps_stamp);
                    break;
                }
                gpio_write(0x0D, 1);
                if (pd0_caps_exchanged == 0) {
                    pd0_rp_level_set(2);
                    delay_ms(4);
                    pd0_power_role_state = 1;
                    pd0_power_role = 1;
                    pd0_data_role = 1;
                    dr_swap_req = 0;
                    pd0_port.tx_msg_id = 0;
                    src_caps_prepare();
                    pd0_ams_state = 4;
                    try_src_count = 2;
                    pd0_cc_line_select();
                    pd0_tx();
                    pd0_port.pending_key = PD_KEY(1, PD_DATA_SOURCE_CAP);
                    delay_ms(2);
                } else {
                    src_caps_step++;
                }
            }
        }
        timer_stamp(&src_caps_stamp);
        return;
    default:
        break;
    }
    src_caps_step = 0;
}

/* Sink-side sequence, stepped by pd0_sink_step (pd0_sink_step_entry set: restart the
 * step's timer). 1: after 120 000 000 ticks call pd0_cc_line_select and post request 4
 * (pd0_recovery_req); 2: after 32 000 000 ticks try pr_swap(); 3: while pr_swap_awaiting = 1,
 * after 60 000 000 ticks finish on pr_swap_stage (1: swap complete, HPD up;
 * 2: post request 4). */
void pd0_sink_task(void)
{
    if (typec_attach_phase == 1 || pd0_ams_state >= 2 || pd1_adapter_state == 2 || pr_swap_stage == 4)
        goto stop;
    switch (pd0_sink_step) {
    case 0:
        goto next;
    case 1:
        if (pd0_sink_step_entry) {
            pd0_sink_step_entry = 0;
            timer_stamp(&pd0_sink_stamp);
        }
        if (timer_elapsed(&pd0_sink_stamp, 120000000UL) == 1) {
            if (pd0_caps_exchanged == 0) {
                pd0_cc_line_select();
                pd0_recovery_req = 4;
                goto stop;
            }
            goto next;
        }
        if (pd0_caps_exchanged != 1)
            return;
        goto next;
    case 2:
        if (pd0_sink_step_entry) {
            pd0_sink_step_entry = 0;
            timer_stamp(&pd0_sink_stamp);
        }
        if (timer_elapsed(&pd0_sink_stamp, 32000000UL) == 1) {
            if (pr_swap() != 1)
                return;
            goto next;
        }
        if (pr_swap_first && pd0_power_role_state == 0)
            return;
        pd0_sink_step_entry = 1;
        pd0_sink_step = 0;
        return;
    case 3:
        if (pd0_sink_step_entry) {
            pd0_sink_step_entry = 0;
            timer_stamp(&pd0_sink_stamp);
        }
        if (pr_swap_awaiting != 1)
            goto next;
        if (timer_elapsed(&pd0_sink_stamp, 60000000UL) != 1)
            return;
        if (pr_swap_stage == 1) {
            pr_swap_stage = 4;
            gpio_write(0x0D, 0);
            pd0_power_settled = 1;
            pr_swap_awaiting = 0;
            pd0_power_role_state = 3;
            if (!hpd_raised)
                hpd_raised = 1;
            pd0_sink_step++;
            return;
        }
        if (pr_swap_stage != 2)
            return;
        pd0_sink_step = 0;
        pd0_recovery_req = 4;
        delay_ms(5);
        pd0_sink_step_entry = 0;
        return;
    default:
        goto stop;
    }
next:
    pd0_sink_step_entry = 1;
    pd0_sink_step++;
    return;
stop:
    pd0_sink_step = 0;
}

/* Power-adapter check while power-port VBUS is present.
 * pd1_adapter_state: 1 start (pins down, go to 2), 2 run pd1_adapter_step's steps: 0 stamp,
 * 1 wait 40 000 000 ticks (skipped while pd1_src_caps_valid), 2 every 20 000 000 ticks
 * pulse pd1_phy_command(0x14) up to three times, then ask for a PR_Swap first if no
 * pd0_detached and still sink, 3 done (pd1_adapter_state = 0, pins 0x0E low / 0x0D high).
 * Losing VBUS in state 2 restarts at 1. */
void pd1_adapter_task(void)
{
    if (vbus_check(1) != 1 || fr_swap_state)
        return;
    if (pd1_adapter_state == 1) {
        pd1_adapter_step = 0;
        src_caps_max_watts = 7;
        gpio_write(0x0D, 0);
        pd1_vbus_drop_armed = 0;
        pd1_adapter_state = 2;
    }
    if (pd1_adapter_state != 2)
        return;
    if (vbus_check(1) == 0) {
        pd1_adapter_step = 0;
        pd1_adapter_state = 1;
        return;
    }
    switch (pd1_adapter_step) {
    case 0:
        pd1_adapter_step++;
        pd1_reset_signal_count = 0;
        break;
    case 1:
        if (pd1_src_caps_valid == 0 && timer_elapsed(&pd1_adapter_stamp, 40000000UL) != 1)
            return;
        pd1_adapter_step++;
        break;
    case 2:
        if (pd1_src_caps_valid == 0) {
            if (timer_elapsed(&pd1_adapter_stamp, 20000000UL) != 1)
                return;
            if (pd1_reset_signal_count <= 2) {
                pd1_reset_signal_count++;
                pd1_phy_command(0x14);
                break;
            }
            if (pd0_detached() == 0 && !pr_swap_first && pd0_power_role == 0)
                pr_swap_first = 1;
            pd1_reset_signal_count = 0;
        }
        pd1_adapter_step++;
        return;
    case 3:
        pd1_reset_signal_count = 0;
        pd1_adapter_step++;
        pd1_adapter_state = 0;
        gpio_write(0x0E, 0);
        gpio_write(0x0D, 1);
        return;
    default:
        pd1_adapter_step = 0;
        return;
    }
    timer_stamp(&pd1_adapter_stamp);
}

/* fr_swap_state sequence (pins 0x04/0x0A) stepped by fr_swap_step. In step 1,
 * fr_swap_state = 2 holds pin 0x04 high until pd0_c72f_bit5_get or 60 000 000 ticks, then
 * hands Timer 2 to the power port (pd_timer_port = 1, fr_swap_state = 3, pd1_adapter_state =
 * 1); fr_swap_state = 3 raises pin 0x0A; step 2 waits for VBUS loss or 40 000 000
 * ticks, step 3 4 000 000 ticks, then drops pin 0x0A and clears fr_swap_state. */
void fr_swap_task(void)
{
    if (fr_swap_state == 0) {
        fr_swap_step = 0;
        return;
    }
    switch (fr_swap_step) {
    case 0:
        fr_swap_step++;
        fr_swap_first_pass = 1;
        timer_stamp(&fr_swap_stamp);
        return;
    case 1:
        if (fr_swap_state == 1) {
            /* The result is not used. */
            timer_elapsed(&fr_swap_stamp, 40000000UL);
            return;
        }
        if (fr_swap_state == 2) {
            if (fr_swap_first_pass == 1) {
                gpio_write(0x04, 1);
                fr_swap_first_pass = 0;
                timer_stamp(&fr_swap_stamp);
            }
            if (pd0_c72f_bit5_get() == 0 && timer_elapsed(&fr_swap_stamp, 60000000UL) != 1)
                return;
            delay_ms(5);
            gpio_write(0x04, 0);
            pd0_cc_pull_flag = 0;
            fr_swap_state = 3;
            pd1_adapter_state = 1;
            pd_timer_port = 1;
            t2_arm_c000();
            return;
        }
        if (fr_swap_state != 3)
            return;
        gpio_write(0x0A, 1);
        break;
    case 2:
        if (vbus_check(1) != 0 && timer_elapsed(&fr_swap_stamp, 40000000UL) != 1)
            return;
        break;
    case 3:
        if (timer_elapsed(&fr_swap_stamp, 4000000UL) != 1)
            return;
        fr_swap_step = 0;
        gpio_write(0x0A, 0);
        fr_swap_state = 0;
        return;
    default:
        fr_swap_step = 0;
        return;
    }
    timer_stamp(&fr_swap_stamp);
    fr_swap_step++;
}

/* Delayed Source_Capabilities (src_caps_delayed: 1 arm, 2 wait 3 200 000
 * ticks, then send and clear). */
void src_caps_delayed_task(void)
{
    if (!src_caps_delayed)
        return;
    if (src_caps_delayed == 1) {
        pd0_port.pending_key = 0xFF;
        src_caps_delayed = 2;
        if (pd0_port.rev == 2)
            pd0_rp_level_set(2);
        if (typec_receptacle == 1)
            pd0_cable_probe();
        timer_stamp(&src_caps_delayed_stamp);
        return;
    }
    if (timer_elapsed(&src_caps_delayed_stamp, 3200000UL) != 1)
        return;
    src_caps_delayed = 0;
    pr_swap_awaiting = 0;
    if (typec_receptacle == 1)
        pd0_cable_rx_stop();
    if (pr_swap_stage == 2)
        pr_swap_stage = 3;
    src_caps_prepare();
    pd0_ams_state = 4;
    pd0_tx();
    pd0_port.pending_key = 0xFF;
    if (pd0_port.rev == 2)
        pd0_rp_level_set(3);
}

/* PD-side pins 0x0E, 0x0D, 0x04, 0x0A, 0x18, 0x13 (and 0x2B) to
 * mode 8, then drive the first six low. */
static const __code uint8_t pd_init_pins[7] = { 0x0E, 0x0D, 0x04, 0x0A, 0x18, 0x13, 0x2B };

void pd_pins_init(void)
{
    uint8_t i;

    for (i = 0; i < 7; i++)
        pin_config(pd_init_pins[i], 8, 0);
    for (i = 0; i < 6; i++)
        gpio_write(pd_init_pins[i], 0);
}

/* Before a contract (pd0_ams_state < 2), with dr_swap_req = 1 or pd0_wait_resend set:
 * after 20 000 000 ticks send DR_Swap (dr_swap, pd0_tx_resend). dr_swap_step steps
 * 0 stamp, 1 wait. */
void dr_swap_task(void)
{
    if (pd0_ams_state >= 2)
        goto stop;
    if (dr_swap_req != 1 && pd0_wait_resend == 0)
        goto stop;
    switch (dr_swap_step) {
    case 1:
        if (timer_elapsed(&dr_swap_stamp, 20000000UL) != 1)
            return;
        dr_swap();
        pd0_tx_resend();
        /* fall through */
    case 0:
        timer_stamp(&dr_swap_stamp);
        dr_swap_step++;
        return;
    default:
        break;
    }
stop:
    dr_swap_step = 0;
}

/* vconn_swap_state 2 -> 3 starts an 8 000 000-tick wait; at its end vconn_swap_state = 4,
 * pd0_vconn_source = 0 and pd0_vconn_off, dr_swap_want run. */
void vconn_swap_task(void)
{
    if (vconn_swap_state == 0 || vconn_swap_state == 4)
        return;
    if (vconn_swap_state == 2) {
        vconn_swap_state = 3;
        timer_stamp(&vconn_swap_stamp);
    }
    if (vconn_swap_state == 3 && timer_elapsed(&vconn_swap_stamp, 8000000UL) == 1) {
        timer_stamp(&vconn_swap_stamp);
        vconn_swap_state = 4;
        pd0_vconn_source = 0;
        pd0_vconn_off();
        dr_swap_want();
    }
}

/* pr_swap_awaiting = 2 held for 33 400 000 ticks posts request 2 (pd0_recovery_req).
 * pr_swap_timeout_step steps 0 stamp, 1 wait, 2 done; pr_swap_awaiting != 2 restarts. */
void pr_swap_timeout_task(void)
{
    switch (pr_swap_timeout_step) {
    case 0:
        timer_stamp(&pr_swap_timeout_stamp);
        break;
    case 1:
        if (pr_swap_awaiting != 2) {
            timer_stamp(&pr_swap_timeout_stamp);
            goto stop;
        }
        if (timer_elapsed(&pr_swap_timeout_stamp, 33400000UL) != 1)
            return;
        pd0_recovery_req = 2;
        break;
    default:
        goto stop;
    }
    pr_swap_timeout_step++;
    return;
stop:
    pr_swap_timeout_step = 0;
}

/* pd0_power_role_state = 6: forget the last sent message; pd0_power_role_state = 3: with
 * pr_swap_awaiting arm the delayed Source_Capabilities (src_caps_delayed), else raise HPD.
 * Then, unless pd0_swap_flags bits 5:4 are set, dr_swap_want, else dr_swap_req = 0. */
void pd0_ps_rdy_acked(void)
{
    if (pd0_power_role_state == 6) {
        pd0_port.pending_key = 0xFF;
        return;
    }
    if (pr_swap_awaiting) {
        if (pd0_power_role_state == 3)
            src_caps_delayed = 1;
    } else if (pd0_power_role_state == 3 && !hpd_raised) {
        hpd_raised = 1;
    }
    if (pd0_swap_flags & 0x30)
        dr_swap_req = 0;
    else
        dr_swap_want();
}

/* cable_5a = 0, pd0_vconn_off, and with typec_receptacle = 1 drive pin 0x13 or 0x18
 * high by pd0_vconn_cc_line (1 / 0); then 20 ms, cable_id_state = 1, pd0_discover_identity, 1 ms. */
void pd0_cable_probe(void)
{
    cable_5a = 0;
    pd0_vconn_off();
    if (typec_receptacle == 1) {
        if (pd0_vconn_cc_line() == 1)
            gpio_write(0x13, 1);
        else if (pd0_vconn_cc_line() == 0)
            gpio_write(0x18, 1);
    }
    delay_ms(0x14);
    cable_id_state = 1;
    pd0_discover_identity();
    delay_ms(1);
}

/* Request 3 (pd0_recovery_req = 3, a PR_Swap refused before Discover Modes):
 * pd0_phy_command(0x14) and pd0_error_recovery, assume a Nintendo Switch dock (switch_docked),
 * then pd0_cc_pull_set(2), pd0_rp_level_set(3), pd0_cc_pull_flag = 1. */
void switch_dock_restart_task(void)
{
    if (pd0_recovery_req != 3)
        return;
    pd0_phy_command(0x14);
    pd0_error_recovery();
    switch_docked = 1;
    pd0_cc_pull_set(2);
    pd0_rp_level_set(3);
    pd0_cc_pull_flag = 1;
}

/* Requests 2 and 4 (pd0_recovery_req), each consumed once and acted on only
 * when pd0_detached is 0: 2 -> pd0_phy_command(0x14), pd0_error_recovery; 4 -> try_src_count = 2,
 * pd0_phy_command(0x14). */
void pd0_hard_reset_task(void)
{
    if (pd0_recovery_req == 2) {
        pd0_recovery_req = 0;
        if (pd0_detached() == 0) {
            pd0_phy_command(0x14);
            pd0_error_recovery();
        }
    }
    if (pd0_recovery_req == 4) {
        pd0_recovery_req = 0;
        if (pd0_detached() == 0) {
            try_src_count = 2;
            pd0_phy_command(0x14);
        }
    }
}


/* Park the current data-port message for pd0_tx_deferred_send to send once the CC line
 * allows a transmission (pd0_tx_allowed). */
static void pd0_tx_defer(uint8_t pending) __reentrant
{
    pd0_port.deferred = 1;
    pd0_port.deferred_msg = pd0_port.tx_msg;
    pd0_port.deferred_key = pending;
}

/* Build a Request in port->request from the received Source_Cap that
 * pd_save_rx_msg copied to port->rx_msg, for the PDO at byte offset
 * port->request_pdo_offset in it: one RDO, operating and maximum current
 * both the PDO's maximum current, object position from the offset, USB
 * communications capable and no USB suspend; flags bit 0 also sets
 * Capability Mismatch when the PDO offers less than 3 A. */
void pd_build_request(__xdata struct pd_port *port, uint8_t flags) __reentrant
{
    uint8_t n = port->request_pdo_offset;
    __xdata uint8_t *pdo;
    __xdata uint8_t *req = port->request;
    uint16_t current;

    if (n > 0x20)
        n = 0x20;
    pdo = &port->rx_msg[n];
    current = (uint16_t)(pdo[1] & 0x03) << 8 | pdo[0];
    req[0] = 0x42;
    req[1] = 0x10;
    req[2] = pdo[0];
    req[3] = (uint8_t)((pdo[0] & 0x3F) << 2) | (pdo[1] & 0x03);
    req[4] = pdo[0] >> 6 | (uint8_t)((pdo[1] & 0x03) << 2);
    req[5] = (uint8_t)(((n >> 2) + 1) << 4) | 0x03;
    if ((flags & 0x01) && current < 300)
        req[5] |= 0x04;
}

/* Start a PR_Swap on the data port when one is pending
 * (pr_swap_first = 1) while we are sink and no swap runs (pd0_power_role_state = 0).
 * With power-port VBUS: unless pd0_detached reports 1 (then pd0_error_recovery), send
 * PR_Swap once the CC line allows (pd0_tx_allowed), polling up to 100 times 1 ms
 * apart, and mark the swap started (pr_swap_first = 2, pd0_power_role_state = 1). Without
 * power-port VBUS (the adapter is gone): drop the pending swap and raise HPD.
 * Returns 0 only when the 100 polls ran out, else 1. */
uint8_t pr_swap(void)
{
    uint8_t tries;

    if (pr_swap_first != 1 || pd0_power_role != 0 || pd0_power_role_state != 0)
        return 1;
    if (vbus_check(1) == 1) {
        gpio_write(0x0D, 1);
        gpio_write(0x0E, 0);
        if (pd0_detached() == 1) {
            pd0_error_recovery();
            return 1;
        }
        if (pd0_power_role_state != 0) {
            pr_swap_first = 2;
            return 1;
        }
        for (tries = 0; tries < 100; tries++) {
            if (pd0_tx_allowed() == 1) {
                pd0_c754_bit2_pulse();
                pr_swap_first = 2;
                pd0_port.tx_msg = PD_MSG(tpl_pr_swap);
                pr_swap_awaiting = 1;
                pd0_power_role_state = 1;
                pr_swap_stage = 1;
                pd0_ams_state = 3;
                pd0_port.pending_key = PD_KEY(0, PD_CTRL_PR_SWAP);
                pd0_tx();
                return 1;
            }
            delay_ms(1);
        }
        return 0;
    }
    pd1_src_caps_valid = 0;
    gpio_write(0x0D, 0);
    gpio_write(0x0E, 1);
    pd1_adapter_state = 1;
    pd0_power_role_state = 0;
    pr_swap_first = 0;
    src_caps_15v_class = 0;
    if (hpd_raised == 0)
        hpd_raised = 1;
    return 1;
}

/* Send a requested DR_Swap (dr_swap_req = 1) on the data port. On PD 3.x
 * the Rp level goes to 2 around the attempt (pd0_rp_level_set(2) .. pd0_rp_level_set(3)). If
 * the CC line does not allow a transmission, the message is parked for
 * pd0_tx_deferred_send. */
void dr_swap(void)
{
    if (pd0_data_role == 0 || dr_swap_req != 1)
        return;
    if (pd0_port.rev >= 2)
        pd0_rp_level_set(2);
    delay_ms(20);
    pd0_wait_resend = 0;
    if (dr_swap_req == 1 && pd0_ams_state < 2) {
        pd0_swap_flags &= 0x0F;
        dr_swap_req = 0;
        pd0_port.tx_msg = PD_MSG(tpl_dr_swap);
        if (pd0_tx_allowed() == 1) {
            pd0_port.pending_key = PD_KEY(0, PD_CTRL_DR_SWAP);
            pd0_tx();
        } else {
            pd0_tx_defer(PD_KEY(0, PD_CTRL_DR_SWAP));
        }
    }
    delay_ms(1);
    if (pd0_port.rev >= 2)
        pd0_rp_level_set(3);
}

/* When pd0_wait_resend = 1, resend the parked message (pd0_port.deferred_msg) now, or park
 * it again if the CC line does not allow a transmission. */
void pd0_tx_resend(void)
{
    if (pd0_wait_resend != 1)
        return;
    if (pr_swap_stage == 1)
        pr_swap_awaiting = 1;
    pd0_port.tx_msg = pd0_port.deferred_msg;
    pd0_wait_resend = 0;
    if (pd0_tx_allowed() == 1)
        pd0_tx();
    else
        pd0_tx_defer(pd0_port.pending_key);
}

/* Send the parked data-port message once the CC line allows. */
void pd0_tx_deferred_send(void)
{
    if (pd0_port.deferred == 1 && pd0_tx_allowed() == 1) {
        pd0_port.deferred = 0;
        pd0_port.tx_msg = pd0_port.deferred_msg;
        pd0_port.pending_key = pd0_port.deferred_key;
        pd0_tx();
    }
}

/* Send the Soft_Reset pd0_tx_timeout asked for (pd0_soft_reset_req), restarting the
 * message ID at 0. */
void pd0_soft_reset(void)
{
    if (pd0_soft_reset_req == 1) {
        pd0_soft_reset_req = 0;
        pd0_port.tx_msg_id = 0;
        pd0_port.pending_key = 0xFF;
        pd0_port.tx_msg = PD_MSG(TPL_SOFT_RESET);
        pd0_tx();
    }
}

/* Data-port transmit timeout, from Timer 2. When a sent message
 * timed out without completing (pd0_port.tx_settled = 0, pd0_port.tx_timer_armed = 1), retransmit the last
 * message (pd0_port.tx_direct, same message ID) while pd0_retry_count < pd0_retry_limit, re-arming
 * Timer 2's reload; after the last retry stop Timer 2, mark the transmission
 * done and ask for a Soft_Reset (pd0_soft_reset_req), or, if the Soft_Reset itself
 * failed, set pd0_recovery_req = 2. */
void pd0_tx_timeout(void)
{
    if (pd0_port.tx_settled != 0 || pd0_port.tx_timer_armed != 1)
        return;
    pd0_tx_first_try = 0;
    pd0_port.tx_timer_armed = 0;
    if (pd0_retry_count < pd0_retry_limit) {
        pd0_port.tx_msg = pd0_port.tx_direct;
        if (pd0_tx_allowed() == 1) {
            pd0_tx();
            t2_load_f1a6();
        } else {
            pd0_tx_defer(pd0_port.pending_key);
        }
        return;
    }
    TR2 = 0;
    pd0_tx_first_try = 1;
    pd0_port.tx_settled = 1;
    pd0_retry_count = 0;
    if (pd0_port.tx_msg == PD_MSG(TPL_SOFT_RESET)) {
        pd0_port.pending_key = 0xFF;
        pd0_recovery_req = 2;
    } else {
        pd0_soft_reset_req = 1;
    }
}

/* Copy a port's received message to port->rx_msg: the two header bytes,
 * then the data objects, at most 0x22 bytes in all. Length: the data-object
 * byte count plus 2; nothing is copied when the count is 0xFE or more. */
void pd_save_rx_msg(__xdata struct pd_port *port) __reentrant
{
    uint8_t i;

    if (port->rx_count >= 0xFE)
        return;
    for (i = 0; i < (uint8_t)(port->rx_count + 2); i++) {
        if (i == 0)
            port->rx_msg[0] = port->rx_header0;
        else if (i == 1)
            port->rx_msg[1] = port->rx_header1;
        else if (i < 0x22)
            port->rx_msg[i] = port->rx_data[i - 2];
    }
}
