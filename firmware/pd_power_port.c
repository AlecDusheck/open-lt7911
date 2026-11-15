// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* USB PD on the power port (PHY block C780..C7DF, state pd1_port).
 *
 * The power port faces the charger: it is always the sink there. Received
 * messages (pd1_rx, from INT1) leave the header in pd1_port (data-object
 * byte count pd1_port.rx_count, pd1_port.rx_extended, message type
 * pd1_port.rx_type, data in pd1_port.rx_data) and pd1_message_handler answers them from the main loop:
 *   Source_Capabilities -> a Request built by pd_build_request for the PDO at offset
 *                          pd1_port.request_pdo_offset (sent from pd1_port.request);
 *   Get_Sink_Cap        -> the fixed Sink_Capabilities template;
 *   Soft_Reset          -> Accept;
 *   swaps, Get_Source_Cap -> Reject; extended/status requests -> Not_Supported;
 *   Vendor_Defined      -> pd1_vdm_reply: Discover Identity / Discover SVIDs ACKs;
 *   PS_RDY              -> the charger's power is up: with power-port VBUS,
 *                          while we are data-port sink and nothing else runs,
 *                          ask for a PR_Swap on the data port
 *                          (pr_swap_first = 1); and if pd0_power_settled is set, hand
 *                          Timer 2 to pd0_t2_ps_rdy (pd_timer_port = 1).
 * pd1_tx sends pd1_port.tx_msg (a code template or an XRAM buffer) and keeps the last
 * one in pd1_port.tx_direct; pd1_tx_timeout, from Timer 2, retransmits it until the retry count
 * pd1_retry_count reaches the limit pd1_retry_limit (2 on PD 3.x, 3 on PD 2.0), then cycles
 * the port (pd1_rp_level_set/pd1_cc_pull_set).
 *
 * Timer 2 is one-shot and shared by both ports (pd_timer_port): when 1 it
 * runs pd0_t2_ps_rdy, which completes the power transition on the data port
 * (PR_Swap state pd0_power_role_state 5 -> 6, power role sink) and sends PS_RDY there.
 */

#include "fw.h"

/* Structured VDM commands (USB PD 3.x table 6-28). */
enum {
    VDM_DISCOVER_IDENTITY = 1,
    VDM_DISCOVER_SVIDS = 2,
};

/* Send a message (code template or XRAM buffer) on the power port. */
static void pd1_send(uint8_t *msg)
{
    pd1_port.tx_msg = msg;
    pd1_tx();
}

/* Answer the power port's received message (see the file
 * comment). Nothing while pd1_port.rx_extended is set. */
void pd1_message_handler(void)
{
    if (pd1_port.rx_extended != 0)
        return;
    if (pd1_port.rx_count != 0) {
        if (pd1_port.rx_type == PD_DATA_VENDOR_DEFINED) {
            pd1_vdm_reply();
        } else if (pd1_port.rx_type == PD_DATA_SOURCE_CAP) {
            pd1_vbus_drop_enable(1);
            pd1_src_caps_valid = 1;
            pd1_ams_state = 4;
            pd_save_rx_msg(&pd1_port);
            pd_build_request(&pd1_port, 0);
            pd1_send(PD_MSG(pd1_port.request));
            pd0_port.pending_key = PD_KEY(1, PD_DATA_REQUEST);
        }
        return;
    }
    switch (pd1_port.rx_type) {
    case PD_CTRL_PS_RDY:
        if (pd1_ams_state == 4)
            pd1_ams_state = 1;
        if (vbus_check(1) == 1 && pr_swap_first == 0 && pd0_power_role == 0 &&
            fr_swap_state == 0 && pd0_detached() == 0)
            pr_swap_first = 1;
        if (pd_timer_port == 0 && pd0_power_settled != 0) {
            dr_swap_want();
            pd_timer_port = 1;
            t2_arm_c000();
        }
        return;
    case PD_CTRL_GET_SINK_CAP:
        pd1_send(PD_MSG(tpl_sink_cap_pd2));
        return;
    case PD_CTRL_SOFT_RESET:
        pd1_port.tx_msg_id = 0;
        pd1_send(PD_MSG(tpl_accept));
        return;
    case PD_CTRL_GET_SOURCE_CAP:
    case PD_CTRL_DR_SWAP:
    case PD_CTRL_PR_SWAP:
    case PD_CTRL_VCONN_SWAP:
    case PD_CTRL_FR_SWAP:
        pd1_send(PD_MSG(tpl_reject));
        break;
    case PD_CTRL_GET_SOURCE_CAP_EXT:
    case PD_CTRL_GET_STATUS:
    case PD_CTRL_GET_PPS_STATUS:
    case PD_CTRL_GET_COUNTRY_CODES:
        pd1_send(PD_MSG(TPL_NOT_SUPPORTED));
        break;
    default:
        return;
    }
    pd0_port.pending_key = 0xFF;
}

/* Timer 2 task while pd_timer_port = 1: finish a power transition
 * and send PS_RDY on the data port. A PR_Swap in state 5 ends as sink
 * (pd0_power_role_state = 6, pd0_power_role = 0). Hands Timer 2 back to the transmit
 * timeouts (pd_timer_port = 0). */
void pd0_t2_ps_rdy(void)
{
    if (pd0_power_role_state == 5) {
        pd0_power_settled = 0;
        pr_swap_awaiting = 2;
        pd0_power_role_state = 6;
        pd0_power_role = 0;
        pd0_cc_pull_set(1);
        delay_ms(2);
    }
    pd1_vbus_drop_armed = 0;
    pd1_vbus_drop_calibrate();
    pd1_vbus_drop_armed = 1;
    pd_timer_port = 0;
    pd1_vbus_drop_event_set(1);
    pd1_vbus_drop_enable(1);
    pd1_vbus_drop_event_set(0);
    if (pr_swap_src_t2_pending == 1) {
        pr_swap_src_t2_pending = 0;
        pr_swap_awaiting = 1;
    }
    if (fr_swap_state == 3)
        pd0_cc_pull_set(1);
    if (pr_swap_stage == 3)
        pr_swap_stage = 4;
    if (pd0_ams_state == 4 || pd0_ams_state == 5)
        pd0_ams_state = 1;
    pd0_port.tx_msg = PD_MSG(tpl_ps_rdy);
    pd0_port.pending_key = PD_KEY(0, PD_CTRL_PS_RDY);
    pd0_tx();
}

/* Power-port transmit timeout, from Timer 2. When a sent message
 * timed out (pd1_port.tx_settled = 0, pd1_port.tx_timer_armed = 1), retransmit it (pd1_port.tx_direct, same message
 * ID) while pd1_retry_count < pd1_retry_limit, re-arming Timer 2's reload; after the last
 * retry cycle the port (pd1_rp_level_set(2), pd1_cc_pull_set(2), 50 ms, pd1_cc_pull_set(1)) and mark
 * the transmission done. */
void pd1_tx_timeout(void)
{
    if (pd1_port.tx_settled != 0 || pd1_port.tx_timer_armed != 1)
        return;
    pd1_tx_first_try = 0;
    pd1_port.tx_timer_armed = 0;
    if (pd1_retry_count < pd1_retry_limit) {
        pd1_send(pd1_port.tx_direct);
        t2_load_f1a6();
        return;
    }
    pd1_rp_level_set(2);
    pd1_cc_pull_set(2);
    delay_ms(50);
    pd1_cc_pull_set(1);
    pd1_tx_first_try = 1;
    pd1_port.tx_settled = 1;
    pd1_retry_count = 0;
}

/* Answer a structured VDM on the power port: Discover Identity with
 * pd1_id_ack (filled by discover_identity_vid for the port's PD revision
 * pd1_port.rev), Discover SVIDs with svids_ack copied to pd1_port.vdm_reply. Other
 * commands get no answer. */
void pd1_vdm_reply(void)
{
    switch (pd1_port.rx_data[0] & 0x1F) {
    case VDM_DISCOVER_IDENTITY:
        discover_identity_vid(pd1_port.rev, pd1_port.rx_data, 0x40);
        pd1_port.tx_msg = PD_MSG(pd1_id_ack);
        break;
    case VDM_DISCOVER_SVIDS:
        copy_bytes(pd1_port.vdm_reply, svids_ack, sizeof svids_ack);
        /* The SVID VDO goes out as 0xFF000000. */
        pd1_port.vdm_reply[8] = 0;
        pd1_port.tx_msg = PD_MSG(pd1_port.vdm_reply);
        break;
    default:
        return;
    }
    pd1_tx();
}

/* Entered from pd0_accept_acked. With pd1_src_caps_valid set, send
 * pd1_request_msg on the power port and give Timer 2 to the transmit timeouts;
 * otherwise wait 30 ms and arm Timer 2 for pd0_t2_ps_rdy. */
void pd0_power_transition_start(void)
{
    pd0_power_settled = 1;
    if (pd1_src_caps_valid > 0) {
        pd1_send(PD_MSG(pd1_request_msg));
        pd_timer_port = 0;
        return;
    }
    delay_ms(30);
    pd_timer_port = 1;
    t2_arm_c000();
}
