// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Vendor Defined Messages on the data port, DP Alt Mode, and the source
 * capabilities / request the chip passes through from the power port.
 *
 * VDMs. pd0_port.rx_data[] holds a received message's data objects, little-endian;
 * for a VDM the first is the VDM header:
 *   [0] bits 0..4 command, bits 6..7 command type (0 = request)
 *   [1] bits 0..2 object position, bits 3..4 version minor, bits 5..6 version
 *       major, bit 7 structured
 *   [2..3] SVID (structured) or VID (unstructured); VDOs follow from [4].
 * The PD layer calls vdm_rx for every Vendor_Defined message; requests go to
 * vdm_dispatch, which answers through pd0_tx (pd0_port.tx_msg names the reply:
 * a code template below or an XRAM buffer):
 *   SVID 0xFFxx  Discover Identity / SVIDs / Modes, Enter / Exit Mode, and
 *                the DP Alt Mode commands DP Status Update and DP Configure.
 *   anything else is ignored (no NAK).
 * switch_docked (a Nintendo Switch dock assumed, see switch_dock_restart_task) changes the
 * Discover Modes reply and the power pass-through.
 * dp_alt_status records the progress of the DP Alt Mode handshake (DP_ALT_*
 * bits); Exit Mode clears the entered/status/configured bits. DP Configure
 * also raises HPD: hpd_raised lets the DP receiver leave RxState 2.
 *
 * HPD to the host goes as a DP Attention (dp_attention): hpd_attention_poll
 * waits a fixed delay after the DP side posts hpd_request, then sends it.
 *
 * Power pass-through. src_caps_prepare builds the Source_Capabilities offered on the
 * data port (src_caps_msg): the power-port source's PDOs with APDOs removed and
 * currents derated (src_caps_from_pd1), or a fixed template. pd1_request_build turns the PDO the
 * data-port sink requested into a Request for the power port (pd1_request_msg).
 *
 * Owns no registers; all I/O goes through pd0_tx.
 */
#include "fw.h"

/* Structured VDM commands. */
#define VDM_DISCOVER_IDENTITY 0x01
#define VDM_DISCOVER_SVIDS    0x02
#define VDM_DISCOVER_MODES    0x03
#define VDM_ENTER_MODE        0x04
#define VDM_EXIT_MODE         0x05
#define VDM_DP_STATUS         0x10
#define VDM_DP_CONFIGURE      0x11

#define VDM_ACK 0x40 /* command type ACK, in VDM header byte 0 */

/* Discover Identity ID header byte 2 (bits 16..23) by negotiated PD
 * revision, set in pd_id_ack_vdm1 (templates: vars.c). */
#define ID_HDR2_PD3 0x60
#define ID_HDR2_PD2 0x00

/* Reply templates: 2-byte message header (pd0_tx fills the message ID), then
 * the data objects, little-endian. */
const __code uint8_t svids_ack[10] = { /* SVIDs ff01 */
    0x4f, 0x26, 0x42, 0x80, 0x00, 0xff, 0x00, 0x00, 0x01, 0xff,
};
static const __code uint8_t modes_ack_switch[10] = { /* DP mode VDO 0x00080805 */
    0x4f, 0x28, 0x43, 0x80, 0x01, 0xff, 0x05, 0x08, 0x08, 0x00,
};
static const __code uint8_t enter_mode_ack[6] = {
    0x4f, 0x1a, 0x44, 0x81, 0x01, 0xff,
};
static const __code uint8_t exit_mode_ack[6] = {
    0x4f, 0x1a, 0x45, 0x81, 0x01, 0xff,
};
static const __code uint8_t dp_status_ack[10] = {    /* HPD low */
    0x4f, 0x2c, 0x50, 0x81, 0x01, 0xff, 0x0a, 0x00, 0x00, 0x00,
};
static const __code uint8_t dp_status_ack_hpd[10] = { /* HPD high */
    0x4f, 0x2c, 0x50, 0x81, 0x01, 0xff, 0x8a, 0x00, 0x00, 0x00,
};
static const __code uint8_t dp_config_ack[6] = {
    0x4f, 0x1e, 0x51, 0x81, 0x01, 0xff,
};
static const __code uint8_t attention_hpd_low[10] = {
    0x4f, 0x22, 0x06, 0x81, 0x01, 0xff, 0x0a, 0x00, 0x00, 0x00,
};
static const __code uint8_t attention_hpd_high[10] = {
    0x4f, 0x22, 0x06, 0x81, 0x01, 0xff, 0x8a, 0x00, 0x00, 0x00,
};
static const __code uint8_t attention_irq_hpd[10] = { /* HPD high + IRQ_HPD */
    0x4f, 0x22, 0x06, 0x81, 0x01, 0xff, 0x8a, 0x01, 0x00, 0x00,
};

/* Source_Capabilities offered on the data port without a power-port source. */
static const __code uint8_t src_caps_default[6] = {  /* 5 V 1.5 A */
    0x41, 0x11, 0x96, 0x90, 0x01, 0x2e,
};
static const __code uint8_t src_caps_switch[10] = {  /* 5 V 1 A, 15 V 1.2 A */
    0x61, 0x21, 0x64, 0x90, 0x01, 0x2a, 0x04, 0xb1, 0x04, 0x00,
};

/* Answer a received VDM request (see the file comment). */
void vdm_dispatch(void)
{
    uint8_t cmd;

    /* Unstructured, or structured with an SVID other than 0xFFxx. */
    if (!(pd0_port.rx_data[1] & 0x80) || pd0_port.rx_data[3] != 0xFF)
        return;

    cmd = pd0_port.rx_data[0] & 0x1F;
    switch (cmd) {
    case VDM_DISCOVER_IDENTITY:
        discover_identity_vid(pd0_port.rev, pd0_port.rx_data, VDM_ACK);
        /* A request with VDM version minor bit set gets the VDM 2.x reply. */
        pd0_port.tx_msg = pd0_port.rx_data[1] & 0x08 ? (uint8_t *)pd_id_ack_vdm2 : (uint8_t *)pd_id_ack_vdm1;
        dp_alt_status |= DP_ALT_IDENTITY;
        break;

    case VDM_DISCOVER_SVIDS:
        dp_alt_status |= DP_ALT_SVIDS;
        pd0_port.tx_msg = (uint8_t *)svids_ack;
        break;

    case VDM_DISCOVER_MODES:
        dp_alt_status |= DP_ALT_MODES;
        if (switch_docked == 1) {
            pd0_port.tx_msg = (uint8_t *)modes_ack_switch;
            break;
        }
        /* A DP mode template; typec_dp_lane_count == 4 drops pin assignment D (C, E only). */
        if (typec_receptacle == 1) {
            copy_bytes(pd0_port.vdm_reply, pd_modes_ack_a, 10);
            pd0_port.vdm_reply[8] = typec_dp_lane_count == 4 ? 0x14 : 0x1C;
        } else {
            copy_bytes(pd0_port.vdm_reply, pd_modes_ack_b, 10);
            pd0_port.vdm_reply[7] = typec_dp_lane_count == 4 ? 0x04 : 0x0C;
        }
        pd0_port.tx_msg = (uint8_t *)pd0_port.vdm_reply;
        break;

    case VDM_ENTER_MODE:
        dp_alt_status |= DP_ALT_ENTERED;
        copy_bytes(pd0_port.vdm_reply, enter_mode_ack, 6);
        vdm_reply_header(VDM_ACK, VDM_ENTER_MODE);
        pd0_port.tx_msg = (uint8_t *)pd0_port.vdm_reply;
        break;

    case VDM_EXIT_MODE:
        copy_bytes(pd0_port.vdm_reply, exit_mode_ack, 6);
        vdm_reply_header(VDM_ACK, VDM_EXIT_MODE);
        pd0_port.tx_msg = (uint8_t *)pd0_port.vdm_reply;
        dp_alt_status &= ~(DP_ALT_ENTERED | DP_ALT_STATUS | DP_ALT_CONFIGURED);
        break;

    case VDM_DP_STATUS:
        dp_alt_status |= DP_ALT_STATUS;
        dp_multi_function_pref = typec_dp_lane_count == 4 ? 0 : 1;
        pd0_port.tx_msg = dp_alt_status & DP_ALT_HPD_HIGH ? (uint8_t *)dp_status_ack_hpd
                                                    : (uint8_t *)dp_status_ack;
        /* DP Status VDO bits 0..1: the partner's DFP_D/UFP_D connected. */
        dp_partner_connected = (pd0_port.rx_data[4] & 0x03) ? 1 : 0;
        break;

    case VDM_DP_CONFIGURE:
        dp_alt_status |= DP_ALT_CONFIGURED;
        hpd_raised = 1;
        if (dp_partner_connected == 1)
            dp_config_pin_assignment(pd0_port.rx_data);
        else
            dp_pin_assignment = 4;
        pd0_port.tx_msg = (uint8_t *)dp_config_ack;
        break;

    default:
        return;
    }
    pd0_tx();
    pd0_port.pending_key = PD_KEY(3, cmd);
}

/* Build the data-port Source_Capabilities (src_caps_msg) from the power
 * port's (pd1_port.rx_msg): drop APDOs, derate each PDO's current by voltage, cap it,
 * and track the largest PDO power in src_caps_max_watts (W). src_caps_15v_class: 3, or 1/2 when a
 * 15 V PDO was capped/derated. src_caps_offset..pdo_15v_class are left holding the offset,
 * end offset, shift index, last PDO's current, voltage, power and 15 V
 * class. */
void src_caps_from_pd1(void) __reentrant
{
    __xdata uint8_t *p;
    uint8_t derate;

    copy_bytes(src_caps_msg, pd1_port.rx_msg, 34);
    src_caps_15v_class = 3;
    src_caps_max_watts = 7;
    src_caps_end = ((pd1_port.rx_msg[1] & 0x70) >> 2) + 2; /* end offset: 2 + 4 * NDO */

    for (src_caps_offset = 2; src_caps_offset < src_caps_end; src_caps_offset += 4) {
        pdo_15v_class = 0;
        p = &src_caps_msg[src_caps_offset];
        if ((p[3] & 0xC0) == 0xC0) {
            /* APDO: remove it and step back onto the PDO moved in. */
            for (src_caps_shift_i = src_caps_offset; src_caps_shift_i + 4 < src_caps_end; src_caps_shift_i++)
                src_caps_msg[src_caps_shift_i] = src_caps_msg[src_caps_shift_i + 4];
            src_caps_msg[1] -= 0x10;
            src_caps_end -= 4;
            src_caps_offset -= 4;
            continue;
        }
        /* Max current (10 mA) bits 0..9, voltage (50 mV) bits 10..19. */
        pdo_current = (uint16_t)(p[1] & 0x03) << 8 | p[0];
        pdo_voltage = (uint16_t)(p[2] & 0x0F) * 64 + ((p[1] & 0xFC) >> 2);

        /* Derate by voltage: subtract a margin (derate_or_50), or cap 15/20 V. */
        derate = 0;
        if (pdo_voltage < 120)
            derate = 100;
        else if (pdo_voltage < 190)
            derate = 50;
        else if (pdo_voltage < 280)
            derate = 66;
        else if (pdo_voltage < 300)
            derate = 58;
        else if (pdo_voltage == 300) {
            if (pdo_current >= 260) {
                pdo_current = 260;
                src_caps_15v_class = 1;
            } else {
                derate = 53;
                src_caps_15v_class = 2;
            }
            pdo_15v_class = src_caps_15v_class;
        } else if (pdo_voltage == 400) {
            if (pdo_current > 500)
                pdo_current = 490;
            else if (pdo_current > 465)
                pdo_current -= 70;
            else if (pdo_current > 325)
                pdo_current -= 50;
            else if (pdo_current > 235)
                pdo_current -= 40;
        } else
            derate = 45;
        if (derate)
            pdo_current = derate_or_50(pdo_current, derate);

        if (pdo_voltage < 400 && pdo_current > 300)
            pdo_current = 325;
        if (cable_5a == 0 && pdo_current >= 300)
            pdo_current = 325;
        if (switch_docked == 1 && pdo_15v_class == 2)
            pdo_current = 260;
        p[0] = (uint8_t)pdo_current;
        p[1] = (p[1] & 0xFC) | ((pdo_current >> 8) & 0x03);

        pdo_watts = (uint32_t)pdo_voltage * pdo_current / 2000;
        if (pdo_watts >= src_caps_max_watts)
            src_caps_max_watts = (uint8_t)pdo_watts;
    }

    if (switch_docked == 1 && src_caps_15v_class == 3)
        copy_bytes(src_caps_msg, src_caps_switch, sizeof src_caps_switch);
    src_caps_msg[0] = (src_caps_msg[0] & 0x0F) | 0x40; /* spec revision 2.0 */
    src_caps_msg[4] &= 0x3F;
    src_caps_msg[5] = 0x2E; /* PDO 1: dual-role power and data, USB comms, unconstrained */
}

/* Build the power-port Request (pd1_request_msg) for the PDO the data-port
 * sink asked for (object position in its Request, pd0_port.rx_data[3] bits 4..6;
 * PDO 1 for the Switch dock), operating = maximum current = the PDO's. */
void pd1_request_build(void)
{
    uint8_t rdo3 = pd0_port.rx_data[3];
    uint8_t pos = (rdo3 & 0x70) >> 4;
    uint16_t cur;

    if (switch_docked == 1 && src_caps_15v_class == 3) {
        pos = 1;
        rdo3 = 0x11; /* object position 1, no USB suspend */
    }
    /* PDO pos of the power port's Source_Capabilities (at 4 * pos - 2 in
     * the message). Position 0 is invalid: its PDO is taken from the last
     * two bytes of the power port's VDM reply (only bytes 2 and 3 of the
     * copy are used below). */
    if (pos == 0) {
        pd1_request_msg[2] = pd1_port.vdm_reply[8];
        pd1_request_msg[3] = pd1_port.vdm_reply[9];
    } else {
        copy_bytes(&pd1_request_msg[2], &pd1_port.rx_msg[pos * 4 - 2], 4);
    }
    cur = (uint16_t)(pd1_request_msg[3] & 0x03) << 8 | pd1_request_msg[2];
    pd1_request_msg[0] = 0x42; /* header: Request, 1 data object */
    pd1_request_msg[1] = 0x12;
    pd1_request_msg[5] = rdo3;
    pd1_request_msg[3] = (pd1_request_msg[3] & 0x03) | (uint8_t)(cur << 2);
    pd1_request_msg[4] = (cur >> 6) & 0x0F;
}

/* Send a DP Attention for an HPD event (1 high, 2 low, 3 IRQ_HPD)
 * when it changes what the host was told, then mark it sent. */
void dp_attention(uint8_t event)
{
    if (event == 2) {
        if (dp_alt_status & DP_ALT_HPD_HIGH) {
            pd0_port.tx_msg = (uint8_t *)attention_hpd_low;
            dp_alt_status &= ~DP_ALT_HPD_HIGH;
            dp_attention_pending = 1;
        }
    } else if (event == 1) {
        if (!(dp_alt_status & DP_ALT_HPD_HIGH)) {
            pd0_port.tx_msg = (uint8_t *)attention_hpd_high;
            dp_alt_status |= DP_ALT_HPD_HIGH;
            dp_attention_pending = 1;
        }
    } else if (event == 3) {
        if (dp_alt_status & DP_ALT_HPD_HIGH) {
            pd0_port.tx_msg = (uint8_t *)attention_irq_hpd;
            dp_attention_pending = 1;
        }
    }
    if (dp_attention_pending == 1) {
        dp_attention_pending = 0;
        pd0_port.tx_vdm = 1;
        dp_multi_function_pref = typec_dp_lane_count == 4 ? 0 : 1;
        pd0_tx();
    }
}

/* Once hpd_request has been pending for 800000 timer ticks, send it
 * as a DP Attention and, if unchanged meanwhile, clear it. */
void hpd_attention_poll(void)
{
    if (pd0_ams_state >= 2 || hpd_request == 0) {
        hpd_attention_timing = 0;
        return;
    }
    if (hpd_attention_timing == 0) {
        timer_stamp(&hpd_attention_stamp);
        hpd_attention_timing = 1;
    }
    if (timer_elapsed(&hpd_attention_stamp, 800000) == 1) {
        hpd_attention_timing = 0;
        hpd_attention_event = hpd_request;
        dp_attention(hpd_attention_event);
        if (hpd_request == hpd_attention_event)
            hpd_request_post(0);
    }
}

/* Before a Discover Identity ACK, set pd_id_ack_vdm1's ID header
 * byte 2 by PD revision (rev >= 2: PD 3.x), its command type to
 * cmd_type, its SVID from the request req, and byte 18 by typec_dp_lane_count. */
void discover_identity_vid(uint8_t rev, const __xdata uint8_t *req, uint8_t cmd_type)
{
    pd_id_ack_vdm1[8] = rev >= 2 ? ID_HDR2_PD3 : ID_HDR2_PD2;
    pd_id_ack_vdm1[2] = (pd_id_ack_vdm1[2] & 0x3F) | cmd_type;
    pd_id_ack_vdm1[4] = req[2];
    pd_id_ack_vdm1[5] = req[3];
    pd_id_ack_vdm1[18] = typec_dp_lane_count == 4 ? 0x08 : 0x0A;
}

/* Fill the VDM header of the reply in pd0_port.vdm_reply: command and type,
 * the request's object position (Enter/Exit Mode; 0 or 7 become 7), and its
 * SVID. */
void vdm_reply_header(uint8_t cmd_type, uint8_t cmd)
{
    uint8_t pos;

    pd0_port.vdm_reply[2] = cmd_type | cmd;
    pd0_port.vdm_reply[3] &= 0xF8;
    if (cmd >= VDM_ENTER_MODE) {
        pos = pd0_port.rx_data[1] & 0x07;
        pd0_port.vdm_reply[3] |= (pos == 0 || pos == 7) ? 0x07 : pos;
    }
    pd0_port.vdm_reply[4] = pd0_port.rx_data[2];
    pd0_port.vdm_reply[5] = pd0_port.rx_data[3];
}

/* Prepare the data-port Source_Capabilities in src_caps_msg and point
 * pd0_port.tx_msg at it. */
void src_caps_prepare(void)
{
    if (pd1_src_caps_valid)
        src_caps_from_pd1();
    else if (switch_docked == 1)
        copy_bytes(src_caps_msg, src_caps_switch, sizeof src_caps_switch);
    else
        copy_bytes(src_caps_msg, src_caps_default, sizeof src_caps_default);
    pd0_port.tx_msg = (uint8_t *)src_caps_msg;
}

/* dp_pin_assignment = the pin assignment a DP Configure request cfg selects:
 * 0x10 (E) if either pin assignment byte names E, else 0x04 (C). */
void dp_config_pin_assignment(const __xdata uint8_t *cfg)
{
    dp_pin_assignment = 0x04;
    if ((cfg[5] & 0x3F) == 0x10 || (cfg[6] & 0x1F) == 0x10)
        dp_pin_assignment = 0x10;
}

/* a - b if a > b, else 50. */
uint16_t derate_or_50(uint16_t a, uint16_t b) __reentrant
{
    return a > b ? a - b : 50;
}

/* Hand a received Vendor_Defined message (msg_type 0x0F) that is a
 * request to vdm_dispatch. */
void vdm_rx(uint8_t msg_type)
{
    if (msg_type == 0x0F && !(pd0_port.rx_data[0] & 0xC0))
        vdm_dispatch();
}
