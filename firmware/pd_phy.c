// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* USB PD PHYs of both ports: init, raw message receive (from INT1) and
 * transmit, MessageID, Timer 2 arming.
 *
 * The chip has two identical PD PHY register blocks 0x80 apart in page C7:
 * C700..C75F for the data port (PD0) and C780..C7DF for the power port
 * (PD1), each with an analog/pin part in page E2 (E2:26..2E for PD0,
 * E2:32..3A for PD1). Each port's protocol state is a struct pd_port
 * (pd0_port, pd1_port) and a set of pd0_ / pd1_ variables. Most functions
 * here come in pairs that run the same sequence on the two ports; each pair
 * says so. The data port does more: PD 3.x revision bits, extended
 * messages, cable (SOP') traffic.
 *
 * Receive (INT1, E5:32 bit 3 data port / bit 2 power port): read and
 * acknowledge the PHY status (C7:36 / C7:B6), then
 *   bit 6  a reset/ordered set was received: its kind is C7:1C[2:0]
 *          (kind 1 = the cable, SOP'; kind 4 latches pd0_recovery_req);
 *   bit 0  a message header is ready in C7:3A/3B (C7:BA/BB): split it into
 *          message type, data byte count, MessageID and Extended bit;
 *   bit 4  data bytes are in the FIFO (C7:23 / C7:A3, a read pops one byte):
 *          read until the status shows bit 5 (end of message), then flag
 *          the message (pd0_port.tx_settled / pd1_port.tx_settled) for the policy code; a GoodCRC is
 *          matched against the message awaiting it (pd0_port.pending_key);
 *   bit 2  the transmission finished: arm the reply timer (Timer 2) or run
 *          the message handler on the received message.
 * Transmit: pd0_tx / pd1_tx copy the message at pd0_port.tx_msg / pd1_port.tx_msg (a code
 * template or an XRAM buffer) into the TX data port (C7:1A / C7:9A),
 * filling in the header's revision, roles and MessageID, then start it
 * (C7:23 / C7:A3 = 1 and a pulse on C7:18 / C7:98 bit 3); pd_tx_retry_arm then
 * sets the port's retry/timeout state and Timer 2's reload.
 */
#include "fw.h"

/* PD header fields (USB PD r3.1 §6.2.1.1): byte 0 bits 4..0 message type,
 * bit 5 port data role, bits 7..6 spec revision; byte 1 bit 0 port power
 * role, bits 3..1 MessageID, bits 6..4 number of data objects, bit 7
 * Extended. */

/* Discover Identity request (header 0x104F: Vendor_Defined, one
 * data object; VDM header 0xFF008001: SVID 0xFF00, structured, Discover
 * Identity), sent by pd0_discover_identity. */
static const __code uint8_t tpl_discover_identity[6] = {
    0x4F, 0x10, 0x01, 0x80, 0x00, 0xFF,
};

/* Data-port PD receive (INT1, E5:32 bit 3). */
void pd0_rx(void)
{
    uint8_t status, b, i;

    status = PD0_STATUS;
    pd0_rx_status = status;
    pd0_rx_status2 = R_C7_1B;
    PD0_STATUS = 0xEB;
    R_C7_1B = 0xFF;

    if (status & 0x40) {
        pd0_rx_index = 0;
        b = R_C7_1C & 0x07;
        pd0_rx_ordered_set = b;
        R_C7_1C = 0xF8;
        R_C7_17 = 0x40;
        R_C7_18 = 0;
        R_C7_19 = 1;
        if (b == 1) {
            cable_id_state = 0;
            R_C7_17 &= ~0x10;
        } else if (cable_id_state) {
            R_C7_19 = 2;
            R_C7_17 = 0x42;
            R_C7_18 = 0x40;
        } else {
            R_C7_17 = 0x50;
            R_C7_18 = 0;
        }
        if (pd0_rx_ordered_set == 4)
            pd0_recovery_req = 1;
    }

    status = pd0_rx_status;
    if (status & 0x01) {
        pd0_port.rx_header1 = PD0_RX_HEADER1;
        pd0_port.rx_header0 = PD0_RX_HEADER0;
        pd0_port.rx_msg_id = pd0_port.rx_header1 & 0x0E;
        pd0_port.rx_type = pd0_port.rx_header0 & 0x1F;
        b = pd0_port.rx_header1 & 0x70;
        pd0_port.rx_count = b >> 2;
        pd0_port.rx_extended = (pd0_port.rx_header1 & 0x80) ? 1 : 0;
    }

    if (status & 0x10) {
        PD0_STATUS = 0x10;
        /* One FIFO read per status poll until bit 5 (end of message). An
         * extended message's first two bytes are its extended header
         * (pd0_port.rx_ext_header0/1). SECURITY: the sender sets the length;
         * bytes 0x22 and 0x23 of a non-extended message go to
         * pd0_port.rx_header0/1, and those past 0x23 all to
         * pd0_rx_overflow. */
        do {
            i = pd0_rx_index;
            if (pd0_port.rx_extended) {
                if (i < 2) {
                    b = PD0_FIFO;
                    if (i == 0)
                        pd0_port.rx_ext_header0 = b;
                    else
                        pd0_port.rx_ext_header1 = b;
                } else {
                    b = PD0_FIFO;
                    if (i < 0x24)
                        pd0_port.rx_data[i - 2] = b;
                    else
                        pd0_rx_overflow = b;
                }
            } else {
                pd0_port.rx_ext_header0 = 0;
                pd0_port.rx_ext_header1 = 0;
                b = PD0_FIFO;
                if (i < sizeof pd0_port.rx_data)
                    pd0_port.rx_data[i] = b;
                else if (i == 0x22)
                    pd0_port.rx_header0 = b;
                else if (i == 0x23)
                    pd0_port.rx_header1 = b;
                else
                    pd0_rx_overflow = b;
            }
            pd0_rx_index++;
            pd0_rx_status |= PD0_STATUS;
        } while (!(pd0_rx_status & 0x20));

        pd0_rx_index = 0;
        pd0_retry_count = 0;
        pd0_port.tx_vdm = 0;
        dp_multi_function_pref = 0;
        if (!pd_timer_port)
            TR2 = 0;
        if (pd0_rx_ordered_set == 1 && dr_swap_req == 2) {
            pd0_data_role = 0;
            dr_swap_req = 0;
        }
        pd0_port.tx_settled = 1;
        if (pd0_port.rx_count || pd0_port.rx_type != 1) {
            pd0_port.rx_msg_pending = 1;
        } else {
            /* GoodCRC: settle the message that awaited it. */
            if (cable_id_state == 1)
                cable_id_state = 2;
            pd0_port.rx_msg_pending = 0;
            pd0_goodcrc_wait = 0;
            if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_ACCEPT)) {
                pd0_accept_acked();
            } else if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_PS_RDY)) {
                pd0_ps_rdy_acked();
            } else if (pd0_port.pending_key == PD_KEY(1, PD_DATA_SOURCE_CAP)) {
                pd0_caps_exchanged = 2;
            } else if (pd0_port.pending_key == PD_KEY(0, PD_CTRL_FR_SWAP)) {
                pd0_power_role = 0;
                pd0_port.pending_key = 0xFF;
                if (fr_swap_state == 1)
                    fr_swap_state = 2;
            }
        }
    }

    if (pd0_rx_status & 0x04) {
        PD0_STATUS = 0x04;
        if (pd0_goodcrc_wait == 1) {
            pd0_goodcrc_wait = 0;
            if (pd0_bist_test_mode == 1) {
                TR2 = 0;
            } else {
                pd0_port.tx_timer_armed = 1;
                t2_load_f1a6();
                TR2 = 1;
            }
        }
        if (pd0_port.rx_msg_pending == 1) {
            pd0_port.tx_vdm = 0;
            dp_multi_function_pref = 0;
            if (pd0_rx_ordered_set == 1) {
                if (!pd0_bist_test_mode)
                    pd_message_handler();
            } else if (cable_id_state && pd0_port.rx_count >= 4 && pd0_port.rx_type == 0x0F &&
                       (pd0_port.rx_data[0] & 0x1F) == 1) {
                /* Discover Identity answered (VDM command 1). */
                cable_id_state = 0;
                cable_5a = ((pd0_port.rx_data[16] & 0x60) == 0x40) ? 1 : 0;
            }
            pd0_port.rx_msg_pending = 0;
        }
    }
}

/* Power-port PD receive (INT1, E5:32 bit 2); pd0_rx's sequence on
 * the C780 block without extended messages and cable traffic. */
void pd1_rx(void)
{
    uint8_t status, b, i;

    status = PD1_STATUS;
    pd1_rx_status = status;
    pd1_rx_status2 = R_C7_9B;
    PD1_STATUS = 0xEB;
    R_C7_9B = 0xFD;

    if ((status & 0x80) && !(R_C7_B0 & 0xF0))
        pd1_detach_handler();

    status = pd1_rx_status;
    if (status & 0x40) {
        pd1_rx_index = 0;
        b = R_C7_9C & 0x07;
        pd1_rx_ordered_set = b;
        R_C7_9C = 0xF8;
        if (b == 1)
            R_C7_97 &= ~0x10;
        else
            R_C7_97 |= 0x10;
    }

    if (status & 0x01) {
        pd1_port.rx_header1 = PD1_RX_HEADER1;
        pd1_port.rx_header0 = PD1_RX_HEADER0;
        pd1_port.rx_msg_id = pd1_port.rx_header1 & 0x0E;
        pd1_port.rx_type = pd1_port.rx_header0 & 0x1F;
        b = pd1_port.rx_header1 & 0x70;
        pd1_port.rx_count = b >> 2;
        pd1_port.rx_extended = (pd1_port.rx_header1 & 0x80) ? 1 : 0;
    }

    if (pd1_rx_status & 0x10) {
        PD1_STATUS = 0x10;
        /* As pd0_rx's non-extended case. */
        do {
            i = pd1_rx_index;
            b = PD1_FIFO;
            if (i < sizeof pd1_port.rx_data)
                pd1_port.rx_data[i] = b;
            else if (i == 0x22)
                pd1_port.rx_header0 = b;
            else if (i == 0x23)
                pd1_port.rx_header1 = b;
            else
                pd1_rx_overflow = b;
            pd1_rx_index++;
            pd1_rx_status |= PD1_STATUS;
        } while (!(pd1_rx_status & 0x20));

        if (!pd_timer_port)
            TR2 = 0;
        pd1_rx_index = 0;
        pd1_retry_count = 0;
        pd1_port.tx_settled = 1;
        if (pd1_rx_ordered_set == 1) {
            if (pd1_port.rx_count || pd1_port.rx_type != 1)
                pd1_port.rx_msg_pending = 1;
        } else {
            pd1_port.rx_msg_pending = 0;
        }
        /* GoodCRC. */
        if (!pd1_port.rx_count && pd1_port.rx_type == 1)
            pd1_goodcrc_wait = 0;
    }

    if (pd1_rx_status2 & 0x02) {
        R_C7_9B |= 0x02;
        if (R_C7_8C & 0x02)
            pd1_vbus_drop_handler();
    }

    if (pd1_rx_status & 0x04) {
        PD1_STATUS = 0x04;
        if (pd1_goodcrc_wait == 1) {
            pd1_goodcrc_wait = 0;
            pd1_port.tx_timer_armed = 1;
            t2_load_f1a6();
            TR2 = 1;
        }
        if (pd1_port.rx_msg_pending == 1) {
            pd1_port.rx_msg_pending = 0;
            if (pd1_rx_ordered_set == 1)
                pd1_message_handler();
        }
    }
}

/* Data-port PD transmit of the message at pd0_port.tx_msg (header, then
 * NDO * 4 data bytes). The header gets the negotiated revision (PD 2.0
 * below pd0_port.rev 2, else 3.0), the data role pd0_data_role, the power role and, on
 * the first try (pd0_tx_first_try == 1), a new MessageID (pd0_port.tx_msg_id); a retry resends
 * pd0_tx_sent_msg_id's. Leaves pd0_port.tx_msg past the message and pd0_port.tx_direct at it. */
void pd0_tx(void) __reentrant
{
    uint8_t i, n, b;

    pd0_port.tx_direct = pd0_port.tx_msg;
    n = pd0_port.tx_msg[1] & 0x70;
    n = (n >> 2) + 2;
    pd0_tx_len = n;
    /* pd0_tx_last_byte/pd0_tx_byte_count get the last byte and index after the loop. */
    i = 0;
    do { /* n >= 2: the loop always runs */
        b = *pd0_port.tx_msg++;
        if (i < 10) {
            if (i == 1) {
                b &= 0xF0;
                if (pd0_tx_first_try == 1) {
                    pd0_tx_sent_msg_id = pd0_port.tx_msg_id;
                    pd0_port.tx_msg_id = pd_next_msg_id(pd0_port.tx_msg_id);
                }
                b |= pd0_tx_sent_msg_id;
                if (pd0_power_role)
                    b |= 0x01;
                else
                    b &= ~0x01;
            } else if (i == 0) {
                b &= 0x1F;
                if (pd0_port.rev < 2)
                    b |= 0x40;
                else
                    b |= 0x80;
                if (pd0_data_role)
                    b |= 0x20;
                else
                    b &= ~0x20;
            } else if (dp_multi_function_pref && i == 6) {
                b |= 0x10;
            } else if (pd0_port.tx_vdm == 1 && i == 3) {
                /* VDM header bit 13: structured VDM version 2.x on PD 3.x. */
                if (pd0_port.rev < 2)
                    b &= ~0x20;
                else
                    b |= 0x20;
            }
        } else if (i == 10) {
            R_C7_19 = 9;
        }
        PD0_TX_DATA = b;
    } while (++i < n);
    pd0_tx_last_byte = b;
    pd0_tx_byte_count = i;
    if (i <= 10)
        R_C7_19 = 9;
    PD0_FIFO = 1;
    REG_PULSE(R_C7_18, 0x08);
    pd_tx_retry_arm(0);
}

/* Power-port PD transmit of the message at pd1_port.tx_msg; pd0_tx's
 * sequence on the C780 block, always PD 2.0, without the VDM fixups. */
void pd1_tx(void) __reentrant
{
    uint8_t i, n, b;

    pd1_port.tx_direct = pd1_port.tx_msg;
    n = pd1_port.tx_msg[1] & 0x70;
    n = (n >> 2) + 2;
    pd1_tx_len = n;
    i = 0;
    do {
        b = *pd1_port.tx_msg++;
        if (i < 10) {
            if (i == 1) {
                b &= 0xF0;
                if (pd1_tx_first_try == 1) {
                    pd1_tx_sent_msg_id = pd1_port.tx_msg_id;
                    pd1_port.tx_msg_id = pd_next_msg_id(pd1_port.tx_msg_id);
                }
                b |= pd1_tx_sent_msg_id;
                if (pd1_power_role)
                    b |= 0x01;
                else
                    b &= ~0x01;
            } else if (i == 0) {
                b = (b & 0x1F) | 0x40;
                if (pd1_data_role)
                    b |= 0x20;
                else
                    b &= ~0x20;
            }
        } else if (i == 10) {
            R_C7_99 = 9;
        }
        PD1_TX_DATA = b;
    } while (++i < n);
    pd1_tx_last_byte = b;
    pd1_tx_byte_count = i;
    if (i <= 10)
        R_C7_99 = 9;
    PD1_FIFO = 1;
    REG_PULSE(R_C7_98, 0x08);
    pd_tx_retry_arm(1);
}

/* After a transmit on port 0 (data) or 1 (power): clear the
 * port's GoodCRC wait and received flag, count a retry unless this was the
 * first try (pd0_tx_first_try / pd1_tx_first_try == 1), set the retry limit (2 on PD 3.x, else
 * 3), mark the next send a first try, load Timer 2 and wait for GoodCRC
 * (the data port only when pd0_caps_exchanged is set). */
void pd_tx_retry_arm(uint8_t port) __reentrant
{
    if (port == 0) {
        pd0_goodcrc_wait = 0;
        pd0_port.tx_settled = 0;
        if (pd0_tx_first_try != 1)
            pd0_retry_count++;
        pd0_retry_limit = (pd0_port.rev < 2) ? 3 : 2;
        pd0_tx_first_try = 1;
        t2_load_f1a6();
        if (pd0_caps_exchanged)
            pd0_goodcrc_wait = 1;
    } else if (port == 1) {
        pd1_goodcrc_wait = 0;
        pd1_port.tx_settled = 0;
        if (pd1_tx_first_try != 1)
            pd1_retry_count++;
        pd1_retry_limit = (pd1_port.rev < 2) ? 3 : 2;
        pd1_tx_first_try = 1;
        t2_load_f1a6();
        pd1_goodcrc_wait = 1;
    }
}

/* Data-port raw send of the 6-byte message at pd0_port.tx_msg (header
 * and one data object) with MessageID 0, power role 0 and data role 0, the
 * revision from pd0_port.rev; C7:19 = 0x0A. */
void pd0_tx_raw(void)
{
    uint8_t i, b;

    for (i = 0; i < 6; i++) {
        b = *pd0_port.tx_msg++;
        if (i == 1) {
            b &= 0xF0;
        } else if (i == 0) {
            b &= 0x1F;
            if (pd0_port.rev < 2)
                b |= 0x40;
            else
                b |= 0x80;
            b &= ~0x20;
        }
        PD0_TX_DATA = b;
    }
    R_C7_19 = 0x0A;
    PD0_FIFO = 1;
    REG_PULSE(R_C7_18, 0x08);
}

/* Power-port PHY registers (C7:8C..9C), the mirror of pd0_phy_regs_init plus
 * C7:A6/A7 and the C7:9B enable bit. */
void pd1_phy_regs_init(void)
{
    R_C7_8C = 0x60;
    R_C7_8D = 0x08;
    R_C7_92 = 0xA0;
    R_C7_93 = 0x0F;
    R_C7_95 = 0xA0;
    R_C7_96 = 0x07;
    R_C7_97 = 0x50;
    R_C7_99 = 0x01;
    R_C7_8C = 0x60 & ~0x02;
    R_C7_9B |= 0x02;
    R_C7_A6 = 0x20;
    R_C7_A7 = 0x0A;
    PD1_STATUS = 0xEB;
    R_C7_9B = 0xFF;
    R_C7_9C = 0xF8;
}

/* Data port: send the Discover Identity request (pd0_tx_raw) up to 10
 * times, 3 ms apart, while cable_id_state == 1 (pd0_rx clears it on an answer);
 * C7:19 = 0x15 before the first and the sixth try. */
void pd0_discover_identity(void)
{
    uint8_t n;

    pd0_phy_command(0x15);
    delay_ms(3);
    for (n = 0; n < 10 && cable_id_state == 1; n++) {
        if (n == 5) {
            pd0_phy_command(0x15);
            delay_ms(10);
        }
        pd0_port.tx_msg_id = 0;
        pd0_port.tx_msg = (uint8_t *)tpl_discover_identity;
        pd0_tx_raw();
        delay_ms(3);
    }
}

/* Power-port analog setup (E2:32/39/3A/44), then up to 3 times
 * pulse C7:D0 (0x30, 1 ms, 0x70, 2 ms) until C7:D5[1:0] reads 1. */
void pd1_phy_c7d0_pulse_init(void)
{
    uint8_t n;

    R_E2_32 = 0x08;
    R_E2_39 &= ~0x10;
    R_E2_3A = 0x78;
    R_E2_44 |= 0x06;
    n = 0;
    do {
        R_C7_D0 = 0x30;
        delay_ms(1);
        R_C7_D0 = 0x70;
        delay_ms(2);
        n++;
    } while (n < 3 && (R_C7_D5 & 0x03) != 1);
}

/* Data-port PHY registers (C7:0C..36). */
void pd0_phy_regs_init(void)
{
    R_C7_0C = 0x60;
    R_C7_0D = 0x08;
    R_C7_12 = 0xA0;
    R_C7_13 = 0x0F;
    R_C7_15 = 0xA0;
    R_C7_16 = 0x06;
    R_C7_17 = 0x50;
    R_C7_19 = 0x01;
    PD0_STATUS = 0xEB;
    R_C7_1B = 0xFF;
    R_C7_1C = 0xF8;
}

/* Data-port analog setup (E2:26..2E, E6:39 bits 5..4); mirror of
 * pd1_phy_e2_init. */
void pd0_phy_e2_init(void)
{
    R_E2_27 = 0x50;
    R_E2_28 &= ~0x04;
    R_E2_29 = 0x1F;
    R_E2_2A = 0x70;
    R_E2_2B = 0x06;
    R_E2_2C = 0x20;
    R_E2_2D = 0x7B;
    R_E2_2E = 0x08;
    R_E6_39 |= 0x30;
    R_E2_26 |= 0x30;
}

/* Power-port analog setup (E2:32..3A, E6:39 bits 2..1); mirror of
 * pd0_phy_e2_init (E2:38 = 0x04 where PD0 has 0x20). */
void pd1_phy_e2_init(void)
{
    R_E2_33 = 0x50;
    R_E2_34 &= ~0x04;
    R_E2_35 = 0x1F;
    R_E2_36 = 0x70;
    R_E2_37 = 0x06;
    R_E2_38 = 0x04;
    R_E2_39 = 0x7B;
    R_E2_3A = 0x08;
    R_E6_39 |= 0x06;
    R_E2_32 |= 0x30;
}

/* Data-port PHY registers C7:2D/2E/42..46; mirror of pd1_phy_regs_ad_init. */
void pd0_phy_regs_2d_init(void)
{
    R_C7_2D = 0x42;
    R_C7_2E = 0x85;
    R_C7_42 = 0x6E;
    R_C7_43 = 0x20;
    R_C7_45 = 0xC8;
    R_C7_46 = 0x6F;
}

/* Power-port PHY registers C7:AD/AE/C2..C6; mirror of pd0_phy_regs_2d_init. */
void pd1_phy_regs_ad_init(void)
{
    R_C7_AD = 0x42;
    R_C7_AE = 0x85;
    R_C7_C2 = 0x6E;
    R_C7_C3 = 0x20;
    R_C7_C5 = 0xC8;
    R_C7_C6 = 0x6F;
}

/* Data-port PHY registers C7:0A/29/2C/3C..3E; mirror of pd1_phy_regs_8a_init. */
void pd0_phy_regs_0a_init(void)
{
    R_C7_0A = 0x10;
    R_C7_29 = 0x46;
    R_C7_2C = 0x62;
    R_C7_3C = 0xA8;
    R_C7_3D = 0x0C;
    R_C7_3E = 0;
}

/* Power-port PHY registers C7:8A/A9/AC/BC..BE; mirror of pd0_phy_regs_0a_init. */
void pd1_phy_regs_8a_init(void)
{
    R_C7_8A = 0x10;
    R_C7_A9 = 0x46;
    R_C7_AC = 0x62;
    R_C7_BC = 0xA8;
    R_C7_BD = 0x0C;
    R_C7_BE = 0;
}

/* Power-port PHY init. */
void pd1_phy_init(void)
{
    pd1_phy_c7d0_pulse_init();
    pd1_phy_regs_a2_init();
    pd1_phy_regs_ad_init();
    irq_enable(0x0B, 1);
    irq_ack(0x0B);
    pd1_phy_regs_init();
    pd1_phy_regs_8a_init();
    pd1_phy_e2_init();
}

/* Data-port PHY registers C7:22/4D/40/41/31; mirror of pd1_phy_regs_a2_init. */
void pd0_phy_regs_22_init(void)
{
    R_C7_22 = 0x0A;
    R_C7_4D = 0x0A;
    R_C7_40 = 0xFD;
    R_C7_41 = 0x75;
    R_C7_31 = 0x07;
}

/* Power-port PHY registers C7:A2/CD/C0/C1/B1; mirror of pd0_phy_regs_22_init. */
void pd1_phy_regs_a2_init(void)
{
    R_C7_A2 = 0x0A;
    R_C7_CD = 0x0A;
    R_C7_C0 = 0xFD;
    R_C7_C1 = 0x75;
    R_C7_B1 = 0x07;
}

/* Data-port PHY init. */
void pd0_phy_init(void)
{
    pd0_phy_regs_22_init();
    pd0_phy_regs_2d_init();
    irq_enable(0x0A, 1);
    irq_ack(0x0A);
    pd0_phy_regs_init();
    pd0_phy_regs_0a_init();
    pd0_phy_e2_init();
}

/* Data port: clear cable_id_state and set the receive mode pd0_rx sets
 * when no cable traffic is expected (C7:19 = 1, C7:17 = 0x40, C7:18 = 0). */
void pd0_cable_rx_stop(void)
{
    cable_id_state = 0;
    R_C7_19 = 1;
    R_C7_17 = 0x40;
    R_C7_18 = 0;
}

/* Set (on == 1) or clear power-port C7:9B bit 1. */
void pd1_vbus_drop_event_set(uint8_t on)
{
    if (on == 1)
        R_C7_9B |= 0x02;
    else
        R_C7_9B &= ~0x02;
}

/* Set (on == 1) or clear power-port C7:8C bit 1. */
void pd1_vbus_drop_enable(uint8_t on)
{
    if (on == 1)
        R_C7_8C |= 0x02;
    else
        R_C7_8C &= ~0x02;
}

/* Next MessageID, kept shifted as in header byte 1 (bits 3..1):
 * id + 2, wrapping to 0 past 0x0E. */
uint8_t pd_next_msg_id(uint8_t id) __reentrant
{
    id += 2;
    return id > 0x0E ? 0 : id;
}

/* Data port: C7:54 = 0xFB, then 0xFF. */
void pd0_c754_bit2_pulse(void)
{
    R_C7_54 = 0xFB;
    R_C7_54 = 0xFF;
}

/* Start Timer 2 from 0xC000. */
void t2_arm_c000(void)
{
    TH2 = 0xC0;
    TL2 = 0;
    TR2 = 1;
}

/* Data port: C7:19 = v, then 1. */
void pd0_phy_command(uint8_t v)
{
    R_C7_19 = v;
    R_C7_19 = 1;
}

/* Power port: C7:99 = v, then 1; mirror of pd0_phy_command. */
void pd1_phy_command(uint8_t v)
{
    R_C7_99 = v;
    R_C7_99 = 1;
}

/* Clear pd0_port.pending_key (no message awaits GoodCRC), then pd0_power_transition_start. */
void pd0_accept_acked(void)
{
    pd0_port.pending_key = 0xFF;
    pd0_power_transition_start();
}

/* Load Timer 2 with 0xF1A6 (the PD reply timeout; not started). */
void t2_load_f1a6(void)
{
    TH2 = 0xF1;
    TL2 = 0xA6;
}

/* Post an HPD request (hpd_request) for the PD side to send as a
 * DP Attention. */
void hpd_request_post(uint8_t req) __reentrant
{
    hpd_request = req;
}
