// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Interrupt handlers: INT0 (DP RX and AUX), INT1 (USB-C PD), Timer 2 (AUX
 * deferred work and PD timers) and the extra vector 0x43.
 *
 * Each handler services the pending sources in a fixed order and
 * acknowledges each one with a write-1-then-0 pulse on its clear register.
 *
 * Some functions the handlers call also run from the main loop, e.g.
 * rx_eq_enable, rx_reset_assert, rx_fec_enable and rx_eq_retry_set from the RX state machine.
 * Those are __reentrant (fw.h): their parameters and locals live on the
 * stack, so an interrupt that lands inside one cannot corrupt the
 * interrupted call. rx_e1_18_1b_cb_23_set runs only from INT0.
 *
 * vbus_check, pd0_cc_line_select and pd0_detached run from both the main loop
 * and INT1 but are not __reentrant: their locals are static, so INT1
 * landing inside one can corrupt the interrupted call.
 */
#include "fw.h"

void int0_isr(void) __interrupt(0)
{
    uint8_t status;

    if (INT0_STATUS & 0x10) {
        REG_PULSE(INT0_CLEAR, 0x10);
        rx_link_rate_index = RX_LINK_CFG & 0x03;
        rx_eq_band_apply(rx_link_rate_index);
        rx_phy_link_rate_set(rx_link_rate_cfg_on);
        rx_eq_retry_set(0);
        REG_PULSE(R_C3_D8, 0x04);
        rx_int0_bit7_flag = 0;
    }
    if (INT0_STATUS & 0x08) {
        REG_PULSE(INT0_CLEAR, 0x08);
        rx_lane_count = rx_lane_count_read();
        REG_PULSE(R_C3_D8, 0x08);
    }
    /* Bit 7 is tested on this same read, before bit 2's acknowledge. */
    status = INT0_STATUS;
    if (status & 0x04) {
        REG_PULSE(INT0_CLEAR, 0x04);
        REG_PULSE(R_C3_D8, 0x80);
    }
    if (status & 0x80) {
        rx_eq_retry_set(0);
        rx_int0_bit7_flag = 1;
        REG_PULSE(INT0_CLEAR, 0x80);
        rx_eq_enable(0);
        rx_e1_18_1b_cb_23_set(1);
        rx_fec_enable(0);
        if (rx_retrain_armed == 1)
            rx_retrain_req = 1;
    }
    if (INT0_STATUS & 0x40) {
        rx_int0_bit7_flag = 0;
        REG_PULSE(INT0_CLEAR, 0x40);
        rx_e1_18_1b_cb_23_set(0);
        rx_reset_assert(1);
        /* Hand the rest of the work to Timer 2: aux_t2_work() runs from the
         * Timer 2 interrupt until aux_t2_count reaches its limit. */
        aux_t2_stage = 2;
        aux_t2_count = 0;
        t2_arm_short();
    }
    if (INT0_STATUS & 0x20) {
        REG_PULSE(INT0_CLEAR, 0x20);
        test_sink_update();
    }
    if (INT0_STATUS & 0x02) {
        REG_PULSE(INT0_CLEAR, 0x02);
        rx_phy_e8_rate_regs_set();
        REG_PULSE(R_C3_D8, 0x10);
    }
    if (INT0_STATUS & 0x01) {
        REG_PULSE(INT0_CLEAR, 0x01);
        aux_request(0);
        REG_PULSE(R_C3_D9, 0x80);
    }
    if (INT0_STATUS2 & 0x10) {
        REG_PULSE(INT0_CLEAR2, 0x10);
        REG_PULSE(R_C2_21, 0x60);
        aux_request(1);
    }
}

void int1_isr(void) __interrupt(2)
{
    if (INT1_STATUS & 0x08) {
        REG_PULSE(INT1_CLEAR, 0x08);
        pd0_rx();
    }
    if (INT1_STATUS & 0x04) {
        REG_PULSE(INT1_CLEAR, 0x04);
        pd1_rx();
    }
    if (INT1_STATUS & 0x20) {
        REG_PULSE(INT1_CLEAR, 0x20);
        if (R_C4_F1 & 0x40) {
            REG_PULSE(R_C4_3E, 0x10);
            /* In Type-C mode HPD goes to the host as a DP Attention. */
            if (rx_state > 4 && typec_mode == 1)
                hpd_request_post(3);
        }
    }
}

void timer2_isr(void) __interrupt(5)
{
    TF2 = 0;
    TR2 = 0;
    while (aux_t2_stage >= 2 && aux_t2_count < 0x3C) {
        aux_t2_work();
        aux_t2_count++;
    }
    if (pd_timer_port == 1) {
        pd0_t2_ps_rdy();
    } else {
        pd0_tx_timeout();
        pd1_tx_timeout();
    }
}

/* Pulse bit m of R_E8_12 when a level reading is strictly between 0x20 and
 * 0x65. */
static void lane_check(uint8_t level, uint8_t m)
{
    if (level > 0x20 && level < 0x65)
        REG_PULSE(R_E8_12, m);
}

/* One pass of the deferred work INT0 bit 6 starts: check the four
 * readings E8:38/3A/3C/3E (likely one per DP lane). */
void aux_t2_work(void)
{
    lane_check(R_E8_38, 0x01);
    lane_check(R_E8_3A, 0x02);
    lane_check(R_E8_3C, 0x04);
    lane_check(R_E8_3E, 0x08);
}

/* Start Timer 2 with a short count (0xFFFC). */
void t2_arm_short(void)
{
    TH2 = 0xFF;
    TL2 = 0xFC;
    TR2 = 1;
}

/* The extra interrupt (vector 0x43) only clears its flag. */
void ext2_isr(void) __interrupt(8)
{
    SFR_91 = 0;
}
