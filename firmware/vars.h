/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026, Alec Dusheck */

/* Firmware state.
 *
 * The linker places every variable; no code depends on where. vars.c
 * defines them (VAR is empty there and extern everywhere else) and holds
 * the initial values; everything else starts at zero.
 *
 * Types: SDCC is little-endian for multi-byte values and puts a generic
 * pointer's type byte last. C code never depends on byte order.
 *
 * Variables touched by a single function are globals because some are read
 * before they are written and so carry their value from one call to the
 * next (e.g. pd0_tx_sent_msg_id in pd0_tx).
 *
 * Variables a reset function clears together are listed under it.
 */
#ifndef VARS_H
#define VARS_H

#include <stdint.h>

#ifndef VAR
#define VAR extern
#endif

/* IRAM (direct-addressed, for the interrupt handlers). DP AUX request as
 * aux_request() parses it from AUX_FIFO, and the AUX work Timer 2 runs after
 * INT0 bit 6. */
VAR __data uint8_t aux_hdr;        /* first FIFO byte: command | address bits 19:16 */
VAR __data uint8_t aux_len;        /* request length */
VAR __data uint8_t aux_i;
VAR __data uint8_t aux_cmd;        /* aux_hdr & 0xF0 */
VAR __data uint8_t aux_fifo_count; /* AUX_FIFO_COUNT at entry */
VAR __data uint8_t aux_reply_i;
/* iram_vars_clear */
VAR __data uint8_t rx_retrain_armed;
VAR __data uint8_t rx_retrain_req;
VAR __data uint8_t rx_link_rate_index;
VAR __data uint8_t aux_t2_stage;   /* >= 2: timer2_isr runs aux_t2_work() */
VAR __data uint8_t fec_config;
VAR __data uint8_t fec_ready;
VAR __data uint8_t aux_addr[3];    /* address bits 19:16, 15:8, 7:0 (I2C address in [2]) */
VAR __data uint8_t aux_data[16];   /* request data */
VAR __data uint16_t aux_t2_count;  /* aux_t2_work() runs while < 0x3C */
VAR __data uint8_t rx_int0_bit7_flag; /* INT0 bit 7 sets it, bits 4 and 6 clear it */

/* Video timing from the DP Main Stream Attributes (msa_timing_read), in pixels and
 * lines. Porches and sync widths are derived from the MSA start and total. */
struct rx_timing {
    uint8_t hsync_neg;  /* MSA HSP: 1 = active-low */
    uint8_t vsync_neg;  /* MSA VSP: 1 = active-low */
    uint16_t hbp;
    uint16_t hsync;
    uint16_t hfp;
    uint16_t hact;
    uint16_t htotal;
    uint16_t vbp;
    uint16_t vsync;
    uint16_t vfp;
    uint16_t vact;
    uint16_t vtotal;
};

/* XRAM. */
VAR __xdata uint8_t edid_buf[512]; /* EDID served to the DP source (512 B), loaded at boot */
VAR __xdata uint8_t dp_attention_pending;
VAR __xdata uint8_t attach_pull_toggle;
VAR __xdata uint8_t attach_state;
VAR __xdata uint32_t attach_stamp;
extern __xdata uint32_t attach_timeout; /* timeout used by attach_timer_task (initially 2000000 ticks) */
VAR __xdata uint8_t pd0_sink_step_entry;
VAR __xdata uint8_t pd0_sink_step;
VAR __xdata uint32_t pd0_sink_stamp;
VAR __xdata uint8_t dr_swap_step;
VAR __xdata uint32_t dr_swap_stamp;
VAR __xdata uint32_t vconn_swap_stamp;
VAR __xdata uint8_t src_caps_first_pass;
VAR __xdata uint8_t src_caps_step;
VAR __xdata uint32_t src_caps_stamp;
VAR __xdata uint8_t pr_swap_timeout_step;
VAR __xdata uint32_t pr_swap_timeout_stamp;
VAR __xdata uint32_t src_caps_delayed_stamp;
VAR __xdata uint8_t hpd_attention_timing;
VAR __xdata uint32_t hpd_attention_stamp;
VAR __xdata uint8_t hpd_attention_event;
/* pd_state_reset and pd_init_clear */
VAR __xdata uint8_t pr_swap_stage;
VAR __xdata uint8_t vconn_swap_state;
VAR __xdata uint8_t pd0_bist_test_mode; /* sink-side PD tasks in pd_main_task run only while 0 */

VAR __xdata uint8_t switch_docked; /* Nintendo Switch dock assumed (a PR_Swap refused before Discover Modes) */
VAR __xdata uint8_t src_caps_15v_class;
/* pd_state_reset: the data port's policy state */
VAR __xdata uint8_t pd0_power_role; /* 0 sink, 1 source */
VAR __xdata uint8_t pd0_data_role;
VAR __xdata uint8_t input_typec; /* non-zero: USB-C input (the PD main task runs only then) */
VAR __xdata uint8_t pd0_power_settled;
VAR __xdata uint8_t dp_multi_function_pref;
VAR __xdata uint8_t pd_timer_port; /* 1: Timer 2 runs the power-port PD timers, else the data-port ones */
VAR __xdata uint8_t typec_attach_phase;
VAR __xdata uint8_t pd0_wait_resend;
VAR __xdata uint8_t pd0_tx_first_try;
VAR __xdata uint8_t pd0_cc_line;
VAR __xdata uint8_t pd0_cc_pull_flag;
VAR __xdata uint8_t pd0_recovery_req;
VAR __xdata uint8_t hpd_raised; /* HPD raised: the DP side may train */
VAR __xdata uint8_t pd0_goodcrc_wait;
VAR __xdata uint8_t typec_receptacle;
VAR __xdata uint8_t pr_swap_first; /* 1: a PR_Swap is wanted, 2: sent or refused */
VAR __xdata uint8_t pr_swap_awaiting;
VAR __xdata uint8_t pd0_power_role_state;
VAR __xdata uint8_t dr_swap_req;
VAR __xdata uint8_t fr_swap_state;
VAR __xdata uint8_t pd0_caps_exchanged;
VAR __xdata uint8_t pd0_swap_flags;
VAR __xdata uint8_t pd0_retry_count;
VAR __xdata uint8_t pd0_ams_state;
VAR __xdata uint8_t dp_alt_status; /* DP Alt Mode status; bit 5 = DP Configure done */
VAR __xdata uint8_t src_caps_send_count;
VAR __xdata uint8_t typec_dp_lane_count;
VAR __xdata uint8_t pd0_retry_limit;
VAR __xdata uint8_t src_caps_delayed;
VAR __xdata uint8_t pr_swap_src_t2_pending;
VAR __xdata uint8_t pd0_vconn_source;
VAR __xdata uint8_t cable_5a;
VAR __xdata uint8_t cable_id_state;
VAR __xdata uint8_t dp_pin_assignment;
VAR __xdata uint8_t try_src_count;
VAR __xdata uint8_t hpd_request; /* HPD request for the PD side, sent as a DP Attention */
VAR __xdata uint8_t pd0_soft_reset_req;

/* State of one PD port: the data port (pd0_port) and the power port
 * (pd1_port). pd_save_rx_msg and pd_build_request work on either. */
struct pd_port {
    uint8_t tx_settled;
    uint8_t rx_msg_pending;
    uint8_t tx_vdm;         /* data port only */
    uint8_t tx_timer_armed;
    uint8_t rx_count;       /* received data-object byte count */
    uint8_t rx_extended;
    uint8_t rx_type;
    uint8_t *tx_msg;        /* message pdN_tx sends: a code template or an XRAM buffer */
    uint8_t rx_data[34];    /* received data objects */
    uint8_t rx_header0;     /* received message header, first byte */
    uint8_t rx_header1;     /* received message header, second byte */
    uint8_t rx_ext_header0;
    uint8_t rx_ext_header1;
    uint8_t request[6];     /* the Request pd_build_request builds */
    uint8_t vdm_reply[10];  /* structured VDM reply */
    uint8_t rx_msg[34];     /* header and data objects pd_save_rx_msg copies */
    uint8_t request_pdo_offset;
    uint8_t rev;            /* negotiated PD revision (data port: >= 2 is PD 3.x) */
    uint8_t rx_msg_id;
    uint8_t pending_key;
    uint8_t tx_msg_id;
    uint8_t *tx_direct;     /* template for the direct-send paths */
    uint8_t *deferred_msg;  /* data port only */
    uint8_t deferred_key;   /* data port only */
    uint8_t deferred;       /* data port only */
};
VAR __xdata struct pd_port pd0_port; /* pd_state_reset clears it */
VAR __xdata struct pd_port pd1_port; /* pd_init_clear clears it */

/* pd_state_reset */
VAR __xdata uint8_t pd1_request_msg[6]; /* power-port Request message (header + RDO) */
VAR __xdata uint8_t src_caps_msg[34]; /* data-port Source_Capabilities message */
VAR __xdata uint8_t src_caps_max_watts;

VAR __xdata uint32_t pd1_adapter_stamp;
VAR __xdata uint8_t pd1_adapter_step;
VAR __xdata uint8_t pd1_reset_signal_count;
VAR __xdata uint32_t fr_swap_stamp;
VAR __xdata uint8_t fr_swap_step;
VAR __xdata uint8_t fr_swap_first_pass;
/* pd_init_clear: the power port's policy state */
VAR __xdata uint8_t pd1_power_role;
VAR __xdata uint8_t pd1_data_role;
VAR __xdata uint8_t pd1_vbus_drop_armed;
VAR __xdata uint8_t pd1_adapter_state;
VAR __xdata uint8_t pd1_src_caps_valid;
VAR __xdata uint8_t pd1_tx_first_try;
VAR __xdata uint8_t pd1_goodcrc_wait;
VAR __xdata uint8_t pd1_retry_count;
VAR __xdata uint8_t pd1_ams_state;
VAR __xdata uint8_t pd1_retry_limit;

/* Snapshots for the host (host_if.c). */
VAR __xdata uint8_t host_mipi_port_num;
VAR __xdata uint8_t host_mipi_lane_num;
VAR __xdata uint8_t host_mipi_fmt;
VAR __xdata uint32_t pclk_khz; /* pixel clock (kHz) published to the host */
VAR __xdata uint32_t host_lmtx_read_clk;
VAR __xdata uint8_t host_audio_channels;
VAR __xdata uint8_t host_audio_word_length;
VAR __xdata uint16_t host_audio_rate; /* width uncertain */
VAR __xdata uint8_t host_color_format;
VAR __xdata uint8_t host_color_depth;
VAR __xdata uint8_t host_color_range;

VAR __xdata uint8_t in_clk_settle_count; /* input-clock settle counter (in_clk_settled) */
extern __xdata uint8_t mipi_fmt_table[75]; /* MIPI pixel formats, 5 bytes each (vars.c) */
/* tx_state_reset */
VAR __xdata uint8_t mipi_cphy;
VAR __xdata uint8_t mipi_out_csi; /* 1 CSI-2, 0 DSI */
VAR __xdata uint8_t mipi_port_num;
VAR __xdata uint8_t mipi_lane_num;
VAR __xdata uint8_t dphy_clk_burst;
VAR __xdata uint8_t dsi_mode;
VAR __xdata uint8_t mipi_fmt;
VAR __xdata uint8_t mipi_port_mode;
VAR __xdata uint8_t mipi_bpp; /* bits per pixel */
VAR __xdata uint8_t tx_state; /* TxState, mirrored to HOST_TX_STATE */
VAR __xdata uint8_t tx_restart_req; /* TX restart request (consumed in TxState 2) */
VAR __xdata uint8_t tx_video_detected; /* RX saw video (TxState 2 -> 3) */
VAR __xdata uint8_t tx_rx_stable; /* RX is stable (TxState 3 -> 4) */
VAR __xdata uint8_t mipi_csi_c8a0_bit6;
VAR __xdata uint8_t mipi_pll_src;
VAR __xdata uint8_t tx_dsc_detected;
VAR __xdata uint32_t mipi_in_clk; /* input pixel clock */
VAR __xdata uint32_t mipi_phy_clk;
VAR __xdata uint32_t lmtx_read_clk; /* measured MIPI TX read clock (kHz) */
VAR __xdata uint8_t mipi_host_gate_en; /* 1: honour HOST_MIPI_GATE (default 0) */
VAR __xdata uint8_t mipi_out_enabled; /* output enabled (TxState 6 -> 7 needs it) */
/* vidchk_clear */
VAR __xdata uint16_t vidchk_htotal;
VAR __xdata uint16_t vidchk_hactive;
VAR __xdata uint16_t vidchk_hfp;
VAR __xdata uint16_t vidchk_hsw;
VAR __xdata uint16_t vidchk_hbp;
VAR __xdata uint16_t vidchk_vactive;
VAR __xdata uint16_t vidchk_vfp;
VAR __xdata uint16_t vidchk_vsw;
VAR __xdata uint16_t vidchk_vbp;
VAR __xdata uint16_t vidchk_hspol;
VAR __xdata uint16_t vidchk_vspol;

/* PD flags and message templates (vars.c). Templates start with a 2-byte
 * message header; data objects are little-endian. */
extern __xdata uint8_t pd_init_req; /* 1: pd_main_task re-runs pd_init */
extern __xdata uint8_t src_caps_ext_msg[30]; /* used through pointers */
extern __xdata uint8_t pd_modes_ack_a[10]; /* Discover Modes ACK template, DP mode VDO 0x00140045 */
extern __xdata uint8_t pd_id_ack_vdm2[26]; /* Discover Identity ACK template, VDM 2.x, data port */
extern __xdata uint8_t pd_modes_ack_b[10]; /* Discover Modes ACK template, mode VDO 0x00000405 */
extern __xdata uint8_t pd1_id_ack[22]; /* Discover Identity ACK template, power port */
extern __xdata uint8_t pd_id_ack_vdm1[22]; /* Discover Identity ACK template, VDM 1.0, data port */

/* dsc_info_clear */
VAR __xdata uint8_t dsc_color_format;
VAR __xdata uint8_t dsc_bpc_half;
VAR __xdata uint8_t dsc_number;
VAR __xdata uint8_t dsc_slice_count;
VAR __xdata uint8_t dsc_ratio;
VAR __xdata uint8_t dsc_bpp_h;
VAR __xdata uint8_t dsc_bpp_l;
VAR __xdata uint8_t dsc_slice_height_h;
VAR __xdata uint8_t dsc_slice_height_l;
VAR __xdata uint8_t dsc_slice_width_h;
VAR __xdata uint8_t dsc_slice_width_l;
VAR __xdata uint8_t dsc_chunk_size_h;
VAR __xdata uint8_t dsc_chunk_size_l;
VAR __xdata struct rx_timing dsc_bypass_timing; /* MSA timing msa_timing_read parses; nothing reads it */

VAR __xdata uint8_t pcr_lock_count;
VAR __xdata uint32_t rx_stamp; /* RX state machine timer stamp */
VAR __xdata uint8_t rx_ecc_poll_count;
/* rx_state_reset */
VAR __xdata uint8_t rx_ref_clock_sel;      /* rx_link_setup arguments */
VAR __xdata uint8_t rx_link_rate_cfg_on;
VAR __xdata uint8_t rx_phy_e2_71_bit2;
VAR __xdata uint8_t rx_max_link_rate_h;
VAR __xdata uint8_t rx_max_link_rate_l;
VAR __xdata uint8_t rx_max_lane_count;
VAR __xdata uint8_t rx_lane_count;
VAR __xdata uint8_t typec_mode; /* 1: USB-C input, HPD goes through PD (hpd_request) */
VAR __xdata uint8_t rx_c3_ed_pattern_sel;
VAR __xdata uint8_t rx_state; /* RxState, mirrored to HOST_RX_STATE */
VAR __xdata uint8_t rx_downspread;
VAR __xdata uint8_t rx_tps3;
VAR __xdata uint8_t rx_tps4;
VAR __xdata uint8_t rx_plug_gate; /* RX leaves state 2 only when 1 */
VAR __xdata uint8_t rx_restart_req; /* RX retrain/restart request (consumed in RxState 4) */
VAR __xdata uint8_t rx_setup_gate;
VAR __xdata uint8_t rx_color_snap_from_vsc;
VAR __xdata uint8_t rx_color_snap_format;
VAR __xdata uint8_t rx_color_snap_colorimetry;
VAR __xdata uint8_t rx_color_snap_depth;
VAR __xdata uint8_t rx_color_snap_range;
VAR __xdata uint8_t msa_snap_htotal_h;
VAR __xdata uint8_t msa_snap_htotal_l;
VAR __xdata uint8_t msa_snap_hact_h;
VAR __xdata uint8_t msa_snap_hact_l;
VAR __xdata uint8_t msa_snap_vtotal_h;
VAR __xdata uint8_t msa_snap_vtotal_l;
VAR __xdata uint8_t msa_snap_vact_h;
VAR __xdata uint8_t msa_snap_vact_l;
VAR __xdata uint8_t rx_eq_fail1;

VAR __xdata uint8_t src_caps_offset;
VAR __xdata uint8_t src_caps_end;
VAR __xdata uint8_t src_caps_shift_i;
VAR __xdata uint16_t pdo_current;
VAR __xdata uint16_t pdo_voltage;
VAR __xdata uint32_t pdo_watts;
VAR __xdata uint8_t pdo_15v_class;

/* audio_state_reset */
VAR __xdata uint8_t audio_rx_stable;
VAR __xdata uint8_t audio_in_sel;
VAR __xdata uint8_t audio_out_sel;
VAR __xdata uint8_t audio_stage; /* audio stage counter (>= 4: audio_gate acts) */
VAR __xdata uint8_t audio_info_valid;
VAR __xdata uint8_t audio_channels;
VAR __xdata uint8_t audio_word_length;
VAR __xdata uint8_t audio_rate_h;
VAR __xdata uint8_t audio_rate_l;
VAR __xdata uint8_t audio_host_gate_en; /* 1: honour HOST_AUDIO_GATE (default 0) */
VAR __xdata uint8_t audio_gate_latch;

VAR __xdata uint8_t hdcp_state; /* 0 none, 0x10 HDCP 1.x, 0x20 HDCP 2.x */
/* hdcp_init */
VAR __xdata uint8_t hdcp_mode;
VAR __xdata uint8_t hdcp_versions;

VAR __xdata uint8_t pd0_tx_last_byte;
VAR __xdata uint8_t pd0_tx_byte_count;
VAR __xdata uint8_t pd0_tx_len;
VAR __xdata uint8_t pd0_tx_sent_msg_id;
VAR __xdata uint8_t pd0_rx_overflow;
VAR __xdata uint8_t pd0_rx_status;
VAR __xdata uint8_t pd0_rx_status2;
VAR __xdata uint8_t pd0_rx_ordered_set;
VAR __xdata uint8_t pd0_rx_index;
VAR __xdata uint8_t pd1_tx_last_byte;
VAR __xdata uint8_t pd1_tx_byte_count;
VAR __xdata uint8_t pd1_tx_len;
VAR __xdata uint8_t pd1_tx_sent_msg_id;
VAR __xdata uint8_t pd1_rx_overflow;
VAR __xdata uint8_t pd1_rx_status;
VAR __xdata uint8_t pd1_rx_status2;
VAR __xdata uint8_t pd1_rx_ordered_set;
VAR __xdata uint8_t pd1_rx_index;

VAR __xdata uint32_t irq_pulse_stamp; /* start of the IRQ pulse */
VAR __xdata uint8_t sys_event_queued; /* SystemEvent waiting for the current IRQ pulse to end */
VAR __xdata uint8_t sys_event; /* current SystemEvent */
VAR __xdata uint8_t irq_pulse_active; /* IRQ pin held low */
VAR __xdata uint8_t sys_event_pending;

/* csc_defaults */
VAR __xdata uint8_t csc_enable;
VAR __xdata uint8_t rx_color_range;
VAR __xdata uint8_t csc_range_cfg;
VAR __xdata uint8_t rx_color_format;
VAR __xdata uint8_t csc_out_format;
VAR __xdata uint8_t csc_range_mode;
VAR __xdata uint8_t csc_conv_sel;

VAR __xdata uint32_t now; /* last latched FREE_COUNTER value */
VAR __xdata uint8_t rx_eq_lane_value[4];
extern __xdata uint8_t dp_partner_connected;
VAR __xdata uint8_t pr_swap_reject_count;
VAR __xdata uint8_t dpcd_ext_caps;
VAR __xdata uint8_t cc_poll_count;


#endif
