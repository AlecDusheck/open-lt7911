/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026, Alec Dusheck */

/* Every function of the firmware, grouped by the file that defines it. */
#ifndef FW_H
#define FW_H

#include <stdbool.h>
#include <stdint.h>

#include "regs.h"
#include "sfr.h"
#include "vars.h"

void main(void);

/* main.c */
void video_poll(void);
void audio_poll(void);
void rx_poll(void);
void tx_poll(void);

/* init.c */
void osc_init(uint8_t);
void pin_e3_1a_1b_mode(uint8_t);
void clock_setup(uint8_t);
void sys_init(void);
void dpcd_ext_caps_detect(void);
void sys_c4_2e_e4_04_mode(uint8_t);
void tx_state_reset(void);
void sys_e4_04_bit1_mode(uint8_t);
void sys_e4_04_bit0_mode(uint8_t);
void audio_state_reset(void);
void rx_state_reset(void);
void irq_init(void);
void sys_c2_a8_bit0_clear(uint8_t);
void t2_init(void);
void state_reset(void);
void event_reset(void);
void pin_05_func_0c_set(void);
void int0_enable(void);
void ext2_enable(void);
uint8_t sys_e1_02_low_read(void);
void int1_enable(void);
void dpcd_ext_caps_init(void);

/* isr.c */
void int0_isr(void) __interrupt(0);
void int1_isr(void) __interrupt(2);
void timer2_isr(void) __interrupt(5);
void aux_t2_work(void);
void t2_arm_short(void);
void ext2_isr(void) __interrupt(8);

/* util.c */
void pin_config(uint8_t, uint8_t, uint8_t);
void gpio_write(uint8_t, uint8_t) __reentrant;
uint8_t timer_elapsed(__xdata uint32_t *, uint32_t);
void counter_latch(void);
void delay_ms(uint16_t) __reentrant; /* not __naked here: callers must save registers */
void timer_stamp(__xdata uint32_t *);
uint8_t irq_pin_level(void);
void delay_short(uint16_t) __reentrant; /* not __naked here: callers must save registers */
void now_reset(void);
void counter_init(void);
void copy_bytes(__xdata uint8_t *, const uint8_t *, uint8_t) __reentrant;
uint32_t udiv32(uint32_t, uint32_t) __reentrant; /* a / b, defined for b = 0 too */
uint16_t udiv16(uint16_t, uint16_t) __reentrant; /* a / b, defined for b = 0 too */

/* SystemEvent codes (sys_event_post). host_event_poll publishes these four
 * to the host; any other code is consumed without an IRQ. */
enum sys_event {
    SYS_EVENT_VIDEO_READY = 5,
    SYS_EVENT_VIDEO_OFF = 6,
    SYS_EVENT_AUDIO_READY = 8,
    SYS_EVENT_AUDIO_OFF = 9,
};

/* Info blocks the getters fill through a pointer. */
struct mipi_tx_info { /* mipi_tx_info_get */
    uint8_t port_num;  /* mipi_port_num */
    uint8_t lane_num;  /* mipi_lane_num */
    uint8_t fmt;       /* mipi_fmt, the code csc_select switches on */
    uint32_t in_clk;   /* mipi_in_clk */
    uint32_t read_clk; /* lmtx_read_clk */
};

struct audio_info { /* audio_info_get */
    uint8_t channels;    /* audio_channels */
    uint8_t word_length; /* audio_word_length */
    uint16_t rate;       /* audio_rate_h:audio_rate_l, the sample rate in kHz */
};

struct video_info { /* video_info_get */
    uint8_t color_format; /* rx_color_snap_format */
    uint8_t color_depth;  /* rx_color_snap_depth */
    uint8_t color_range;  /* rx_color_snap_range */
};

struct audio_config { /* audio_config_get */
    uint8_t in_type;      /* audio_in_type() argument (audio_in_sel) */
    uint8_t out_type;     /* audio_out_type() argument (audio_out_sel) */
    uint8_t host_gate_en; /* audio_host_gate_en */
};

struct input_config { /* input_config_get */
    uint8_t typec;          /* input_typec */
    uint8_t receptacle;     /* typec_receptacle */
    uint8_t dp_lane_count;  /* typec_dp_lane_count */
    uint8_t pin_assignment; /* dp_pin_assignment */
};

/* host_if.c */
void host_publish_info(uint8_t);
uint16_t rx_timing_read(uint8_t);
void irq_pulse(void);
void host_version_init(void);
void mipi_info_snapshot(void);
void host_event_poll(void);
void mipi_tx_info_get(__xdata struct mipi_tx_info *);
void audio_info_snapshot(void);
void video_info_snapshot(void);
void sys_event_post(uint8_t);
void audio_info_get(__xdata struct audio_info *);
void video_info_get(__xdata struct video_info *);
uint8_t host_audio_gate(uint8_t, uint8_t);
uint8_t host_mipi_gate(uint8_t, uint8_t);
uint8_t sys_event_is_pending(void);
void sys_event_clear(void);
uint8_t sys_event_get(void);
void host_marker_publish(void);

/* The host marker, E0:A8: two AUX requests macOS sends a display and other
 * hosts do not. */
extern volatile __xdata uint8_t host_marker;
#define HOST_MARK_DISPLAYID  0x01 /* I2C-over-AUX request to 7-bit 0x52 (DisplayID) */
#define HOST_MARK_DPCD_0303C 0x02 /* native read of DPCD 0x0303C */
#define HOST_MARK_ALIVE      0x80

/* flash.c */
void flash_read(uint32_t, __xdata uint8_t *, uint16_t);

/* edid.c */
void edid_load(void);
void edid_ram_upload(const __xdata uint8_t *);

/* RxState (rx_state, mirrored to HOST_RX_STATE = E0:AE): the values are a
 * host contract. dprx.c describes each state. */
enum rx_state_id {
    RX_STATE_INIT = 0x01,
    RX_STATE_WAIT_PLUG = 0x02,
    RX_STATE_3 = 0x03,
    RX_STATE_SETUP = 0x04,
    RX_STATE_5 = 0x05,
    RX_STATE_EQ = 0x06,
    RX_STATE_DESKEW = 0x07,
    RX_STATE_8 = 0x08,
    RX_STATE_ECC = 0x09,
    RX_STATE_MSA = 0x0A,
    RX_STATE_DSC = 0x0B,
    RX_STATE_PCR = 0x0C,
    RX_STATE_STABLE = 0x0D,
};

/* dprx.c */
void rx_pixel_clock(void) __reentrant;
void rx_state_machine(void);
void rx_eq_band(uint8_t);
void rx_phy_lane_config(uint8_t, uint8_t);
void rx_events(void);
void rx_training_info(void);
void rx_vco_setup(uint8_t, uint16_t);
void irq_enable(uint8_t, uint8_t);
void irq_ack(uint8_t);
void rx_config_load(void);
void rx_lane_routing_setup(void);
void rx_pclk_pll_band(void);
void rx_pcr_init(void);
void rx_dsc_mode_config(uint8_t);
void aux_controller_mode(uint8_t);
void set_rx_state(uint8_t);
void rx_c3_setup_ed_pattern(uint8_t);
void rx_rterm(void);
void rx_lane_map_set(uint8_t, uint8_t);
void rx_plug(void);
void rx_c3_regs_init(uint8_t);
void rx_phy_setup(uint8_t, uint16_t, uint8_t, uint8_t);
void rx_dsc_path_config(void);
void rx_rate_count(void);
void rx_phy_e8_defaults(void);
uint8_t rx_lanes_ready(void);
uint8_t rx_lanes_eq_finished(void);
uint8_t rx_lanes_eq_passed(void);
void dpcd_addr_latch_a(uint16_t);
void rx_ref_clock_select(uint8_t);
void rx_pixel_clock_reset(void);
void dpcd_addr_latch_b(uint16_t);
void rx_soft_reset_pulse(uint8_t);
uint8_t rx_fec_check(void);
void rx_config_default(void);
uint8_t rx_deskew_ok(void);
uint8_t rx_config_get(__xdata uint8_t *);
uint8_t rx_pcr_stable(void);
void rx_phy_lane_select(uint8_t);
void rx_e6_5f_onehot_set(uint8_t);
void rx_phy_e2_6c_bit2_set(uint8_t);
void rx_c3_be_setup(void);
uint8_t rx_dsc_detect(void);
void test_sink_crc_start(void);
void rx_init_regs_clear(void);
void rx_phy_e6_e2_setup(void);
void rx_eq_enable(uint8_t) __reentrant;
void rx_link_setup(uint8_t, uint16_t, uint8_t, uint8_t);
void rx_reset_assert(uint8_t) __reentrant;
void rx_fec_enable(uint8_t) __reentrant;
void dsc_info_clear(void);
void rx_irqs_enable(void);
void aux_c2_regs_setup(void);
void aux_controller_init(void);
void rx_eq_init(void);
void rx_phy_e2_6c_bit6_set(uint8_t);
void rx_phy_e2_71_bit2_set(uint8_t);
uint8_t rx_ecc_ok(void);
void rx_ecc_recover_pulse(void);
void rx_eq_restart_pulse(void);
uint8_t rx_typec_routing_class(void);
void rx_c1_81_from_c0_f6(void);
void rx_phy_e3_e6_setup(void);
void rx_e3_1f_bit0_set(uint8_t);
void rx_eq_retry_set(uint8_t) __reentrant;
void iram_vars_clear(void);
uint8_t rx_pcr_locked(void);
uint8_t rx_eq_retry_get(void);
uint8_t rx_lane_count_read(void);
void rx_link_reset_pulse(void);
void rx_phy_analog_init(void);
void aux_c2_0c_bit7_clear(void);
void rx_e6_38_low_bits_set(void);
void hpd_irq_event_enable(void);
void rx_pclk_e297_bit7_clear(void);
uint8_t dp_pin_assignment_get(void);

/* dprx_aux.c */
void dpcd_init(uint8_t);
void aux_request(uint8_t);
void rx_phy_e8_rate_regs_set(void);
void test_sink_update(void);
void rx_phy_link_rate_set(uint8_t);
void rx_e1_18_1b_cb_23_set(uint8_t);
void rx_eq_band_apply(uint8_t);
void dpcd_write_max_lane_count(uint8_t);
void dpcd_write(uint8_t, uint8_t);
void dpcd_page(uint8_t) __reentrant;
uint8_t dpcd_ext_caps_on(void);
void dpcd_init_default(void);

/* dprx_video.c */
struct rx_color;
void dsc_pps(void);
void msa_timing_read(__xdata struct rx_timing *) __reentrant;
void dsc_bypass_info(void);
void rx_color_read(__xdata struct rx_color *);
uint8_t irq_pending(uint8_t);
uint8_t rx_timing_changed(void);
void csc_select(void);
void rx_video_info(void);
uint8_t rx_color_changed(void);
void csc_program(uint8_t, uint8_t, uint8_t, uint8_t);
void tx_video_event(uint8_t);
void rx_video_event(uint8_t);
void csc_range_mode_select(void);
void csc_config_load(void);
void audio_video_event(uint8_t);
void csc_setup(void);
uint8_t csc_conv_field_sel0(uint8_t);
uint8_t csc_conv_field(uint8_t, uint8_t);
void csc_disable(void);
uint8_t csc_conv_field_sel2(uint8_t);
void csc_defaults(void);
uint8_t csc_config_get(__xdata uint8_t *);
uint8_t csc_conv_field_sel4(uint8_t);
void csc_ycbcr_to_rgb_enable(uint8_t, uint8_t);
void csc_rgb_to_ycbcr_enable(uint8_t, uint8_t);
void csc_422_to_444_enable(void);
void csc_444_to_422_enable(void);

/* dprx_hdcp.c */
uint8_t hdcp_config_get(__xdata uint8_t *);
void hdcp_repeater(uint8_t);
void hdcp_decoder(uint8_t);
void hdcp_state_poll(void);
void hdcp_init(void);
void hdcp_config_load(void);
uint8_t hdcp_active_version(void);
void hdcp_decoder_disable(uint8_t);
void hdcp_repeater_disable(uint8_t);
uint8_t rx_state_get(void);

/* dprx_hpd.c */
void hpd_pin(uint8_t, uint8_t);
void hpd_low(void);
uint8_t hpd_gate(void);
uint8_t rx_plug_detect(void);
void hpd_irq(void);
void hpd_high(void);
uint8_t rx_plug_sense(void);

/* mipi_tx.c */
void dsi_panel_init(void);
void rprd_cal(uint8_t, uint8_t);
void tx_state_machine(void);
void mipi_tx_config_apply(void);
void mipi_port0_analog_setup(uint8_t, uint8_t);
void mipi_port1_analog_setup(uint8_t, uint8_t);
void mipi_config_finalize(void);
void tx_output_gate(void);
void dsi_send_packet(uint8_t, uint8_t, const __code uint8_t *);
uint8_t mipi_config_get(__xdata uint8_t *);
void mipi_phy_analog_setup(void);
void mipi_fmt_set(uint8_t);
void mipi_output_enable(uint8_t, uint8_t);
void mipi_config_defaults(void);
void set_tx_state(uint8_t);
void dphy_ctl_ab_ae_pulse(void);
void dphy_clk_mode(uint8_t);
void dphy_clk_enable(uint8_t, uint8_t);
void mipi_phy_analog_enable(uint8_t);
void mipi_ports_lanes_setup(void);
void mipi_phy_type_setup(void);
void mipi_out_type_setup(void);
void mipi_lane_map_set(void);
void tx_event_notify(uint8_t);
void mipi_port_count_set(uint8_t);
void tx_c8_d5_bits45_enable(uint8_t);
void mipi_data_type_set(uint8_t, uint8_t);
void dphy_ports_ab_bd_bits54_set(uint8_t, uint8_t);
void cphy_ports_ab_bd_bits76_set(uint8_t, uint8_t);
void dphy_e2_c7c8_e2e3_set(void);
void dsi_packet_setup(void);
void dsi_send_packet_wait(uint8_t, uint8_t, const __code uint8_t *);
void mipi_word_count_set(uint16_t, uint16_t);
void rx_setup_gate_set(uint8_t);
void cphy_ports_analog_setup(uint8_t, uint8_t);
void dphy_ports_analog_setup(uint8_t, uint8_t);
void tx_c8_0f_bit5_set(uint8_t);
void mipi_cphy_select(uint8_t);
void tx_c8_a0_bit6_set(uint8_t);
void tx_c8_70_bit3_set(uint8_t);
void mipi_phy_analog_type_set(uint8_t);
void tx_c8_7a_bit7_set(uint8_t);
void tx_c8_7a_bit6_set(uint8_t);
void tx_c8_05_bit0_clear(uint8_t);
void tx_c8_71_enable(uint8_t);
void tx_c8_84_bit3_set(uint8_t);
void dsi_packet_nowait_set(uint8_t);
void mipi_port_mode_set(uint8_t);
void mipi_lane_count_set(uint8_t);
void dsi_mode_set(uint8_t);
void tx_set_c8_0f_low5(uint8_t);
void dsi_packet_finish(void);
void tx_c8_a0_bit7_on(uint8_t);
void dsi_panel_init_if_dsi(void);
void dphy_ctl_ab_ae_pulse_if_dphy(void);
void tx_c8_aa_write_aa(void);
void tx_set_c8_0d(uint8_t);
void tx_set_c8_0e(uint8_t);

/* mipi_clk.c */
/* Clocks the MIPI TX PLL setup works from (mipi_pll_configure, mipi_pll). */
struct mipi_pll_clocks {
    uint32_t in_clk;    /* input pixel clock */
    uint32_t data_rate; /* PHY clock to reach */
    uint32_t ref_clk;   /* reference clock */
};
void tx_set_hactive(uint16_t);
void mipi_rdly_update(void);
void tx_timing_check(void);
void vidchk_clear(void);
void mipi_pll(uint8_t, const __xdata struct mipi_pll_clocks *);
uint16_t tx_freerun_hact(void) __reentrant;
void mipi_phy_clk_calc(void);
float ceil_f(float);
uint32_t clk_meter_read(uint8_t);
void mipi_tx_pll_setup(void);
uint8_t lmtx_pll_band(void);
uint8_t in_clk_settled(void);
void mipi_phy_timing_setup(void);
uint8_t lmtx_read_clk_measure(void);
void tx_c8_d4_dd_fixed_set(void);
void tx_set_c8_d8_reg(uint16_t);
void tx_set_c8_d8(uint16_t);
void tx_set_c8_da(uint16_t);
void tx_set_hss_reg(uint16_t);
void mipi_pll_configure(uint8_t, const __xdata struct mipi_pll_clocks *);
uint8_t mipi_fmt_bpp(uint8_t);
uint8_t clk_meter_done(uint8_t);
void tx_set_vfp(uint16_t);
void tx_set_hactive_quarter(uint16_t);
void tx_set_freerun_hfp(uint16_t);
void tx_set_c8_d4(uint16_t);
uint16_t tx_get_c8_c1(void);
void clk_meter_select(uint8_t, uint8_t);
void mipi_pll_src_set(uint8_t);
void tx_set_hspol(uint8_t);
void tx_set_vspol(uint8_t);
void tx_freerun_enable(uint8_t);
uint16_t tx_get_hss_reg(void);
uint16_t tx_get_hactive_quarter(void);
void tx_set_vactive(uint16_t);
void tx_set_vsw_vbp(uint16_t);
void tx_set_vsw(uint16_t);
void mipi_rdly_write(uint16_t);
int16_t abs16(int16_t);
uint8_t in_clk_meter_src_bit(void);
void tx_set_c8_d6(uint8_t);
void tx_set_c8_d7(uint8_t);
void tx_set_c8_dc(uint8_t);
void tx_set_c8_a6(uint8_t);
void tx_set_c8_a9(uint8_t);
void tx_set_c8_a9_cphy(uint8_t);
void tx_set_hss(uint16_t);

/* audio.c */
void audio_in_type(uint8_t);
void pin_field_or(uint8_t, uint8_t);
void audio_out_type(uint8_t);
void audio_task(void);
uint16_t audio_rate_read(void);
void audio_on_off(uint8_t, uint8_t);
void audio_gate(void);
void audio_stage_step(void);
uint8_t audio_channel_count_read(void);
void audio_config_load(void);
uint8_t audio_fs(void);
uint8_t audio_word_length_read(void);
void set_audio_stage(uint8_t);
uint8_t audio_config_get(__xdata struct audio_config *);
void audio_path_setup_finish(void);
void audio_event_post(uint8_t);
void audio_ca_2f_bit2_set(uint8_t);
void audio_config_default(void);
uint8_t audio_ca_8d_bit1_get(void);
uint8_t audio_c0_f1_bit4_get(void);
uint8_t tx_state_get(void);

/* typec_input.c */
void input_config_load(void);
void input_select(void);
uint8_t input_config_get(__xdata struct input_config *);

/* pd_phy.c */
void pd0_rx(void);
void pd1_rx(void);
void pd0_tx(void) __reentrant;
void pd1_tx(void) __reentrant;
void pd_tx_retry_arm(uint8_t) __reentrant;
void pd0_tx_raw(void);
void pd1_phy_regs_init(void);
void pd0_discover_identity(void);
void pd1_phy_c7d0_pulse_init(void);
void pd0_phy_regs_init(void);
void pd0_phy_e2_init(void);
void pd1_phy_e2_init(void);
void pd0_phy_regs_2d_init(void);
void pd1_phy_regs_ad_init(void);
void pd0_phy_regs_0a_init(void);
void pd1_phy_regs_8a_init(void);
void pd1_phy_init(void);
void pd0_phy_regs_22_init(void);
void pd1_phy_regs_a2_init(void);
void pd0_phy_init(void);
void pd0_cable_rx_stop(void);
void pd1_vbus_drop_event_set(uint8_t);
void pd1_vbus_drop_enable(uint8_t);
uint8_t pd_next_msg_id(uint8_t) __reentrant;
void pd0_c754_bit2_pulse(void);
void t2_arm_c000(void);
void pd0_phy_command(uint8_t);
void pd1_phy_command(uint8_t);
void pd0_accept_acked(void);
void t2_load_f1a6(void);
void hpd_request_post(uint8_t) __reentrant;

/* pd_cc.c */
void pd_state_reset(void);
void attach_timer_task(void);
void typec_attach(uint8_t);
void pd0_cc_line_select(void);
void pd0_error_recovery(void);
void pd1_vbus_drop_calibrate(void);
void pd0_cc_line_track(void);
uint8_t pd0_detached(void);
uint8_t vbus_check(uint8_t);
void attach_pull_apply(uint8_t);
uint8_t pd0_cc_wait_dfp(void);
uint8_t pd0_cc_wait_ufp(void);
void pd0_source_detach(void);
void typec_orient_e2_02_set(uint8_t) __reentrant;
uint8_t pd0_vconn_cc_line(void);
uint8_t pd0_tx_allowed(void);
void pd1_cc_line_track(void);
void pd0_cc_pull_set(uint8_t) __reentrant;
void pd1_cc_pull_set(uint8_t);
uint8_t pd0_cc_open(void);
void pd0_cc_enable(uint8_t);
void pd0_rp_level_set(uint8_t) __reentrant;
void pd1_rp_level_set(uint8_t);
uint8_t pd0_c72f_bit5_get(void);

/* USB PD message types (USB PD 3.x tables 6-5, 6-6, 6-53). The PD message
 * handlers switch on exactly this numbering. */
enum {
    PD_CTRL_GOODCRC = 0x01,
    PD_CTRL_GOTOMIN = 0x02,
    PD_CTRL_ACCEPT = 0x03,
    PD_CTRL_REJECT = 0x04,
    PD_CTRL_PING = 0x05,
    PD_CTRL_PS_RDY = 0x06,
    PD_CTRL_GET_SOURCE_CAP = 0x07,
    PD_CTRL_GET_SINK_CAP = 0x08,
    PD_CTRL_DR_SWAP = 0x09,
    PD_CTRL_PR_SWAP = 0x0A,
    PD_CTRL_VCONN_SWAP = 0x0B,
    PD_CTRL_WAIT = 0x0C,
    PD_CTRL_SOFT_RESET = 0x0D,
    PD_CTRL_NOT_SUPPORTED = 0x10,
    PD_CTRL_GET_SOURCE_CAP_EXT = 0x11,
    PD_CTRL_GET_STATUS = 0x12,
    PD_CTRL_FR_SWAP = 0x13,
    PD_CTRL_GET_PPS_STATUS = 0x14,
    PD_CTRL_GET_COUNTRY_CODES = 0x15,
};
enum {
    PD_DATA_SOURCE_CAP = 0x01,
    PD_DATA_REQUEST = 0x02,
    PD_DATA_BIST = 0x03,
    PD_DATA_SINK_CAP = 0x04,
    PD_DATA_ALERT = 0x06,
    PD_DATA_GET_COUNTRY_INFO = 0x07,
    PD_DATA_ENTER_USB = 0x08,
    PD_DATA_EPR_REQUEST = 0x09,
    PD_DATA_EPR_MODE = 0x0A,
    PD_DATA_VENDOR_DEFINED = 0x0F,
};
enum {
    PD_EXT_GET_BATTERY_CAP = 0x03,
    PD_EXT_GET_BATTERY_STATUS = 0x04,
    PD_EXT_GET_MANUFACTURER_INFO = 0x06,
    PD_EXT_SECURITY_REQUEST = 0x08,
    PD_EXT_FW_UPDATE_REQUEST = 0x0A,
};

/* The key pd0_port.pending_key holds for a sent message: kind 0 (control), 1
 * (data) or 3 (VDM reply) and the message type or VDM command. */
#define PD_KEY(kind, type) ((uint8_t)(((kind) << 5) | (type)))

/* dp_alt_status bits. */
#define DP_ALT_IDENTITY   0x01 /* Discover Identity answered */
#define DP_ALT_SVIDS      0x02 /* Discover SVIDs answered */
#define DP_ALT_MODES      0x04 /* Discover Modes answered */
#define DP_ALT_ENTERED    0x08 /* Enter Mode answered */
#define DP_ALT_STATUS     0x10 /* DP Status Update answered */
#define DP_ALT_CONFIGURED 0x20 /* DP Configure answered */
#define DP_ALT_HPD_HIGH   0x40 /* HPD high as last reported to the host */

/* PD message templates in code space: the 2-byte header (little-endian;
 * byte 0 type | data role bit 5 | spec revision bits 7:6, byte 1 object
 * count bits 6:4 | message ID bits 3:1 | power role bit 0), then 4 bytes per
 * data object. pd0_tx/pd1_tx take the length from the object count and fill
 * in the revision, message ID and roles. PD_MSG stores one (or an XRAM
 * buffer) in a generic message pointer. */
#define PD_MSG(p) ((uint8_t *)(p))

/* pd_policy.c */
void pd_init_clear(void);
void pd_message_handler(void);
void src_caps_send_task(void);
void pd_build_request(__xdata struct pd_port *, uint8_t) __reentrant;
void pd0_sink_task(void);
void pd1_adapter_task(void);
void fr_swap_task(void);
uint8_t pr_swap(void);
void dr_swap(void);
void pd_save_rx_msg(__xdata struct pd_port *) __reentrant;
void pd_main_task(void);
void pd0_tx_timeout(void);
void src_caps_delayed_task(void);
void pd_pins_init(void);
void dr_swap_task(void);
void vconn_swap_task(void);
void pr_swap_timeout_task(void);
void pd0_tx_resend(void);
void pd0_ps_rdy_acked(void);
void pd0_cable_probe(void);
void pd1_vbus_drop_handler(void);
void switch_dock_restart_task(void);
void pd0_hard_reset_task(void);
void pd1_adapter_lost_reset(void);
void pd0_tx_deferred_send(void);
void pd0_soft_reset(void);
void pd1_detach_handler(void);
void dr_swap_want(void);
void cable_id_reset(void);
void pd0_fr_swap_signal(uint8_t);
void pd0_vconn_off(void);
void pd0_recovery_task(void);
void pd_init(void);
extern const __code uint8_t tpl_sink_cap_pd2[6];
extern const __code uint8_t tpl_ps_rdy[2];
extern const __code uint8_t tpl_accept[2];
extern const __code uint8_t tpl_reject[2];
extern const __code uint8_t tpl_not_supported_soft_reset[4];
#define TPL_NOT_SUPPORTED (&tpl_not_supported_soft_reset[0])
#define TPL_SOFT_RESET (&tpl_not_supported_soft_reset[2])

/* pd_vdm.c */
void vdm_dispatch(void);
void src_caps_from_pd1(void) __reentrant;
void pd1_request_build(void);
void dp_attention(uint8_t);
void hpd_attention_poll(void);
void discover_identity_vid(uint8_t, const __xdata uint8_t *, uint8_t);
void vdm_reply_header(uint8_t, uint8_t);
void src_caps_prepare(void);
void dp_config_pin_assignment(const __xdata uint8_t *);
uint16_t derate_or_50(uint16_t, uint16_t) __reentrant;
void vdm_rx(uint8_t);
extern const __code uint8_t svids_ack[10]; /* Discover SVIDs ACK, SVID 0xFF01 */

/* pd_power_port.c */
void pd1_message_handler(void);
void pd0_t2_ps_rdy(void);
void pd1_tx_timeout(void);
void pd1_vdm_reply(void);
void pd0_power_transition_start(void);

#endif
