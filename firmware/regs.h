/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026, Alec Dusheck */

/* LT7911EXC hardware registers.
 *
 * The 8051 reaches every register with movx in XDATA 0xC000..0xE8FF; XRAM is
 * 0x0000..0x1FFF. XDATA 0xPPRR is I2C page PP register RR (the page selector
 * is register 0xFF of I2C slave 0x41). While the MCU runs, the host can read
 * only E0:80..DF and write only E0:B0/B1; the HOST_* registers are therefore
 * the firmware's whole external contract, together with the IRQ pin (GPIO
 * 0x0B, active-low pulse) and the flash layout.
 *
 * Every register the firmware touches is declared here, so no file needs a
 * raw address. Registers whose role is proven have a name; the rest are
 * R_<page>_<reg> until their role is known. Writes have side effects: keep
 * the order, count and values of reads and writes exactly.
 */
#ifndef REGS_H
#define REGS_H

#include <stdint.h>

/* Set then clear bits in a register from one read, as the hardware's
 * write-1-then-0 acknowledge and strobe bits expect: read once, write
 * (v | m), write (v | m) & ~m. Re-reading between the two writes would not
 * be the same bus sequence. */
#define REG_PULSE(reg, m)                     \
    do {                                      \
        uint8_t pulse_ = (reg) | (m);         \
        (reg) = pulse_;                       \
        (reg) = pulse_ & (uint8_t)~(m);       \
    } while (0)

/* C0: DP RX stream and secondary data: VSC SDP buffer C0:A8..BF, MSA MISC0 C0:EB, VSC valid C0:EC, DSC PPS C0:05/32..35. */
__xdata __at(0xC005) volatile uint8_t R_C0_05;
__xdata __at(0xC010) volatile uint8_t R_C0_10;
__xdata __at(0xC015) volatile uint8_t R_C0_15;
__xdata __at(0xC016) volatile uint8_t R_C0_16;
__xdata __at(0xC017) volatile uint8_t R_C0_17;
__xdata __at(0xC018) volatile uint8_t R_C0_18;
__xdata __at(0xC019) volatile uint8_t R_C0_19;
__xdata __at(0xC02C) volatile uint8_t R_C0_2C;
__xdata __at(0xC032) volatile uint8_t R_C0_32;
__xdata __at(0xC033) volatile uint8_t R_C0_33;
__xdata __at(0xC034) volatile uint8_t R_C0_34;
__xdata __at(0xC035) volatile uint8_t R_C0_35;
__xdata __at(0xC0A9) volatile uint8_t R_C0_A9;
__xdata __at(0xC0AA) volatile uint8_t R_C0_AA;
__xdata __at(0xC0AB) volatile uint8_t R_C0_AB;
__xdata __at(0xC0BC) volatile uint8_t R_C0_BC;
__xdata __at(0xC0BD) volatile uint8_t R_C0_BD;
__xdata __at(0xC0DB) volatile uint8_t R_C0_DB;
__xdata __at(0xC0DC) volatile uint8_t R_C0_DC;
__xdata __at(0xC0DD) volatile uint8_t R_C0_DD;
__xdata __at(0xC0DE) volatile uint8_t R_C0_DE;
__xdata __at(0xC0DF) volatile uint8_t R_C0_DF;
__xdata __at(0xC0E0) volatile uint8_t R_C0_E0;
__xdata __at(0xC0E1) volatile uint8_t R_C0_E1;
__xdata __at(0xC0E2) volatile uint8_t R_C0_E2;
__xdata __at(0xC0E3) volatile uint8_t R_C0_E3;
__xdata __at(0xC0E4) volatile uint8_t R_C0_E4;
__xdata __at(0xC0E5) volatile uint8_t R_C0_E5;
__xdata __at(0xC0E6) volatile uint8_t R_C0_E6;
__xdata __at(0xC0E7) volatile uint8_t R_C0_E7;
__xdata __at(0xC0E8) volatile uint8_t R_C0_E8;
__xdata __at(0xC0E9) volatile uint8_t R_C0_E9;
__xdata __at(0xC0EA) volatile uint8_t R_C0_EA;
__xdata __at(0xC0EB) volatile uint8_t R_C0_EB;
__xdata __at(0xC0EC) volatile uint8_t R_C0_EC;
__xdata __at(0xC0ED) volatile uint8_t R_C0_ED;
__xdata __at(0xC0EE) volatile uint8_t R_C0_EE;
__xdata __at(0xC0EF) volatile uint8_t R_C0_EF;
__xdata __at(0xC0F1) volatile uint8_t R_C0_F1;
__xdata __at(0xC0F5) volatile uint8_t R_C0_F5;
__xdata __at(0xC0F6) volatile uint8_t R_C0_F6;
__xdata __at(0xC0F7) volatile uint8_t R_C0_F7;
__xdata __at(0xC0F8) volatile uint8_t R_C0_F8;
__xdata __at(0xC0F9) volatile uint8_t R_C0_F9;

/* C1: DP RX clock recovery and link: C1:FB..FD pixel-clock setup, C1:E5 bit 7 = pixel-clock (PCR) lock. */
__xdata __at(0xC105) volatile uint8_t R_C1_05;
__xdata __at(0xC13F) volatile uint8_t R_C1_3F;
__xdata __at(0xC141) volatile uint8_t R_C1_41;
__xdata __at(0xC14A) volatile uint8_t R_C1_4A;
__xdata __at(0xC162) volatile uint8_t R_C1_62;
__xdata __at(0xC181) volatile uint8_t R_C1_81;
__xdata __at(0xC186) volatile uint8_t R_C1_86;
__xdata __at(0xC18C) volatile uint8_t R_C1_8C;
__xdata __at(0xC18D) volatile uint8_t R_C1_8D;
__xdata __at(0xC19C) volatile uint8_t R_C1_9C;
__xdata __at(0xC19E) volatile uint8_t R_C1_9E;
__xdata __at(0xC1AD) volatile uint8_t R_C1_AD;
__xdata __at(0xC1B0) volatile uint8_t R_C1_B0;
__xdata __at(0xC1B1) volatile uint8_t R_C1_B1;
__xdata __at(0xC1B2) volatile uint8_t R_C1_B2;
__xdata __at(0xC1B3) volatile uint8_t R_C1_B3;
__xdata __at(0xC1BD) volatile uint8_t R_C1_BD;
__xdata __at(0xC1C7) volatile uint8_t R_C1_C7;
__xdata __at(0xC1C8) volatile uint8_t R_C1_C8;
__xdata __at(0xC1CA) volatile uint8_t R_C1_CA;
__xdata __at(0xC1CD) volatile uint8_t R_C1_CD;
__xdata __at(0xC1CE) volatile uint8_t R_C1_CE;
__xdata __at(0xC1CF) volatile uint8_t R_C1_CF;
__xdata __at(0xC1D0) volatile uint8_t R_C1_D0;
__xdata __at(0xC1D6) volatile uint8_t R_C1_D6;
__xdata __at(0xC1D7) volatile uint8_t R_C1_D7;
__xdata __at(0xC1E2) volatile uint8_t R_C1_E2;
__xdata __at(0xC1E5) volatile uint8_t R_C1_E5;
__xdata __at(0xC1F2) volatile uint8_t R_C1_F2;
__xdata __at(0xC1FA) volatile uint8_t R_C1_FA;
__xdata __at(0xC1FB) volatile uint8_t R_C1_FB;
__xdata __at(0xC1FC) volatile uint8_t R_C1_FC;
__xdata __at(0xC1FD) volatile uint8_t R_C1_FD;

/* C2: DP AUX controller and the EDID RAM. */
__xdata __at(0xC200) volatile uint8_t R_C2_00;
__xdata __at(0xC202) volatile uint8_t R_C2_02;
__xdata __at(0xC203) volatile uint8_t R_C2_03;
__xdata __at(0xC204) volatile uint8_t R_C2_04;
__xdata __at(0xC205) volatile uint8_t R_C2_05;
__xdata __at(0xC207) volatile uint8_t R_C2_07;
__xdata __at(0xC208) volatile uint8_t R_C2_08;
__xdata __at(0xC209) volatile uint8_t R_C2_09;
__xdata __at(0xC20C) volatile uint8_t R_C2_0C;
__xdata __at(0xC20E) volatile uint8_t EDID_RAM_DATA; /* AUX EDID RAM write port (C2:02 bit 4 and C2:05 = 2 select it) */
__xdata __at(0xC210) volatile uint8_t AUX_FIFO; /* AUX request FIFO; a read pops one byte */
__xdata __at(0xC211) volatile uint8_t R_C2_11;
__xdata __at(0xC212) volatile uint8_t R_C2_12;
__xdata __at(0xC213) volatile uint8_t R_C2_13;
__xdata __at(0xC214) volatile uint8_t R_C2_14;
__xdata __at(0xC220) volatile uint8_t AUX_FIFO_COUNT; /* bytes waiting in AUX_FIFO, bits 4:0 */
__xdata __at(0xC221) volatile uint8_t R_C2_21;
__xdata __at(0xC223) volatile uint8_t R_C2_23;
__xdata __at(0xC23C) volatile uint8_t R_C2_3C;
__xdata __at(0xC23F) volatile uint8_t R_C2_3F;
__xdata __at(0xC2A8) volatile uint8_t R_C2_A8;
__xdata __at(0xC2B3) volatile uint8_t R_C2_B3;
__xdata __at(0xC2B5) volatile uint8_t R_C2_B5;
__xdata __at(0xC2B6) volatile uint8_t R_C2_B6;
__xdata __at(0xC2B9) volatile uint8_t R_C2_B9;
__xdata __at(0xC2BB) volatile uint8_t R_C2_BB;

/* C3: DPCD window and link training: lane EQ results C3:90..97 and C3:F3..FA. */
__xdata __at(0xC303) volatile uint8_t DPCD_ADDR_H; /* DPCD window address bits 19:16 in bits 3:0 */
__xdata __at(0xC304) volatile uint8_t DPCD_ADDR_M; /* DPCD window address bits 15:8 */
__xdata __at(0xC305) volatile uint8_t DPCD_ADDR_L; /* DPCD window address bits 7:0 */
__xdata __at(0xC30C) volatile uint8_t R_C3_0C;
__xdata __at(0xC315) volatile uint8_t R_C3_15;
__xdata __at(0xC31B) volatile uint8_t R_C3_1B;
__xdata __at(0xC31F) volatile uint8_t R_C3_1F;
__xdata __at(0xC321) volatile uint8_t R_C3_21;
__xdata __at(0xC323) volatile uint8_t R_C3_23;
__xdata __at(0xC325) volatile uint8_t R_C3_25;
__xdata __at(0xC32A) volatile uint8_t R_C3_2A;
__xdata __at(0xC339) volatile uint8_t R_C3_39;
__xdata __at(0xC353) volatile uint8_t DPCD_WDATA; /* DPCD window write data */
__xdata __at(0xC354) volatile uint8_t DPCD_RDATA; /* DPCD window read data */
__xdata __at(0xC35D) volatile uint8_t R_C3_5D;
__xdata __at(0xC360) volatile uint8_t R_C3_60;
__xdata __at(0xC36A) volatile uint8_t R_C3_6A;
__xdata __at(0xC38A) volatile uint8_t R_C3_8A;
__xdata __at(0xC390) volatile uint8_t R_C3_90;
__xdata __at(0xC391) volatile uint8_t R_C3_91;
__xdata __at(0xC392) volatile uint8_t R_C3_92;
__xdata __at(0xC393) volatile uint8_t R_C3_93;
__xdata __at(0xC394) volatile uint8_t R_C3_94;
__xdata __at(0xC395) volatile uint8_t R_C3_95;
__xdata __at(0xC396) volatile uint8_t R_C3_96;
__xdata __at(0xC397) volatile uint8_t R_C3_97;
__xdata __at(0xC39C) volatile uint8_t R_C3_9C;
__xdata __at(0xC39E) volatile uint8_t R_C3_9E;
__xdata __at(0xC39F) volatile uint8_t R_C3_9F;
__xdata __at(0xC3AB) volatile uint8_t R_C3_AB;
__xdata __at(0xC3B8) volatile uint8_t R_C3_B8;
__xdata __at(0xC3BB) volatile uint8_t R_C3_BB;
__xdata __at(0xC3BC) volatile uint8_t R_C3_BC;
__xdata __at(0xC3BD) volatile uint8_t R_C3_BD;
__xdata __at(0xC3BE) volatile uint8_t R_C3_BE;
__xdata __at(0xC3BF) volatile uint8_t R_C3_BF;
__xdata __at(0xC3C2) volatile uint8_t R_C3_C2;
__xdata __at(0xC3C3) volatile uint8_t R_C3_C3;
__xdata __at(0xC3C4) volatile uint8_t R_C3_C4;
__xdata __at(0xC3C5) volatile uint8_t R_C3_C5;
__xdata __at(0xC3C9) volatile uint8_t R_C3_C9;
__xdata __at(0xC3CA) volatile uint8_t RX_LINK_CFG;
__xdata __at(0xC3D4) volatile uint8_t R_C3_D4;
__xdata __at(0xC3D6) volatile uint8_t R_C3_D6;
__xdata __at(0xC3D7) volatile uint8_t R_C3_D7;
__xdata __at(0xC3D8) volatile uint8_t R_C3_D8;
__xdata __at(0xC3D9) volatile uint8_t R_C3_D9;
__xdata __at(0xC3E1) volatile uint8_t R_C3_E1;
__xdata __at(0xC3E8) volatile uint8_t R_C3_E8;
__xdata __at(0xC3EA) volatile uint8_t R_C3_EA;
__xdata __at(0xC3EC) volatile uint8_t R_C3_EC;
__xdata __at(0xC3ED) volatile uint8_t R_C3_ED;
__xdata __at(0xC3F3) volatile uint8_t R_C3_F3;
__xdata __at(0xC3F4) volatile uint8_t R_C3_F4;
__xdata __at(0xC3F5) volatile uint8_t R_C3_F5;
__xdata __at(0xC3F6) volatile uint8_t R_C3_F6;
__xdata __at(0xC3F7) volatile uint8_t R_C3_F7;
__xdata __at(0xC3F8) volatile uint8_t R_C3_F8;
__xdata __at(0xC3F9) volatile uint8_t R_C3_F9;
__xdata __at(0xC3FA) volatile uint8_t R_C3_FA;
__xdata __at(0xC3FD) volatile uint8_t R_C3_FD;

/* C4: HDCP and DP RX miscellany. */
__xdata __at(0xC412) volatile uint8_t R_C4_12;
__xdata __at(0xC413) volatile uint8_t R_C4_13;
__xdata __at(0xC414) volatile uint8_t R_C4_14;
__xdata __at(0xC415) volatile uint8_t R_C4_15;
__xdata __at(0xC416) volatile uint8_t R_C4_16;
__xdata __at(0xC421) volatile uint8_t R_C4_21;
__xdata __at(0xC422) volatile uint8_t R_C4_22;
__xdata __at(0xC42E) volatile uint8_t R_C4_2E;
__xdata __at(0xC431) volatile uint8_t R_C4_31;
__xdata __at(0xC436) volatile uint8_t R_C4_36;
__xdata __at(0xC437) volatile uint8_t R_C4_37;
__xdata __at(0xC438) volatile uint8_t R_C4_38;
__xdata __at(0xC439) volatile uint8_t R_C4_39;
__xdata __at(0xC43C) volatile uint8_t R_C4_3C;
__xdata __at(0xC43E) volatile uint8_t R_C4_3E;
__xdata __at(0xC442) volatile uint8_t R_C4_42;
__xdata __at(0xC4B3) volatile uint8_t R_C4_B3;
__xdata __at(0xC4B7) volatile uint8_t R_C4_B7;
__xdata __at(0xC4BA) volatile uint8_t R_C4_BA;
__xdata __at(0xC4F0) volatile uint8_t R_C4_F0;
__xdata __at(0xC4F1) volatile uint8_t R_C4_F1;

/* C5: Video path, used by the video-info code; meaning unknown. */
__xdata __at(0xC50B) volatile uint8_t R_C5_0B;
__xdata __at(0xC50C) volatile uint8_t R_C5_0C;
__xdata __at(0xC50D) volatile uint8_t R_C5_0D;
__xdata __at(0xC510) volatile uint8_t R_C5_10;
__xdata __at(0xC511) volatile uint8_t R_C5_11;
__xdata __at(0xC512) volatile uint8_t R_C5_12;

/* C7: USB-C PD PHYs: two identical blocks 0x80 apart, C700..C75F the data port (PD0) and C780..C7DF the power port (PD1). CC/VBUS status C7:2F/30 and C7:AF/B0 (C7:AF bit 4 = power-port VBUS). */
__xdata __at(0xC70A) volatile uint8_t R_C7_0A;
__xdata __at(0xC70C) volatile uint8_t R_C7_0C;
__xdata __at(0xC70D) volatile uint8_t R_C7_0D;
__xdata __at(0xC712) volatile uint8_t R_C7_12;
__xdata __at(0xC713) volatile uint8_t R_C7_13;
__xdata __at(0xC715) volatile uint8_t R_C7_15;
__xdata __at(0xC716) volatile uint8_t R_C7_16;
__xdata __at(0xC717) volatile uint8_t R_C7_17;
__xdata __at(0xC718) volatile uint8_t R_C7_18;
__xdata __at(0xC719) volatile uint8_t R_C7_19;
__xdata __at(0xC71A) volatile uint8_t PD0_TX_DATA;
__xdata __at(0xC71B) volatile uint8_t R_C7_1B;
__xdata __at(0xC71C) volatile uint8_t R_C7_1C;
__xdata __at(0xC722) volatile uint8_t R_C7_22;
__xdata __at(0xC723) volatile uint8_t PD0_FIFO; /* data-port PD: a read pops a received byte; writing 1 sends the bytes queued in PD0_TX_DATA */
__xdata __at(0xC729) volatile uint8_t R_C7_29;
__xdata __at(0xC72C) volatile uint8_t R_C7_2C;
__xdata __at(0xC72D) volatile uint8_t R_C7_2D;
__xdata __at(0xC72E) volatile uint8_t R_C7_2E;
__xdata __at(0xC72F) volatile uint8_t R_C7_2F;
__xdata __at(0xC730) volatile uint8_t R_C7_30;
__xdata __at(0xC731) volatile uint8_t R_C7_31;
__xdata __at(0xC736) volatile uint8_t PD0_STATUS;
__xdata __at(0xC73A) volatile uint8_t PD0_RX_HEADER0;
__xdata __at(0xC73B) volatile uint8_t PD0_RX_HEADER1;
__xdata __at(0xC73C) volatile uint8_t R_C7_3C;
__xdata __at(0xC73D) volatile uint8_t R_C7_3D;
__xdata __at(0xC73E) volatile uint8_t R_C7_3E;
__xdata __at(0xC740) volatile uint8_t R_C7_40;
__xdata __at(0xC741) volatile uint8_t R_C7_41;
__xdata __at(0xC742) volatile uint8_t R_C7_42;
__xdata __at(0xC743) volatile uint8_t R_C7_43;
__xdata __at(0xC744) volatile uint8_t R_C7_44;
__xdata __at(0xC745) volatile uint8_t R_C7_45;
__xdata __at(0xC746) volatile uint8_t R_C7_46;
__xdata __at(0xC74D) volatile uint8_t R_C7_4D;
__xdata __at(0xC754) volatile uint8_t R_C7_54;
__xdata __at(0xC78A) volatile uint8_t R_C7_8A;
__xdata __at(0xC78C) volatile uint8_t R_C7_8C;
__xdata __at(0xC78D) volatile uint8_t R_C7_8D;
__xdata __at(0xC792) volatile uint8_t R_C7_92;
__xdata __at(0xC793) volatile uint8_t R_C7_93;
__xdata __at(0xC795) volatile uint8_t R_C7_95;
__xdata __at(0xC796) volatile uint8_t R_C7_96;
__xdata __at(0xC797) volatile uint8_t R_C7_97;
__xdata __at(0xC798) volatile uint8_t R_C7_98;
__xdata __at(0xC799) volatile uint8_t R_C7_99;
__xdata __at(0xC79A) volatile uint8_t PD1_TX_DATA;
__xdata __at(0xC79B) volatile uint8_t R_C7_9B;
__xdata __at(0xC79C) volatile uint8_t R_C7_9C;
__xdata __at(0xC7A2) volatile uint8_t R_C7_A2;
__xdata __at(0xC7A3) volatile uint8_t PD1_FIFO; /* power-port PD FIFO, as PD0_FIFO */
__xdata __at(0xC7A6) volatile uint8_t R_C7_A6;
__xdata __at(0xC7A7) volatile uint8_t R_C7_A7;
__xdata __at(0xC7A9) volatile uint8_t R_C7_A9;
__xdata __at(0xC7AC) volatile uint8_t R_C7_AC;
__xdata __at(0xC7AD) volatile uint8_t R_C7_AD;
__xdata __at(0xC7AE) volatile uint8_t R_C7_AE;
__xdata __at(0xC7AF) volatile uint8_t R_C7_AF;
__xdata __at(0xC7B0) volatile uint8_t R_C7_B0;
__xdata __at(0xC7B1) volatile uint8_t R_C7_B1;
__xdata __at(0xC7B6) volatile uint8_t PD1_STATUS;
__xdata __at(0xC7BA) volatile uint8_t PD1_RX_HEADER0;
__xdata __at(0xC7BB) volatile uint8_t PD1_RX_HEADER1;
__xdata __at(0xC7BC) volatile uint8_t R_C7_BC;
__xdata __at(0xC7BD) volatile uint8_t R_C7_BD;
__xdata __at(0xC7BE) volatile uint8_t R_C7_BE;
__xdata __at(0xC7C0) volatile uint8_t R_C7_C0;
__xdata __at(0xC7C1) volatile uint8_t R_C7_C1;
__xdata __at(0xC7C2) volatile uint8_t R_C7_C2;
__xdata __at(0xC7C3) volatile uint8_t R_C7_C3;
__xdata __at(0xC7C5) volatile uint8_t R_C7_C5;
__xdata __at(0xC7C6) volatile uint8_t R_C7_C6;
__xdata __at(0xC7CD) volatile uint8_t R_C7_CD;
__xdata __at(0xC7D0) volatile uint8_t R_C7_D0;
__xdata __at(0xC7D5) volatile uint8_t R_C7_D5;

/* C8: MIPI TX: D-PHY, CSI-2/DSI timing and output control. */
__xdata __at(0xC805) volatile uint8_t R_C8_05;
__xdata __at(0xC808) volatile uint8_t R_C8_08;
__xdata __at(0xC809) volatile uint8_t R_C8_09;
__xdata __at(0xC80A) volatile uint8_t R_C8_0A;
__xdata __at(0xC80B) volatile uint8_t R_C8_0B;
__xdata __at(0xC80C) volatile uint8_t R_C8_0C;
__xdata __at(0xC80D) volatile uint8_t R_C8_0D;
__xdata __at(0xC80E) volatile uint8_t R_C8_0E;
__xdata __at(0xC80F) volatile uint8_t R_C8_0F;
__xdata __at(0xC811) volatile uint8_t R_C8_11;
__xdata __at(0xC830) volatile uint8_t R_C8_30;
__xdata __at(0xC831) volatile uint8_t R_C8_31;
__xdata __at(0xC832) volatile uint8_t R_C8_32;
__xdata __at(0xC833) volatile uint8_t R_C8_33;
__xdata __at(0xC85C) volatile uint8_t R_C8_5C;
__xdata __at(0xC85D) volatile uint8_t R_C8_5D;
__xdata __at(0xC85E) volatile uint8_t R_C8_5E;
__xdata __at(0xC85F) volatile uint8_t R_C8_5F;
__xdata __at(0xC870) volatile uint8_t R_C8_70;
__xdata __at(0xC871) volatile uint8_t R_C8_71;
__xdata __at(0xC87A) volatile uint8_t R_C8_7A;
__xdata __at(0xC87B) volatile uint8_t R_C8_7B;
__xdata __at(0xC87C) volatile uint8_t R_C8_7C;
__xdata __at(0xC87D) volatile uint8_t R_C8_7D;
__xdata __at(0xC87E) volatile uint8_t R_C8_7E;
__xdata __at(0xC87F) volatile uint8_t R_C8_7F;
__xdata __at(0xC880) volatile uint8_t R_C8_80;
__xdata __at(0xC881) volatile uint8_t R_C8_81;
__xdata __at(0xC882) volatile uint8_t MIPI_RDLY_H;
__xdata __at(0xC883) volatile uint8_t MIPI_RDLY_L;
__xdata __at(0xC884) volatile uint8_t R_C8_84;
__xdata __at(0xC885) volatile uint8_t R_C8_85;
__xdata __at(0xC886) volatile uint8_t R_C8_86;
__xdata __at(0xC887) volatile uint8_t R_C8_87;
__xdata __at(0xC888) volatile uint8_t R_C8_88;
__xdata __at(0xC889) volatile uint8_t R_C8_89;
__xdata __at(0xC895) volatile uint8_t R_C8_95;
__xdata __at(0xC896) volatile uint8_t R_C8_96;
__xdata __at(0xC897) volatile uint8_t R_C8_97;
__xdata __at(0xC898) volatile uint8_t R_C8_98;
__xdata __at(0xC899) volatile uint8_t R_C8_99;
__xdata __at(0xC89A) volatile uint8_t R_C8_9A;
__xdata __at(0xC89C) volatile uint8_t R_C8_9C;
__xdata __at(0xC8A0) volatile uint8_t R_C8_A0;
__xdata __at(0xC8A3) volatile uint8_t R_C8_A3;
__xdata __at(0xC8A6) volatile uint8_t R_C8_A6;
__xdata __at(0xC8A9) volatile uint8_t R_C8_A9;
__xdata __at(0xC8AA) volatile uint8_t R_C8_AA;
__xdata __at(0xC8AB) volatile uint8_t R_C8_AB;
__xdata __at(0xC8AC) volatile uint8_t R_C8_AC;
__xdata __at(0xC8AD) volatile uint8_t R_C8_AD;
__xdata __at(0xC8AE) volatile uint8_t R_C8_AE;
__xdata __at(0xC8B6) volatile uint8_t R_C8_B6;
__xdata __at(0xC8C1) volatile uint8_t R_C8_C1;
__xdata __at(0xC8C2) volatile uint8_t R_C8_C2;
__xdata __at(0xC8D4) volatile uint8_t R_C8_D4;
__xdata __at(0xC8D5) volatile uint8_t R_C8_D5;
__xdata __at(0xC8D6) volatile uint8_t R_C8_D6;
__xdata __at(0xC8D7) volatile uint8_t R_C8_D7;
__xdata __at(0xC8D8) volatile uint8_t R_C8_D8;
__xdata __at(0xC8DA) volatile uint8_t R_C8_DA;
__xdata __at(0xC8DC) volatile uint8_t R_C8_DC;
__xdata __at(0xC8DD) volatile uint8_t R_C8_DD;
__xdata __at(0xC8EC) volatile uint8_t R_C8_EC;

/* C9: MIPI TX PLL. */
__xdata __at(0xC920) volatile uint8_t R_C9_20;
__xdata __at(0xC925) volatile uint8_t R_C9_25;
__xdata __at(0xC926) volatile uint8_t R_C9_26;
__xdata __at(0xC928) volatile uint8_t R_C9_28;
__xdata __at(0xC929) volatile uint8_t R_C9_29;
__xdata __at(0xC955) volatile uint8_t R_C9_55;
__xdata __at(0xC990) volatile uint8_t R_C9_90;
__xdata __at(0xC996) volatile uint8_t R_C9_96;
__xdata __at(0xC99D) volatile uint8_t R_C9_9D;
__xdata __at(0xC99E) volatile uint8_t R_C9_9E;

/* CA: Audio. */
__xdata __at(0xCA05) volatile uint8_t R_CA_05;
__xdata __at(0xCA08) volatile uint8_t R_CA_08;
__xdata __at(0xCA0A) volatile uint8_t R_CA_0A;
__xdata __at(0xCA0B) volatile uint8_t R_CA_0B;
__xdata __at(0xCA0C) volatile uint8_t R_CA_0C;
__xdata __at(0xCA0D) volatile uint8_t R_CA_0D;
__xdata __at(0xCA0E) volatile uint8_t R_CA_0E;
__xdata __at(0xCA11) volatile uint8_t R_CA_11;
__xdata __at(0xCA17) volatile uint8_t R_CA_17;
__xdata __at(0xCA1A) volatile uint8_t R_CA_1A;
__xdata __at(0xCA25) volatile uint8_t R_CA_25;
__xdata __at(0xCA26) volatile uint8_t R_CA_26;
__xdata __at(0xCA28) volatile uint8_t R_CA_28;
__xdata __at(0xCA2A) volatile uint8_t R_CA_2A;
__xdata __at(0xCA2B) volatile uint8_t R_CA_2B;
__xdata __at(0xCA2C) volatile uint8_t R_CA_2C;
__xdata __at(0xCA2D) volatile uint8_t R_CA_2D;
__xdata __at(0xCA2F) volatile uint8_t R_CA_2F;
__xdata __at(0xCA3A) volatile uint8_t R_CA_3A;
__xdata __at(0xCA42) volatile uint8_t R_CA_42;
__xdata __at(0xCA4A) volatile uint8_t R_CA_4A;
__xdata __at(0xCA4B) volatile uint8_t R_CA_4B;
__xdata __at(0xCA50) volatile uint8_t R_CA_50;
__xdata __at(0xCA53) volatile uint8_t R_CA_53;
__xdata __at(0xCA5A) volatile uint8_t R_CA_5A;
__xdata __at(0xCA60) volatile uint8_t R_CA_60;
__xdata __at(0xCA61) volatile uint8_t R_CA_61;
__xdata __at(0xCA7C) volatile uint8_t R_CA_7C;
__xdata __at(0xCA81) volatile uint8_t R_CA_81;
__xdata __at(0xCA8D) volatile uint8_t R_CA_8D;
__xdata __at(0xCA97) volatile uint8_t R_CA_97;

/* CB: DP RX PHY side; meaning unknown. */
__xdata __at(0xCB14) volatile uint8_t R_CB_14;
__xdata __at(0xCB1D) volatile uint8_t R_CB_1D;
__xdata __at(0xCB1E) volatile uint8_t R_CB_1E;
__xdata __at(0xCB23) volatile uint8_t R_CB_23;
__xdata __at(0xCB25) volatile uint8_t R_CB_25;
__xdata __at(0xCB26) volatile uint8_t R_CB_26;

/* CD: Meaning unknown (written in RX state 1). */
__xdata __at(0xCD12) volatile uint8_t R_CD_12;
__xdata __at(0xCD60) volatile uint8_t R_CD_60;
__xdata __at(0xCD6D) volatile uint8_t R_CD_6D;
__xdata __at(0xCD6E) volatile uint8_t R_CD_6E;
__xdata __at(0xCD6F) volatile uint8_t R_CD_6F;
__xdata __at(0xCDF2) volatile uint8_t R_CD_F2;
__xdata __at(0xCDF4) volatile uint8_t R_CD_F4;

/* E0: MCU system: the host mailbox E0:80..B1 (the only registers the host reads; see HOST_*), the SPI flash engine, the free-running counter. */
__xdata __at(0xE02A) volatile uint8_t COUNTER_CTRL; /* bit 2 latches FREE_COUNTER into the read registers */
__xdata __at(0xE04C) volatile uint8_t FREE_COUNTER[4]; /* free-running counter, big-endian; valid while COUNTER_CTRL bit 2 is set */
__xdata __at(0xE051) volatile uint8_t FLASH_STROBE; /* SPI flash engine: write 1 then 0 to run the command */
__xdata __at(0xE054) volatile uint8_t FLASH_FORMAT; /* SPI command format: 0x01 opcode only, 0x05 opcode + address, 0x45 read */
__xdata __at(0xE055) volatile uint8_t FLASH_OPCODE; /* SPI opcode (0x03 read) */
__xdata __at(0xE056) volatile uint8_t FLASH_LEN; /* read length */
__xdata __at(0xE05A) volatile uint8_t FLASH_ADDR_H; /* flash address bits 23:16 */
__xdata __at(0xE05B) volatile uint8_t FLASH_ADDR_M; /* flash address bits 15:8 */
__xdata __at(0xE05C) volatile uint8_t FLASH_ADDR_L; /* flash address bits 7:0 */
__xdata __at(0xE05E) volatile uint8_t FLASH_RDATA; /* read FIFO */
__xdata __at(0xE080) volatile uint8_t HOST_80; /* host mailbox, written 0 at boot */
__xdata __at(0xE081) volatile uint8_t HOST_FW_VER_0; /* firmware version byte 0 (host reads E0:81..83) */
__xdata __at(0xE082) volatile uint8_t HOST_FW_VER_1; /* firmware version byte 1 */
__xdata __at(0xE083) volatile uint8_t HOST_FW_VER_2; /* firmware version byte 2 */
__xdata __at(0xE084) volatile uint8_t HOST_INT_TYPE; /* interrupt type the host reads on the IRQ pulse: 0 video off, 1 video ready, 2 audio off, 3 audio ready */
__xdata __at(0xE085) volatile uint8_t HOST_PCLK_H; /* pixel clock in kHz, 24-bit big-endian */
__xdata __at(0xE086) volatile uint8_t HOST_PCLK_M;
__xdata __at(0xE087) volatile uint8_t HOST_PCLK_L;
__xdata __at(0xE088) volatile uint8_t HOST_HTOTAL_H; /* horizontal total, big-endian */
__xdata __at(0xE089) volatile uint8_t HOST_HTOTAL_L;
__xdata __at(0xE08A) volatile uint8_t HOST_VTOTAL_H; /* vertical total, big-endian */
__xdata __at(0xE08B) volatile uint8_t HOST_VTOTAL_L;
__xdata __at(0xE08C) volatile uint8_t HOST_WIDTH_H; /* active width, big-endian */
__xdata __at(0xE08D) volatile uint8_t HOST_WIDTH_L;
__xdata __at(0xE08E) volatile uint8_t HOST_HEIGHT_H; /* active height, big-endian */
__xdata __at(0xE08F) volatile uint8_t HOST_HEIGHT_L;
__xdata __at(0xE090) volatile uint8_t R_E0_90;
__xdata __at(0xE091) volatile uint8_t R_E0_91;
__xdata __at(0xE092) volatile uint8_t R_E0_92;
__xdata __at(0xE093) volatile uint8_t R_E0_93;
__xdata __at(0xE094) volatile uint8_t R_E0_94;
__xdata __at(0xE095) volatile uint8_t R_E0_95;
__xdata __at(0xE096) volatile uint8_t R_E0_96;
__xdata __at(0xE097) volatile uint8_t R_E0_97;
__xdata __at(0xE098) volatile uint8_t R_E0_98;
__xdata __at(0xE099) volatile uint8_t R_E0_99;
__xdata __at(0xE09A) volatile uint8_t R_E0_9A;
__xdata __at(0xE09B) volatile uint8_t R_E0_9B;
__xdata __at(0xE09C) volatile uint8_t R_E0_9C;
__xdata __at(0xE09D) volatile uint8_t R_E0_9D;
__xdata __at(0xE09E) volatile uint8_t HOST_HDCP; /* bit 7: HDCP key valid */
__xdata __at(0xE0A0) volatile uint8_t R_E0_A0;
__xdata __at(0xE0A1) volatile uint8_t R_E0_A1;
__xdata __at(0xE0A2) volatile uint8_t R_E0_A2;
__xdata __at(0xE0A3) volatile uint8_t R_E0_A3;
__xdata __at(0xE0A4) volatile uint8_t HOST_COLOR_SPACE; /* colour-space code 0/1/2 */
__xdata __at(0xE0A5) volatile uint8_t R_E0_A5;
__xdata __at(0xE0A6) volatile uint8_t R_E0_A6;
__xdata __at(0xE0A8) volatile uint8_t HOST_MARKER; /* host marker (host_marker_publish) */
__xdata __at(0xE0AA) volatile uint8_t R_E0_AA;
__xdata __at(0xE0AB) volatile uint8_t R_E0_AB;
__xdata __at(0xE0AD) volatile uint8_t HOST_AUDIO_FS; /* audio sample rate code */
__xdata __at(0xE0AE) volatile uint8_t HOST_RX_STATE; /* mirror of rx_state; the host driver and tests read it */
__xdata __at(0xE0AF) volatile uint8_t HOST_TX_STATE; /* mirror of tx_state */
__xdata __at(0xE0B0) volatile uint8_t HOST_MIPI_GATE; /* host to MCU: MIPI output gate */
__xdata __at(0xE0B1) volatile uint8_t HOST_AUDIO_GATE; /* host to MCU: audio output gate */

/* E1: Read at boot and by the HPD code; E1:02 bits 3:0 look like a board strap. */
__xdata __at(0xE102) volatile uint8_t R_E1_02;
__xdata __at(0xE118) volatile uint8_t R_E1_18;
__xdata __at(0xE119) volatile uint8_t R_E1_19;
__xdata __at(0xE11A) volatile uint8_t R_E1_1A;
__xdata __at(0xE11B) volatile uint8_t R_E1_1B;

/* E2: System and analog: clocks E2:05..0F, pin config E2:22..43, Type-C analog E2:2D/39, DP RX PHY E2:48..9F, MIPI analog E2:A2..E3. */
__xdata __at(0xE202) volatile uint8_t R_E2_02;
__xdata __at(0xE205) volatile uint8_t R_E2_05;
__xdata __at(0xE206) volatile uint8_t R_E2_06;
__xdata __at(0xE208) volatile uint8_t R_E2_08;
__xdata __at(0xE209) volatile uint8_t R_E2_09;
__xdata __at(0xE20A) volatile uint8_t R_E2_0A;
__xdata __at(0xE20B) volatile uint8_t R_E2_0B;
__xdata __at(0xE20F) volatile uint8_t R_E2_0F;
__xdata __at(0xE222) volatile uint8_t R_E2_22;
__xdata __at(0xE223) volatile uint8_t R_E2_23;
__xdata __at(0xE225) volatile uint8_t R_E2_25;
__xdata __at(0xE226) volatile uint8_t R_E2_26;
__xdata __at(0xE227) volatile uint8_t R_E2_27;
__xdata __at(0xE228) volatile uint8_t R_E2_28;
__xdata __at(0xE229) volatile uint8_t R_E2_29;
__xdata __at(0xE22A) volatile uint8_t R_E2_2A;
__xdata __at(0xE22B) volatile uint8_t R_E2_2B;
__xdata __at(0xE22C) volatile uint8_t R_E2_2C;
__xdata __at(0xE22D) volatile uint8_t R_E2_2D;
__xdata __at(0xE22E) volatile uint8_t R_E2_2E;
__xdata __at(0xE22F) volatile uint8_t R_E2_2F;
__xdata __at(0xE230) volatile uint8_t R_E2_30;
__xdata __at(0xE231) volatile uint8_t R_E2_31;
__xdata __at(0xE232) volatile uint8_t R_E2_32;
__xdata __at(0xE233) volatile uint8_t R_E2_33;
__xdata __at(0xE234) volatile uint8_t R_E2_34;
__xdata __at(0xE235) volatile uint8_t R_E2_35;
__xdata __at(0xE236) volatile uint8_t R_E2_36;
__xdata __at(0xE237) volatile uint8_t R_E2_37;
__xdata __at(0xE238) volatile uint8_t R_E2_38;
__xdata __at(0xE239) volatile uint8_t R_E2_39;
__xdata __at(0xE23A) volatile uint8_t R_E2_3A;
__xdata __at(0xE23B) volatile uint8_t R_E2_3B;
__xdata __at(0xE23C) volatile uint8_t R_E2_3C;
__xdata __at(0xE23D) volatile uint8_t R_E2_3D;
__xdata __at(0xE23F) volatile uint8_t R_E2_3F;
__xdata __at(0xE240) volatile uint8_t R_E2_40;
__xdata __at(0xE243) volatile uint8_t R_E2_43;
__xdata __at(0xE244) volatile uint8_t R_E2_44;
__xdata __at(0xE247) volatile uint8_t R_E2_47;
__xdata __at(0xE248) volatile uint8_t R_E2_48;
__xdata __at(0xE24C) volatile uint8_t R_E2_4C;
__xdata __at(0xE24D) volatile uint8_t R_E2_4D;
__xdata __at(0xE261) volatile uint8_t R_E2_61;
__xdata __at(0xE26C) volatile uint8_t R_E2_6C;
__xdata __at(0xE270) volatile uint8_t R_E2_70;
__xdata __at(0xE271) volatile uint8_t R_E2_71;
__xdata __at(0xE272) volatile uint8_t R_E2_72;
__xdata __at(0xE275) volatile uint8_t R_E2_75;
__xdata __at(0xE27A) volatile uint8_t R_E2_7A;
__xdata __at(0xE27B) volatile uint8_t R_E2_7B;
__xdata __at(0xE27E) volatile uint8_t R_E2_7E;
__xdata __at(0xE27F) volatile uint8_t R_E2_7F;
__xdata __at(0xE283) volatile uint8_t R_E2_83;
__xdata __at(0xE284) volatile uint8_t R_E2_84;
__xdata __at(0xE295) volatile uint8_t R_E2_95;
__xdata __at(0xE297) volatile uint8_t R_E2_97;
__xdata __at(0xE298) volatile uint8_t R_E2_98;
__xdata __at(0xE299) volatile uint8_t R_E2_99;
__xdata __at(0xE29A) volatile uint8_t R_E2_9A;
__xdata __at(0xE29B) volatile uint8_t R_E2_9B;
__xdata __at(0xE29D) volatile uint8_t R_E2_9D;
__xdata __at(0xE29E) volatile uint8_t R_E2_9E;
__xdata __at(0xE29F) volatile uint8_t R_E2_9F;
__xdata __at(0xE2A0) volatile uint8_t R_E2_A0;
__xdata __at(0xE2A2) volatile uint8_t R_E2_A2;
__xdata __at(0xE2A3) volatile uint8_t R_E2_A3;
__xdata __at(0xE2A4) volatile uint8_t R_E2_A4;
__xdata __at(0xE2A5) volatile uint8_t R_E2_A5;
__xdata __at(0xE2A9) volatile uint8_t R_E2_A9;
__xdata __at(0xE2AA) volatile uint8_t R_E2_AA;
__xdata __at(0xE2AB) volatile uint8_t R_E2_AB;
__xdata __at(0xE2AF) volatile uint8_t R_E2_AF;
__xdata __at(0xE2B0) volatile uint8_t R_E2_B0;
__xdata __at(0xE2B1) volatile uint8_t R_E2_B1;
__xdata __at(0xE2B2) volatile uint8_t R_E2_B2;
__xdata __at(0xE2B4) volatile uint8_t R_E2_B4;
__xdata __at(0xE2B5) volatile uint8_t R_E2_B5;
__xdata __at(0xE2B6) volatile uint8_t R_E2_B6;
__xdata __at(0xE2B7) volatile uint8_t R_E2_B7;
__xdata __at(0xE2BB) volatile uint8_t R_E2_BB;
__xdata __at(0xE2BC) volatile uint8_t R_E2_BC;
__xdata __at(0xE2BD) volatile uint8_t R_E2_BD;
__xdata __at(0xE2C7) volatile uint8_t R_E2_C7;
__xdata __at(0xE2C8) volatile uint8_t R_E2_C8;
__xdata __at(0xE2E2) volatile uint8_t R_E2_E2;
__xdata __at(0xE2E3) volatile uint8_t R_E2_E3;

/* E3: Pins and the MIPI PLL: E3:02..05 PLL, E3:0A..35 pin config and levels (E3:22 HPD drive, E3:2E bit 0 the IRQ pin level). */
__xdata __at(0xE302) volatile uint8_t R_E3_02;
__xdata __at(0xE303) volatile uint8_t R_E3_03;
__xdata __at(0xE305) volatile uint8_t R_E3_05;
__xdata __at(0xE30A) volatile uint8_t R_E3_0A;
__xdata __at(0xE30C) volatile uint8_t R_E3_0C;
__xdata __at(0xE30D) volatile uint8_t R_E3_0D;
__xdata __at(0xE30E) volatile uint8_t R_E3_0E;
__xdata __at(0xE30F) volatile uint8_t R_E3_0F;
__xdata __at(0xE310) volatile uint8_t R_E3_10;
__xdata __at(0xE311) volatile uint8_t R_E3_11;
__xdata __at(0xE312) volatile uint8_t R_E3_12;
__xdata __at(0xE313) volatile uint8_t R_E3_13;
__xdata __at(0xE314) volatile uint8_t R_E3_14;
__xdata __at(0xE315) volatile uint8_t R_E3_15;
__xdata __at(0xE316) volatile uint8_t R_E3_16;
__xdata __at(0xE317) volatile uint8_t R_E3_17;
__xdata __at(0xE318) volatile uint8_t R_E3_18;
__xdata __at(0xE319) volatile uint8_t R_E3_19;
__xdata __at(0xE31A) volatile uint8_t R_E3_1A;
__xdata __at(0xE31B) volatile uint8_t R_E3_1B;
__xdata __at(0xE31C) volatile uint8_t R_E3_1C;
__xdata __at(0xE31D) volatile uint8_t R_E3_1D;
__xdata __at(0xE31E) volatile uint8_t R_E3_1E;
__xdata __at(0xE31F) volatile uint8_t R_E3_1F;
__xdata __at(0xE321) volatile uint8_t R_E3_21;
__xdata __at(0xE322) volatile uint8_t R_E3_22;
__xdata __at(0xE323) volatile uint8_t R_E3_23;
__xdata __at(0xE32B) volatile uint8_t R_E3_2B;
__xdata __at(0xE32D) volatile uint8_t R_E3_2D;
__xdata __at(0xE32E) volatile uint8_t R_E3_2E;
__xdata __at(0xE32F) volatile uint8_t R_E3_2F;
__xdata __at(0xE330) volatile uint8_t R_E3_30;
__xdata __at(0xE331) volatile uint8_t R_E3_31;
__xdata __at(0xE332) volatile uint8_t R_E3_32;
__xdata __at(0xE334) volatile uint8_t R_E3_34;
__xdata __at(0xE335) volatile uint8_t R_E3_35;

/* E4: DP RX video timing measurement E4:7A..A3. */
__xdata __at(0xE400) volatile uint8_t R_E4_00;
__xdata __at(0xE401) volatile uint8_t R_E4_01;
__xdata __at(0xE404) volatile uint8_t R_E4_04;
__xdata __at(0xE47A) volatile uint8_t R_E4_7A_8D[0x14]; /* the ten 16-bit big-endian timing measurements rx_timing_read returns */
__xdata __at(0xE47A) volatile uint8_t R_E4_7A;
__xdata __at(0xE47C) volatile uint8_t R_E4_7C;
__xdata __at(0xE47E) volatile uint8_t R_E4_7E;
__xdata __at(0xE480) volatile uint8_t R_E4_80;
__xdata __at(0xE482) volatile uint8_t R_E4_82;
__xdata __at(0xE484) volatile uint8_t R_E4_84;
__xdata __at(0xE486) volatile uint8_t R_E4_86;
__xdata __at(0xE488) volatile uint8_t R_E4_88;
__xdata __at(0xE48A) volatile uint8_t R_E4_8A;
__xdata __at(0xE48C) volatile uint8_t R_E4_8C;
__xdata __at(0xE4A3) volatile uint8_t R_E4_A3;

/* E5: Interrupt controller (INT0/INT1 status and acknowledge) and clock-measurement counters E5:40..5D. */
__xdata __at(0xE501) volatile uint8_t INT0_MASK;
__xdata __at(0xE505) volatile uint8_t R_E5_05;
__xdata __at(0xE50A) volatile uint8_t INT1_MASK;
__xdata __at(0xE519) volatile uint8_t INT0_CLEAR; /* write 1 then 0 to a bit to acknowledge it in INT0_STATUS */
__xdata __at(0xE51A) volatile uint8_t INT1_CLEAR; /* write 1 then 0 to a bit to acknowledge it in INT1_STATUS */
__xdata __at(0xE51C) volatile uint8_t R_E5_1C;
__xdata __at(0xE51D) volatile uint8_t INT0_CLEAR2; /* acknowledge for INT0_STATUS2 */
__xdata __at(0xE531) volatile uint8_t INT0_STATUS; /* INT0 sources: bit 0 native AUX request, bits 1..7 other DP RX events */
__xdata __at(0xE532) volatile uint8_t INT1_STATUS; /* INT1 sources: bit 3 data-port PD, bit 2 power-port PD, bit 5 DP */
__xdata __at(0xE535) volatile uint8_t INT0_STATUS2; /* INT0 sources: bit 4 I2C-over-AUX request */
__xdata __at(0xE540) volatile uint8_t CLK_METER0_CTL;
__xdata __at(0xE545) volatile uint8_t R_E5_45;
__xdata __at(0xE546) volatile uint8_t R_E5_46;
__xdata __at(0xE547) volatile uint8_t R_E5_47;
__xdata __at(0xE54A) volatile uint8_t CLK_METER0_STATUS;
__xdata __at(0xE54B) volatile uint8_t CLK_METER0_CNT_H;
__xdata __at(0xE54C) volatile uint8_t CLK_METER0_CNT_M;
__xdata __at(0xE54D) volatile uint8_t CLK_METER0_CNT_L;
__xdata __at(0xE550) volatile uint8_t CLK_METER1_CTL;
__xdata __at(0xE555) volatile uint8_t R_E5_55;
__xdata __at(0xE556) volatile uint8_t R_E5_56;
__xdata __at(0xE557) volatile uint8_t R_E5_57;
__xdata __at(0xE55A) volatile uint8_t CLK_METER1_STATUS;
__xdata __at(0xE55B) volatile uint8_t CLK_METER1_CNT_H;
__xdata __at(0xE55C) volatile uint8_t CLK_METER1_CNT_M;
__xdata __at(0xE55D) volatile uint8_t CLK_METER1_CNT_L;

/* E6: Analog miscellany. */
__xdata __at(0xE633) volatile uint8_t R_E6_33;
__xdata __at(0xE638) volatile uint8_t R_E6_38;
__xdata __at(0xE639) volatile uint8_t R_E6_39;
__xdata __at(0xE63A) volatile uint8_t R_E6_3A;
__xdata __at(0xE63B) volatile uint8_t R_E6_3B;
__xdata __at(0xE64D) volatile uint8_t R_E6_4D[0x13]; /* E6:4D..5F: one 4-bit field per pin, indexed as pin_config()'s pins */
__xdata __at(0xE65F) volatile uint8_t R_E6_5F;

/* E7: Oscillator measurement. */
__xdata __at(0xE73C) volatile uint8_t R_E7_3C;
__xdata __at(0xE73D) volatile uint8_t R_E7_3D;
__xdata __at(0xE73E) volatile uint8_t R_E7_3E;
__xdata __at(0xE740) volatile uint8_t R_E7_40;
__xdata __at(0xE749) volatile uint8_t R_E7_49;
__xdata __at(0xE74A) volatile uint8_t R_E7_4A;
__xdata __at(0xE74B) volatile uint8_t R_E7_4B;

/* E8: DP RX PHY and equaliser. */
__xdata __at(0xE80F) volatile uint8_t R_E8_0F;
__xdata __at(0xE811) volatile uint8_t R_E8_11;
__xdata __at(0xE812) volatile uint8_t R_E8_12;
__xdata __at(0xE814) volatile uint8_t R_E8_14;
__xdata __at(0xE821) volatile uint8_t R_E8_21;
__xdata __at(0xE822) volatile uint8_t R_E8_22;
__xdata __at(0xE838) volatile uint8_t R_E8_38;
__xdata __at(0xE83A) volatile uint8_t R_E8_3A;
__xdata __at(0xE83C) volatile uint8_t R_E8_3C;
__xdata __at(0xE83E) volatile uint8_t R_E8_3E;
__xdata __at(0xE840) volatile uint8_t R_E8_40;
__xdata __at(0xE841) volatile uint8_t R_E8_41;
__xdata __at(0xE854) volatile uint8_t R_E8_54;
__xdata __at(0xE855) volatile uint8_t R_E8_55;
__xdata __at(0xE85A) volatile uint8_t R_E8_5A;
__xdata __at(0xE85B) volatile uint8_t R_E8_5B;
__xdata __at(0xE85E) volatile uint8_t R_E8_5E;
__xdata __at(0xE85F) volatile uint8_t R_E8_5F;
__xdata __at(0xE860) volatile uint8_t R_E8_60;
__xdata __at(0xE861) volatile uint8_t R_E8_61;
__xdata __at(0xE864) volatile uint8_t R_E8_64;
__xdata __at(0xE865) volatile uint8_t R_E8_65;
__xdata __at(0xE868) volatile uint8_t R_E8_68;
__xdata __at(0xE869) volatile uint8_t R_E8_69;
__xdata __at(0xE86E) volatile uint8_t R_E8_6E;
__xdata __at(0xE870) volatile uint8_t R_E8_70;
__xdata __at(0xE872) volatile uint8_t R_E8_72;
__xdata __at(0xE874) volatile uint8_t R_E8_74;
__xdata __at(0xE879) volatile uint8_t R_E8_79;
__xdata __at(0xE87A) volatile uint8_t R_E8_7A;
__xdata __at(0xE87B) volatile uint8_t R_E8_7B;
__xdata __at(0xE888) volatile uint8_t R_E8_88;
__xdata __at(0xE889) volatile uint8_t R_E8_89;
__xdata __at(0xE88A) volatile uint8_t R_E8_8A;
__xdata __at(0xE88B) volatile uint8_t R_E8_8B;
__xdata __at(0xE88C) volatile uint8_t R_E8_8C;
__xdata __at(0xE88D) volatile uint8_t R_E8_8D;
__xdata __at(0xE88E) volatile uint8_t R_E8_8E;
__xdata __at(0xE88F) volatile uint8_t R_E8_8F;
__xdata __at(0xE891) volatile uint8_t R_E8_91;
__xdata __at(0xE898) volatile uint8_t R_E8_98;
__xdata __at(0xE899) volatile uint8_t R_E8_99;
__xdata __at(0xE8D0) volatile uint8_t R_E8_D0;
__xdata __at(0xE8D9) volatile uint8_t R_E8_D9;
__xdata __at(0xE8DA) volatile uint8_t R_E8_DA;
__xdata __at(0xE8E0) volatile uint8_t R_E8_E0;
__xdata __at(0xE8E2) volatile uint8_t R_E8_E2;
__xdata __at(0xE8E4) volatile uint8_t R_E8_E4;
__xdata __at(0xE8E5) volatile uint8_t R_E8_E5;
__xdata __at(0xE8E6) volatile uint8_t R_E8_E6;
__xdata __at(0xE8EE) volatile uint8_t R_E8_EE;

#endif
