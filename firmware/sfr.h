/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2026, Alec Dusheck */

/* 8051 special function registers the firmware uses.
 *
 * The core is an 8052 with Timer 2 and one extra interrupt (vector 0x43).
 * Ports P0..P3, Timers 0 and 1, the UART and the T2 capture/reload registers
 * are unused: all pins are memory-mapped (regs.h, pages E2/E3).
 */
#ifndef SFR_H
#define SFR_H

/* Interrupt enable and priority. */
__sfr __at(0xA8) IE;
__sbit __at(0xA8) EX0;  /* INT0: DP RX and AUX */
__sbit __at(0xAA) EX1;  /* INT1: USB-C PD and DP */
__sbit __at(0xAD) ET2;  /* Timer 2: AUX deferred work and PD timers */
__sbit __at(0xAF) EA;
__sfr __at(0xB8) IP;
__sbit __at(0xB8) PX0;

/* External interrupt trigger mode: 0 = level. */
__sfr __at(0x88) TCON;
__sbit __at(0x88) IT0;
__sbit __at(0x8A) IT1;

/* Timer 2, used as a one-shot: load TH2:TL2, set TR2; the ISR stops it. */
__sfr __at(0xC8) T2CON;
__sbit __at(0xCA) TR2;
__sbit __at(0xCF) TF2;
__sfr __at(0xCC) TL2;
__sfr __at(0xCD) TH2;

/* Non-standard SFRs. They look like the Dallas-style extended interrupt
 * registers (0x91 EXIF flags, 0xE8 EIE enable with bit 0 = vector 0x43,
 * 0xF8 EIP priority), which matches how they are used, but that is not
 * proven. */
__sfr __at(0x91) SFR_91;
__sfr __at(0xE8) SFR_E8;
__sfr __at(0xF8) SFR_F8;

#endif
