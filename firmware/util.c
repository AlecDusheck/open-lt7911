// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* Pins, busy-wait delays and the free-running counter.
 *
 * Pins: the chip's pins are numbered 0x00..0x18, 0x22 and 0x27..0x2A. Each
 * has a function-select nibble (pin_config) in a register of pages E2/E3,
 * plus one more bit pin_config sets or clears, and a two-bit GPIO field
 * (gpio_write): an enable bit and, just below it, the output level. GPIO
 * 0x0B is the host IRQ line (active-low pulse), its level bit E3:2E bit 0.
 *
 * Time: E0:4C..4F is a free-running 32-bit counter, big-endian, readable
 * while COUNTER_CTRL bit 2 holds it. counter_latch copies it into `now`;
 * a timeout is a 32-bit stamp (timer_stamp) checked with timer_elapsed.
 * The tick unit is not known. These run only from the main loop, so `now`
 * changes only through them.
 */
#include "fw.h"

struct pin_field {
    volatile __xdata uint8_t *reg;
    uint8_t mask;
};

/* Function-select register of pins 0x00..0x15 (index = pin) and
 * 0x27..0x2A (index 0x16..0x19); mask is the extra bit. Pins 0x16..0x18
 * and 0x22 are laid out differently (pin_config). */
static const struct pin_field __code pin_mux[] = {
    { &R_E2_31, 0x10 }, { &R_E2_22, 0x10 }, { &R_E2_23, 0x10 }, { &R_E2_3D, 0x10 },
    { &R_E3_0A, 0x10 }, { &R_E3_0C, 0x10 }, { &R_E3_0D, 0x10 }, { &R_E3_0E, 0x10 },
    { &R_E3_0F, 0x10 }, { &R_E3_10, 0x20 }, { &R_E3_11, 0x10 }, { &R_E3_12, 0x10 },
    { &R_E3_13, 0x10 }, { &R_E3_14, 0x10 }, { &R_E3_15, 0x10 }, { &R_E3_16, 0x10 },
    { &R_E3_17, 0x10 }, { &R_E3_18, 0x10 }, { &R_E3_19, 0x10 }, { &R_E3_1B, 0x10 },
    { &R_E3_1C, 0x10 }, { &R_E3_1D, 0x10 },
    { &R_E2_2F, 0x10 }, { &R_E2_30, 0x10 }, { &R_E2_3B, 0x10 }, { &R_E2_3C, 0x10 },
};

/* GPIO field of pins 0x00..0x18 (index = pin), 0x22 (0x19) and 0x27..0x2A
 * (0x1A..0x1D); mask is the enable bit, the level bit is the one below. */
static const struct pin_field __code pin_gpio[] = {
    { &R_E2_40, 0x80 }, { &R_E2_25, 0x20 }, { &R_E2_25, 0x08 }, { &R_E2_43, 0x08 },
    { &R_E3_2B, 0x80 }, { &R_E3_2D, 0x20 }, { &R_E3_2D, 0x08 }, { &R_E3_2D, 0x02 },
    { &R_E3_2E, 0x80 }, { &R_E3_2E, 0x20 }, { &R_E3_2E, 0x08 }, { &R_E3_2E, 0x02 },
    { &R_E3_2F, 0x80 }, { &R_E3_2F, 0x20 }, { &R_E3_2F, 0x08 }, { &R_E3_2F, 0x02 },
    { &R_E3_30, 0x80 }, { &R_E3_30, 0x20 }, { &R_E3_30, 0x08 }, { &R_E3_31, 0x02 },
    { &R_E3_32, 0x80 }, { &R_E3_32, 0x20 }, { &R_E3_34, 0x80 }, { &R_E3_34, 0x08 },
    { &R_E3_34, 0x20 },
    { &R_E3_34, 0x02 },
    { &R_E2_3F, 0x08 }, { &R_E2_3F, 0x02 }, { &R_E2_43, 0x80 }, { &R_E2_43, 0x20 },
};

/* Set or clear mask in *reg (one read, one write). */
static void reg_bit(volatile __xdata uint8_t *reg, uint8_t mask, uint8_t set) __reentrant
{
    if (set)
        *reg |= mask;
    else
        *reg &= (uint8_t)~mask;
}

/* Select function func of a pin and set or clear its extra bit
 * (bit). Usual layout: clear the low nibble (one write), OR in func (a
 * second write), then the extra bit. Unknown pins do nothing. */
void pin_config(uint8_t pin, uint8_t func, uint8_t bit)
{
    volatile __xdata uint8_t *reg;
    uint8_t v;

    switch (pin) {
    case 0x16: /* E3:21 bits 1:0 = func[3:2], bits 7:6 = func[1:0] */
        v = R_E3_21 & 0x3C;
        R_E3_21 = v;
        v |= (func & 0x0C) >> 2;
        R_E3_21 = v;
        R_E3_21 = v | (uint8_t)((func & 0x03) << 6);
        reg_bit(&R_E3_22, 0x40, bit);
        return;
    case 0x17: /* E3:21 bits 5:2 */
        v = R_E3_21 & 0xC3;
        R_E3_21 = v;
        R_E3_21 = v | (uint8_t)(func << 2);
        reg_bit(&R_E3_22, 0x80, bit);
        return;
    case 0x18: /* E3:23 bits 7:4 */
        v = R_E3_23 & 0x0F;
        R_E3_23 = v;
        R_E3_23 = v | (uint8_t)(func << 4);
        reg_bit(&R_E3_22, 0x01, bit);
        return;
    case 0x22: /* E3:22 bits 5:2 */
        v = R_E3_22 & 0xC1;
        R_E3_22 = v;
        R_E3_22 = v | (uint8_t)(func << 2);
        reg_bit(&R_E3_22, 0x02, bit);
        return;
    }
    if (pin >= 0x27 && pin <= 0x2A)
        pin -= 0x27 - 0x16;
    else if (pin > 0x15)
        return;
    reg = pin_mux[pin].reg;
    v = *reg & 0xF0;
    *reg = v;
    *reg = v | func;
    reg_bit(reg, pin_mux[pin].mask, bit);
}

/* Drive a pin as a GPIO: set its enable bit, then its level bit to
 * level (non-zero = high). Unknown pins do nothing. */
void gpio_write(uint8_t pin, uint8_t level) __reentrant
{
    volatile __xdata uint8_t *reg;
    uint8_t en;

    if (pin == 0x22)
        pin = 0x19;
    else if (pin >= 0x27 && pin <= 0x2A)
        pin -= 0x27 - 0x1A;
    else if (pin > 0x18)
        return;
    reg = pin_gpio[pin].reg;
    en = pin_gpio[pin].mask;
    *reg |= en;
    reg_bit(reg, en >> 1, level);
}

/* Level of the host IRQ pin's output latch (GPIO 0x0B): 0 or 1. */
uint8_t irq_pin_level(void)
{
    return R_E3_2E & 0x01;
}

/* Busy-wait ms units of 600 inner loops each. The argument moves from
 * DPL/DPH into R7/R6, 4 cycles per call.
 * fw.h declares both delays without __naked: SDCC skips the caller's
 * register saves around a call to a function it knows is naked. */
void delay_ms(uint16_t ms) __reentrant __naked
{
    (void)ms;
    __asm
        mov  r7, dpl
        mov  r6, dph
    00001$:
        mov  a, r7
        dec  r7
        mov  r2, ar6
        jnz  00002$
        dec  r6
    00002$:
        orl  a, r2
        jz   00004$
        clr  a
        mov  r4, a
        mov  r5, a
    00003$:
        clr  c
        mov  a, r5
        subb a, #0x58
        mov  a, r4
        subb a, #0x02
        jnc  00001$
        inc  r5
        cjne r5, #0x00, 00005$
        inc  r4
    00005$:
        sjmp 00003$
    00004$:
        ret
    __endasm;
}

/* Busy-wait n loops. The argument moves from DPL/DPH into R7/R6, 4 cycles
 * per call. Assembly callers enter at _delay_short_r7r6 with n already in
 * R7 (low) and R6. */
void delay_short(uint16_t n) __reentrant __naked
{
    (void)n;
    __asm
        mov  r7, dpl
        mov  r6, dph
    _delay_short_r7r6::
        mov  a, r7
        dec  r7
        mov  r4, ar6
        jnz  00002$
        dec  r6
    00002$:
        orl  a, r4
        jnz  _delay_short_r7r6
        ret
    __endasm;
}

/* memcpy for code an interrupt can reach: SDCC's memcpy keeps its
 * arguments at fixed addresses, so an ISR copy would corrupt a main-loop
 * one it interrupts. src is a code template or XRAM. */
void copy_bytes(__xdata uint8_t *dst, const uint8_t *src, uint8_t n) __reentrant
{
    while (n--)
        *dst++ = *src++;
}

/* Counter control at boot. */
void counter_init(void)
{
    COUNTER_CTRL = 0x28;
}

/* Zero the latched counter. */
void now_reset(void)
{
    now = 0;
}

/* Latch the free-running counter into now. */
void counter_latch(void)
{
    uint32_t t;

    COUNTER_CTRL |= 0x04;
    t = FREE_COUNTER[0];
    t = t << 8 | FREE_COUNTER[1];
    t = t << 8 | FREE_COUNTER[2];
    now = t << 8 | FREE_COUNTER[3];
    COUNTER_CTRL &= (uint8_t)~0x04;
}

/* Store the current counter in *stamp. */
void timer_stamp(__xdata uint32_t *stamp)
{
    counter_latch();
    *stamp = now;
}

/* 1 once the counter has passed *stamp + ticks, else 0. If the
 * counter is behind *stamp (it wrapped) or *stamp + ticks overflows, the
 * stamp restarts at the current counter and the result is 0. */
uint8_t timer_elapsed(__xdata uint32_t *stamp, uint32_t ticks)
{
    uint32_t end;

    counter_latch();
    end = *stamp;
    if (end <= now) {
        end += ticks;
        if (end >= ticks) /* *stamp + ticks did not overflow */
            return now > end;
    }
    *stamp = now;
    return 0;
}

/* The core's DIV AB of a by 0, which the divisions below use for a zero
 * divisor: the 8051 leaves the quotient undefined, so ask the core. */
static uint8_t div_ab_by_0(uint8_t a) __naked
{
    (void)a;
    __asm
        mov  a, dpl
        mov  b, #0
        div  ab
        mov  dpl, a
        ret
    __endasm;
}

/* a / b, defined for b = 0 too: the top byte's DIV AB by 0, then 24 one
 * bits. */
uint32_t udiv32(uint32_t a, uint32_t b) __reentrant
{
    if (b)
        return a / b;
    return (uint32_t)div_ab_by_0(a >> 24) << 24 | 0xFFFFFFUL;
}

/* a / b, defined for b = 0 too: from 256 up a itself, else a's DIV AB by
 * 0. */
uint16_t udiv16(uint16_t a, uint16_t b) __reentrant
{
    if (b)
        return a / b;
    return a > 0xFF ? a : div_ab_by_0((uint8_t)a);
}
