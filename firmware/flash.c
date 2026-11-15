// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* SPI flash reads through the E0:5x engine, the same sequence the Linux
 * driver uses (lt7911_read_flash): format 0x45, opcode 0x03 (read), length,
 * 24-bit address, a 1-then-0 strobe on E0:51, a short wait, then the bytes
 * from the E0:5E FIFO. One command reads at most 32 bytes. */
#include "fw.h"

/* Read len bytes from flash address addr into dst, 32 at a time. */
void flash_read(uint32_t addr, __xdata uint8_t *dst, uint16_t len)
{
    /* Only address bits 23:0 reach the engine: keep them as 8 + 16 bits. */
    uint8_t hi = (uint8_t)(addr >> 16);
    uint16_t lo = (uint16_t)addr;
    uint8_t n, i;

    while (len) {
        n = len > 0x20 ? 0x20 : (uint8_t)len;
        FLASH_FORMAT = 0x45;
        FLASH_OPCODE = 0x03;
        FLASH_LEN = n;
        FLASH_ADDR_H = hi;
        FLASH_ADDR_M = (uint8_t)(lo >> 8);
        FLASH_ADDR_L = (uint8_t)lo;
        FLASH_STROBE = 0x01;
        FLASH_STROBE = 0x00;
        delay_short(100);
        for (i = 0; i < n; i++)
            dst[i] = FLASH_RDATA;
        lo += n;
        if (lo < n)
            hi++;
        dst += n;
        len -= n;
    }
}
