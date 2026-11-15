// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026, Alec Dusheck */

/* The EDID served to the DP source. At boot edid_load() fills edid_buf
 * (512 B) from the flash slot at 0x60000 when it starts 00 FF FF FF, else
 * from the built-in EDID; edid_ram_upload() streams it into the AUX EDID
 * RAM when the RX state machine needs it. */
#include <string.h>

#include "fw.h"

/* The built-in EDID's identity (build.env): EDID_MANUFACTURER_0..2 the PNP
 * id's letters, EDID_PRODUCT, EDID_SERIAL, EDID_NAME_0..12 the monitor name
 * descriptor's text. */
#if !defined(EDID_MANUFACTURER_2) || !defined(EDID_PRODUCT) || !defined(EDID_SERIAL) || !defined(EDID_NAME_12)
#error "EDID_MANUFACTURER, EDID_PRODUCT, EDID_SERIAL and EDID_NAME are required (build.env)"
#endif
#define EDID_IS_LETTER(c) ((c) >= 'A' && (c) <= 'Z')
#if !EDID_IS_LETTER(EDID_MANUFACTURER_0) || !EDID_IS_LETTER(EDID_MANUFACTURER_1) \
    || !EDID_IS_LETTER(EDID_MANUFACTURER_2) || EDID_MANUFACTURER_3 != '\n'
#error "EDID_MANUFACTURER must be three letters A..Z (build.env)"
#endif

#define EDID_FLASH_ADDR 0x60000UL

/* Base block bytes 8..15: manufacturer (PNP id, three letters of 5 bits,
 * A = 1, big-endian), product code and serial number (little-endian). */
#define EDID_PNP_HI ((uint8_t)((EDID_MANUFACTURER_0 - '@') << 2 | (EDID_MANUFACTURER_1 - '@') >> 3))
#define EDID_PNP_LO ((uint8_t)(((EDID_MANUFACTURER_1 - '@') & 0x07) << 5 | (EDID_MANUFACTURER_2 - '@')))
#define EDID_SERIAL_L ((unsigned long)EDID_SERIAL)
#define EDID_ID                                                                  \
    EDID_PNP_HI, EDID_PNP_LO, (EDID_PRODUCT & 0xFF), (EDID_PRODUCT >> 8),         \
    (EDID_SERIAL_L & 0xFF), ((EDID_SERIAL_L >> 8) & 0xFF),                      \
    ((EDID_SERIAL_L >> 16) & 0xFF), (EDID_SERIAL_L >> 24)
#define EDID_NAME                                                                \
    EDID_NAME_0, EDID_NAME_1, EDID_NAME_2, EDID_NAME_3, EDID_NAME_4, EDID_NAME_5, \
    EDID_NAME_6, EDID_NAME_7, EDID_NAME_8, EDID_NAME_9, EDID_NAME_10,             \
    EDID_NAME_11, EDID_NAME_12

/* Base block checksum: the block sums to 0 (mod 256). 0x3C: the sum of the
 * base block's other 106 bytes (all but the identity and the checksum). */
#define EDID_ID_SUM                                                              \
    (0UL + EDID_PNP_HI + EDID_PNP_LO + EDID_PRODUCT + (EDID_PRODUCT >> 8)         \
     + EDID_SERIAL_L + (EDID_SERIAL_L >> 8) + (EDID_SERIAL_L >> 16)              \
     + (EDID_SERIAL_L >> 24) + EDID_NAME_0 + EDID_NAME_1 + EDID_NAME_2            \
     + EDID_NAME_3 + EDID_NAME_4 + EDID_NAME_5 + EDID_NAME_6 + EDID_NAME_7        \
     + EDID_NAME_8 + EDID_NAME_9 + EDID_NAME_10 + EDID_NAME_11 + EDID_NAME_12)
#define EDID_CHECKSUM ((0UL - 0x3C - EDID_ID_SUM) & 0xFF)

/* Built-in EDID (base block + CEA extension); the second 256 bytes of the
 * 512-byte image are zero. */
static const __code uint8_t edid_fallback[256] = {
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, EDID_ID,
    0x06, 0x24, 0x01, 0x03, 0x81, 0x3C, 0x22, 0x78, 0x2E, 0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26,
    0x0F, 0x50, 0x54, 0x21, 0x08, 0x00, 0xD1, 0xC0, 0xA9, 0xC0, 0xD1, 0x00, 0x81, 0x80, 0x81, 0x40,
    0x81, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
    0x8A, 0x00, 0x55, 0x50, 0x21, 0x00, 0x00, 0x1E, 0x33, 0x2D, 0x00, 0x50, 0xA0, 0xA0, 0x15, 0x50,
    0x08, 0x20, 0x78, 0x00, 0x55, 0x50, 0x21, 0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0xFC, 0x00, EDID_NAME,
    0x00, 0x00, 0x00, 0xFD,
    0x00, 0x17, 0x64, 0x16, 0xA0, 0x3C, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, EDID_CHECKSUM,
    0x02, 0x03, 0x2C, 0xF1, 0x48, 0x5F, 0x01, 0x04, 0x13, 0x3E, 0x1F, 0x22, 0x10, 0x23, 0x09, 0x07,
    0x07, 0x83, 0x01, 0x00, 0x00, 0x67, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x18, 0x44, 0x67, 0xD8, 0x5D,
    0xC4, 0x01, 0x78, 0x80, 0x00, 0xE3, 0x05, 0xE3, 0x01, 0xE2, 0x00, 0xCB, 0xA3, 0x5B, 0x00, 0x50,
    0xA0, 0xA0, 0x29, 0x50, 0x08, 0x20, 0xB8, 0x04, 0x55, 0x50, 0x21, 0x00, 0x00, 0x1A, 0x75, 0x89,
    0x00, 0x50, 0xA0, 0xA0, 0x29, 0x50, 0x08, 0x20, 0xB8, 0x04, 0x55, 0x50, 0x21, 0x00, 0x00, 0x1A,
    0x2F, 0x7A, 0x70, 0x50, 0xD0, 0xA0, 0x29, 0x50, 0x08, 0x20, 0xB8, 0x04, 0x55, 0xFA, 0x20, 0x00,
    0x00, 0x1A, 0xA8, 0xA5, 0x00, 0x50, 0xF0, 0x70, 0x08, 0x80, 0x08, 0x20, 0x13, 0x00, 0x55, 0x50,
    0x21, 0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29,
};

/* Load edid_buf from the flash slot, or the built-in EDID when the
 * slot does not start with an EDID header (00 FF FF FF). */
void edid_load(void)
{
    flash_read(EDID_FLASH_ADDR, edid_buf, sizeof(edid_buf));
    if (edid_buf[0] == 0x00 && edid_buf[1] == 0xFF && edid_buf[2] == 0xFF && edid_buf[3] == 0xFF)
        return;
    memcpy(edid_buf, edid_fallback, sizeof(edid_fallback));
    memset(edid_buf + sizeof(edid_fallback), 0, sizeof(edid_buf) - sizeof(edid_fallback));
}

/* Stream 512 bytes into the AUX EDID RAM (C2:02 bit 4 set and
 * C2:05 = 2 select the write port; C2:05 = 0 ends). C2:02 bit 4 stays set. */
void edid_ram_upload(const __xdata uint8_t *edid)
{
    uint16_t i;

    R_C2_02 |= 0x10;
    R_C2_05 = 0x02;
    for (i = 0; i < 0x200; i++)
        EDID_RAM_DATA = edid[i];
    R_C2_05 = 0x00;
}
