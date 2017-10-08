#ifndef __FONT_8x8_H__
#define __FONT_8x8_H__

/* 
 * Taken from ArduiPi_OLED.cpp
 * This font can be freely used without any restriction(It is placed in public domain)
 * 8x8 Font ASCII 32 - 127 Implemented
 * Note: each uint8_t represents a Horizontal bit pattern (not a vertical 'scan')
 * with bit 0 is the topmost bit and bit 7 the lowest.
 */
const uint8_t font_8x8[][8] =  {
  /*  0 - 31 */
  {0x00,0x3C,0x42,0x3C,0x00,0x3C,0x42,0x3C}, /* 00h: NUL */
  {0x00,0x3C,0x42,0x3C,0x00,0x44,0x7E,0x40}, /* 01h: SOH */
  {0x00,0x3C,0x42,0x3C,0x00,0x64,0x52,0x4C}, /* 02h: STX */
  {0x00,0x3C,0x42,0x3C,0x00,0x49,0x49,0x34}, /* 03h: ETX */
  {0x00,0x3C,0x42,0x3C,0x00,0x0E,0x08,0x7F}, /* 04h: EOT */
  {0x00,0x3C,0x42,0x3C,0x00,0x4E,0x4A,0x32}, /* 05h: ENQ */
  {0x00,0x3C,0x42,0x3C,0x00,0x7C,0x4A,0x32}, /* 06h: ACK */
  {0x00,0x3C,0x42,0x3C,0x00,0x02,0x72,0x0E}, /* 07h: BEL */
  {0x00,0x3C,0x42,0x3C,0x00,0x3C,0x49,0x3C}, /* 08h: BS  */
  {0x00,0x3C,0x42,0x3C,0x00,0x4C,0x4A,0x3E}, /* 09h: TAB */
  {0x00,0x3C,0x42,0x3C,0x00,0x7C,0x0A,0x7C}, /* 0Ah: LF  */
  {0x00,0x3C,0x42,0x3C,0x00,0x7E,0x4A,0x34}, /* 0Bh: VT  */
  {0x00,0x3C,0x42,0x3C,0x00,0x3C,0x42,0x4C}, /* 0Ch: FF  */
  {0x00,0x3C,0x42,0x3C,0x00,0x7E,0x42,0x34}, /* 0Dh: CR  */
  {0x00,0x3C,0x42,0x3C,0x00,0x7E,0x4A,0x4A}, /* 0Eh: SO  */
  {0x00,0x3C,0x42,0x3C,0x00,0x7E,0x0A,0x0A}, /* 0Fh: SI  */
  {0x00,0x44,0x7E,0x40,0x00,0x3C,0x42,0x3C}, /* 10h: DLE */
  {0x00,0x44,0x7E,0x40,0x00,0x44,0x7E,0x40}, /* 11h: DC1 */
  {0x00,0x44,0x7E,0x40,0x00,0x64,0x52,0x4C}, /* 12h: DC2 */
  {0x00,0x44,0x7E,0x40,0x00,0x49,0x49,0x34}, /* 13h: DC3 */
  {0x00,0x44,0x7E,0x40,0x00,0x0E,0x08,0x7F}, /* 14h: DC4 */
  {0x00,0x44,0x7E,0x40,0x00,0x4E,0x4A,0x32}, /* 15h: NAK */
  {0x00,0x44,0x7E,0x40,0x00,0x7C,0x4A,0x32}, /* 16h: SYN */
  {0x00,0x44,0x7E,0x40,0x00,0x02,0x72,0x0E}, /* 17h: ETB */
  {0x00,0x44,0x7E,0x40,0x00,0x3C,0x49,0x3C}, /* 18h: CAN */
  {0x00,0x44,0x7E,0x40,0x00,0x4C,0x4A,0x3E}, /* 19h: EM  */
  {0x00,0x44,0x7E,0x40,0x00,0x7C,0x0A,0x7C}, /* 1Ah: SUB */
  {0x00,0x44,0x7E,0x40,0x00,0x7E,0x4A,0x34}, /* 1Bh: ESC */
  {0x00,0x44,0x7E,0x40,0x00,0x3C,0x42,0x4C}, /* 1Ch: FS  */
  {0x00,0x44,0x7E,0x40,0x00,0x7E,0x42,0x34}, /* 1Dh: GS  */
  {0x00,0x44,0x7E,0x40,0x00,0x7E,0x4A,0x4A}, /* 1Eh: RS  */
  {0x00,0x44,0x7E,0x40,0x00,0x7E,0x0A,0x0A}  /* 1Fh: US  */
  /* 32 - 63 */
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
  {0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
  {0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
  {0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
  {0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
  {0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
  {0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
  {0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
  {0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
  {0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
  {0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
  {0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
  {0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
  {0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
  {0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
  {0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
  {0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
  {0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
  {0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
  {0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
  {0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
  /* 64 - 95 */
  {0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
  {0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
  {0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
  {0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
  {0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
  {0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
  {0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
  {0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
  {0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
  {0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
  {0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
  {0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
  {0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
  {0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
  {0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
  {0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
  {0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
  {0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
  {0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
  {0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
  {0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
  /* 96 - 127 */
  {0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
  {0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
  {0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
  {0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
  {0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
  {0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
  {0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
  {0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
  {0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
  {0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
  {0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
  {0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
  {0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
  {0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
  {0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
  {0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
  {0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
  {0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
  {0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
  {0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
  {0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
  {0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
  {0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00},
  /* 128 - 159 */
  {0x00,0x3C,0x49,0x3C,0x00,0x3C,0x42,0x3C}, /* 80h */
  {0x00,0x3C,0x49,0x3C,0x00,0x44,0x7E,0x40}, /* 81h */
  {0x00,0x3C,0x49,0x3C,0x00,0x64,0x52,0x4C}, /* 82h */
  {0x00,0x3C,0x49,0x3C,0x00,0x49,0x49,0x34}, /* 83h */
  {0x00,0x3C,0x49,0x3C,0x00,0x0E,0x08,0x7F}, /* 84h */
  {0x00,0x78,0x15,0x15,0x15,0x78,0x00,0x00}, /* 133: Å */
  {0x00,0x7E,0x09,0x7F,0x49,0x49,0x00,0x00}, /* 134: Æ */
  {0x00,0x3C,0x49,0x3C,0x00,0x02,0x72,0x0E}, /* 87h */
  {0x00,0x3C,0x49,0x3C,0x00,0x3C,0x49,0x3C}, /* 88h */
  {0x00,0x3C,0x49,0x3C,0x00,0x4C,0x4A,0x3E}, /* 89h */
  {0x00,0x3C,0x49,0x3C,0x00,0x7C,0x0A,0x7C}, /* 8Ah */
  {0x00,0x3C,0x49,0x3C,0x00,0x7E,0x4A,0x34}, /* 8Bh */
  {0x00,0x3C,0x49,0x3C,0x00,0x3C,0x42,0x42}, /* 8Ch */
  {0x00,0x3C,0x49,0x3C,0x00,0x7E,0x42,0x3C}, /* 8Dh */
  {0x00,0x3C,0x49,0x3C,0x00,0x7E,0x4A,0x4A}, /* 8Eh */
  {0x00,0x3C,0x49,0x3C,0x00,0x7E,0x0A,0x0A}, /* 8Fh */
  {0x00,0x4C,0x4A,0x3E,0x00,0x3C,0x42,0x3C}, /* 90h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x44,0x7E,0x40}, /* 91h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x64,0x52,0x4C}, /* 92h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x49,0x49,0x34}, /* 93h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x0E,0x08,0x7F}, /* 94h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x4E,0x4A,0x32}, /* 95h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x7C,0x4A,0x32}, /* 96h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x02,0x72,0x0E}, /* 97h */
  {0x00,0x57,0x21,0x5D,0x42,0x3D,0x00,0x00}, /* 152: Ø */
  {0x00,0x4C,0x4A,0x3E,0x00,0x4C,0x4A,0x3E}, /* 99h */
  {0x00,0x4C,0x4A,0x3E,0x00,0x7C,0x0A,0x7C}, /* 9Ah */
  {0x00,0x4C,0x4A,0x3E,0x00,0x7E,0x4A,0x34}, /* 9Bh */
  {0x00,0x4C,0x4A,0x3E,0x00,0x3C,0x42,0x4C}, /* 9Ch */
  {0x00,0x4C,0x4A,0x3E,0x00,0x7E,0x42,0x34}, /* 9Dh */
  {0x00,0x4C,0x4A,0x3E,0x00,0x7E,0x4A,0x4A}, /* 9Eh */
  {0x00,0x4C,0x4A,0x3E,0x00,0x7E,0x0A,0x0A}, /* 9Fh */
	/* 160 - 191 */
  {0x00,0x7C,0x0A,0x7C,0x00,0x3C,0x42,0x3C}, /* A0h */
  {0x00,0x7C,0x0A,0x7C,0x00,0x44,0x7E,0x40}, /* A1h */
  {0x00,0x7C,0x0A,0x7C,0x00,0x64,0x52,0x4C}, /* A2h */
  {0x00,0x7C,0x0A,0x7C,0x00,0x49,0x49,0x34}, /* A3h */
  {0x00,0x7C,0x0A,0x7C,0x00,0x0E,0x08,0x7F}, /* A4h */
  {0x00,0x20,0x54,0x55,0x55,0x78,0x00,0x00}, /* 165: å */
  {0x00,0x24,0x54,0x38,0x54,0x58,0x00,0x00}, /* 166: æ */
  {0x00,0x7C,0x0A,0x7C,0x00,0x02,0x72,0x0E}, /* A7h */
  {0x00,0x7C,0x0A,0x7C,0x00,0x3C,0x49,0x3C}, /* A8h *
  {0x00,0x7C,0x0A,0x7C,0x00,0x4C,0x4A,0x3E}, /* A9h */
  {0x00,0x7C,0x0A,0x7C,0x00,0x7C,0x0A,0x7C}, /* AAh */
  {0x00,0x7C,0x0A,0x7C,0x00,0x7E,0x4A,0x34}, /* ABh */
  {0x00,0x7C,0x0A,0x7C,0x00,0x3C,0x42,0x4C}, /* ACh */
  {0x00,0x7C,0x0A,0x7C,0x00,0x7E,0x42,0x34}, /* ADh */
  {0x00,0x7C,0x0A,0x7C,0x00,0x7E,0x4A,0x4A}, /* AEh */
  {0x00,0x7C,0x0A,0x7C,0x00,0x7E,0x0A,0x0A}, /* AFh */
  {0x00,0x7E,0x4A,0x34,0x00,0x3C,0x42,0x3C}, /* B0h */
  {0x00,0x7E,0x4A,0x34,0x00,0x44,0x7E,0x40}, /* B1h */
  {0x00,0x7E,0x4A,0x34,0x00,0x64,0x52,0x4C}, /* B2h */
  {0x00,0x7E,0x4A,0x34,0x00,0x49,0x49,0x34}, /* B3h */
  {0x00,0x7E,0x4A,0x34,0x00,0x0E,0x08,0x7F}, /* B4h */
  {0x00,0x7E,0x4A,0x34,0x00,0x4E,0x4A,0x32}, /* B5h */
  {0x00,0x7E,0x4A,0x34,0x00,0x7C,0x4A,0x32}, /* B6h */
  {0x00,0x7E,0x4A,0x34,0x00,0x02,0x72,0x0E}, /* B7h */
  {0x00,0x58,0x24,0x54,0x48,0x34,0x00,0x00}, /* 184: ø */
  {0x00,0x7E,0x4A,0x34,0x00,0x4C,0x4A,0x3E}, /* B9h */
  {0x00,0x7E,0x4A,0x34,0x00,0x7C,0x0A,0x7C}, /* BAh */
  {0x00,0x7E,0x4A,0x34,0x00,0x7E,0x4A,0x34}, /* BBh */
  {0x00,0x7E,0x4A,0x34,0x00,0x3C,0x42,0x4C}, /* BCh */
  {0x00,0x7E,0x4A,0x34,0x00,0x7E,0x42,0x34}, /* BDh */
  {0x00,0x7E,0x4A,0x34,0x00,0x7E,0x4A,0x4A}, /* BEh */
  {0x00,0x7E,0x4A,0x34,0x00,0x7E,0x0A,0x0A}, /* BFh */
  /* 192 - 223 */    
  {0x00,0x3C,0x42,0x4C,0x00,0x3C,0x42,0x3C}, /* C0h */
  {0x00,0x3C,0x42,0x4C,0x00,0x44,0x7E,0x40}, /* C1h */
  {0x00,0x3C,0x42,0x4C,0x00,0x64,0x52,0x4C}, /* C2h */
  {0x00,0x3C,0x42,0x4C,0x00,0x49,0x49,0x34}, /* C3h */
  {0x00,0x3C,0x42,0x4C,0x00,0x0E,0x08,0x7F}, /* C4h */
  {0x00,0x3C,0x42,0x4C,0x00,0x4E,0x4A,0x32}, /* C5h */
  {0x00,0x3C,0x42,0x4C,0x00,0x7C,0x4A,0x32}, /* C6h */
  {0x00,0x3C,0x42,0x4C,0x00,0x02,0x72,0x0E}, /* C7h */
  {0x00,0x3C,0x42,0x4C,0x00,0x3C,0x49,0x3C}, /* C8h *
  {0x00,0x3C,0x42,0x4C,0x00,0x4C,0x4A,0x3E}, /* C9h */
  {0x00,0x3C,0x42,0x4C,0x00,0x7C,0x0A,0x7C}, /* CAh */
  {0x00,0x3C,0x42,0x4C,0x00,0x7E,0x4A,0x34}, /* CBh */
  {0x00,0x3C,0x42,0x4C,0x00,0x3C,0x42,0x4C}, /* CCh */
  {0x00,0x3C,0x42,0x4C,0x00,0x7E,0x42,0x34}, /* CDh */
  {0x00,0x3C,0x42,0x4C,0x00,0x7E,0x4A,0x4A}, /* CEh */
  {0x00,0x3C,0x42,0x4C,0x00,0x7E,0x0A,0x0A}, /* CFh */
  {0x00,0x7E,0x42,0x34,0x00,0x3C,0x42,0x3C}, /* D0h */
  {0x00,0x7E,0x42,0x34,0x00,0x44,0x7E,0x40}, /* D1h */
  {0x00,0x7E,0x42,0x34,0x00,0x64,0x52,0x4C}, /* D2h */
  {0x00,0x7E,0x42,0x34,0x00,0x49,0x49,0x34}, /* D3h */
  {0x00,0x7E,0x42,0x34,0x00,0x0E,0x08,0x7F}, /* D4h */
  {0x00,0x7E,0x42,0x34,0x00,0x4E,0x4A,0x32}, /* D5h */
  {0x00,0x7E,0x42,0x34,0x00,0x7C,0x4A,0x32}, /* D6h */
  {0x00,0x7E,0x42,0x34,0x00,0x02,0x72,0x0E}, /* D7h */
  {0x00,0x7E,0x42,0x34,0x00,0x3C,0x49,0x3C}, /* D8h *
  {0x00,0x7E,0x42,0x34,0x00,0x4C,0x4A,0x3E}, /* D9h */
  {0x00,0x7E,0x42,0x34,0x00,0x7C,0x0A,0x7C}, /* DAh */
  {0x00,0x7E,0x42,0x34,0x00,0x7E,0x4A,0x34}, /* DBh */
  {0x00,0x7E,0x42,0x34,0x00,0x3C,0x42,0x4C}, /* DCh */
  {0x00,0x7E,0x42,0x34,0x00,0x7E,0x42,0x34}, /* DDh */
  {0x00,0x7E,0x42,0x34,0x00,0x7E,0x4A,0x4A}, /* DEh */
  {0x00,0x7E,0x42,0x34,0x00,0x7E,0x0A,0x0A}, /* DFh */
  /* 224 - 255 */
  {0x00,0x7E,0x4A,0x4A,0x00,0x3C,0x42,0x3C}, /* E0h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x44,0x7E,0x40}, /* E1h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x64,0x52,0x4C}, /* E2h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x49,0x49,0x34}, /* E3h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x0E,0x08,0x7F}, /* E4h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x4E,0x4A,0x32}, /* E5h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x7C,0x4A,0x32}, /* E6h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x02,0x72,0x0E}, /* E7h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x3C,0x49,0x3C}, /* E8h *
  {0x00,0x7E,0x4A,0x4A,0x00,0x4C,0x4A,0x3E}, /* E9h */
  {0x00,0x7E,0x4A,0x4A,0x00,0x7C,0x0A,0x7C}, /* EAh */
  {0x00,0x7E,0x4A,0x4A,0x00,0x7E,0x4A,0x34}, /* EBh */
  {0x00,0x7E,0x4A,0x4A,0x00,0x3C,0x42,0x4C}, /* ECh */
  {0x00,0x7E,0x4A,0x4A,0x00,0x7E,0x42,0x34}, /* EDh */
  {0x00,0x7E,0x4A,0x4A,0x00,0x7E,0x4A,0x4A}, /* EEh */
  {0x00,0x7E,0x4A,0x4A,0x00,0x7E,0x0A,0x0A}, /* EFh */
  {0x00,0x7E,0x0A,0x0A,0x00,0x3C,0x42,0x3C}, /* F0h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x44,0x7E,0x40}, /* F1h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x64,0x52,0x4C}, /* F2h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x49,0x49,0x34}, /* F3h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x0E,0x08,0x7F}, /* F4h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x4E,0x4A,0x32}, /* F5h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x7C,0x4A,0x32}, /* F6h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x02,0x72,0x0E}, /* F7h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x3C,0x49,0x3C}, /* F8h *
  {0x00,0x7E,0x0A,0x0A,0x00,0x4C,0x4A,0x3E}, /* F9h */
  {0x00,0x7E,0x0A,0x0A,0x00,0x7C,0x0A,0x7C}, /* FAh */
  {0x00,0x7E,0x0A,0x0A,0x00,0x7E,0x4A,0x34}, /* FBh */
  {0x00,0x7E,0x0A,0x0A,0x00,0x3C,0x42,0x4C}, /* FCh */
  {0x00,0x7E,0x0A,0x0A,0x00,0x7E,0x42,0x34}, /* FDh */
  {0x00,0x7E,0x0A,0x0A,0x00,0x7E,0x4A,0x4A}, /* FEh */
  {0x00,0x7E,0x0A,0x0A,0x00,0x7E,0x0A,0x0A}  /* FFh */
};

#endif