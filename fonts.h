#ifndef __FONTS_H__
#define __FONTS_H__

#include <stdint.h>
#include <stdio.h>

/*
 * name:   font name (not used)
 * x:      glyph width in pixels
 * y:      glyph height in pixels
 * hspace: recommended spacing when placing glyphs next to each other
 *   (e.g. strings)
 * vspace: recommended spacing when placing glyphs above/below to each
 *   other (e.g. two chars)
 * first:  Code for first implemented glyph. If space char is the
 *   first glyph, set start to 32.
 * last:   Code of last implemented glyph.
 * glyphs: array of bitpatterns forming the glyphs.
 *   Glyph bitmaps must be arranged as follows:
 *   1) There must be <x>*roundup(<y>/8) uint8_t's per glyph.
 *   2) Each uint8_t contains an N-bit column of pixels.
 *   3) The first <x> uint8_t's contain the topmost <y>%8 pixels of a
 *      column. Following uint8_t's contain 8 pixels of a row
 *   4) Columns are stored in a left-to-right sequence.
 *   EXAMPLE: x=4, y=12. first=last=0. There will be 4 * 2 = 8 uint8_t
 *   to represent this glyph.
 *   glyphs[] = {0xF0, 0x80, 0x00, 0x00,
 *               0xFF, 0xFF, 0xFC, 0x80};
 *   Image will show a triangle starting with 12 pixels set in the
 *   leftmost column, dropping to 9, 6, and finally 3 pixels set in
 *   the final column.
 *
 * This (very) unusual layout is dictated by how data is stored in the
 * SSD1306 controllers display buffer (the GDDRAM).
 */

enum ftypes {
  FIXED    = 0, /* fixed-spacing */
  VARIABLE = 1  /* proportional-spacing */
};

typedef struct font {
  char *name; /* font name */
  enum ftypes ftype;
  int x, y;           /* size of each glyph in px */
  int hspace, vspace; /* recommended spacing when placing two glyphs next to each other */
  int first, last;    /* ASCII code of first & last glyph in glyph array */
  uint8_t *offset;    /* Prop font: start of glyph */
  uint8_t *width;     /* Prop font: width of glyph */
  uint8_t *glyphs;    /* ptr to arr of vscans size (last-first+1)*x*((y+7)>>3) */
} font_t;

extern const font_t fixed_7x5;
extern const font_t font_7px;
extern const font_t fixed_8x8;
extern const font_t font_8px;
extern const font_t fixed_21x14;
extern const font_t font_21px;

#endif
