#include <stdint.h>

typedef struct font {
  char *name; /* font name */
  int x, y; /* size of each glyph in px */
  int first, last; /* first & last glyph in glypharr */
  uint8_t *glyphs; /* ptr to arr of size (last-first+1)*x*((y+7)>>3) */
} font_t;
