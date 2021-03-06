/*
 * Solomon Systec Products SSD1306 128x64/32 Dot Matric
 * OLED/PLED Segment/Common Driver Controller
 * http://www.solomon-systech.com/
 *
 * Driver for Raspberry Pi 3 written by
 * Kai Harrekilde-Petersen (C) 2017
 *
 * 0.91" 128x32 module bought off Aliexpress.com
 * Connect VCC to 5V as the module has internal 3.3V LDO (Torex XC6206)
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "i2c.h"
#include "fonts.h"
#include "ssd1306.h"
#include "ssd1306_regs.h"

#define VERSION "0.0.4"

static int __width, __height;
uint8_t *gfxbuf = NULL; /* Graphics buffer */
int ssd_x, ssd_y; /* coords inside OLED display. (0,0) is the Upper/Left corner */
font_t *ssd_font;

void ssd130x_power(int fd, int on)
{
  /* Powering up not controlled in SW */
  return;
}

void ssd130x_reset(int fd)
{
  /* No-op */
  return;
}

void ssd_cmd1(int fd, uint8_t c0)
{
  i2c_wr8(fd, SSD_C, c0);
}

void ssd_cmd2(int fd, uint8_t c0, uint8_t c1)
{
  i2c_wr16(fd, SSD_C, (c1<<8) | c0);
}

void ssd_cmd3(int fd, uint8_t c0, uint8_t c1, uint8_t c2)
{
  uint8_t cmd[3] = {c0, c1, c2};
  i2c_wr_blk(fd, SSD_C, 3, (uint8_t *)&cmd);
}

void ssd_cmd_blk(int fd, int len, uint8_t *cmd)
{
  i2c_wr_blk(fd, SSD_C, len, cmd);
}

void ssd_dat1(int fd, uint8_t d0)
{
  i2c_wr8(fd, SSD_D, d0);
}

void ssd_dat2(int fd, uint8_t d0, uint8_t d1)
{
  i2c_wr16(fd, SSD_D, (d1 << 8) | d0);
}

void ssd_dat_blk(int fd, int len, uint8_t *data)
{
  i2c_wr_blk(fd, SSD_D, len, data);
}

void ssd130x_init(int fd, int w, int h, uint8_t dclk, uint8_t c, uint8_t pc)
{
  if(w!=128 || (h!=32 && h!=64)) {
    exit(-1);
  }
  __width=w;
  __height=h;
/*

 * Software initialization must NOT be done according to <SSD1306.pdf,
 * pg64>, as it doesn't work (trust me on this one).
 * Instead use the sequence from
 * <https://github.com/galpavlin/STM32-SSD1306/128x32/src/oled.c>:
 *  1) Set Display OFF (0xAE)
 *  2) Set Display Clock (0xD5, 0x80)
 *  3) Set MUX ratio (0xA8, h-1) [x32=0x1F, x64=0x3F]
 *  4) Set Display offset (0xD3, 0x00)
 *  5) Set Display Start Line (0x40)
 *  6) Set Charge-pump Internal (0x8D, 0x14)
 *  7) Addressing Mode = PAGE (0x20)
 *  8) Set Segment re-map (0xA1)
 *  9) Set COM Output Scan Dir reversed (0xC8)
 * 10) Set COM pin HW config (0xDA, 0x02/0x12) [x32=0x02, x64=0x12]
 * 11) Set Contrast Control (0x81, 0x8F) [x32=0x8F, x64=0xCF]
 * 12) Set precharge period (0xD9, 0xF1)
 * 13) Set VCOMH deselect level (0xDB, 0x40)
 * 14) Set Entire display on (0xA4)
 * 15) Set Display Normal (0xA6)
 * 16) Set Scrolling = OFF (0x2E)
 * 17) Set Display ON (0xAF)
 * 18) /Clear Screen/
 */
  ssd_cmd2(fd, SSD_CMD_LOCK, 0x12);    /* 0xFD: LOCK = unlock (ssd1309 only, but doesn't harm ssd1306) */
  ssd_cmd1(fd, SSD_DISP_SLEEP);        /* 0xAE: DISP_ENTIRE_OFF */
  ssd_cmd2(fd, SSD_DCLK_DIV, dclk);    /* 0xD5: DCLK_DIV = 0x80 */
  ssd_cmd2(fd, SSD_MUX_RATIO, h-1);    /* 0xA8: MUX_RATIO = h-1 (x32=0x1F, x64=0x3F) */
  ssd_cmd2(fd, SSD_DISP_OFFSET, 0);    /* 0xD3: DISP_OFFSET = 0 */
  ssd_cmd1(fd, SSD_DISP_ST_LINE | 0);  /* 0x40: DISP_STARTLINE = 0 */
  ssd_cmd2(fd, SSD_CHARGEPUMP, 0x14);  /* 0x8D: CHARGEPUMP = 0x14 */
  ssd_cmd2(fd, SSD_ADDR_MODE, 0);      /* 0x20: ADDR_MODE = 0h */
  ssd_cmd1(fd, SSD_SEG_REMAP127);      /* 0xA1: SEG_REMAP = Y */
  ssd_cmd1(fd, SSD_COM_SCAN_REV);      /* 0xC8: COM_SCAN = Reverse (7..0) */
  ssd_cmd2(fd, SSD_COM_HW_CFG, (h==32) ? 0x02 : 0x12); /* 0xDA: COM_HW PINS = 0x02 / 0x12 */
  ssd_cmd2(fd, SSD_CONTRAST, c);   /* 0x81: CONTRAST = 0x8F */
  ssd_cmd2(fd, SSD_PRECHARGE, pc);     /* 0xD9: PRECHARGE = 0xF1 */
  ssd_cmd2(fd, SSD_VCOM_LVL, 0x40);    /* 0xDB: VCOMH LEVEL = 0x40 */
  ssd_cmd1(fd, SSD_DISP_ENT_NORM);     /* 0xA4: DISP_ENTIRE_RESUME */
  ssd_cmd1(fd, SSD_DISP_NORM);         /* 0xA6: DISP_NORMAL */
  ssd_cmd1(fd, SSD_SCROLL_OFF);        /* 0x2E: SCROLL = DEACTIVATE */
  ssd_cmd1(fd, SSD_DISP_AWAKE);        /* 0xAF: DISP = ON */

  /* If not allocated, allocate gfx buffer; else: zero contents */
  if(NULL==gfxbuf) {
    gfxbuf = calloc((size_t) w*h/8, sizeof(uint8_t));
    if(NULL==gfxbuf) {
      puts("Unable to allocate memory for graphics buffer");
      exit(-1);
    }
  } else {
    memset(gfxbuf, 0, (size_t) w*h/8);
  }
  /* Update GDDRAM from gfxbuf */
  ssd_disp_update(fd);
}

void ssd_disp_awake(int fd, int on)
{
  if(on) {
    i2c_wr8(fd, SSD_C, SSD_DISP_AWAKE);
  } else {
    i2c_wr8(fd, SSD_C, SSD_DISP_SLEEP);
  }
}

void ssd_disp_update(int fd)
{
  int i;
  int len=I2C_BLK_MAX;

  /* Tell which area we want to update */
  ssd_cmd3(fd, SSD_ADDR_COL, 0, __width-1);
  ssd_cmd3(fd, SSD_ADDR_PAGE, 0, (__height/8)-1);
  /* Copy from gfxbuf to display */
  for(i=0;i<__width*__height/8;i+=len) {
    ssd_dat_blk(fd, len, (uint8_t *) &gfxbuf[i]);
  }
}

void disp_init_triangle()
{
  static uint8_t img[128*32/8] = {
    /* Row 0-7, col 0-127, bit0 = topmost row, bit7 = lowest row */
    0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01,
    /* Row 8-15, col 0-127 */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Row 16-23, col 0-127 */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Row 24-31, col 0-127 */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  memcpy(gfxbuf,&img,sizeof(img)); /* dst, src, sz */
}

void disp_init_adafruit()
{
  static uint8_t adafruit[128*32/8] = {
    /* Row 0-7, col 0-127 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Row 8-15, col 0-127 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
    0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
    0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Row 16-23, col 0-127 */
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
    0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
    0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
    0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
    0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
    0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
    0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* Row 24-31, col 0-127 */
    0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
    0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
    0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
    0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(gfxbuf,&adafruit,sizeof(adafruit)); /* dst, src, sz */

}

void ssd_disp_clear() {
  memset(gfxbuf, 0, (size_t) ssd_width()*ssd_height()/8);
}

int ssd_width()
{
  return __width;
}

int ssd_height()
{
  return __height;
}

/*
 * The most basic function, set a single pixel
 */
void ssd_plot(uint8_t x, uint8_t y, int color)
{
  if (x<__width && y<__height) {
    int line = y >> 3;
    uint8_t byte = 1 << (y%8);
    if(color==COLOR_BLK) gfxbuf[line*__width+x] &= ~byte;
    if(color==COLOR_WHT) gfxbuf[line*__width+x] |=  byte;
    if(color==COLOR_INV) gfxbuf[line*__width+x] ^= byte;
  }
}

void ssd_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, int color)
{
  uint64_t vmask;
  int x,y, idx;
  uint8_t tmp;

  /* Ensure that x0<=x1 and y0<=y1 */
  if(x0>x1) { tmp=x0; x0=x1; x1=tmp; }
  if(y0>y1) { tmp=y0; y0=y1; y1=tmp; }
  if(x1>=__width || y1>=__height) return;

  vmask = (~(~0ULL << (y1-y0+1))) << (y0 % 8);
  for(y=y0>>3;y<=(y1+7)>>3;y++) {
    idx = x0 + __width*y;
    for(x=x0;x<=x1;x++) {
      tmp = gfxbuf[idx];
      if(color==COLOR_BLK) tmp &= ~vmask;
      if(color==COLOR_WHT) tmp |=  vmask;
      if(color==COLOR_INV) tmp ^=  vmask;
      gfxbuf[idx++] = tmp;
    }
    vmask >>= 8;
  }
}

void ssd_set_xy(uint8_t x, uint8_t y)
{
  ssd_x = x;
  ssd_y = y;
}

void ssd_set_font(const font_t *font)
{
  ssd_font = (font_t *) font;
}

/*
 * Print a single glyph from the default font
 * Note: does NOT move the (x,y) coor
 */
void ssd_putc(char ch)
{
  uint64_t vscan;
  uint64_t vmask;
  uint64_t mask;
  uint8_t *glyphptr;
  int x,y,x_s,x_w;
  int lsl;
  int idx;
  int c;
  if (ch<ssd_font->first || ch>ssd_font->last) {
    /* Character out of range - abort */
    return;
  }
  /* point to the first uint8_t of the chosen character */
  c = ch - ssd_font->first;
  if(ssd_font->ftype == FIXED) {
    idx   = c * ssd_font->x * ((ssd_font->y+7) >> 3);
    x_s = 0;
    x_w = ssd_font->x;
  } else if(ssd_font->ftype == VARIABLE) {
    idx   = c * ssd_font->x * ((ssd_font->y+7) >> 3);
    x_s = ssd_font->offset[c];
    x_w = ssd_font->width[c];
  } else {
    /* Unknown font type; abort */
    return;
  }
  glyphptr = ssd_font->glyphs+idx;
  lsl = ssd_y - ((8 - ssd_font->y % 8) % 8);
  mask = (~(~0ULL << ssd_font->y)) << ssd_y;
  /* For each column, assemble vscan, align it, and "print" it */
  for(x=0;x<x_w;x++) {
    vmask = mask;
    vscan = 0;
    /* assemble complete vertical scan */
    for(y=((ssd_font->y+7)>>3)-1;y>=0;y--) {
      vscan = (vscan << 8) | glyphptr[x+x_s+y*ssd_font->x];
    }
    /* align vscan to match destination uint8_t's */
    vscan = (lsl>=0) ? vscan << lsl : vscan >> -lsl;
    vmask >>= (ssd_y & ~0x7);
    vscan >>= (ssd_y & ~0x7);
    for(y=ssd_y>>3; y<=(ssd_font->y+ssd_y-1)>>3; y++) {
      int idx = __width*y + ssd_x+x;
      uint8_t tmp = gfxbuf[idx];
      uint8_t t0 = tmp;
      tmp &= ~vmask; /* set pixels in glyph area */
      tmp |=  vscan; /* apply glyph */
      vmask >>= 8;
      vscan >>= 8;
      gfxbuf[idx] = tmp;  /* writeback */
    }
  }
}

/*
 * ssd_puts(): C POSIX Library wannabe
 *
 * Returns the numbers of characters printed (ssd_puts() breaks when
 * it cannot print the next complete character.
 *
 * Handles both fixed-pitch and variable-pitch (proportional) fonts.
 * Updates the display (x,y) cursor so puts() can be called repeatedly
 * and get the expected results.
 */
int ssd_puts(const char *str)
{
  int i=0;
  char j;
  int  x_w;
  while(j=str[i]) {
    /* Determine if we can print the glyph here or need to move to next line */
    if(ssd_font->ftype == FIXED) {
      x_w = ssd_font->x;
    } else if(ssd_font->ftype==VARIABLE) {
      x_w = ssd_font->width[j-ssd_font->first];
    } else {
      return(-1);
    }
    /* Height check */
    if((ssd_y + ssd_font->y) > ssd_height()) {
      break;
    }
    /* Width check */
    if((ssd_x + x_w) > ssd_width()) {
      /* If too wide, print glyph on next line (if possible) */
      ssd_x = 0;
      ssd_y += ssd_font->y + ssd_font->vspace;
      if ((ssd_y + ssd_font->y) > ssd_height()) {
	/* Out of display space; return number of chars printed */
	return i;
      }
    }
    /* Both W & H checks fall-through to print char here */
    ssd_putc(str[i]);

    /* Move cursor */
    ssd_x += x_w + ssd_font->hspace;
    if ((ssd_x + x_w) > ssd_width()) {
      ssd_x = 0;
      ssd_y += ssd_font->y + ssd_font->vspace;
    }
    i++;
  }
  return i;
}

/* 
 * ssd_strlen(): Compute length of string in pixels
 * Assumes that string isn't wrapped.
 */
int ssd_strlen(const char *str)
{
  int i=0;
  int px=0;
  char ch;
  while(ch=str[i]) {
    if(ch==0) {
      return px;
    }
    if(ssd_font->ftype==FIXED) {
      px += ssd_font->x;
    } else if(ssd_font->ftype==VARIABLE) {
      px += ssd_font->width[ch-ssd_font->first];
    } else {
      return -1;
    }
    if(str[i+1]!=0) px += ssd_font->hspace;
    i++;
  }
  return px;
}

void ssd130x_scroll_h(int fd, int dir)
{
  if(dir==0) {
    ssd_cmd1(fd, SSD_SCROLL_OFF);
  } else if (dir==1) {
    uint8_t hscroll[] = {SSD_SCROLL_H_R, 0, 0, 0, 0xF, 0, 0xFF};
    ssd_cmd_blk(fd, sizeof(hscroll), (uint8_t *)&hscroll);
    ssd_cmd1(fd, SSD_SCROLL_ON);
  } else if (dir==-11) {
    uint8_t hscroll[] = {SSD_SCROLL_H_L, 0, 0, 0, 0xF, 0, 0xFF};
    ssd_cmd_blk(fd, sizeof(hscroll), (uint8_t *)&hscroll);
    ssd_cmd1(fd, SSD_SCROLL_ON);
  }
}
 
int main (int argc, char **argv)
{
  int height = 64;
  uint8_t dclk=0x80;
  uint8_t c=0xCF;
  uint8_t pc=0xF1;
  if(argc>1) {
    int tmp = atoi(argv[1]);
    if((tmp==64) || (tmp==32)) {
      height=tmp;
    }
  }
  if(argc>2) {
    int tmp = atoi(argv[2]);
    printf("argv[2]=%s / 0x%02X ",argv[2], tmp);
    dclk = (uint8_t) (0xFF & tmp);
    printf("DCLK=0x%02X\n",dclk);
  }
  if(argc>3) {
    int tmp = atoi(argv[3]);
    c = (uint8_t) (0xFF & tmp);
  }
  if(argc>4) {
    int tmp = atoi(argv[3]);
    pc = (uint8_t) (0xFF & tmp);
  }
  int i,j;
  int x,y;
  int disp = i2c_init_dev(RA_SSD1306);
  if(-1==disp) {
    puts("Failed to setup OLED display\n");
    exit(-1);
  }
  
  printf("Initializing 128 x %2d display\n",height);
  ssd130x_init(disp,128,height,dclk,c, pc);
  ssd_disp_update(disp);

#if 0
  /* Font/glyph printing tests with all fonts and all implemented
   * characters
   */
  const font_t *tests1[] = {&fixed_7x5, &fixed_8x8, &fixed_21x14, &font_7px, &font_8px, &font_21px};
  const font_t *tests1[] = {&fixed_21x14};
  for(i=0;i<sizeof(tests1) / sizeof(font_t *);i++) {
    char str[260];
    int len, idx, tmp;
    int ch_x;
    ssd_set_font(tests1[i]);
    x=0;
    y=0;
    len = ssd_font->last-ssd_font->first+1;
    idx = 0;
    tmp = 0;
    for(j=ssd_font->first;j<=ssd_font->last;j++) {
      str[idx++] = j;
    }
    str[idx] = 0;
    if (0==ssd_font->first) str[0]=' ';
    idx = 0;
    while(len>0) {
      ssd_disp_clear();
      ssd_set_xy(0,0);
      tmp = ssd_puts(&str[idx]);
      idx += tmp;
      len -= tmp;
      ssd_disp_update(disp);
      getc(stdin);
    }
  }
#endif

#if 1
  /* Font/glyph printing tests with all fonts and all implemented
   * characters
   */
  const font_t *tests4[] = {&fixed_7x5, &fixed_8x8, &fixed_9x7, &fixed_12x9, &fixed_16x10, &fixed_21x14, &font_7px, &font_8px, &font_9px, &font_12px, &font_16px, &font_21px};
  for(i=0;i<sizeof(tests4) / sizeof(font_t *);i++) {
    char str[260];
    int len, idx, tmp;
    int ch_x;
    ssd_set_font(tests4[i]);
    printf("Setting font %s\n",tests4[i]->name);
    x=0;
    y=0;
    len = ssd_font->last-ssd_font->first+1;
    idx = 0;
    tmp = 0;
    for(j=ssd_font->first;j<=ssd_font->last;j++) {
      str[idx++] = j;
    }
    str[idx] = 0;
    if (0==ssd_font->first) str[0]=' ';
    idx = 0;
    while(len>0) {
      printf("idx=%3d\r",idx);
      ssd_disp_clear();
      ssd_set_xy(0,0);
      tmp = ssd_puts(&str[idx]);
      idx += tmp;
      len -= tmp;
      ssd_disp_update(disp);
      getc(stdin);
    }
  }
#endif

#if 0
  /*
   * Test that we can print glyphs in any row (x)
   */
  const font_t *tests2[] = {&fixed_7x5, &fixed_8x8, &fixed_21x14};
  for(i=0;i<sizeof(tests2)/sizeof(font_t *);i++) {
    ssd_set_font(tests2[i]);
    ssd_disp_clear();
    x = 0;
    y = 0;
    for(j=0;j<ssd_height();j++) {
      ssd_set_xy(x,y);
      ssd_putc(j+'A');
      x += ssd_font->x + ssd_font->hspace;
      if ((x + ssd_font->x) > ssd_width()) {
	x = 0;
	if ((y + ssd_font->y) > ssd_height()) {
	  y = 0;
	  ssd_disp_update(disp);
	  getc(stdin);
	  if (j!=ssd_font->last) ssd_disp_clear();
	}
	y += 1;
      }
      y += 1;
    }
    ssd_disp_update(disp);
    getc(stdin);
  }
#endif

#if 0
  /*
   * Test printing two strings with puts() - should be indistinguishable from a single puts()
   */
  const font_t *tests3[] = {&fixed_7x5, &fixed_8x8, &fixed_21x14, &font_21px};
  const char str1[] = "@ABC";
  const char str2[] = "DEF";
  for(i=0;i<sizeof(tests3)/sizeof(font_t *);i++) {
    ssd_disp_clear();
    ssd_set_font(tests3[i]);
    ssd_disp_clear();
    ssd_set_xy(0,0);
    ssd_puts(str1);
    ssd_puts(str2);
    ssd_disp_update(disp);
    getc(stdin);
  }
#endif
  
#if 0
  /* ssd_rect() testing */
  ssd_set_xy(0,0);
  ssd_set_font(&fixed_7x5);
  char str[] = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
  ssd_puts(str);
  ssd_disp_update(disp);
  getc(stdin);
  ssd_rect(0, 0,127,31,COLOR_INV);
  ssd_rect(0, 7,127, 8,COLOR_BLK);
  ssd_disp_update(disp);
#endif

#if 0
  /* GPS laptimer normal view */
  char status[33];
  char min[3], sec[3], csec[3];
  int laptime;
  int ngps, spd, hh, mm, rec;
  laptime = (1 * 100 + 8)*100 + 86;
  ngps = 0; spd=0; hh=12; mm=42; rec=0;

  ssd_disp_clear();
  ssd_set_font(&fixed_7x5);
  /* Number of Satellites */
  ssd_set_xy(5,0);
  sprintf(status, "S%02d", ngps);
  ssd_puts(status);
  /* Speed in mph/kph */
  ssd_set_xy((5+6+6)+18,0);
  sprintf(status, "%03d", spd);
  ssd_puts(status);
  /* Time of Day */
  ssd_set_xy((5+6+6)+(18+18)+18,0);
  sprintf(status, "%02d:%02d", hh, mm);
  ssd_puts(status);
  /* Recording status (Circle/Dot) */
  ssd_set_font(&fixed_8x8);
  ssd_set_xy(115,0);
  ssd_putc((rec)? 131 : 130);
  /* Laptime MM:SS.SS */
  ssd_set_font(&font_21px);
  ssd_set_xy(5,9);
  sprintf(status, "%02d:%02d.%02d", laptime/10000, (laptime/100) % 100, laptime % 100);
  ssd_puts(status);
  sprintf(status, "%c%02d.%02d", '-', 1, 14);
  int len = ssd_strlen(status);
  ssd_set_xy(ssd_width()-5-len,9+4+21);
  ssd_puts(status);
  ssd_disp_update(disp);
#endif
}

/* ************************************************************
 * End of file.
 * ************************************************************/
