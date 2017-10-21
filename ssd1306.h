/*
 * Solomon Systec Products SSD1306 128x64/32 Dot Matric
 * OLED/PLED Segment/Common Driver Controller
 * http://www.solomon-systech.com/
 *
 * Driver for Raspberry Pi 3 written by
 * Kai Harrekilde-Petersen (C) 2017
 *
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "fonts.h"

/* SSD1306 I2C address */
#define RA_SSD1306       (0x3C)
#define RA_SSD1309       (0x3C)

#define COLOR_BLK (0)
#define COLOR_WHT (1)
#define COLOR_INV (2)

void ssd130x_power(int fd, int on);
void ssd130x_reset(int fd);
void ssd130x_init(int fd, int w, int h);
void ssd_disp_clear();
void ssd_disp_awake(int fd, int awake);
void ssd_disp_update(int fd);
void ssd_disp_init(void);
int  ssd_width();
int  ssd_height();
void ssd_plot(uint8_t x, uint8_t y, int color);
void ssd_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, int color);
void ssd_set_xy(uint8_t x, uint8_t y);
void ssd_set_font(const font_t *font);
void ssd_putc(char ch);
int  ssd_puts(const char *str);
int  ssd_strlen(const char *str);

#endif

/* ************************************************************
 * End of file.
 * ************************************************************/
