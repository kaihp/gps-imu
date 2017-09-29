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

/* SSD1306 I2C address */
#define RA_SSD1306       (0x3C)

#define SSD_LCD_WIDTH  (128)
#define SSD_LCD_HEIGHT (32)

void ssd130x_power(int fd, int on);
void ssd130x_reset(int fd);
void ssd130x_init(int fd);
void ssd_disp_awake(int fd, int awake);
void ssd_disp_init(int fd, int c);
void ssd_plot(int16_t x, int16_t y, int col);
void ssd_coor(uint8_t row, uint8_t col);
void ssd_putc(char ch);
void ssd_puts(const char *str);

#endif

/* ************************************************************
 * End of file.
 * ************************************************************/
