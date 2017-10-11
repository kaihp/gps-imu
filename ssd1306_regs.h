/*
 * Solomon Systec Products SSD1306 128x64/32 Dot Matric
 * OLED/PLED Segment/Common Driver Controller
 * http://www.solomon-systech.com/
 *
 * Driver for Raspberry Pi 3 written by
 * Kai Harrekilde-Petersen (C) 2017
 *
 */
#ifndef __SSD1306_REGS_H__
#define __SSD1306_REGS_H__
#include <stdint.h>

#include "ssd1306.h"

/* Co D/C# */
/* For some reason, 0x80 as Co doesn't work */
#define SSD_C  (0x00)
#define SSD_D  (0x40)
#define SSD_CD (0xC0)

/* Registers, ordered by address */
#define SSD_COL_PAGE_LO   (0x00)
#define SSD_COL_PAGE_HI   (0x10)
#define SSD_ADDR_MODE     (0x20)
#define ADDR_MODE_HORIZ   (0x00)
#define ADDR_MODE_VERT    (0x01)
#define ADDR_MODE_PAGE    (0x02)
#define SSD_ADDR_COL      (0x21)
#define SSD_ADDR_PAGE     (0x22)
#define SSD_SCROLL_H_R    (0x26)
#define SSD_SCROLL_H_L    (0x27)
#define SSD_SCROLL_VH_R   (0x29)
#define SSD_SCROLL_VH_L   (0x2A)
#define SSD_SCROLL_OFF    (0x2E)
#define SSD_SCROLL_ON     (0x2F)
#define SSD_DISP_ST_LINE  (0x40)
#define SSD_CONTRAST      (0x81)
#define SSD_CHARGEPUMP    (0x8D)
#define SSD_SEG_REMAP0    (0xA0)
#define SSD_SEG_REMAP127  (0xA1)
#define SSD_SCROLL_V_AREA (0xA3)
#define SSD_DISP_ENT_NORM (0xA4)
#define SSD_DISP_ENT_ON   (0xA5)
#define SSD_DISP_NORM     (0xA6)
#define SSD_DISP_INV      (0xA7)
#define SSD_MUX_RATIO     (0xA8)
#define SSD_DISP_SLEEP    (0xAE)
#define SSD_DISP_AWAKE    (0xAF)
#define SSD_PAGE_START    (0xB0)
#define SSD_COM_SCAN_NORM (0xC0)
#define SSD_COM_SCAN_REV  (0xC8)
#define SSD_DISP_OFFSET   (0xD3)
#define SSD_DCLK_DIV      (0xD5)
#define SSD_PRECHARGE     (0xD9)
#define SSD_COM_HW_CFG    (0xDA)
#define SSD_VCOM_LVL      (0xDB)
#define SSD_NOP           (0xE3)

#endif

/* ************************************************************
 * End of file.
 * ************************************************************/
