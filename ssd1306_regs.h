#ifndef __SSD1306_REGS_H__
#define __SSD1306_REGS_H__

/* Solomon Systech SSD1306 128 x 64 / 32 Dot Matrix OLED/PLED
   Segment/Common Driver with Controller */

/* SSD1306 I2C address */
#define RA_SSD1306       (0x3C)
/*calibration parameters */
#define SSD_DISP_START (0x40)
#define SSD_MUX        (0xA8)
#define SSD_CONT_CFG   (0x81)
#define SSD_CHRG_PUMP  (0x8D)
#define SSD_SEG_REMAP  (0xA1)
#define SSD_DISP_ONOF  (0xA4)
#define SSD_DISP_NORM  (0xA6)
#define SSD_DISP_OFF   (0xAE)
#define SSD_DISP_ON    (0xAF)
#define SSD_COM_DIR    (0xC8)
#define SSD_DISP_OFF   (0xD3)
#define SSD_DSP_CLK    (0xD5)
#define SSD_PRECHRG    (0xD9)
#define SSD_COM_CONF   (0xDA)
#define SSD_VCOMH_LVL  (0xDB)

#define SSD_CONTRAST     (0x81)
#define SSD_DISP_ENT_ON  (0xA4)
#define SSD_DISP_ENT_OFF (0xA5)
#define SSD_DISP_NORM    (0xA6)
#define SSD_DISP_INV     (0xA7)
#define SSD_DISP_SLEEP   (0xAE)
#define SSD_DISP_AWAKE   (0xAF)
#define SSD_H_SCROLL_R   (0x26)
#define SSD_H_SCROLL_L   (0x27)
#define SSD_VH_SCROLL_R  (0x29)
#define SSD_VH_SCROLL_L  (0x2A)
#define SSD_SCROLL_OFF   (0x2E)
#define SSD_SCROLL_ON    (0x2F)
#define SSD_VSCROLL_AREA (0xA3)
#define SSD_COL_PAGE_LO  (0x00)
#define SSD_COL_PAGE_HI  (0x10)
#define SSD_MEMADDR_MODE (0x20)
#define SSD_COL_ADDR     (0x21)
#define SSD_PAGE_ADDR    (0x22)
#define SSD_PAGE_START   (0xB0)
#define SSD_DISP_ST_LINE (0x40)
#define SSD_SEG_REMAP    (0xA0)
#define SSD_MUX_RATIO    (0xA8)
#define SSD_SCAN_DIR_NOR (0xC0)
#define SSD_SCAN_DIR_REM (0xC8)
#define SSD_DISP_OFFSET  (0xD3)
#define SSD_PIN_CFG      (0xDA)
#define SSD_DCLK_DIV     (0xD5)
#define SSD_PCHRG_PER    (0xD9)
#define SSD_VCOM_LVL     (0xDB)
#define SSD_NOP          (0xE3)


#endif
