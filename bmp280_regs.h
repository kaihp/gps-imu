#ifndef __BMP280_REGS_H__
#define __BMP280_REGS_H__

/* BMP-280 I2c address */
#define RA_BMP (0x76)
/*calibration parameters */
#define BMP_CAL_DIG_T1_LSB (0x88)
#define BMP_CAL_DIG_T1_MSB (0x89)
#define BMP_CAL_DIG_T2_LSB (0x8A)
#define BMP_CAL_DIG_T2_MSB (0x8B)
#define BMP_CAL_DIG_T3_LSB (0x8C)
#define BMP_CAL_DIG_T3_MSB (0x8D)
#define BMP_CAL_DIG_P1_LSB (0x8E)
#define BMP_CAL_DIG_P1_MSB (0x8F)
#define BMP_CAL_DIG_P2_LSB (0x90)
#define BMP_CAL_DIG_P2_MSB (0x91)
#define BMP_CAL_DIG_P3_LSB (0x92)
#define BMP_CAL_DIG_P3_MSB (0x93)
#define BMP_CAL_DIG_P4_LSB (0x94)
#define BMP_CAL_DIG_P4_MSB (0x95)
#define BMP_CAL_DIG_P5_LSB (0x96)
#define BMP_CAL_DIG_P5_MSB (0x97)
#define BMP_CAL_DIG_P6_LSB (0x98)
#define BMP_CAL_DIG_P6_MSB (0x99)
#define BMP_CAL_DIG_P7_LSB (0x9A)
#define BMP_CAL_DIG_P7_MSB (0x9B)
#define BMP_CAL_DIG_P8_LSB (0x9C)
#define BMP_CAL_DIG_P8_MSB (0x9D)
#define BMP_CAL_DIG_P9_LSB (0x9E)
#define BMP_CAL_DIG_P9_MSB (0x9F)
/* BMP280 Register Map (cf BST-BMP280-DS001-18, pg 24) */
#define BMP_CHIP_ID        (0xD0)
#define BMP_RESET          (0xE0)
#define BMP_STATUS         (0xF3)
#define BMP_CTRL_MEAS      (0xF4)
#define BMP_CONFIG         (0xF5)
#define BMP_PRESS_MSB      (0xF7)
#define BMP_PRESS_LSB      (0xF8)
#define BMP_PRESS_XLSB     (0xF9)
#define BMP_TEMP_MSB       (0xFA)
#define BMP_TEMP_LSB       (0xFB)
#define BMP_TEMP_XLSB      (0xFC)


#endif
