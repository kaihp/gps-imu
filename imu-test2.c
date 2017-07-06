#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <wiringPi.h>
#include "mpu9250_regs.h"
#include "bmp280_regs.h"

/* HW-types of size 8, 16, 32, and 64 bits */
typedef int8_t   s8_t;
typedef uint8_t  u8_t;
typedef int16_t  s16_t;
typedef uint16_t u16_t;
typedef int32_t  s32_t;
typedef uint32_t u32_t;
typedef int64_t  u64_t;
typedef uint64_t s64_t;

typedef struct {
  u16_t x,y,z;
} coor_t;

typedef struct {
  u8_t h;
  u8_t l;
} hl_t;

typedef union {
  hl_t b;
  u16_t u;
  s16_t s;
} coorhl_t;

typedef struct {
  u8_t msb, lsb, xsb;
} bmp280_raw_t;

typedef union {
  bmp280_raw_t b;
  u32_t w;
} bmp280_data_t;

/* Calibration data, stored on-chip in NV memory */
typedef struct {
  u16_t dig_T1;
  s16_t dig_T2, dig_T3;
  u16_t dig_P1;
  s16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_cal_t;

int mpu, bmp; /* FD's for wiringPi*() functions */
bmp280_cal_t bmp_cal;
s32_t bmp_temp_fine;

#define i2c_wr8(fd,reg,data) wiringPiI2CWriteReg8(fd,reg,data)
#define i2c_wr16(fd,reg,data) wiringPiI2CWriteReg16(fd,reg,data)

#define i2c_rd8(fd,reg) wiringPiI2CReadReg8(fd,reg)
#define i2c_rd16(fd,reg) wiringPiI2CReadReg16(fd,reg)

__inline__ u16_t byteswap(u16_t data)
{
  coorhl_t tmp;
  u8_t h,l;
  tmp.u = data;
  h = tmp.b.h;
  l = tmp.b.l;
  tmp.b.l = h;
  tmp.b.h = l;
  return tmp.u;
}

u16_t mpu_rd16(int fd, int reg)
{
  int res = wiringPiI2CReadReg16(fd,reg);
  if(res<0) {
    perror(strerror(res));
    exit(res);
  }
  /* due to endianness, swap H & L bytes - the stupid way */
  return byteswap((u16_t) res);
}

float to_float(int res, u16_t raw)
{
  s16_t raw_s = (s16_t) raw;
  return (float) (1.0*res*raw_s)/32768.0;
}


#define GRAV_RES (8)
#define GYRO_RES (1000)

void mpu9250_init(void)
{
  i2c_wr8(mpu,MPU_ACCEL_CONFIG,2<<3); /* ACCEL_FS_SEL=0b10 (+/-8g) */
  i2c_wr8(mpu,MPU_GYRO_CONFIG,2<<3);  /* GYRO_FS_SEL=0b10  (+/-1000dps) */

  return;
}

void bmp280_init(void)
{
  int res;
  res = i2c_rd8(bmp,BMP_CHIP_ID);
  if(res==-1) {
    puts("Unable to talk to BMP280 sensor.");
    exit(-1);
  }
  if(res!=0x58) {
    puts("Unable to recognize BMP280 sensor.");
    exit(-1);
  }

  /* In order to initialize, set mode to NORMAL, with 1x oversampling (ULP mode) */
  i2c_wr8(bmp,BMP_CTRL_MEAS,(1<<5)|(1<<2)|(3<<0)); /* osrs_p=1x, osrs_t=1x, mode=NORMAL */
  /* Set standby duration = 0.5ms, no IIR filter, SPI /not/ enabled */
  i2c_wr8(bmp,BMP_CONFIG,(0<<5)|(0<<2)|(0<<0));
  /* Fetch calibration values from NVmem */
  bmp_cal.dig_T1 = i2c_rd16(bmp,BMP_CAL_DIG_T1_LSB);
  bmp_cal.dig_T2 = i2c_rd16(bmp,BMP_CAL_DIG_T2_LSB);
  bmp_cal.dig_T3 = i2c_rd16(bmp,BMP_CAL_DIG_T3_LSB);
  bmp_cal.dig_P1 = i2c_rd16(bmp,BMP_CAL_DIG_P1_LSB);
  bmp_cal.dig_P2 = i2c_rd16(bmp,BMP_CAL_DIG_P2_LSB);
  bmp_cal.dig_P3 = i2c_rd16(bmp,BMP_CAL_DIG_P3_LSB);
  bmp_cal.dig_P4 = i2c_rd16(bmp,BMP_CAL_DIG_P4_LSB);
  bmp_cal.dig_P5 = i2c_rd16(bmp,BMP_CAL_DIG_P5_LSB);
  bmp_cal.dig_P6 = i2c_rd16(bmp,BMP_CAL_DIG_P6_LSB);
  bmp_cal.dig_P7 = i2c_rd16(bmp,BMP_CAL_DIG_P7_LSB);
  bmp_cal.dig_P8 = i2c_rd16(bmp,BMP_CAL_DIG_P8_LSB);
  bmp_cal.dig_P9 = i2c_rd16(bmp,BMP_CAL_DIG_P9_LSB);
  return;
}

void bmp_cal_ref_load(void)
{
  bmp_cal.dig_T1 =  27504;
  bmp_cal.dig_T2 =  26435;
  bmp_cal.dig_T3 =  -1000;
  bmp_cal.dig_P1 =  36477;
  bmp_cal.dig_P2 = -10685;
  bmp_cal.dig_P3 =   3024;
  bmp_cal.dig_P4 =   2855;
  bmp_cal.dig_P5 =    140;
  bmp_cal.dig_P6 =     -7;
  bmp_cal.dig_P7 =  15500;
  bmp_cal.dig_P8 = -14600;
  bmp_cal.dig_P9 =   6000;
  return;
}
void bmp_cal_dump(void)
{
  printf("dig_T1=%04x dig_T2=%04x dig_T3=%04x\n",(u16_t) bmp_cal.dig_T1, (u16_t) bmp_cal.dig_T2, (u16_t) bmp_cal.dig_T3);
  printf("dig_P1=%04x dig_P2=%04x dig_P3=%04x\n",(u16_t) bmp_cal.dig_P1, (u16_t) bmp_cal.dig_P2, (u16_t) bmp_cal.dig_P3);
  printf("dig_P4=%04x dig_P5=%04x dig_P6=%04x\n",(u16_t) bmp_cal.dig_P4, (u16_t) bmp_cal.dig_P5, (u16_t) bmp_cal.dig_P6);
  printf("dig_P7=%04x dig_P8=%04x dig_P9=%04x\n",(u16_t) bmp_cal.dig_P7, (u16_t) bmp_cal.dig_P8, (u16_t) bmp_cal.dig_P9);

  return;
}

u32_t bmp_rd_raw(u8_t reg)
{
  bmp280_data_t tmp = {.w = 0};
  tmp.w = (byteswap(i2c_rd16(bmp,reg)) << 8) | (i2c_rd8(bmp,reg+2));
  return tmp.w;
}

/* Returns temperature in 0.01 degrees Centigrade */
s32_t bmp_rd_temp(s32_t adc_t)
{
  /* Code from BST-BMP280-DS001-187, page 22 */
  s32_t var1, var2;
  var1 =  ((((adc_t >> 3) - ((s32_t) bmp_cal.dig_T1 << 1))) * ((s32_t) bmp_cal.dig_T2)) >> 11;
  var2 = (((((adc_t >> 4) - ((s32_t) bmp_cal.dig_T1)) * ((adc_t>>4) - ((s32_t) bmp_cal.dig_T1))) >> 12) * ((s32_t) bmp_cal.dig_T3)) >> 14;
  bmp_temp_fine = var1+var2;
  return (bmp_temp_fine*5 + 128) >> 8;
}

/* Returns absolute Pressure in Pascal (Pa) */
u32_t bmp_rd_pres(u32_t adc_p)
{
  /* 64bit Fixed-point version, using code on pg 22 */
  /* NB: returns value in Q24.8 format; user must divide by 256 to get Pascal */
  s64_t var1 = 0;
  s64_t var2 = 0;
  s64_t p = 0;
  var1 = ((s64_t)bmp_temp_fine) - 128000;
  var2 = var1 * var1 * (s64_t)bmp_cal.dig_P6;
  var2 = var2 + ((var1 * (s64_t)bmp_cal.dig_P5) << 17);
  var2 = var2 + (((s64_t)bmp_cal.dig_P4) << 35);
  var1 = ((var1 * var1 * (s64_t)bmp_cal.dig_P3) >> 8) + ((var1 * (s64_t)bmp_cal.dig_P2) << 12);
  var1 = (((1LL << 47) + var1) * ((s64_t)bmp_cal.dig_P1)) >> 33;
  if (var1 == 0) {
    return 0;
  }
  p = (1<<20) - adc_p;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((s64_t)bmp_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((s64_t)bmp_cal.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((s64_t)bmp_cal.dig_P7) << 4);
  return (u32_t) p;
}

/* Takes pressure in Q24.8 format (from bmp_rd_pres()) and returns computed altitude */
/* presssure/altitude formula from http://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html */
/* Pressure = 101325*(1-2.25577e-5*altitude)**5.25588 */
/* https://en.wikipedia.org/wiki/Pressure_altitude says:*/
/* Altitude [m] = (1-(pressure/1013.25)**0.190284)*476924.05 [P in hPa]. 1013.25 is the 'standard 1atm' pressure */
s32_t pres2alt(u32_t pres)
{
  double tmp = ((double) pres)/256.0;
  tmp = pow(1.0-tmp/1013.25,0.190284)*476924.05;
  return (s32_t) (tmp*256);
}


int main (void)
{
  int i;
  wiringPiSetup();
  mpu=wiringPiI2CSetup(RA_MPU);
  bmp=wiringPiI2CSetup(RA_BMP);
  if(mpu==-1) {
    puts("Failed to setup the MPU device\n");
    exit(-1);
  }
  if(bmp==-1) {
    puts("Failed to setup the BMP device\n");
    exit(-1);
  }

  mpu9250_init();
  bmp280_init();
#if defined(BMP) && defined(DEBUG)
  /*  bmp_cal_ref_load(); */
  bmp_cal_dump();
#endif

  coor_t accel_offset, gyro_offset;

  i=0;
  do {
    coor_t accel = {0,0,0}, gyro = {0,0,0};
    s32_t raw_temp = 0, temp = 0;
    u32_t raw_pres = 0, pres = 0;

#if defined(ACC)
    accel.x = mpu_rd16(mpu,MPU_ACCEL_XOUT_H);
    accel.y = mpu_rd16(mpu,MPU_ACCEL_YOUT_H);
    accel.z = mpu_rd16(mpu,MPU_ACCEL_ZOUT_H);
#endif

#if defined(GYRO)
    gyro.x  = mpu_rd16(mpu,MPU_GYRO_XOUT_H);
    gyro.y  = mpu_rd16(mpu,MPU_GYRO_YOUT_H);
    gyro.z  = mpu_rd16(mpu,MPU_GYRO_ZOUT_H);
#endif

#if defined(BMP)
    raw_temp = bmp_rd_raw(BMP_TEMP_MSB) >> 4;
    raw_pres = (s32_t) (bmp_rd_raw(BMP_PRESS_MSB) >> 4);
    temp = bmp_rd_temp(raw_temp);
    pres = bmp_rd_pres(raw_pres);
#endif

#ifdef ACC
    printf("Accel: %+5.2f/%+5.2f/%+5.2f ",
	   to_float(GRAV_RES,accel.x), to_float(GRAV_RES,accel.y), to_float(GRAV_RES,accel.z));
# ifdef DEBUG
    printf("%04x/%04x/%04x ", accel.x, accel.y, accel.z);
# endif
#endif

#ifdef GYRO
    printf("Gyro: %+7.2f/%+7.2f/%+7.2f ",
	   to_float(GYRO_RES,gyro.x), to_float(GYRO_RES,gyro.y), to_float(GYRO_RES,gyro.z));
# ifdef DEBUG
    printf("%04x/%04x/%04x ", gyro.x, gyro.y, gyro.z);
# endif
#endif

#ifdef BMP
    printf("Temp: %+7.2f Pres: %7.3f ", temp/100.0, pres/25600.0);
# ifdef DEBUG
    printf("%06x %06x ", (u32_t) raw_temp, raw_pres);
# endif
#endif
    putchar('\r');
  } while(i<10);
}
