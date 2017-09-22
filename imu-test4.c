#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "i2c.h"

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

typedef struct {
  bmp280_raw_t pres;
  bmp280_raw_t temp;
} bmp280_pres_temp_t;

typedef union {
  bmp280_raw_t b;
  u32_t w;
} bmp280_data_t;

/* Calibration data, stored on-chip in NV memory, addr 0x88-0x9f */
typedef struct {
  u16_t dig_T1;
  s16_t dig_T2, dig_T3;
  u16_t dig_P1;
  s16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_cal_t;

int mpu, bmp; /* FD's for i2c functions */
bmp280_cal_t bmp_cal;
s32_t bmp_temp_fine;


static inline u16_t byteswap(u16_t data)
{
  return (data >> 8) | (0xFF & data) << 8;
}

u16_t mpu_rd16(int fd, int reg)
{
  int res = i2c_rd16(fd,reg);
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
  int res;
  res = i2c_rd8(mpu,MPU_WHO_AM_I);
  if(res<0) {
    puts("Unable to talk to MPU9250 sensor (bad WHOAMI value).");
    exit(-1);
  }
  if(res!=MPU_WHOAMI_MAGIC) {
    puts("Unable to recognize MPU9250 sensor (bad MAGIC value).");
    exit(-1);
  }
    
  i2c_wr8(mpu,MPU_ACCEL_CONFIG,2<<3); /* ACCEL_FS_SEL=0b10 (+/-8g) */
  i2c_wr8(mpu,MPU_GYRO_CONFIG,2<<3);  /* GYRO_FS_SEL=0b10  (+/-1000dps) */

  return;
}

void bmp280_init(void)
{
  int res;
  res = i2c_rd8(bmp,BMP_CHIP_ID);
  if(res<0) {
    puts("Unable to talk to BMP280 sensor (bad CHIPID value).");
    exit(-1);
  }
  if(res!=BMP_CHIPID_MAGIC) {
    puts("Unable to recognize BMP280 sensor (bad MAGIC value).");
    exit(-1);
  }

  /* In order to initialize, set mode to NORMAL, with 1x oversampling (ULP mode) */
  i2c_wr8(bmp,BMP_CTRL_MEAS,(1<<5)|(1<<2)|(3<<0)); /* [0xF4]=0x27, osrs_p=1x, osrs_t=1x, mode=NORMAL */
  /* Set standby duration = 0.5ms, no IIR filter, SPI /not/ enabled */
  i2c_wr8(bmp,BMP_CONFIG,(0<<5)|(0<<2)|(0<<0)); /* [0xF5]=0x00 */
  /* Fetch calibration values from NVmem */
#define CAL_SZ (1+BMP_CAL_DIG_P9_MSB-BMP_CAL_DIG_T1_LSB)
  u8_t buf[1+CAL_SZ]; /* Add 1 byte for size in [0] */
  buf[0] = CAL_SZ; /* how many bytes to (burst) read */
  res =  i2c_rd_blk(bmp,BMP_CAL_DIG_T1_LSB,buf[0],(u8_t *)&buf);
  if((res != CAL_SZ) || (buf[0] != CAL_SZ)){
    puts("Unable to read calibration values in a BLOCK_READ");
    printf("Expected %d bytes, got %d.\n",CAL_SZ, buf[0]);
    exit(-1);
  }
  /* buf[0] contains the amount of bytes read; so skip that */
  bmp_cal.dig_T1 = (u16_t) (buf[ 2] <<  8 | buf[ 1]);
  bmp_cal.dig_T2 = (s16_t) (buf[ 4] <<  8 | buf[ 3]);
  bmp_cal.dig_T3 = (s16_t) (buf[ 6] <<  8 | buf[ 5]);
  bmp_cal.dig_P1 = (u16_t) (buf[ 8] <<  8 | buf[ 7]);
  bmp_cal.dig_P2 = (s16_t) (buf[10] <<  8 | buf[ 9]);
  bmp_cal.dig_P3 = (s16_t) (buf[12] <<  8 | buf[11]);
  bmp_cal.dig_P4 = (s16_t) (buf[14] <<  8 | buf[13]);
  bmp_cal.dig_P5 = (s16_t) (buf[16] <<  8 | buf[15]);
  bmp_cal.dig_P6 = (s16_t) (buf[18] <<  8 | buf[17]);
  bmp_cal.dig_P7 = (s16_t) (buf[20] <<  8 | buf[19]);
  bmp_cal.dig_P8 = (s16_t) (buf[22] <<  8 | buf[21]);
  bmp_cal.dig_P9 = (s16_t) (buf[24] <<  8 | buf[23]);
  return;
}

void bmp_cal_ref_load(void)
{
  /* Reference calibration values from section 3.12 (pg 22--23) of the manual */
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
  printf("dig_T1 = %6d\n",bmp_cal.dig_T1);
  printf("dig_T2 = %6d\n",bmp_cal.dig_T2);
  printf("dig_T3 = %6d\n",bmp_cal.dig_T3);
  printf("dig_P1 = %6d\n",bmp_cal.dig_P1);
  printf("dig_P2 = %6d\n",bmp_cal.dig_P2);
  printf("dig_P3 = %6d\n",bmp_cal.dig_P3);
  printf("dig_P4 = %6d\n",bmp_cal.dig_P4);
  printf("dig_P5 = %6d\n",bmp_cal.dig_P5);
  printf("dig_P6 = %6d\n",bmp_cal.dig_P6);
  printf("dig_P7 = %6d\n",bmp_cal.dig_P7);
  printf("dig_P8 = %6d\n",bmp_cal.dig_P8);
  printf("dig_P9 = %6d\n",bmp_cal.dig_P9);
  return;
}

void bmp_rd_raw_pres_temp(bmp280_pres_temp_t *res)
{
  union i2c_data tmp;
  i2c_rd_blk(bmp,BMP_PRESS_MSB,sizeof(bmp280_pres_temp_t),(u8_t *) &tmp);
  res->pres.msb = tmp.blk[1];
  res->pres.lsb = tmp.blk[2];
  res->pres.xsb = tmp.blk[3];
  res->temp.msb = tmp.blk[4];
  res->temp.lsb = tmp.blk[5];
  res->temp.xsb = tmp.blk[6];
  return;
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
  /* Hack! Seems like the GCC compiler doesn't correctly ASR a negative 64bit value. This sets the topmost 19 bits. */
  var2 |= (~0ULL<<(64-19));
  p = ((p + var1 + var2) >> 8) + (((s64_t)bmp_cal.dig_P7) << 4);
  return (u32_t) p;
}

void bmp_rd_pres_temp(s32_t *raw_pres, s32_t *raw_temp, s32_t *pres, s32_t *temp)
{
  bmp280_pres_temp_t raw;
  bmp_rd_raw_pres_temp(&raw);
  *raw_pres = (raw.pres.msb << 12) | (raw.pres.lsb << 4) | (raw.pres.xsb >> 4);
  *raw_temp = (raw.temp.msb << 12) | (raw.temp.lsb << 4) | (raw.temp.xsb >> 4);
  *temp = bmp_rd_temp(*raw_temp);
  *pres = bmp_rd_pres(*raw_pres);
  return;
}


/* Takes pressure (Pa) in Q24.8 format (from bmp_rd_pres()) and returns computed altitude */
/* presssure/altitude formula from http://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html */
/* Pressure = 101325*(1-2.25577e-5*altitude)**5.25588 */
/* https://en.wikipedia.org/wiki/Pressure_altitude says:*/
/* Altitude [m] = (1-(pressure/1013.25)**0.190284)*476924.05 [P in hPa]. 1013.25 is the 'standard 1atm' pressure */
s32_t pres2alt(u32_t pres)
{
  double tmp = ((double) pres)/25600.0;
  tmp = (1.0-pow(tmp/1013.25,0.190284))* /*476924.05 */ (145366.45*0.3048);
  return (s32_t) (tmp*256);
}


int main (void)
{
  int i;
  mpu = i2c_init_dev(RA_MPU);
  bmp = i2c_init_dev(RA_BMP);
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
  puts("Done init");
#if defined(BMP)
#if 0
  bmp_cal_ref_load();
# endif
# ifdef DEBUG
  bmp_cal_dump();
# endif
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
# if 0
    raw_temp = 519888;
    raw_pres = 415148;
    temp = bmp_rd_temp(raw_temp);
    pres = bmp_rd_pres(raw_pres);
# else
    bmp_rd_pres_temp(&raw_pres, &raw_temp, &pres, &temp);
# endif
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
    s32_t alt = pres2alt(pres);
    printf("Temp: %+7.2f Pres: %7.3f Alt: %7.3f ", temp/100.0, pres/25600.0, alt/256.0);
# ifdef DEBUG
    printf("%06x %06x %06x ", (u32_t) raw_temp, raw_pres, (u32_t) alt);
# endif
#endif
    putchar('\r');
  } while(i<10);
}
