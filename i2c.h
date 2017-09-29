#include <stdint.h>

#ifndef __LOCAL_I2C_H__
#define __LOCAL_I2C_H__

#define I2C_BLK_MAX (32)

union i2c_data {
  uint8_t  byte;
  uint16_t word;
  uint8_t  blk[I2C_BLK_MAX+2];
};

int i2c_init_dev(int devId);
int i2c_rd8(int fd, int addr);
int i2c_rd16(int fd, int addr);
int i2c_rd_blk(int fd, int addr, int length, uint8_t *data);
int i2c_wr(int fd, int cmd);
int i2c_wr8(int fd, int addr, uint8_t data);
int i2c_wr16(int fd, int addr, uint16_t data);
int i2c_wr_blk(int fd, int addr, int length, uint8_t *data);

#endif
