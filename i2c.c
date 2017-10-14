#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "i2c.h"

/* I2C definitions */
#define I2C_SLAVE 0x0703
#define I2C_SMBUS 0x0720 /* SMBus-level access */
#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types */
#define I2C_SMBUS_QUICK            0
#define I2C_SMBUS_BYTE             1
#define I2C_SMBUS_BYTE_DATA        2
#define I2C_SMBUS_WORD_DATA        3
#define I2C_SMBUS_PROC_CALL        4
#define I2C_SMBUS_BLOCK_DATA       5
#define I2C_SMBUS_I2C_BLOCK_BROKEN 6
#define I2C_SMBUS_BLOCK_PROC_CALL  7 /* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA   8

int i2c_init_dev(int devId)
{
  int fd;
  const char *dev = "/dev/i2c-1";
  fd = open(dev, O_RDWR);
  if (fd<0) {
    puts("Error: couldn't open I2C device. Did you forget sudo?");
    return fd;
  }
  if(ioctl(fd, I2C_SLAVE, devId) < 0) {
    printf("Unable to select I2C device 0x%02x\n",devId);
  }
  return fd;
}

static inline int i2c_smbus_access(int fd, char rw, uint8_t cmd, int size, union i2c_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = rw;
  args.command    = cmd;
  args.size       = size;
  args.data       = (union i2c_smbus_data *)data;
  return ioctl(fd, I2C_SMBUS, &args) ;
}

int i2c_rd8(int fd, int addr)
{
  union i2c_data buf;
  if(0==i2c_smbus_access(fd, I2C_SMBUS_READ, addr, I2C_SMBUS_BYTE_DATA, &buf)) {
    return buf.byte;
  }
  return -1;
}

int i2c_rd16(int fd, int addr)
{
  union i2c_data buf;
  if(0==i2c_smbus_access(fd, I2C_SMBUS_READ, addr, I2C_SMBUS_WORD_DATA, &buf)) {
    return buf.word;
  }
  return -1;
}

int i2c_rd_blk(int fd, int addr, int length, uint8_t *data)
{
  uint8_t buf[2+I2C_BLK_MAX];
  int full, partial;
  int nbytes=0;
  /* Chop xfer into blocks of I2C_BLK_MAX bytes */
  full = length/I2C_BLK_MAX;
  partial = length & (I2C_BLK_MAX-1);
  while(full--) {
    buf[0] = I2C_BLK_MAX;
    if(0==i2c_smbus_access(fd, I2C_SMBUS_READ, addr, I2C_SMBUS_I2C_BLOCK_DATA, (union i2c_data *) buf)) {
      memcpy(&data[nbytes],&buf[1],I2C_BLK_MAX); /* dst,src,size_t */
      nbytes+=I2C_BLK_MAX;
    } else {
      return -nbytes;
    }
  }
  if(partial!=0) {
    buf[0] = partial;
    if(0==i2c_smbus_access(fd, I2C_SMBUS_READ, addr, I2C_SMBUS_I2C_BLOCK_DATA, (union i2c_data *) buf)) {
      memcpy(&data[nbytes],&buf[1],partial); /* dst,src,size_t */
      nbytes+=partial;
    } else {
      return -nbytes;
    }
  }
  return nbytes;
}

int i2c_wr(int fd, int cmd)
{
  union i2c_data buf = {.word = 0xDEAD};
  if(0==i2c_smbus_access(fd, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE, &buf)) {
    return buf.byte;
  }
  return -1;
}

int i2c_wr8(int fd, int addr, uint8_t data)
{
  union i2c_data buf = {.byte = data};
  if(0==i2c_smbus_access(fd, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, &buf)) {
    return buf.byte;
  }
  return -1;
}

int i2c_wr16(int fd, int addr, uint16_t data)
{
  union i2c_data buf = {.word = data};
  if(0==i2c_smbus_access(fd, I2C_SMBUS_WRITE, addr, I2C_SMBUS_WORD_DATA, &buf)) {
    return buf.byte;
  }
  return -1;
}

int i2c_wr_blk(int fd, int addr, int length, uint8_t *data)
{
  uint8_t buf[2+I2C_BLK_MAX];
  int full, partial;
  int nbytes=0;
  /* Chop xfer into blocks of I2C_BLK_MAX bytes */
  full = length/I2C_BLK_MAX;
  partial = length & (I2C_BLK_MAX-1);
  while(full--) {
    buf[0] = I2C_BLK_MAX;
    memcpy(&buf[1],&data[nbytes],I2C_BLK_MAX); /* dst,src,size_t */
    if(0==i2c_smbus_access(fd, I2C_SMBUS_WRITE, addr, I2C_SMBUS_I2C_BLOCK_DATA, (union i2c_data *) buf)) {
      nbytes+=I2C_BLK_MAX;
    } else {
      return -nbytes;
    }
  }
  if(partial!=0) {
    buf[0] = partial;
    memcpy(&buf[1],&data[nbytes],partial); /* dst,src,size_t */
    if(0==i2c_smbus_access(fd, I2C_SMBUS_WRITE, addr, I2C_SMBUS_I2C_BLOCK_DATA, (union i2c_data *) buf)) {
      nbytes+=partial;
    } else {
      return -nbytes;
    }
  }
  return nbytes;
}

