/*
 * "Board Support Package" for Raspberry Pi 3
 *
 * Written by Kai Harrekilde-Petersen, 2017.
 *
 * This is a very limited BSP for the Raspberry Pi3. 
 * It assumes that it is running on top of the Raspbian/Stretch
 * distro, so the system as such as already been set up and only I/O
 * access to I2C and GPIOs needs to be set up.
 */

#include <unistd.h>
#include <sys/time.h>
#include "board.h"
#if 0
#include "i2c.h"
#include "gpio.h"
#endif

void board_init() 
{
  // Configure Interrupts
  // GPIO_Config();  

  // Configure I2C
  // I2cMaster_Init(); 
}

void mdelay(unsigned long nTime)
{
  usleep(nTime * 1000);
  return;
}

int get_tick_count(unsigned long *count)
{
  struct timeval tv;
  int status = gettimeofday(&tv, NULL);
  if(0==status) {
    count[0] = (unsigned long) (tv.tv_sec*1000 + tv.tv_usec/1000);
    return 0;
  }
  return -1;
}
