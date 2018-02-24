/*
 * GPIO pin test. Program to test the setup, setting, and checking the GPIO pins.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>

#include "gpio.h"

/* Input GPIOs */
#define SW1   0
#define SW2   1
#define SW3   2
#define SW4   3
#define NORTH 4
#define EAST  5
#define SOUTH 6
#define WEST  7
#define MID   8
#define PPS   9
/* Logical -> physical port translation:
 * SW1-4, N, E, S, W, Mid, 1PPS 
 */
#define NIN 10
int pin_in[NIN] = {4, 17, 21, 19, 16, 6, 5, 12, 13, 26};

/* Output GPIOs */
#define RSTN  0
/* Logical -> physical port translation:
 * RSTN
 */
#define NOUT 1
int pin_out[NOUT] = {20};

int main(int argc, char **argv)
{
  int i;
	if(0!=geteuid()) {
		printf("%s: This program needs to be run as root or under sudo.\n",argv[0]);
		exit(1);
	}

  if(0>gpio_init()) {
    puts("Failed to setup GPIO interface\n");
    exit(-1);
  }

	for(i=0;i<NIN;i++) {
		gpio_mode(pin_in[i],GPIO_INPUT,GPIO_TRI);
	}
	for(i=0;i<NOUT;i++) {
		gpio_mode(pin_out[i],GPIO_OUTPUT,GPIO_TRI);
		gpio_write(pin_out[i],1);
	}
	gpio_write(pin_out[RSTN],0);
	delaymsec(1);
	gpio_write(pin_out[RSTN],1);

	int vals[10];
	i=0;
  do {
		for(int j=0;j<NIN;j++) {
			vals[j]=gpio_read(pin_in[j]);
			putchar(vals[j]);
		}
    putchar('\r');
		i++;
  } while(i<10);
}
