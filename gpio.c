#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include "gpio.h"

#define	PAGE_SIZE  (4*1024)
#define	BLOCK_SIZE (4*1024)

#define MASK(bit,len) ((~(~0ULL<<len))<<bit)
#define barrier() __asm__ __volatile__ ("" : : : "memory")
#define NGPIO 64

/* 
 * The BCM2835 has 54 GPIO pins.
 * BCM2835 data sheet, Page 90 onwards.
 * There are 6 control registers, each control the functions of a block
 * of 10 pins.
 * Each control register has 10 sets of 3 bits per GPIO pin - the ALT values
 *
 * 000 = GPIO Pin X is an input
 * 001 = GPIO Pin X is an output
 * 100 = GPIO Pin X takes alternate function 0
 * 101 = GPIO Pin X takes alternate function 1
 * 110 = GPIO Pin X takes alternate function 2
 * 111 = GPIO Pin X takes alternate function 3
 * 011 = GPIO Pin X takes alternate function 4
 * 010 = GPIO Pin X takes alternate function 5
 *
 * So the 3 bits for port X are:
 *	X / 10 + ((X % 10) * 3)
 */

/* Port function select bits */
#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

/* /dev/mem & /dev/gpiomem offsets */
#define	GPIO_PERIPH_BASE	0x3F000000
#define GPIO_PADS_OFFSET  0x00100000

/* wiringPi has these as "volatile" but I doubt that is really GCC-safe. I'll
	 put a memory barrier() around accesses instead */
static unsigned int gpio_base = 0;
static unsigned int gpio_pads;
static uint32_t *gpio;
static uint32_t *pads;

/* Translate from the Schematic names to the physical BCM GPIO pins */

static int pin2gpio[NGPIO] = {
  17, 18, 27, 22, /* GPIO  0 -  3 */
	23, 24, 25,  4, /* GPIO  4 -  7 */
   2,  3,				  /* GPIO  8 -  9 aka SDA0, SCL0 */
	 8,  7,         /* GPIO 10 - 11 aka SPI_CE_1, SPI_CE_0 */
  10,  9, 11,     /* GPIO 12 - 14 aka SPI_MOSI, SPI_MISO & SPI_SCLK */
  14, 15,         /* GPIO 15 - 16 aka UART TXD0, RXD0 */
  28, 29, 30, 31,	/* GPIO 17 - 20	*/
   5,  6, 13, 19, /* GPIO 21 - 24 */
  26, 12, 16, 20, /* GPIO 25 - 28 */
  21,  0,  1,     /* GPIO 29 - 31 */
	/* Padding: */
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	/* 32 - 47 */
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	/* 48 - 63 */
};

/*
 * gpio2fsel:
 * Map a BCM_GPIO pin to it's Function Selection control port. (GPFSEL
 * 0-5) Groups of 10 - 3 bits per Function - 30 bits per port
 */
static uint8_t gpio2fsel[NGPIO] = {
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
} ;


/*
 * gpio2shift:
 * Define the shift up for the 3 bits per pin in each GPFSEL port
 */
static uint8_t gpio2shift[NGPIO] = {
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
};

#define GPPUD 37

static uint8_t gpio2pudclk[NGPIO] = {
  38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
  39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
} ;

/* gpio2set:
 * (Word) offset to the GPIO Set registers for each GPIO pin
 */
static uint8_t gpio2set[NGPIO] = {
   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
   8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
} ;

/* gpio2clr:
 * (Word) offset to the GPIO Clear registers for each GPIO pin
 */
static uint8_t gpio2clr[NGPIO] = {
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
};

/* gpio2lev
 * (Word) offset to the GPIO Input Level registers for each GPIO pin
 */
static uint8_t gpio2lev[NGPIO] = {
  13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
};

int gpio_init(void)
{
	int res = -1;
	int fd;

	gpio_base = GPIO_PERIPH_BASE;
	/* First try /dev/mem, then /dev/gpiomem if that fails */
	fd = open("/dev/mem",O_RDWR | O_SYNC | O_CLOEXEC);
	if(0>fd) {
		fd = open("/dev/gpiomem",O_RDWR | O_SYNC | O_CLOEXEC);
		if(0>fd) {
			return res;
		}
		gpio_base = 0;
	}

	gpio_pads = gpio_base + GPIO_PADS_OFFSET;

	/* Map the hardware */
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpio_base);
  if(MAP_FAILED==gpio) return res;

	//	The drive pads
  pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpio_pads);
  if(MAP_FAILED==pads) return res;

#if 0
  initialiseEpoch();
#endif

	return 0;
}

#if 0
/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin
 *	The Arduino only has pull-ups and these are enabled by writing 1
 *	to a port when in input mode - this paradigm doesn't quite apply
 *	here though.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

	/**/ if (wiringPiMode == WPI_MODE_PINS)
		pin = pinToGpio [pin] ;
	else if (wiringPiMode == WPI_MODE_PHYS)
		pin = physToGpio [pin] ;
	else if (wiringPiMode != WPI_MODE_GPIO)
		return ;

	*(gpio + GPPUD) = pud & 3;
	delayusec(5);
	*(gpio + gpio2pudclk[pin]) = 1 << (pin & 31);
	delayusec(5);
    
	*(gpio + GPPUD) = 0;
	delayusec(5);
	*(gpio + gpio2pudclk[pin]) = 0;
	delayusec(5) ;
}


int digitalRead(int pin)
{
  char c;
  struct wiringPiNodeStruct *node = wiringPiNodes;

	if(wiringPiMode == WPI_MODE_GPIO_SYS) {
		// Sys mode
		if (sysFds [pin] == -1)	return GPIO_LOW;

		lseek  (sysFds [pin], 0L, SEEK_SET);
		read   (sysFds [pin], &c, 1);
		return (c == '0') ? GPIO_LOW : GPIO_HIGH;
	} else if (wiringPiMode == WPI_MODE_PINS)
		pin = pinToGpio [pin];
	else if (wiringPiMode == WPI_MODE_PHYS)
		pin = physToGpio [pin];
	else if (wiringPiMode != WPI_MODE_GPIO)
		return GPIO_LOW;

	if ((*(gpio + gpioToGPLEV [pin]) & (1 << (pin & 31))) != 0)
		return GPIO_HIGH;
	else
		return GPIO_LOW;
  } else {
	if ((node = wiringPiFindNode (pin)) == NULL)
		return GPIO_LOW;
	return node->digitalRead (node, pin);
}

/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */

void digitalWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] != -1)
      {
	if (value == GPIO_LOW)
	  write (sysFds [pin], "0\n", 2);
	else
	  write (sysFds [pin], "1\n", 2);
      }
      return;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin];
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin];
    else if (wiringPiMode != WPI_MODE_GPIO)
      return;

    if (value == GPIO_LOW)
      *(gpio + gpioToGPCLR [pin]) = 1 << (pin & 31);
    else
      *(gpio + gpioToGPSET [pin]) = 1 << (pin & 31);
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite (node, pin, value);
  }
}
#endif

int gpio_mode(int pin, int mode, int pull)
{
	int fSel, shift;
	fSel  = gpio2fsel[pin];
	shift = gpio2shift[pin];

	if(mode == GPIO_INPUT) {
		/* Set pin mode */
		barrier();
		/* Sets bits to zero = input */
		*(gpio+fSel) = (*(gpio+fSel) & ~(7 << shift));
		/* Set pull-up/down */
    *(gpio+GPPUD) = pull & 3;
		delayusec(5);
		barrier();
    *(gpio+gpio2pudclk[pin]) = 1 << (pin & 31);
		delayusec(5);
    
		barrier();
    *(gpio+GPPUD)             = 0;
		delayusec(5);
		barrier();
    *(gpio+gpio2pudclk[pin]) = 0;
		delayusec(5);

	}	else if(mode==GPIO_OUTPUT) {
		barrier();
		*(gpio+fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift);
	}
	barrier();
	return 0;
}

void gpio_write(int pin, int val)
{
	*(gpio + ((val==GPIO_LOW)? gpio2clr[pin] : gpio2set[pin])) = 1 << (pin & 31);
}

int gpio_read(int pin)
{
	return (*(gpio+gpio2lev[pin]) & (1 << (pin & 31))) != 0;
}

int gpio_isr_set(int pin, void (*fn)(void))
{
	return 0;
}

int gpio_isr_del(int pin)
{
	return 0;
}

int gpio_ena(int pin)
{
	return 0;
}

int gpio_dis(int pin)
{
	return 0;
}


void busywait(unsigned int delay)
{
	struct timeval tNow, tLong, tEnd;
	gettimeofday(&tNow,NULL);
	tLong.tv_sec  = delay / 1000000;
	tLong.tv_usec = delay % 1000000;
	timeradd(&tNow,&tLong,&tEnd);
	while(timercmp(&tNow,&tEnd,<)) gettimeofday(&tNow,NULL);
	return;
}


void delayusec(unsigned int delay)
{
	struct timespec sleeper;
	unsigned int usec = delay % 1000000;
	unsigned int secs = delay / 1000000;

	if(0==delay) return;
	if(delay<100) {
		busywait(delay);
	} else {
		sleeper.tv_sec  = secs;
		sleeper.tv_nsec = (long)(usec * 1000L);
		nanosleep(&sleeper,NULL);
	}
	return;
}

void delaymsec(unsigned int delay)
{
  struct timespec sleeper;
  sleeper.tv_sec  = (time_t)(delay / 1000) ;
  sleeper.tv_nsec = (long) ((delay % 1000) * 1000000);
  nanosleep(&sleeper, NULL);
	return;
}
