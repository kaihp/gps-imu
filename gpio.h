/*
 * GPIO related constants & prototypes
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

#define GPIO_LOW  0
#define GPIO_HIGH 1

#define GPIO_TRI 0
#define GPIO_PD  1
#define GPIO_PU  2

#define GPIO_IRQ_EDGE_SETUP 0
#define GPIO_IRQ_EDGE_FALL  1
#define GPIO_IRQ_EDGE_RISE  2
#define GPIO_IRQ_EDGE_BOTH  3

extern int  gpio_init(void);
extern int  gpio_mode(int pin, int mode, int pull);
extern void gpio_write(int pin, int val);
extern int  gpio_read(int pin);
extern int  gpio_isr_set(int pin, void (*fn)(void));
extern int  gpio_isr_del(int pin);
extern int  gpio_isr_ena(int pin);
extern int  gpio_isr_dis(int pin);

extern void busywait(unsigned int delay);
extern void delayusec(unsigned int delay);
extern void delaymsec(unsigned int delay);

#endif
