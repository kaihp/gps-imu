CC=gcc
CFLAGS=-g
LDFLAGS=-g
LDLIBS=-lm

%.c: %.fnt
	./fontcompile $< $@

ssd1306.o: ssd1306.c ssd1306.h ssd1306_regs.h
i2c.o: i2c.c i2c.h
imu-test4.o: imu-test4.c mpu9250_regs.h bmp280_regs.h i2c.h

imu-test4: imu-test4.o i2c.o

ssd-test1: ssd1306.o i2c.o font_7x5.o font_8x8.o font_16x10.o font_21x14.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LOADLIBES) $(LDLIBS)
