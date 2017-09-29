CC=gcc
CFLAGS=-g

ssd1306.o: ssd1306.c ssd1306.h ssd1306_regs.h
	$(CC) $(CFLAGS) -c $<

i2c.o: i2c.c i2c.h
imu-test4.o: imu-test4.c mpu9250_regs.h bmp280_regs.h i2c.h font_8x8.h

imu-test4: imu-test4.o i2c.o
	$(CC) $(CFLAGS) -o $@ $^

ssd-test1: ssd1306.o i2c.o
	$(CC) $(CFLAGS) -o $@ $^
