CC=gcc
OPTIONS = -mthumb -mcpu=cortex-m4
DEFINES = -DEMPL -DEMPL_TARGET_RPI3 -DMPU9250
INCDIRS = -iquote../../../bsp
CPPFLAGS = $(OPTIONS) $(DEFINES) $(INCDIRS)

SRCS = inv_mpu.c inv_mpu_dmp_motion_driver.c
OBJS = $(SRCS:.c=.o)

.PHONY: default all compile clean help

default: help

help:
	@echo "Targets: compile clean help"

clean:
	-rm -f $(OBJS)

compile: $(OBJS)

all: compile
