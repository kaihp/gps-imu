CC=gcc
OPTIONS = -mthumb -mcpu=cortex-m4
DEFINES = -DEMPL -DEMPL_TARGET_RPI3 -DMPU9250
INCDIRS = -iquote../core/driver/eMPL -iquote../core/driver/include -iquote../core/driver/rpi  -iquote../core/eMPL-hal -iquote../core/mllite -iquote../core/mpl

CPPFLAGS= $(OPTIONS) $(DEFINES) $(INCDIRS)

SRCS = board.c
OBJS = $(SRCS:.c=.o)

.PHONY: default all compile clean help

default: help

help:
	@echo "Targets: compile clean help"

clean:
	-rm -f $(OBJS)

compile: $(OBJS)

all: compile
