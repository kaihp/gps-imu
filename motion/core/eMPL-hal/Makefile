CC=gcc
INCDIRS = -iquote../mllite -iquote../driver/include
DEFINES = -DEMPL_TARGET_RPI3 -DMPU9250
CPPFLAGS = $(DEFINES) $(INCDIRS)

SRCS = eMPL_outputs.c
OBJS = $(SRCS:.c=.o)

.PHONY: default all compile clean help

default: help

help:
	@echo "Targets: compile clean help"

clean:
	-rm -f $(OBJS)

compile: $(OBJS)

all: compile
