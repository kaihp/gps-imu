CC=gcc
OPTIONS = -mthumb -mcpu=cortex-m4
DEFINES = -DEMPL -DEMPL_TARGET_RPI3 -DMPU9250
INCDIRS = -iquote../mllite -iquote../driver/include
CPPFLAGS = $(OPTIONS) $(DEFINES) $(INCDIRS)

SRCS = data_builder.c hal_outputs.c message_layer.c mlmath.c ml_math_func.c mpl.c results_holder.c start_manager.c storage_manager.c
OBJS = $(SRCS:.c=.o)

.PHONY: default all compile clean help

default: help

help:
	@echo "Targets: compile clean help"

clean:
	-rm -f $(OBJS)

compile: $(OBJS)

all: compile
