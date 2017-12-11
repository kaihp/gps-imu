#!/usr/bin/make -f
OPTIONS = -mthumb -mcpu=cortex-m4
DEFINES = -DEMPL -DEMPL_TARGET_RPI3 -DMPU9250

DIRS = bsp core user

.PHONY: default all compile clean help

default: help

help:
	@echo "Targets: compile clean help"

clean:
	for dir in $(DIRS); do $(MAKE) -C $$dir clean; done

compile:
	for dir in $(DIRS); do $(MAKE) -C $$dir compile; done

$(DIRS):
	OPTIONS="$(OPTIONS)" $(MAKE) -C $@ compile

all: compile
