#!/usr/bin/make -f

DIRS = driver eMPL-hal mllite

.PHONY: default all compile clean help

default: help

help:
	@echo "Targets: compile clean help"

clean:
	for dir in $(DIRS); do $(MAKE) -C $$dir clean; done

compile:
	for dir in $(DIRS); do $(MAKE) -C $$dir compile; done

$(DIRS):
	$(MAKE) -C $@ compile

all: compile
