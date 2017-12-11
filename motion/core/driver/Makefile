OPTIONS = -mthumb -mcpu=cortex-m4
DEFINES = -DEMPL -DEMPL_TARGET_RPI3 -DMPU9250

DIRS = eMPL rpi

.PHONY: default all compile clean help $(DIRS)

default: help

help:
	@echo "Targets: compile clean help"

clean:
	for dir in $(DIRS); do $(MAKE) -C $$dir clean; done

compile: $(DIRS)

$(DIRS):
	$(MAKE) -C $@ compile

all: compile
