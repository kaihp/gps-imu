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
