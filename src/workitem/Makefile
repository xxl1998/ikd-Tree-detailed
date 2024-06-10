PROJ=workitem

CFLAGS=-Wall -DUNIX -g -DDEBUG

# Check for 32-bit vs 64-bit
PROC_TYPE = $(strip $(shell uname -m | grep 64))

ROOT_DIR := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
CFLAGS += -DROOT_DIR=\"$(ROOT_DIR)\"

# Linux OS
LIBS=-lOpenCL
ifeq ($(PROC_TYPE),)
	CFLAGS+=-m32
else
	CFLAGS+=-m64
endif

$(PROJ): $(PROJ).cpp
	g++ $(CFLAGS) -o $@ $^ $(INC_DIRS:%=-I%) $(LIB_DIRS:%=-L%) $(LIBS)

.PHONY: clean

clean:
	rm $(PROJ)
