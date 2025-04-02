# Makefile for ND100 VM

CC= gcc
#CFLAGS = -ggdb

# Nodebug
#CFLAGS = -Wall -O3 -pg -fno-aggressive-loop-optimizations -ggdb 

# Debug
# profiling for gprof: -pg 
CFLAGS = -Wall -O0 -fno-aggressive-loop-optimizations -ggdb -DDEBUG

# Source files
SRCS = nd100em.c \
		cpu.c \
		cpu_mms.c \
		iox/devicemanager.c \
		iox/device.c \
		iox/deviceRTC.c \
		iox/devicePapertape.c \
		iox/deviceFloppyPIO.c \
		iox/deviceSMD.c \
		iox/diskSMD.c \
		iox/deviceTerminal.c \
		iox/panel.c \
		io_new.c \
		floppy.c \
		nd100lib.c \
		float.c \
		mon.c \
		trace.c \
		decode.c \
		iox/deviceFloppyDMA.c
		

# Object files
OBJS = $(SRCS:.c=.o)

# Include files that should trigger recompilation when changed
INCLUDES = nd100em.h \
          cpu.h \
          cpu_mms.h \
          io_new.h \
          nd100.h \
          nd100lib.h \
          iox/device.h \
          iox/devicemanager.h \
          iox/devicePapertape.h \
          iox/deviceFloppyPIO.h \
          iox/deviceTerminal.h \
		  iox/deviceSMD.h \
		  iox/diskSMD.h \
          iox/deviceRTC.h \
          iox/panel.h \
		  floppy.h \
		  float.h \
		  mon.h \
		  trace.h \
		  decode.h
		  

all: nd100em

clean:
	rm -f $(OBJS) nd100em core

# Pattern rule for compiling source files
%.o: %.c $(INCLUDES)
	$(CC) $(CFLAGS) -c $< -o $@

# Special rule for iox directory files
iox/%.o: iox/%.c $(INCLUDES)
	$(CC) $(CFLAGS) -c $< -o $@

nd100em: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -lconfig -lm -o nd100em

