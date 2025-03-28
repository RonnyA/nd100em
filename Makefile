# Makefile for ND100 VM

CC= gcc
#CFLAGS = -ggdb
CFLAGS = -Wall -O3 -pg -fno-aggressive-loop-optimizations -ggdb

SRCS = main.c \
       cpu.c \
       memory.c \
       io_new.c \
       iox/device.c \
       iox/devicemanager.c \
       iox/devicePapertape.c \
       iox/deviceFloppyPIO.c \
       iox/deviceTerminal.c \
       iox/deviceRTC.c \
       iox/panel.c

OBJS = $(SRCS:.c=.o)

all: nd100em

clean:
	rm -f cpu.o mon.o trace.o decode.o float.o floppy.o io_new.o nd100lib.o nd100em.o iox/device.o iox/devicemanager.o iox/devicePapertape.o iox/deviceFloppyPIO.o iox/deviceTerminal.o iox/deviceRTC.o iox/panel.o nd100em core

cpu.o: cpu.c cpu.h nd100.h
	$(CC) $(CFLAGS) -c cpu.c

memory.o: memory.c memory.h nd100.h
	$(CC) $(CFLAGS) -c memory.c

io_new.o: io_new.c io_new.h nd100.h
	$(CC) $(CFLAGS) -c io_new.c

iox/device.o: iox/device.c iox/device.h
	$(CC) $(CFLAGS) -c iox/device.c -o iox/device.o

iox/devicemanager.o: iox/devicemanager.c iox/devicemanager.h iox/device.h
	$(CC) $(CFLAGS) -c iox/devicemanager.c -o iox/devicemanager.o

iox/devicePapertape.o: iox/devicePapertape.c iox/devicePapertape.h iox/device.h
	$(CC) $(CFLAGS) -c iox/devicePapertape.c -o iox/devicePapertape.o

iox/deviceFloppyPIO.o: iox/deviceFloppyPIO.c iox/deviceFloppyPIO.h iox/device.h
	$(CC) $(CFLAGS) -c iox/deviceFloppyPIO.c -o iox/deviceFloppyPIO.o

iox/deviceTerminal.o: iox/deviceTerminal.c iox/deviceTerminal.h iox/device.h
	$(CC) $(CFLAGS) -c iox/deviceTerminal.c -o iox/deviceTerminal.o

iox/deviceRTC.o: iox/deviceRTC.c iox/deviceRTC.h iox/device.h
	$(CC) $(CFLAGS) -c iox/deviceRTC.c -o iox/deviceRTC.o

iox/panel.o: iox/panel.c iox/panel.h iox/device.h
	$(CC) $(CFLAGS) -c iox/panel.c -o iox/panel.o

nd100lib.o: nd100lib.c nd100lib.h nd100.h
	$(CC) $(CFLAGS) -c nd100lib.c

nd100em: nd100em.o nd100lib.o cpu.o mon.o decode.o float.o floppy.o io_new.o trace.o iox/device.o iox/devicemanager.o iox/devicePapertape.o iox/deviceFloppyPIO.o iox/deviceTerminal.o iox/deviceRTC.o iox/panel.o
	$(CC) $(CFLAGS) -pthread nd100em.o nd100lib.o cpu.o mon.o decode.o float.o floppy.o io_new.o trace.o iox/device.o iox/devicemanager.o iox/devicePapertape.o iox/deviceFloppyPIO.o iox/deviceTerminal.o iox/deviceRTC.o iox/panel.o -lconfig -lm -o nd100em

# Compilation rules
$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/iox/%.o: iox/%.c
	$(CC) $(CFLAGS) -c $< -o $@

