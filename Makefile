# Makefile for PicoBorgRev advanced motor controller PIC code

# Use the small devices C compiler
CC=sdcc

# Compile for PIC16F1824
CCFLAGS=-mpic14 -p16f1824 --use-non-free

# Linker flags
LDFLAGS=-Wl,--no-cinit-warnings,-O2,-q

# Use pk2cmd for programming
PROGRAMMER=pk2cmd

# Program for PIC16F1824
PROGRAMMERFLAGS=-M -PPIC16f1824


all: firmware clean

firmware: picoborgrev.hex

picoborgrev.hex: picoborgrev.o
	$(CC) $(CCFLAGS) $(LDFLAGS) -o picoborgrev.hex picoborgrev.o

picoborgrev.o:
	$(CC) $(CCFLAGS) -c picoborgrev.c

upload: program

program: firmware
	$(PROGRAMMER) $(PROGRAMMERFLAGS) -Fpicoborgrev.hex

clean:
	rm -f *.asm *.cod *.lst *.o

distclean: clean
	rm -f *.hex
