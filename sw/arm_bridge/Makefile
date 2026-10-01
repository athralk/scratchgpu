# Bare-metal Cortex-A9 bridge for the ZC702 (built in WSL, run by xsdb on Windows).
CC = $(HOME)/opt/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin/arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-a9 -marm -O2 -ffreestanding -nostdlib -Wall

bridge.elf: start.S bridge.c link.ld
	$(CC) $(CFLAGS) -T link.ld start.S bridge.c -o $@

clean:
	rm -f bridge.elf
