# tgc examples:  make sim   (translate, compile for Synapse + TinyGPU, run on the SoC model)
CC      = riscv64-unknown-elf-gcc
CFLAGS  = -O3 -mabi=ilp32d -ffreestanding -fno-builtin -nostdlib -Wall -ffp-contract=off
DEMO    = ../demo
SIM     = ../../cpu/sim/soc/obj/Vsoc_sim_top

kernels.c: kernels.tgc tgc.py
	python3 tgc.py $< -o $@

tgc.elf: kernels.c main.c $(DEMO)/crt0.S $(DEMO)/link.ld
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr_zve32f_zvl512b -c kernels.c -o kernels.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -c main.c -o main.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -c $(DEMO)/crt0.S -o crt0.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -T $(DEMO)/link.ld crt0.o main.o kernels.o -lgcc -o $@

tgc.hex: tgc.elf
	riscv64-unknown-elf-objcopy -O binary $< tgc.bin
	python3 -c "d=open('tgc.bin','rb').read(); d+=b'\0'*(-len(d)%4); \
open('$@','w').write('@00000000\n'+''.join('%08x\n'%int.from_bytes(d[i:i+4],'little') for i in range(0,len(d),4)))"

sim: tgc.hex
	$(SIM) +hex=tgc.hex +tohost=$$(riscv64-unknown-elf-nm tgc.elf | awk '/ tohost$$/{print $$1}') +max_cycles=50000000

clean:
	rm -f *.o kernels.c tgc.elf tgc.bin tgc.hex
.PHONY: sim clean
