# Synapse-32 + TinyGPU v2 demo.   make        -> demo.elf / demo.hex / demo.bin
#                                 make sim    -> run on the SoC simulator
CC      = riscv64-unknown-elf-gcc
OBJCOPY = riscv64-unknown-elf-objcopy
CFLAGS  = -O3 -mabi=ilp32d -ffreestanding -fno-builtin -nostdlib -Wall
SIM     = ../../cpu/sim/soc/obj/Vsoc_sim_top

demo.elf: crt0.S main.c kernels_scalar.c kernels_vector.c common.h link.ld
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -c crt0.S -o crt0.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -c main.c -o main.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -c kernels_scalar.c -o kernels_scalar.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr_zve32f_zvl512b -c kernels_vector.c -o kernels_vector.o
	$(CC) $(CFLAGS) -march=rv32imafd_zicsr -T link.ld crt0.o main.o kernels_scalar.o kernels_vector.o -lgcc -o $@

demo.bin: demo.elf
	$(OBJCOPY) -O binary $< $@

demo.hex: demo.bin
	python3 -c "import sys; d=open('$<','rb').read(); d+=b'\0'*(-len(d)%4); \
open('$@','w').write('@00000000\n'+''.join('%08x\n'%int.from_bytes(d[i:i+4],'little') for i in range(0,len(d),4)))"

sim: demo.hex
	$(MAKE) -s -C ../../cpu/sim/soc
	$(SIM) +hex=demo.hex +tohost=$$(riscv64-unknown-elf-nm demo.elf | awk '/ tohost$$/{print $$1}') +max_cycles=200000000

clean:
	rm -f *.o demo.elf demo.bin demo.hex
.PHONY: sim clean
