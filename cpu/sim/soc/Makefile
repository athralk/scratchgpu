# Standalone Verilator build of the FPGA SoC with the behavioral DDR model.
#   make            -> obj/Vsoc_sim_top
#   make run HEX=<image.hex> TOHOST=80001000
ROOT := $(abspath ../..)
RTL  := $(filter-out %/top.v %/unified_mem.v,$(shell find $(ROOT)/rtl $(ROOT)/../gpu/rtl -name '*.v'))
TB   := $(ROOT)/tests/testbenches/axi_ddr_model.v $(ROOT)/tests/testbenches/soc_sim_top.v

obj/Vsoc_sim_top: $(RTL) $(TB) main.cpp
	verilator --cc --exe --build -j 0 -O3 --x-assign fast --x-initial fast \
	  -Wno-fatal -Wno-lint -Wno-style -Wno-SYNCASYNCNET \
	  -I$(ROOT)/rtl/include --top-module soc_sim_top -Mdir obj \
	  -CFLAGS -O2 $(RTL) $(TB) main.cpp

# Linux boots: 256 MB of DDR (the DTS memory node), UART output on stdout.
obj_linux/Vsoc_sim_top: $(RTL) $(TB) main.cpp
	verilator --cc --exe --build -j 0 -O3 --x-assign fast --x-initial fast \
	  -Wno-fatal -Wno-lint -Wno-style -Wno-SYNCASYNCNET -GDDR_WORDS=67108864 \
	  -I$(ROOT)/rtl/include --top-module soc_sim_top -Mdir obj_linux \
	  -CFLAGS -O2 $(RTL) $(TB) main.cpp

linux: obj_linux/Vsoc_sim_top

run: obj/Vsoc_sim_top
	./obj/Vsoc_sim_top +hex=$(HEX) +tohost=$(TOHOST)

# Boot Synapse Linux (Buildroot output) on the 256 MB model; the console is the UART echo.
#   make boot-linux [IMG=~/opt/br-synapse32/images/synapse.hex] [CYCLES=2000000000]
IMG    ?= $(HOME)/opt/br-synapse32/images/synapse.hex
CYCLES ?= 2000000000
boot-linux: obj_linux/Vsoc_sim_top
	./obj_linux/Vsoc_sim_top +hex=$(IMG) +tohost=0 +max_cycles=$(CYCLES)

clean:
	rm -rf obj obj_linux
.PHONY: run clean linux boot-linux
