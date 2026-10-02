# Accuracy test of the fast divider (gpu/rtl/fp_rcpdiv.v) against the host's IEEE results.
#   make            build and run 200k random cases
ROOT := $(abspath ../..)
obj/Vfp_rcpdiv_tb: fp_rcpdiv_tb.v fp_rcpdiv_tb.cpp $(ROOT)/gpu/rtl/fp_rcpdiv.v $(ROOT)/cpu/rtl/include/fp_seed_rom.vh
	verilator --cc --exe --build -j 0 -O3 -Wno-fatal -Wno-lint -Wno-style -I$(ROOT)/cpu/rtl/include \
	  --top-module fp_rcpdiv_tb -Mdir obj -CFLAGS "-O2 -frounding-math" fp_rcpdiv_tb.v $(ROOT)/gpu/rtl/fp_rcpdiv.v fp_rcpdiv_tb.cpp
run: obj/Vfp_rcpdiv_tb
	./obj/Vfp_rcpdiv_tb $(N)
.DEFAULT_GOAL := run
N ?= 200000
.PHONY: run
