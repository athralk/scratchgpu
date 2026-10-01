# ScratchGPU: Synapse-32 + TinyGPU v2 on the ZC702

A small RISC-V computer for the Zynq-7020 (ZC702): the **Synapse-32** RV32IMAFD core with
caches and an Sv32 MMU, plus **TinyGPU v2**, an RVV vector unit (Zve32f, VLEN 512) that the
CPU drives with its own instruction stream. Parallel loops run on the GPU because the CPU
issues vector instructions to it, whether hand-written (intrinsics) or produced by the
compiler from plain C (`-O3 -march=..._zve32f_zvl512b`). There is no GPU driver and no
offload API.

```
                 ┌────────────── Synapse-32 (RV32IMA + Zicsr, 5-stage) ──────────────┐
                 │  IF ─ ID ─ EX ─ MEM ─ WB      mul/div unit      vsetvl, CSRs        │
                 └──┬──────────┬───────────────────────┬─────────────────────────────┘
          I-TLB/I$ 8K│    D-TLB/D$ 8K (write-through)   │ issue (from MEM, in order, precise)
                     │          │  ▲                    ▼
                     │          │  │ vector/FP loads  TinyGPU v2  (FP/vector coprocessor)
                     │          │  └──────────────── · 4-entry queue
                     │          │                    · VRF 32 x 512b in 16 LUTRAM banks
                     │          │                    · 8 SIMD lanes (int ALU, DSP mul, FP32 FMA)
                     │          │                    · element engine: all of Zve32f
                     │          │                    · scalar F/D: FP regs, FP64 FMA, div/sqrt
                     ▼          ▼
               AXI3 master (64-bit) ── S_AXI_HP0 ──> Zynq PS DDR (Synapse RAM at DDR 0x2000_0000)
               CLINT · PLIC · 16550 UART (PS UART0 via EMIO) · control block (M_AXI_GP0)
```

## Layout

| Path | What |
|---|---|
| `cpu/rtl/` | Synapse-32 core (`riscv_cpu.v`), MMU/TLB/page walker (`mmu/`), caches (`cache/`), SoC (`soc/soc_top.v`) |
| `cpu/rtl/core_modules/vec_decode.v` | Vector + FP instruction decode/legality, shared by core and GPU |
| `gpu/rtl/` | TinyGPU v2: `vpu_top.v` (sequencer, FP regs), `vpu_lanes.v`, `vpu_vrf.v`, `vpu_fp32.v`, `vpu_fp64.v`, `fp_fma.v`, `fp32_fma.v` |
| `cpu/sim/soc/` | Fast Verilator model of the whole SoC with a DDR model (`make`, `make linux`) |
| `cpu/tests/` | cocotb regression (`./run.sh -n 14 unit_tests system_tests`) and riscv-tests on the SoC (`system_tests/test_soc_isa.py`) |
| `cpu/tools/vgen.py`, `run_vtests.py` | Random vector/FP programs, checked against Spike signature by signature |
| `sw/demo/` | CPU+GPU demo: matmul int32/fp32 and auto-vectorized saxpy (`make sim`) |
| `sw/arm_bridge/` | Bare-metal ARM program for the board: releases Synapse, bridges its UART to the USB-UART |
| `sw/br2-synapse/` | Buildroot external tree for Synapse Linux (kernel, OpenSBI, BusyBox) |
| `tools/sync_vivado.sh` | Copies RTL and board software to `D:/ScratchGPU_Vivado` |

## Verification status

- riscv-tests rv32 ui/um/ua/uf/ud/si/mi, physical and virtual memory: 140/140 on the SoC (caches, TLBs, AXI DDR model), also under slow / stalling DDR.
- Random vector + FP programs vs Spike (`run_vtests.py`): all classes (integer, mask, permute, reduction, widening/narrowing, every load/store form, FP32 vector incl. `.vf`, scalar F/D with all rounding modes and flags): 400/400 mixed, plus 200+ per class.
- Compact FMA vs the reference FMA: 0 mismatches in 4M vectors; generic divide/sqrt vs FP32 one: 0 in 400k.

## Running things

```bash
# whole-SoC simulator and the demo (prints over the UART model)
make -C cpu/sim/soc && make -C sw/demo sim

# random vector/FP tests against Spike (needs ~/opt/riscv-isa-sim, see below)
python3 cpu/tools/run_vtests.py --seeds 0-199 --jobs 14

# ISA tests on the SoC, and the cocotb suite
cd cpu/tests && ./run.sh -n 14 system_tests/test_soc_isa.py
./run.sh -n 14 unit_tests system_tests
```

Tools live in `~/opt` (xPack RISC-V GCC 14, xPack ARM GCC, riscv-tests, Spike, dtc), linked
into `~/.local/bin`.

## On the board (Vivado on Windows)

1. `tools/sync_vivado.sh` (WSL) copies RTL and software to `D:/ScratchGPU_Vivado`.
2. In the Vivado Tcl console: `source D:/ScratchGPU_Vivado/create_project.tcl`, then
   `source D:/ScratchGPU_Vivado/build.tcl` (bitstream, XSA, reports in `out/`).
3. Open a terminal on the ZC702 USB-UART (115200 8N1), then from a Windows shell:
   `xsdb D:/ScratchGPU_Vivado/run_on_board.tcl` (programs the PL, loads `sw/demo.bin`
   into DDR, starts the ARM bridge, which releases Synapse).
