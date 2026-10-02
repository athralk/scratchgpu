# ScratchGPU: Synapse-32 + TinyGPU v2 on the ZC702

ScratchGPU is a small RISC-V computer that fits in the programmable logic of the Zynq-7020 on
a Xilinx ZC702 board. It has two halves that work as one machine:

- **Synapse-32**: a 5-stage in-order RISC-V CPU (RV32IMA + Zicsr/Zifencei) with 8 KB
  instruction and data caches, TLBs and an Sv32 MMU. It runs OpenSBI and Linux.
- **TinyGPU v2**: a vector coprocessor that implements the RISC-V vector extension
  (RVV 1.0, the `Zve32f` subset: 8/16/32-bit integers and FP32, 512-bit registers). It also
  holds the scalar floating-point unit (F and D), so it is the CPU's FPU as well.

The GPU is not a device behind a driver. It executes **instructions from the CPU's own
instruction stream**. When the CPU meets a vector or floating-point instruction, it hands
it to TinyGPU and carries on. A "GPU kernel" is therefore an ordinary function compiled
with vector instructions in it. The compiler can generate those from plain C loops, you
can write them by hand with RVV intrinsics, or the `tgc` mini-language can generate them
from CUDA-style kernels.

The dual-core ARM in the Zynq is the **service processor**. It powers up the DDR memory,
loads Synapse's software into DDR, releases Synapse from reset, and relays its console to
your laptop over the board's USB-UART.

Contents:
1. [The big picture](#1-the-big-picture)
2. [How Synapse and TinyGPU are connected](#2-how-synapse-and-tinygpu-are-connected)
3. [What the interconnect lets you do](#3-what-the-interconnect-lets-you-do)
4. [How it goes on the chip](#4-how-it-goes-on-the-chip)
5. [How Linux runs on it](#5-how-linux-runs-on-it)
6. [What is left for Linux](#6-what-is-left-for-linux)
7. [What was installed, and why](#7-what-was-installed-and-why)
8. [Repository layout](#8-repository-layout)
9. [Building, testing and running](#9-building-testing-and-running)
10. [Verification status and known limits](#10-verification-status-and-known-limits)

---

## 1. The big picture

```
 ┌──────────────────────────────── Zynq-7020 (ZC702) ─────────────────────────────────┐
 │                                                                                    │
 │  PS (hard silicon)                         PL (FPGA fabric) : soc_top              │
 │  ┌──────────────────────┐                  ┌─────────────────────────────────────┐ │
 │  │ 2x ARM Cortex-A9     │  M_AXI_GP0       │ ps_ctrl  (reset, ID, PC readback)   │ │
 │  │  bare-metal "bridge" ├─────────────────►│                                     │ │
 │  │                      │                  │  Synapse-32 CPU ──issue──► TinyGPU  │ │
 │  │ UART1 ◄── USB-UART ──┼── laptop         │   I$ D$ TLBs        ◄─resp── v2     │ │
 │  │ UART0 ◄──── EMIO ────┼─────────────────►│  16550 UART                         │ │
 │  │                      │                  │  CLINT timer, PLIC                  │ │
 │  │ DDR controller       │  S_AXI_HP0       │                                     │ │
 │  │   1 GB DDR3  ◄───────┼──────────────────┤  AXI3 master (64-bit)               │ │
 │  └──────────────────────┘                  └─────────────────────────────────────┘ │
 │        FCLK_CLK0 = 50 MHz clocks all of the PL                                     │
 └────────────────────────────────────────────────────────────────────────────────────┘
```

- **Memory.** Synapse has no memory of its own. All its RAM is a 256 MB window of the
  board's DDR3, reached through the PS's high-performance port S_AXI_HP0. Synapse
  physical address `0x8000_0000` maps to DDR `0x2000_0000`, so the ARM keeps the lower
  512 MB.
- **Console.** Synapse's 16550-compatible UART is wired through EMIO to the PS's UART0. The
  ARM bridge program copies bytes between UART0 and UART1, the board's USB-UART, so
  Synapse's console appears on your laptop at 115200 8N1.
- **Control.** A small register block on M_AXI_GP0 (`ps_ctrl`, at ARM address
  `0x4000_0000`) holds Synapse in reset until the ARM has loaded its image:

  | Offset | Register | Meaning |
  |---|---|---|
  | 0x00 | CTRL | bit 0 = Synapse reset (1 = held; resets to 1) |
  | 0x04 | PC | Synapse's current fetch PC (read-only, for bring-up) |
  | 0x08 | ID | `0x53594E35` = "SYN5": proves the bitstream is loaded |
  | 0x0C | SCRATCH | free read/write register |
  | 0x10 | DDRBASE | DDR address that Synapse `0x8000_0000` maps to |

- **Synapse's memory map:**

  | Address | What |
  |---|---|
  | `0x0200_0000` | CLINT (`mtime`, `mtimecmp`, software interrupt) |
  | `0x0C00_0000` | PLIC (external interrupts: UART) |
  | `0x2000_0000` | 16550 UART (`ns16550a` in the device tree) |
  | `0x8000_0000` - `0x8FFF_FFFF` | 256 MB DRAM (DDR `0x2000_0000` and up) |

---

## 2. How Synapse and TinyGPU are connected

### 2.1 The idea: a coprocessor on the instruction stream

The old TinyGPU was a separate 4x4 int64 matrix engine that needed its own command
interface. TinyGPU v2 is instead **tightly coupled** to the CPU pipeline. The CPU decodes
every instruction. Anything in the vector (OP-V) or floating-point (OP-FP, FMA, FLW/FSW,
FLD/FSD) opcode space is sent to TinyGPU from the CPU's **MEM stage**, in program order,
together with everything TinyGPU needs to execute it.

There is no driver, no command queue in memory, no interrupt to signal "done", and no
copying of data. TinyGPU reads and writes the same memory as the CPU, through the CPU's own
data cache.

```
  Synapse pipeline                                      TinyGPU v2 (gpu/rtl/vpu_top.v)
  IF ─ ID ─ EX ─ MEM ─ WB                               ┌──────────────────────────────┐
                 │                                      │ 4-entry instruction queue    │
                 │ vec_issue_valid / ready              │ decoder (vec_decode.v, same  │
                 │ instr, rs1, rs2, vl, vtype,  ───────►│   file the CPU uses)         │
                 │ vstart, vxrm, frm                    │ VRF: 32 x 512-bit registers  │
                 │                                      │ 4 SIMD lanes (fast path)     │
                 │ vec_resp_valid, data, fault,  ◄──────│ element engine (all of RVV)  │
                 │ fault_addr, fault_vstart, vl         │ scalar F/D unit + FP regs    │
                 │                                      │ FP32 / FP64 divide-sqrt      │
                 │ vec_busy, fflags_set, vxsat_set ◄────│                              │
                 │                                      └───────┬──────────────────────┘
                 │                                              │ vector/FP loads & stores
                 ▼                                              ▼
            D-TLB + D-cache  ◄──────── shared data port (soc_top muxes CPU / TinyGPU)
```

### 2.2 The signals

| Direction | Signal | Meaning |
|---|---|---|
| CPU → GPU | `vec_issue_valid/ready` | handshake for one instruction |
| | `vec_issue_instr` | the 32-bit instruction |
| | `vec_issue_rs1`, `rs2` | integer operands (for memory ops, `rs1` is already the effective address) |
| | `vec_issue_vl`, `vtype`, `vstart` | vector state, owned by the CPU's CSRs |
| | `vec_issue_vxrm`, `frm` | fixed-point and FP rounding modes |
| GPU → CPU | `vec_resp_valid`, `data` | result for instructions that write an integer register |
| | `vec_resp_fault`, `fault_addr`, `fault_vstart` | a vector load/store hit a page fault at element `vstart` |
| | `vec_resp_vl_valid`, `vl` | fault-only-first load trimmed `vl` |
| | `vec_busy` | GPU still has work queued (fences and CSR reads wait on it) |
| | `vec_fflags_set`, `vec_vxsat_set` | accrued FP exception flags / fixed-point saturation |

### 2.3 Who owns what

- **The CPU owns all architectural control state.**
  - The CSRs `vl`, `vtype`, `vstart`, `vxrm`, `vxsat`, `fflags`, `frm` and `fcsr`.
  - The `mstatus.FS`/`VS` dirty bits that Linux uses for lazy context switching.
  - `vsetvli` executes in the CPU itself (`vsetvl_unit.v`), so setting up a loop never
    waits for the GPU.
  - Illegal vector/FP instructions trap in the CPU before they are issued.
- **TinyGPU owns the data.**
  - The 32 vector registers (512 bits each) and the 32 floating-point registers
    (64 bits each).
  - All vector and FP arithmetic.

### 2.4 Three kinds of instructions

| Kind | Examples | What the CPU does |
|---|---|---|
| **Fire-and-forget** | `vadd.vv`, `vfmacc.vf`, `fadd.d` | issues and continues at once; up to 4 instructions queue in the GPU |
| **Returns a value** | `vmv.x.s`, `vcpop.m`, `vfirst.m`, `fmv.x.w`, `feq.s`, `fcvt.w.s` | issues, then holds MEM until `vec_resp_valid` brings the integer result |
| **Touches memory** | `vle32.v`, `vluxei32.v`, `vsse16.v`, `flw`, `fsd` | issues with the address, waits for completion; a page fault comes back with the faulting element so the trap is **precise** (`vstart` set, the instruction can be restarted) |

Because every instruction issues in order from MEM, and memory instructions complete before
the CPU moves on, all of these hold:
- Memory ordering between scalar and vector code is automatic.
- Traps are precise.
- Linux can preempt a process in the middle of a vector loop.

### 2.5 Inside TinyGPU v2

- **Vector register file:** 32 registers × 512 bits (VLEN = 512) in 16 LUT-RAM banks with
  three read ports and one write port, about 1k LUTs. A shadow copy of `v0` in flip-flops
  feeds the mask bits.
- **SIMD lanes (fast path):** 4 lanes for 32-bit elementwise integer and FP32 operations.
  Each lane has an integer ALU, a 33x33 multiplier on DSPs, and a fused multiply-add (FMA).
  The lanes are pipelined (4 stages) and take one beat of 4 elements per clock.
- **Element engine (complete path):** runs every Zve32f instruction one element at a time:
  - 8/16-bit elements, widening/narrowing, fixed-point;
  - reductions, masks, slides, gathers, compress;
  - FP conversions, compares, divides, square roots, `vfrec7`/`vfrsqrt7`.
  - Each element takes 5 clocks:
    1. fetch the operands into registers;
    2. compute for 3 cycles;
    3. capture the result, then write it.
  - This is what lets the deep FP logic meet 50 MHz.
- **Scalar F/D:** 32 × 64-bit FP registers (NaN-boxed singles), an FP64 FMA, and iterative
  FP32/FP64 divide/sqrt, bit-exact with Spike including all rounding modes and flags.
- **Vector memory:** goes element by element through the CPU's D-cache and D-TLB. While
  one access is in flight, the next element's address is already prepared, so cache hits
  stream at one element per clock. Because the D-cache is write-through, CPU and GPU always
  see the same memory with no coherence protocol.

---

## 3. What the interconnect lets you do

### 3.1 Write parallel code in three ways

**Plain C, auto-vectorized by GCC.** Compile with
`-O3 -march=rv32imafd_zve32f_zvl512b -mabi=ilp32d` and loops like this become vector code:
```c
for (int i = 0; i < n; i++) y[i] = a * x[i] + y[i];
```

**RVV intrinsics**, for full control:
```c
for (size_t vl; n > 0; n -= vl, x += vl, y += vl) {
    vl = __riscv_vsetvl_e32m8(n);
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(x, vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(y, vl);
    __riscv_vse32_v_f32m8(y, __riscv_vfmacc_vf_f32m8(vy, a, vx, vl), vl);
}
```

**`tgc` kernels (`sw/tgc`)**: a CUDA-like syntax. You write what happens to *one* element
(`tid`), and `tgc` produces the vector loop:
```c
__kernel void saxpy(float a, const float *x, float *y) {
    y[tid] = a * x[tid] + y[tid];
}
// in C:  launch(saxpy, n, a, x, y);   -- a direct function call, no runtime
```
Branches inside a kernel become vector compares and masked operations. `__kernel2d`
gives you `tx`/`ty`. The examples include saxpy, ReLU, a Mandelbrot renderer and an
image blend.

### 3.2 Things this design makes cheap

- **No launch cost.** A kernel call costs the same as a function call, so even short
  loops (100 elements) gain. A driver-based GPU needs thousands of elements to pay back
  its launch cost.
- **Mixed scalar/vector code.** Scalar code can read a vector result one instruction
  later, for example `vfredusum` then `vfmv.f.s`. Pointer-chasing code and vector math
  share the same data structures in the same memory.
- **Linux processes use the GPU like any other CPU feature.** The kernel saves and restores
  the vector registers on context switches, so every process can use it at once without a
  scheduler or allocator.
- **Floating point everywhere.** The scalar FPU lives in the same unit, so ordinary C
  `float`/`double` code also runs in hardware.

### 3.3 Measured on the simulator

32x32 matrix multiply and saxpy, scalar code vs the same computation on TinyGPU (4 lanes),
with bit-identical results:

| Workload | Scalar | TinyGPU | Speed-up |
|---|---|---|---|
| int32 matmul | 409k cycles | 71k cycles | 5.7x |
| fp32 matmul | 850k cycles | 77k cycles | 11.0x |
| saxpy (plain C, auto-vectorized) | 165k cycles | 50k cycles | 3.3x |

The fp32 speed-up is partly inflated: after the timing fix (§10), scalar FP ops take about
6 cycles each, which slows the scalar baseline it is compared against.

### 3.4 Where the limits are (and what would lift them)

| Limit | Effect | Possible improvement |
|---|---|---|
| Vector memory is 32 bits per clock through the D-cache | memory-bound kernels (saxpy) gain ~3x, compute-bound ones (matmul) ~6-11x | 64/128-bit vector port, or a scratchpad for `__shared__` data |
| One vector instruction runs at a time | no overlap between a long load and arithmetic | chaining / a second issue slot |
| Element engine is 5 clocks per element | 8/16-bit and complex ops are slow | pipeline the element engine (overlap elements) |
| 4 lanes | half the ALUs of the original 4x4 plan | LUT budget: 8 lanes did not fit next to everything else |

---

## 4. How it goes on the chip

### 4.1 From RTL to bitstream (Vivado on Windows)

The RTL lives in this WSL repository. Vivado runs on Windows and reads a synced copy:

1. In WSL, `tools/sync_vivado.sh` copies `cpu/rtl`, `gpu/rtl`, the timing constraints and the
   board software to `D:/ScratchGPU_Vivado`.
2. Open the Vivado project `D:/Vivado_Projects/Scratch_gpu`. In the Tcl console, run
   `source D:/ScratchGPU_Vivado/add_design.tcl`. This:
   - adds the RTL;
   - adds `constraints/soc_timing.xdc`;
   - builds the block design `system`: PS7 with the ZC702 preset, FCLK0 = 50 MHz, HP0 and
     GP0 enabled, UART0 on EMIO, with `soc_top` as a module reference.
3. Run `source D:/ScratchGPU_Vivado/build.tcl` for synthesis, implementation and the
   bitstream. It writes the bitstream, `ps7_init.tcl` and reports to `D:/ScratchGPU_Vivado/out/`.
4. If timing fails, `source D:/ScratchGPU_Vivado/report_timing.tcl` (read-only) writes the
   failing paths, grouped by module, to `out/timing_endpoints.txt`.

Resources at the last full implementation (before the timing fix): **46,732 / 53,200 LUTs
(87.8 %)**, 8.8k flip-flops (8 %). The timing fix adds flip-flops and almost no LUTs.

### 4.2 Running something on the board

```
xsdb D:/ScratchGPU_Vivado/run_on_board.tcl              # the CPU+GPU demo
xsdb D:/ScratchGPU_Vivado/run_on_board.tcl <image.bin>  # any Synapse image (e.g. Linux)
```
Open a serial terminal on the ZC702 USB-UART (115200 8N1) first. Over JTAG, the script:

1. resets the PS and runs `ps7_init` (DDR, clocks, MIO);
2. programs the bitstream;
3. copies the Synapse image into DDR at `0x2000_0000`, which Synapse sees as `0x8000_0000`;
4. starts `sw/arm_bridge` on ARM core 0. The bridge checks the "SYN5" ID, releases Synapse
   from reset, and relays the UARTs.

Synapse then fetches its first instruction from `0x8000_0000`.

### 4.3 Clock and reset

- Everything in the PL runs on FCLK_CLK0 = 50 MHz. That is also the timer frequency, so
  `timebase-frequency = 50000000` in the device tree.
- `FCLK_RESET0_N` resets the PL. The ARM's CTRL register holds Synapse in reset until the
  image is in place.

---

## 5. How Linux runs on it

### 5.1 Boot chain

```
 DDR 0x2000_0000 (= Synapse 0x8000_0000)
 ┌───────────────────────────────────────────────────────────────────────────┐
 │ OpenSBI 1.5.1  (M-mode firmware, "fw_payload": firmware + kernel in one) │
 │   └─ embedded device tree  synapse32.dtb                                 │
 │ Linux 6.12.48 Image at 0x8040_0000 (S-mode)                               │
 │   └─ built-in initramfs: BusyBox + glibc root filesystem                  │
 └───────────────────────────────────────────────────────────────────────────┘
```

1. **OpenSBI** starts in machine mode at `0x8000_0000`. It:
   - sets up the CLINT timer and trap delegation;
   - emulates misaligned accesses (Synapse traps on them);
   - prints its banner on the UART;
   - jumps to Linux in supervisor mode with the device tree's address in `a1`.
2. **Linux** reads the device tree:
   - CPU: `rv32imafd_zicsr_zifencei_zve32f`;
   - 256 MB of RAM at `0x8000_0000`;
   - CLINT, PLIC, and the ns16550a UART at `0x2000_0000`.

   It then turns on the Sv32 MMU (Synapse's TLBs and hardware page walker) and uses
   SBI calls for the timer.
3. The kernel unpacks the **initramfs** and runs `/init` (BusyBox), which gives you a shell
   on `ttyS0`, the console you see through the ARM bridge.

### 5.2 Why the hardware looks the way it does

- **F and D are required.** Standard Linux userlands (glibc, the `ilp32d` ABI) assume
  hardware floating point, and the kernel saves FP registers on context switches. That is
  why TinyGPU also implements scalar F/D.
- **The vector extension is usable from Linux.** Since 6.10 the kernel supports `Zve32x`
  machines, saving and restoring the vector registers lazily via `mstatus.VS`. That makes
  TinyGPU available to every process.
- **Sv32 + Svade.** The page walker never writes PTEs. Linux sets the Accessed/Dirty
  bits itself after the resulting page faults, which keeps the walker small.
- **Interrupts.** The timer goes through the CLINT and OpenSBI. The UART interrupt goes
  through the PLIC.

### 5.3 The software build (Buildroot)

`sw/br2-synapse` is a Buildroot *external tree*. From it, Buildroot builds everything:
- a cross compiler (GCC 14 + glibc for `rv32imafd`/`ilp32d`);
- the kernel, configured as `rv32_defconfig` plus `board/synapse32/linux.fragment`
  (no compressed instructions, vector on, FP on, PCI/USB/etc. off);
- OpenSBI with the device tree embedded;
- BusyBox and the initramfs.

`board/synapse32/post-image.sh` then writes two images:
- `synapse.bin`: the raw image the ARM loads into DDR;
- `synapse.hex`: for the simulator.

```bash
cd ~/opt/buildroot
make O=~/opt/br-synapse32 BR2_EXTERNAL=~/ScratchGPU/sw/br2-synapse synapse32_defconfig
env -i HOME=$HOME PATH=/usr/bin:/bin make O=~/opt/br-synapse32 -j12   # first build ~1 h
# result: ~/opt/br-synapse32/images/synapse.bin and synapse.hex
```

Building with a clean `PATH` matters. The WSL `PATH` contains Windows directories with
spaces, which Buildroot refuses.

### 5.4 Booting it in simulation

```bash
make -C cpu/sim/soc linux        # Verilator model with 256 MB of DDR
make -C cpu/sim/soc boot-linux   # console output appears on stdout
```
The simulator runs at about 200k Synapse clocks per second (measured on the demo), so one
second of Linux time (50M clocks) takes about 4-5 minutes of wall time.

---

## 6. What is left for Linux

Status at the time of writing:
- OpenSBI and Linux boot in the simulator up to ~2.9 s of kernel time: memory setup, MMU on,
  timer, the UART console, and the network stack initialised.
- The boot then stops with `Kernel panic - not syncing: junk at the end of compressed
  archive` while unpacking the initramfs.

| # | Task | Why | Size |
|---|---|---|---|
| 1 | **Fix the initramfs panic** | the unpacked cpio stream is corrupt. Either a hardware bug in a path the tests do not yet cover, or a packaging problem. The new random stress tests (scalar memory against a 32 KB window, timer interrupts every 20-275 cycles) pass; the next step is to unpack the same archive in a bare-metal test, and to try an uncompressed / smaller initramfs | 1-3 days |
| 2 | **Trim the kernel** | the `Image` is 30 MB because `rv32_defconfig` enables media, filesystems and network drivers we can never use. That makes each simulated boot (and each JTAG load) slow | hours |
| 3 | **Make the kernel see the vector unit** | the boot log lists `adfim` but no vector extension. Check that `zve32x`/`zve32f` in `riscv,isa-extensions` is accepted and that `has_vector()` is true, then run a vector program under Linux | hours to a day |
| 4 | **First boot on the board** | load `synapse.bin` with `run_on_board.tcl` (JTAG; a 30 MB image takes a minute or two; a trimmed one much less) and get a shell on the USB-UART | depends on 1-3; a day of bring-up |
| 5 | Persistent root filesystem (virtio-blk) | today the root filesystem is a RAM disk built into the kernel; changes are lost at reset. Plan: a virtio-mmio register page in the PL, served by a program on the ARM that stores blocks in a file on the SD card | 1-2 weeks |
| 6 | Networking (virtio-net) | `ssh` into Synapse, through the ARM's Ethernet | ~1 week after 5 |
| 7 | Display (VNC) | a `simple-framebuffer` in Synapse DDR; the ARM serves it with VNC over Ethernet; your laptop shows it full-screen on the HDMI monitor | ~1 week after 6 |
| 8 | Boot from SD without JTAG | the ARM runs from the SD card (FSBL + bridge) and loads `synapse.bin` from it | a few days |

Items 1-4 give you "Linux shell on the board". Items 5-8 turn it into a usable small
computer.

---

## 7. What was installed, and why

Everything except the apt packages lives under `~/opt`, with `~/.local/bin` symlinks.
Nothing was installed system-wide on Windows.

| What | Where | Size | Why |
|---|---|---|---|
| xPack RISC-V GCC 14 | `~/opt/xpack-riscv-none-elf-gcc-*` | 1.3 GB | bare-metal compiler for tests, the demo and `tgc`; GCC 14 is needed for RVV intrinsics and auto-vectorization |
| xPack ARM GCC 14 | `~/opt/xpack-arm-none-eabi-gcc-*` | 1.0 GB | builds `sw/arm_bridge` (the ARM service program) |
| Spike (riscv-isa-sim) | `~/opt/riscv-isa-sim` (source), `~/opt/spike` (build) | 1.8 GB | the official RISC-V reference simulator; every random vector/FP test compares our hardware against it |
| riscv-tests | `~/opt/riscv-tests` | 16 MB | the standard ISA test suite (140 tests run on the SoC) |
| dtc | `~/opt/dtc-src` | 4 MB | compiles the device tree (`synapse32.dts` → `.dtb`) |
| Buildroot | `~/opt/buildroot` | 1.2 GB | the Linux build system |
| Buildroot output | `~/opt/br-synapse32` | **12 GB** | its own RISC-V Linux cross-compiler, kernel, OpenSBI, BusyBox and all intermediate files. This is most of the WSL growth you saw, and the long build with "lots of stuff flashing" |
| apt packages | system | small | Buildroot's host requirements (`build-essential`, `flex`, `bison`, `bc`, `cpio`, `rsync`, `unzip`, `libncurses-dev`, `libssl-dev`, ...) |
| Python packages | repo `.venv` | small | cocotb, pytest, pytest-xdist for the legacy test suite |

Reclaiming space safely, when no builds are running:
```bash
rm -rf ~/ScratchGPU/cpu/tests/.xdist ~/ScratchGPU/cpu/tests/sim_build*   # test build caches
rm -rf ~/opt/riscv-isa-sim/build                                        # Spike build tree
ln -sf ~/opt/spike/bin/spike ~/.local/bin/spike
# ~/opt/br-synapse32/build (11 GB) is needed for incremental Linux rebuilds; deleting it
# means the next build starts from scratch (~1 h).
```
WSL does not give freed space back to Windows automatically. Compact the virtual disk
afterwards (`wsl --shutdown`, then `Optimize-VHD`, or `diskpart` → `compact vdisk`).

---

## 8. Repository layout

| Path | What |
|---|---|
| `cpu/rtl/riscv_cpu.v` | Synapse-32 pipeline, coprocessor issue interface |
| `cpu/rtl/core_modules/` | ALU, iterative mul/div, CSRs (incl. vector/FP CSRs), decoder, `vec_decode.v` (shared with the GPU), `vsetvl_unit.v`, CLINT timer, PLIC, UART |
| `cpu/rtl/cache/` | 8 KB 2-way I-cache and write-through D-cache with store buffer |
| `cpu/rtl/mmu/` | `tlb.v`, `ptw.v` (Sv32 walker), permission checks |
| `cpu/rtl/soc/` | `soc_top.v` (everything in the PL), `axi_mem_master.v`, `ps_ctrl.v`, `soc_timing.xdc` |
| `gpu/rtl/` | TinyGPU v2: `vpu_top.v`, `vpu_lanes.v`, `vpu_vrf.v`, `vpu_fp32.v`, `vpu_fp64.v`, `fp32_fma.v`, `fp_fma.v` |
| `cpu/sim/soc/` | Verilator model of the whole SoC with a DDR model |
| `cpu/tests/` | cocotb suite, ISA tests on the SoC, testbenches |
| `cpu/tools/` | `vgen.py` + `run_vtests.py` (random tests vs Spike) |
| `sw/demo/` | CPU+GPU demo (matmul, saxpy) |
| `sw/tgc/` | the `tgc` kernel translator and examples |
| `sw/arm_bridge/` | ARM service program |
| `sw/br2-synapse/` | Buildroot external tree (defconfig, kernel fragment, device tree, scripts) |
| `tools/sync_vivado.sh` | copies RTL, constraints and software to `D:/ScratchGPU_Vivado` |

---

## 9. Building, testing and running

```bash
# Whole-SoC simulator, then the demo
make -C cpu/sim/soc
make -C sw/demo sim
make -C sw/tgc sim                       # tgc kernels incl. ASCII Mandelbrot

# ISA tests on the SoC (140 tests)
cd cpu/tests && ./run.sh -n 12 system_tests/test_soc_isa.py

# Random vector/FP/memory tests against Spike
python3 cpu/tools/run_vtests.py --seeds 0-399 --jobs 12 \
    --classes int,mask,perm,red,widen,mem,fp,sfp,smem
python3 cpu/tools/run_vtests.py --seeds 0-79 --irq      # with timer interrupts

# Legacy cocotb suite
cd cpu/tests && ./run.sh -n 12 unit_tests system_tests --ignore=system_tests/test_soc_isa.py
```

---

## 10. Verification status and known limits

- **riscv-tests** rv32 ui/um/ua/uf/ud/si/mi, with physical and virtual memory: 140/140 on
  the SoC.
- **Random tests vs Spike:** every vector class plus scalar F/D and scalar memory stress:
  400/400. With frequent timer interrupts: 80/80.
- **FP units:** the pipelined FMA matches the original FMA exactly (0 mismatches in 150k
  random and edge-case operands). Divide/sqrt matches the reference (0 in 1M).
- **Timing:** the first implementation missed 50 MHz by 40 ns, because the vector unit did a
  whole element (register read, FMA, write-back) in one clock. Now:
  - the lanes are pipelined;
  - the element engine and scalar FP unit run in phases;
  - `soc_timing.xdc` gives those phases their multicycle budget.

  This is verified in simulation and awaits the next Vivado run.
- **Not yet on hardware:** no bitstream has run on the board yet.
- **Linux:** boots in simulation to the initramfs stage; see §6.
