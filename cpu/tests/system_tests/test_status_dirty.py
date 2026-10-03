"""mstatus.FS / mstatus.VS dirty tracking on the FPGA SoC (sim/soc, no cocotb).

Scalar FP runs on the vector unit, but per the privileged spec it may only dirty FS: a VS that turns
Dirty after scalar FP made Linux save vector state (to a NULL buffer) for tasks that never used the
vector unit. Each check starts from FS = VS = Initial, runs one instruction class and checks both
fields. Pass = the program writes 1 to tohost; a failing check writes its odd code.

The dirty bits are set when the instruction issues from MEM, so a csrr right behind it still reads
the old value (Spike does not lag). Traps read the status many cycles later, so Linux never sees
this; the checks wait a few cycles (settle) before reading.
"""
import subprocess
from pathlib import Path

import pytest

PROGRAM = r"""
    .section .text.init
    .globl _start
_start:
    li s0, (3 << 9) | (3 << 13)          # VS and FS masks
    li s1, (1 << 9) | (1 << 13)          # VS = FS = Initial
    la s2, fdata

    .macro initial
    csrc mstatus, s0
    csrs mstatus, s1
    .endm
    .macro settle
    nop
    nop
    nop
    nop
    .endm
    # fail with code if (mstatus & s0) != expected
    .macro expect bits, code
    settle
    csrr t0, mstatus
    and t0, t0, s0
    li t1, \bits
    li a0, \code
    bne t0, t1, fail
    .endm

    initial
    fadd.s f1, f2, f3                    # scalar FP arithmetic
    expect (1 << 9) | (3 << 13), 3

    initial
    fld f4, 0(s2)                        # scalar FP load
    expect (1 << 9) | (3 << 13), 5

    initial
    fsd f4, 8(s2)                        # scalar FP store (FS may turn Dirty, VS must not)
    settle
    csrr t0, mstatus
    srli t0, t0, 9
    andi t0, t0, 3
    li t1, 1
    li a0, 7
    bne t0, t1, fail

    initial
    fmv.x.w a1, f4                       # FP -> integer register (reads FP state only)
    settle
    csrr t0, mstatus
    srli t0, t0, 9
    andi t0, t0, 3
    li t1, 1
    li a0, 9
    bne t0, t1, fail

    initial
    fdiv.d f5, f4, f4                    # long-latency FP (the divider)
    expect (1 << 9) | (3 << 13), 11

    vsetvli t2, zero, e32, m1, ta, ma
    initial
    vadd.vv v1, v2, v3                   # a real vector instruction dirties VS
    settle
    csrr t0, mstatus
    srli t0, t0, 9
    andi t0, t0, 3
    li t1, 3
    li a0, 13
    bne t0, t1, fail

    initial
    vle32.v v4, (s2)                     # vector load: VS Dirty
    settle
    csrr t0, mstatus
    srli t0, t0, 9
    andi t0, t0, 3
    li t1, 3
    li a0, 15
    bne t0, t1, fail

    li a0, 1
fail:
    la t0, tohost
    sw a0, 0(t0)
1:  j 1b

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .globl fromhost
fromhost: .dword 0

    .data
    .align 6
fdata:
    .double 3.0
    .double 0.0
    .fill 16, 4, 0
"""


def _cpu_root() -> Path:
    cur = Path(__file__).resolve()
    while not (cur / "rtl").exists():
        cur = cur.parent
    return cur


@pytest.fixture(scope="module")
def soc_sim():
    sim_dir = _cpu_root() / "sim" / "soc"
    subprocess.run(["make", "-s", "-C", str(sim_dir)], check=True)
    return sim_dir / "obj" / "Vsoc_sim_top"


def runCocotbTests(soc_sim, tmp_path):
    """Scalar FP leaves VS Initial; vector instructions make it Dirty."""
    src, elf, binf, hexf = (tmp_path / n for n in ("t.S", "t.elf", "t.bin", "t.hex"))
    src.write_text(PROGRAM)
    subprocess.run(["riscv64-unknown-elf-gcc", "-march=rv32imafd_zve32f_zvl512b", "-mabi=ilp32", "-nostdlib",
                    "-T", str(_cpu_root() / "tools" / "vtest_link.ld"), str(src), "-o", str(elf)], check=True)
    subprocess.run(["riscv64-unknown-elf-objcopy", "-O", "binary", str(elf), str(binf)], check=True)
    data = binf.read_bytes()
    data += b"\0" * (-len(data) % 4)
    with open(hexf, "w") as f:
        f.write("@00000000\n")
        for i in range(0, len(data), 4):
            f.write("%08x\n" % int.from_bytes(data[i:i + 4], "little"))
    nm = subprocess.run(["riscv64-unknown-elf-nm", str(elf)], check=True, capture_output=True, text=True)
    tohost = next(int(l.split()[0], 16) for l in nm.stdout.splitlines() if l.endswith(" tohost"))
    proc = subprocess.run([str(soc_sim), f"+hex={hexf}", f"+tohost={tohost:x}", "+max_cycles=200000"],
                          capture_output=True, text=True, timeout=120)
    out = proc.stdout + proc.stderr
    assert proc.returncode == 0, f"check failed (tohost code in the log):\n{out[-1500:]}"
