import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

MASK32 = 0xFFFFFFFF
INSTR = {"MUL": 0x30, "MULH": 0x31, "MULHSU": 0x32, "MULHU": 0x33,
         "DIV": 0x34, "DIVU": 0x35, "REM": 0x36, "REMU": 0x37}


def s32(v):
    return v - (1 << 32) if v & 0x80000000 else v


def reference(op, a, b):
    if op == "MUL":
        return (s32(a) * s32(b)) & MASK32
    if op == "MULH":
        return ((s32(a) * s32(b)) >> 32) & MASK32
    if op == "MULHSU":
        return ((s32(a) * b) >> 32) & MASK32
    if op == "MULHU":
        return ((a * b) >> 32) & MASK32
    if op in ("DIV", "REM"):
        sa, sb = s32(a), s32(b)
        if sb == 0:
            return MASK32 if op == "DIV" else a
        if sa == -(1 << 31) and sb == -1:
            return 0x80000000 if op == "DIV" else 0
        q = abs(sa) // abs(sb)
        q = -q if (sa < 0) ^ (sb < 0) else q
        return (q & MASK32) if op == "DIV" else ((sa - q * sb) & MASK32)
    if b == 0:
        return MASK32 if op == "DIVU" else a
    return (a // b) if op == "DIVU" else (a % b)


async def reset(dut):
    dut.rst.value = 1
    dut.req.value = 0
    dut.kill.value = 0
    dut.advance.value = 1
    dut.instr_id.value = 0
    dut.a.value = 0
    dut.b.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst.value = 0


async def run_op(dut, op, a, b, hold_cycles=0):
    """Drive one instruction like EX does: hold req until ready, then advance."""
    dut.req.value = 1
    dut.instr_id.value = INSTR[op]
    dut.a.value = a
    dut.b.value = b
    dut.advance.value = 0 if hold_cycles else 1
    cycles = 0
    while True:
        await ReadOnly()
        if int(dut.ready.value):
            break
        await RisingEdge(dut.clk)
        cycles += 1
        assert cycles < 40, f"{op} never completed"
    got = int(dut.result.value)
    # MEM may hold the result for a few cycles; it must stay put.
    for _ in range(hold_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(dut.ready.value) and int(dut.result.value) == got
    await RisingEdge(dut.clk)
    dut.advance.value = 1
    if hold_cycles:
        await RisingEdge(dut.clk)
    dut.req.value = 0
    return got, cycles


@cocotb.test()
async def test_directed_and_random(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)
    corners = [0, 1, 2, 3, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0xFFFFFFFE, 0x12345678]
    cases = [(op, a, b) for op in INSTR for a in corners for b in corners]
    rng = random.Random(1)
    cases += [(rng.choice(list(INSTR)), rng.getrandbits(32), rng.getrandbits(32)) for _ in range(600)]
    for i, (op, a, b) in enumerate(cases):
        got, _ = await run_op(dut, op, a, b, hold_cycles=(2 if i % 17 == 0 else 0))
        exp = reference(op, a, b)
        assert got == exp, f"{op} {a:#010x},{b:#010x}: got {got:#010x} expected {exp:#010x}"


@cocotb.test()
async def test_back_to_back_and_kill(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)
    # A divide killed mid-flight must not leak into the next instruction.
    dut.req.value = 1
    dut.instr_id.value = INSTR["DIVU"]
    dut.a.value = 1000
    dut.b.value = 7
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.kill.value = 1
    await RisingEdge(dut.clk)
    dut.kill.value = 0
    got, _ = await run_op(dut, "MUL", 6, 7)
    assert got == 42
    # Identical back-to-back ops each compute afresh (EX reloads the same id).
    got1, c1 = await run_op(dut, "REMU", 100, 9)
    got2, c2 = await run_op(dut, "REMU", 100, 9)
    assert got1 == got2 == 1 and c1 == c2 and c1 > 30


def runCocotbTests():
    from cocotb_test.simulator import run
    root_dir = os.getcwd()
    while not os.path.exists(os.path.join(root_dir, "rtl")):
        root_dir = os.path.dirname(root_dir)
    rtl_dir = os.path.join(root_dir, "rtl")
    run(
        verilog_sources=[os.path.join(rtl_dir, "core_modules", "muldiv.v")],
        toplevel="muldiv",
        module="test_muldiv",
        simulator="verilator",
        includes=[os.path.join(rtl_dir, "include")],
        extra_env={"PYTHON3": sys.executable},
    )
