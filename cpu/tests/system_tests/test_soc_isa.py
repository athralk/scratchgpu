"""riscv-tests on the FPGA SoC (soc_top: caches, TLBs, page walker, AXI DDR model).

Runs every rv32 ui/um/ua/si/mi test (-p physical and -v virtual-memory variants) found in
RISCV_TESTS_DIR (default ~/opt/riscv-tests/isa) on the standalone Verilator build in
sim/soc (no cocotb: ~150x faster). Pass = the test writes 1 to tohost.
Set SOC_ISA_FILTER to a regex to run a subset, SOC_PLUSARGS for extra plusargs (+trace).
"""
import os
import re
import subprocess
from pathlib import Path

import pytest

MAX_CYCLES = int(os.environ.get("SOC_ISA_MAX_CYCLES", "4000000"))
SKIP = {"ma_data"}  # misaligned accesses trap on this core (allowed by the spec)
# vpu_top FAST_DIV=1 (the default): divide/sqrt are approximate (within 1-2 ulp) and raise the
# inexact flag even for exact results such as sqrt(10000), which these tests check bit for bit.
# Set SOC_EXACT_DIV=1 after building with FAST_DIV=0 to require them to pass.
APPROX = set() if os.environ.get("SOC_EXACT_DIV") == "1" else {"fdiv"}


def _repo_root() -> Path:
    cur = Path(__file__).resolve()
    while not (cur / "rtl").exists():
        cur = cur.parent
    return cur


def _elfs():
    d = Path(os.environ.get("RISCV_TESTS_DIR", str(Path.home() / "opt/riscv-tests/isa")))
    if not d.exists():
        return []
    pat = re.compile(os.environ.get("SOC_ISA_FILTER", r"^rv32(ui|um|ua|uf|ud|si|mi)-[pv]-\w+$"))
    return [f for f in sorted(d.iterdir())
            if pat.match(f.name) and f.suffix == "" and f.name.split("-")[-1] not in SKIP]


@pytest.fixture(scope="session")
def soc_sim():
    sim_dir = _repo_root() / "sim" / "soc"
    subprocess.run(["make", "-s", "-C", str(sim_dir)], check=True)
    return sim_dir / "obj" / "Vsoc_sim_top"


def _elf_to_hex(elf: Path, out_dir: Path) -> tuple[Path, int]:
    out_dir.mkdir(parents=True, exist_ok=True)
    binf = out_dir / (elf.name + ".bin")
    hexf = out_dir / (elf.name + ".hex")
    subprocess.run(["riscv64-unknown-elf-objcopy", "-O", "binary", str(elf), str(binf)], check=True)
    data = binf.read_bytes()
    binf.write_bytes(data + b"\0" * (-len(data) % 4))  # --reverse-bytes=4 needs whole words
    subprocess.run(["riscv64-unknown-elf-objcopy", "-I", "binary", "-O", "verilog",
                    "--verilog-data-width=4", "--reverse-bytes=4", str(binf), str(hexf)], check=True)
    nm = subprocess.run(["riscv64-unknown-elf-nm", str(elf)], check=True, capture_output=True, text=True)
    tohost = next(int(l.split()[0], 16) for l in nm.stdout.splitlines() if l.endswith(" tohost"))
    return hexf, tohost


def _params():
    return [pytest.param(f, marks=pytest.mark.xfail(strict=True, reason="approximate divider (FAST_DIV)"))
            if f.name.split("-")[-1] in APPROX else f for f in _elfs()]


@pytest.mark.parametrize("elf", _params(), ids=lambda p: p.name)
def runCocotbTests(elf, soc_sim, tmp_path):
    hexf, tohost = _elf_to_hex(elf, tmp_path)
    cmd = [str(soc_sim), f"+hex={hexf}", f"+tohost={tohost:x}", f"+max_cycles={MAX_CYCLES}"]
    cmd += os.environ.get("SOC_PLUSARGS", "").split()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    tail = (proc.stdout + proc.stderr)[-2000:]
    assert proc.returncode == 0, f"{elf.name}: rc={proc.returncode}\n{tail}"
