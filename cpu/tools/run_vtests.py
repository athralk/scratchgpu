#!/usr/bin/env python3
"""Run random vector tests on Spike and the SoC RTL and compare signatures.

    run_vtests.py --seeds 0-99 [--classes int,mask,...] [--jobs 12] [--keep]
"""
import argparse
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SIM = ROOT / "sim" / "soc" / "obj" / "Vsoc_sim_top"
LINK = HERE / "vtest_link.ld"
ISA = "rv32imafd_zve32f_zvl512b"
MARCH = "rv32imafd_zve32f_zvl512b"


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def run_one(seed, args, work):
    d = Path(work) / f"s{seed}"
    d.mkdir(parents=True, exist_ok=True)
    src, elf = d / "t.S", d / "t.elf"
    sh([sys.executable, str(HERE / "vgen.py"), "--seed", str(seed), "--out", str(src),
        "--classes", args.classes, "--blocks", str(args.blocks), "--ops", str(args.ops)])
    r = sh(["riscv64-unknown-elf-gcc", f"-march={MARCH}", "-mabi=ilp32", "-nostdlib",
            "-T", str(LINK), str(src), "-o", str(elf)])
    if r.returncode:
        return seed, "BUILD", r.stderr[-1500:]
    nm = {l.split()[2]: int(l.split()[0], 16) for l in sh(["riscv64-unknown-elf-nm", str(elf)]).stdout.splitlines()
          if len(l.split()) == 3}
    b, e = nm["begin_signature"], nm["end_signature"]
    r = sh(["spike", f"--isa={ISA}", f"+signature={d / 'ref.sig'}", "--signature-granularity=4", str(elf)],
           timeout=120)
    if r.returncode:
        return seed, "SPIKE", r.stderr[-800:]
    binf = d / "t.bin"
    sh(["riscv64-unknown-elf-objcopy", "-O", "binary", str(elf), str(binf)])
    data = binf.read_bytes()
    data += b"\0" * (-len(data) % 4)
    with open(d / "t.hex", "w") as f:
        f.write("@00000000\n")
        for i in range(0, len(data), 4):
            f.write("%08x\n" % int.from_bytes(data[i:i + 4], "little"))
    r = sh([str(SIM), f"+hex={d / 't.hex'}", f"+tohost={nm['tohost']:x}", f"+signature={d / 'dut.sig'}",
            f"+sig_begin={b:x}", f"+sig_end={e:x}", "+max_cycles=20000000"] + os.environ.get("SOC_PLUSARGS", "").split(), timeout=600)
    if r.returncode != 0 or not (d / "dut.sig").exists():
        return seed, "DUT", (r.stdout + r.stderr)[-800:]
    ref = (d / "ref.sig").read_text().split()
    dut = (d / "dut.sig").read_text().split()
    if ref != dut:
        diffs = [i for i, (x, y) in enumerate(zip(ref, dut)) if x != y]
        i = diffs[0]
        addr = b + 4 * i
        where = "ssig" if addr < nm["vsig"] else ("vsig" if addr < nm["msig"] else "msig")
        off = addr - nm[where]
        info = f"{len(diffs)} words differ; first at {where}+{off:#x}"
        if where != "ssig":
            info += f" (block {off // 2048}, v{(off % 2048) // 64}, byte {off % 64})"
        info += f": ref {ref[i]} dut {dut[i]}"
        return seed, "MISMATCH", info
    if not args.keep:
        for p in d.iterdir():
            p.unlink()
        d.rmdir()
    return seed, "PASS", ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-49")
    ap.add_argument("--classes", default="int,mask,perm,red,widen,mem,fp,sfp")
    ap.add_argument("--blocks", type=int, default=12)
    ap.add_argument("--ops", type=int, default=6)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--work", default=None)
    a = ap.parse_args()
    lo, _, hi = a.seeds.partition("-")
    seeds = range(int(lo), int(hi or lo) + 1)
    work = a.work or tempfile.mkdtemp(prefix="vtests_")
    fails = 0
    with ThreadPoolExecutor(a.jobs) as ex:
        for seed, status, info in ex.map(lambda s: run_one(s, a, work), seeds):
            if status != "PASS":
                fails += 1
                print(f"seed {seed}: {status} {info}")
    print(f"{len(seeds) - fails}/{len(seeds)} passed (work dir {work})")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
