#!/usr/bin/env python3
"""Random RVV (Zve32x + Zve32f .vv) test generator for TinyGPU v2, checked against Spike.

Each test is a sequence of blocks. A block picks SEW/LMUL/vl, loads all 32 vector registers
from random data (whole-register loads), runs a few random legal instructions, then stores
all 32 registers plus scalar results into the signature. Spike and the RTL must produce the
same signature.

    vgen.py --seed N --out dir/           -> dir/test.S
"""
import argparse
import random
import struct

VLENB = 64
CLASSES = ["int", "mask", "perm", "red", "widen", "mem", "fp", "sfp", "smem"]


class Gen:
    def __init__(self, seed, classes, blocks, ops, irq=False, exact_div=False):
        self.irq = irq
        self.exact_div = exact_div
        self.r = random.Random(seed)
        self.classes = classes
        self.blocks = blocks
        self.ops = ops
        self.lines = []
        self.nscalar = 0          # scalar result slots used

    # Divide / square root are approximate on the default hardware (vpu_top FAST_DIV=1), so they
    # are left out of the bit-exact comparison unless --exact-div (gpu/tb checks their accuracy).
    APPROX_OPS = {"vfdiv", "vfrdiv", "vfsqrt.v", "fdiv", "fsqrt"}

    def ch(self, ops):
        return self.r.choice(ops if self.exact_div else [o for o in ops if o not in self.APPROX_OPS])

    def emit(self, s):
        self.lines.append("  " + s)

    # ---------------- vtype ----------------
    def pick_vtype(self, need_widen=False, fp=False):
        r = self.r
        if fp:
            sew = 32
        elif need_widen:
            sew = r.choice([8, 16])
        else:
            sew = r.choice([8, 16, 32])
        lmuls = ["m1", "m2", "m4", "m8"]
        if sew == 8:
            lmuls += ["mf2", "mf4"]
        elif sew == 16:
            lmuls += ["mf2"]
        if need_widen:
            lmuls = [l for l in lmuls if l != "m8"]   # 2*LMUL must be <= 8
        lmul = r.choice(lmuls)
        lm = {"m1": 1, "m2": 2, "m4": 4, "m8": 8, "mf2": 0.5, "mf4": 0.25}[lmul]
        vlmax = int(VLENB * 8 // sew * lm)
        vl = r.choice([vlmax, r.randint(0, vlmax), r.randint(1, max(1, vlmax)), 1])
        return sew, lmul, lm, vlmax, vl

    def group(self, lm, avoid=(), allow_v0=True):
        """Random register group base aligned to EMUL lm, not overlapping avoid groups."""
        n = max(1, int(lm))
        cands = [b for b in range(0, 32, n) if (allow_v0 or b != 0)]
        cands = [b for b in cands if b + n <= 32 and
                 not any(b < a0 + an and a0 < b + n for (a0, an) in avoid)]
        b = self.r.choice(cands)
        return b

    def rand_x(self, reg):
        r = self.r
        v = r.choice([r.getrandbits(32), r.randint(0, 40), r.randint(-40, 40) & 0xFFFFFFFF,
                      0x80000000, 0x7FFFFFFF, 0xFFFFFFFF, r.randint(0, 600)])
        self.emit(f"li {reg}, {v}")

    def store_scalar(self, reg):
        off = 4 * self.nscalar - getattr(self, "s2_base", 0)
        if off > 2044:                       # keep within the 12-bit store immediate
            self.emit("addi s2, s2, 2040")
            self.s2_base = getattr(self, "s2_base", 0) + 2040
            off -= 2040
        self.emit(f"sw {reg}, {off}(s2)")
        self.nscalar += 1

    # ---------------- instruction pickers ----------------
    def mask_suffix(self, vd, allow=True):
        if allow and self.r.random() < 0.5 and vd != 0:
            return ", v0.t"
        return ""

    def gen_int(self, sew, lm):
        r = self.r
        op = r.choice(["vadd", "vsub", "vrsub", "vminu", "vmin", "vmaxu", "vmax", "vand", "vor",
                       "vxor", "vsll", "vsrl", "vsra", "vsaddu", "vsadd", "vssubu", "vssub",
                       "vssrl", "vssra", "vsmul", "vaaddu", "vaadd", "vasubu", "vasub", "vmul",
                       "vmulh", "vmulhu", "vmulhsu", "vdiv", "vdivu", "vrem", "vremu", "vmacc",
                       "vnmsac", "vmadd", "vnmsub", "vmerge", "vmv", "vadc", "vsbc"])
        forms = {
            "vrsub": ["vx", "vi"], "vsub": ["vv", "vx"], "vssubu": ["vv", "vx"], "vssub": ["vv", "vx"],
            "vminu": ["vv", "vx"], "vmin": ["vv", "vx"], "vmaxu": ["vv", "vx"], "vmax": ["vv", "vx"],
            "vsmul": ["vv", "vx"], "vaaddu": ["vv", "vx"], "vaadd": ["vv", "vx"], "vasubu": ["vv", "vx"],
            "vasub": ["vv", "vx"], "vmul": ["vv", "vx"], "vmulh": ["vv", "vx"], "vmulhu": ["vv", "vx"],
            "vmulhsu": ["vv", "vx"], "vdiv": ["vv", "vx"], "vdivu": ["vv", "vx"], "vrem": ["vv", "vx"],
            "vremu": ["vv", "vx"], "vmacc": ["vv", "vx"], "vnmsac": ["vv", "vx"], "vmadd": ["vv", "vx"],
            "vnmsub": ["vv", "vx"], "vsbc": ["vvm", "vxm"], "vadc": ["vvm", "vxm", "vim"],
            "vmerge": ["vvm", "vxm", "vim"], "vmv": ["v.v", "v.x", "v.i"],
        }.get(op, ["vv", "vx", "vi"])
        form = r.choice(forms)
        vd = self.group(lm, allow_v0=False)
        vs2 = self.group(lm)
        vs1 = self.group(lm)
        self.rand_x("a1")
        imm = r.randint(-16, 15) if op not in ("vsll", "vsrl", "vsra", "vssrl", "vssra") else r.randint(0, 31)
        if op == "vmv":
            src = {"v.v": f"v{vs1}", "v.x": "a1", "v.i": str(imm)}[form]
            self.emit(f"vmv.{form} v{vd}, {src}")
        elif form in ("vvm", "vxm", "vim"):
            src = {"vvm": f"v{vs1}", "vxm": "a1", "vim": str(imm)}[form]
            self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}, v0")
        else:
            src = {"vv": f"v{vs1}", "vx": "a1", "vi": str(imm)}[form]
            if op in ("vmacc", "vnmsac", "vmadd", "vnmsub"):
                self.emit(f"{op}.{form} v{vd}, {src}, v{vs2}{self.mask_suffix(vd)}")
            else:
                self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}{self.mask_suffix(vd)}")

    def gen_maskdest(self, sew, lm):
        r = self.r
        op = r.choice(["vmseq", "vmsne", "vmsltu", "vmslt", "vmsleu", "vmsle", "vmsgtu", "vmsgt",
                       "vmadc", "vmsbc"])
        n = max(1, int(lm))
        vs2 = self.group(lm)
        vs1 = self.group(lm)
        vd = self.group(1, avoid=[(vs2, n), (vs1, n)])
        self.rand_x("a1")
        imm = r.randint(-16, 15)
        if op in ("vmadc", "vmsbc"):
            forms = ["vv", "vx"] + (["vi"] if op == "vmadc" else [])
            form = r.choice(forms)
            src = {"vv": f"v{vs1}", "vx": "a1", "vi": str(imm)}[form]
            if r.random() < 0.5:
                self.emit(f"{op}.{form}m v{vd}, v{vs2}, {src}, v0")
            else:
                self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}")
            return
        forms = {"vmsltu": ["vv", "vx"], "vmslt": ["vv", "vx"], "vmsgtu": ["vx", "vi"],
                 "vmsgt": ["vx", "vi"]}.get(op, ["vv", "vx", "vi"])
        form = r.choice(forms)
        src = {"vv": f"v{vs1}", "vx": "a1", "vi": str(imm)}[form]
        self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}{self.mask_suffix(vd)}")

    def gen_mask(self, sew, lm):
        r = self.r
        kind = r.choice(["logic", "logic", "cpop", "first", "set", "iota", "vid", "cmp"])
        if kind == "cmp":
            return self.gen_maskdest(sew, lm)
        if kind == "logic":
            op = r.choice(["vmand", "vmnand", "vmandn", "vmxor", "vmor", "vmnor", "vmorn", "vmxnor"])
            self.emit(f"{op}.mm v{r.randrange(32)}, v{r.randrange(32)}, v{r.randrange(32)}")
        elif kind in ("cpop", "first"):
            self.emit(f"v{'cpop' if kind == 'cpop' else 'first'}.m a2, v{r.randrange(32)}{self.mask_suffix(1)}")
            self.store_scalar("a2")
        elif kind == "set":
            op = r.choice(["vmsbf", "vmsif", "vmsof"])
            vs2 = r.randrange(32)
            vd = r.choice([x for x in range(1, 32) if x != vs2])
            self.emit(f"{op}.m v{vd}, v{vs2}{self.mask_suffix(vd)}")
        elif kind == "iota":
            vs2 = r.randrange(32)
            vd = self.group(lm, avoid=[(vs2, 1), (0, 1)], allow_v0=False)
            self.emit(f"viota.m v{vd}, v{vs2}{self.mask_suffix(vd)}")
        else:
            vd = self.group(lm, allow_v0=False)
            self.emit(f"vid.v v{vd}{self.mask_suffix(vd)}")

    def gen_perm(self, sew, lm):
        r = self.r
        n = max(1, int(lm))
        kind = r.choice(["slideup", "slidedown", "slide1up", "slide1down", "gather", "gather16",
                         "compress", "vmvnr", "mvxs", "mvsx", "ext"])
        vs2 = self.group(lm)
        self.rand_x("a1")
        if kind in ("slideup", "slidedown"):
            vd = self.group(lm, avoid=[(vs2, n)] if kind == "slideup" else [], allow_v0=False)
            if r.random() < 0.5:
                self.emit(f"li a1, {r.randint(0, 70)}")
                self.emit(f"v{kind}.vx v{vd}, v{vs2}, a1{self.mask_suffix(vd)}")
            else:
                self.emit(f"v{kind}.vi v{vd}, v{vs2}, {r.randint(0, 31)}{self.mask_suffix(vd)}")
        elif kind in ("slide1up", "slide1down"):
            vd = self.group(lm, avoid=[(vs2, n)] if kind == "slide1up" else [], allow_v0=False)
            self.emit(f"v{kind}.vx v{vd}, v{vs2}, a1{self.mask_suffix(vd)}")
        elif kind == "gather":
            vs1 = self.group(lm)
            vd = self.group(lm, avoid=[(vs2, n), (vs1, n)], allow_v0=False)
            form = r.choice(["vv", "vx", "vi"])
            if form == "vx":
                self.emit(f"li a1, {r.randint(0, 80)}")
            src = {"vv": f"v{vs1}", "vx": "a1", "vi": str(r.randint(0, 31))}[form]
            self.emit(f"vrgather.{form} v{vd}, v{vs2}, {src}{self.mask_suffix(vd)}")
        elif kind == "gather16":
            # index EEW=16: EMUL = 16/SEW * LMUL
            el = 16 / sew * lm
            if el > 8 or el < 0.125:
                return self.gen_int(sew, lm)
            ne = max(1, int(el))
            vs1 = self.group(el)
            vd = self.group(lm, avoid=[(vs2, n), (vs1, ne)], allow_v0=False)
            self.emit(f"vrgatherei16.vv v{vd}, v{vs2}, v{vs1}{self.mask_suffix(vd)}")
        elif kind == "compress":
            vs1 = r.randrange(32)
            vd = self.group(lm, avoid=[(vs2, n), (vs1, 1)])
            self.emit(f"vcompress.vm v{vd}, v{vs2}, v{vs1}")
        elif kind == "vmvnr":
            k = r.choice([1, 2, 4, 8])
            self.emit(f"vmv{k}r.v v{r.randrange(0, 32, k)}, v{r.randrange(0, 32, k)}")
        elif kind == "mvxs":
            self.emit(f"vmv.x.s a2, v{r.randrange(32)}")
            self.store_scalar("a2")
        elif kind == "mvsx":
            self.emit(f"vmv.s.x v{r.randrange(32)}, a1")
        else:
            f = r.choice([2, 4]) if sew == 32 else 2
            if sew == 8:
                return self.gen_int(sew, lm)
            el = lm / f
            if el < 0.125 or (sew == 16 and f == 4):
                return self.gen_int(sew, lm)
            vs2 = self.group(el)
            vd = self.group(lm, avoid=[(vs2, max(1, int(el)))], allow_v0=False)
            self.emit(f"v{r.choice(['z', 's'])}ext.vf{f} v{vd}, v{vs2}{self.mask_suffix(vd)}")

    def gen_red(self, sew, lm):
        r = self.r
        op = r.choice(["vredsum", "vredand", "vredor", "vredxor", "vredminu", "vredmin",
                       "vredmaxu", "vredmax"])
        self.emit(f"{op}.vs v{r.randrange(32)}, v{self.group(lm)}, v{r.randrange(32)}{self.mask_suffix(1)}")

    def gen_widen(self, sew, lm):
        r = self.r
        n = max(1, int(lm))
        nw = max(1, int(2 * lm))
        kind = r.choice(["w", "w", "wv", "narrow", "wred"])
        self.rand_x("a1")
        if kind == "w":
            op = r.choice(["vwaddu", "vwadd", "vwsubu", "vwsub", "vwmulu", "vwmul", "vwmulsu",
                           "vwmaccu", "vwmacc", "vwmaccsu", "vwmaccus"])
            vs2 = self.group(lm)
            vs1 = self.group(lm)
            vd = self.group(2 * lm, avoid=[(vs2, n), (vs1, n)], allow_v0=False)
            form = "vx" if op == "vwmaccus" else r.choice(["vv", "vx"])
            src = f"v{vs1}" if form == "vv" else "a1"
            if op.startswith("vwmacc"):
                self.emit(f"{op}.{form} v{vd}, {src}, v{vs2}{self.mask_suffix(vd)}")
            else:
                self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}{self.mask_suffix(vd)}")
        elif kind == "wv":
            op = r.choice(["vwaddu", "vwadd", "vwsubu", "vwsub"])
            vs2 = self.group(2 * lm)
            vs1 = self.group(lm)
            vd = self.group(2 * lm, avoid=[(vs1, n)], allow_v0=False)
            form = r.choice(["wv", "wx"])
            src = f"v{vs1}" if form == "wv" else "a1"
            self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}{self.mask_suffix(vd)}")
        elif kind == "narrow":
            op = r.choice(["vnsrl", "vnsra", "vnclipu", "vnclip"])
            vs2 = self.group(2 * lm)
            vs1 = self.group(lm)
            vd = self.group(lm, avoid=[(vs2, nw)], allow_v0=False)
            form = r.choice(["wv", "wx", "wi"])
            src = {"wv": f"v{vs1}", "wx": "a1", "wi": str(r.randint(0, 31))}[form]
            self.emit(f"{op}.{form} v{vd}, v{vs2}, {src}{self.mask_suffix(vd)}")
        else:
            op = r.choice(["vwredsumu", "vwredsum"])
            self.emit(f"{op}.vs v{r.randrange(32)}, v{self.group(lm)}, v{r.randrange(32)}{self.mask_suffix(1)}")

    def load_f(self, freg):
        # random single in f[freg] via its bit pattern
        self.emit(f"li t3, 0x{self.fp_bits():08x}")
        self.emit(f"fmv.w.x {freg}, t3")

    def fp_bits(self):
        r = self.r
        k = r.random()
        if k < 0.2:
            return r.choice([0x00000000, 0x80000000, 0x7F800000, 0xFF800000, 0x7FC00000, 0x7F800001,
                             0x00000001, 0x3F800000, 0xBF800000, 0x7F7FFFFF, 0x00800000])
        if k < 0.8:
            return struct.unpack("<I", struct.pack("<f", r.uniform(-500, 500)))[0]
        return r.getrandbits(32)

    def gen_fp(self, sew, lm):
        r = self.r
        kind = r.choice(["arith", "arith", "fma", "cmp", "unary", "red", "vf", "vf", "fmv"])
        if kind == "vf":
            self.load_f("fa0")
            vs2 = self.group(lm)
            vd = self.group(lm, allow_v0=False)
            op = self.ch(["vfadd", "vfsub", "vfrsub", "vfmul", "vfdiv", "vfrdiv", "vfmin", "vfmax",
                           "vfsgnj", "vfsgnjn", "vfsgnjx", "vfmacc", "vfnmacc", "vfmsac", "vfnmsac",
                           "vfmadd", "vfnmadd", "vfmsub", "vfnmsub", "vmfeq", "vmfne", "vmflt", "vmfle",
                           "vmfgt", "vmfge", "vfslide1up", "vfslide1down", "vfmerge"])
            if op.startswith("vmf"):
                n = max(1, int(lm))
                vdm = self.group(1, avoid=[(vs2, n)])
                self.emit(f"{op}.vf v{vdm}, v{vs2}, fa0{self.mask_suffix(vdm)}")
            elif op == "vfmerge":
                self.emit(f"vfmerge.vfm v{vd}, v{vs2}, fa0, v0")
            elif op in ("vfmacc", "vfnmacc", "vfmsac", "vfnmsac", "vfmadd", "vfnmadd", "vfmsub", "vfnmsub"):
                self.emit(f"{op}.vf v{vd}, fa0, v{vs2}{self.mask_suffix(vd)}")
            else:
                if op == "vfslide1up":
                    vd = self.group(lm, avoid=[(vs2, max(1, int(lm)))], allow_v0=False)
                self.emit(f"{op}.vf v{vd}, v{vs2}, fa0{self.mask_suffix(vd)}")
            return
        if kind == "fmv":
            self.load_f("fa1")
            k = r.choice(["vfmv.v.f", "vfmv.s.f", "vfmv.f.s"])
            vd = self.group(lm, allow_v0=False)
            if k == "vfmv.v.f":
                self.emit(f"vfmv.v.f v{vd}, fa1")
            elif k == "vfmv.s.f":
                self.emit(f"vfmv.s.f v{r.randrange(32)}, fa1")
            else:
                self.emit(f"vfmv.f.s fa2, v{r.randrange(32)}")
                self.emit("fmv.x.w a2, fa2")
                self.store_scalar("a2")
            return
        vs2 = self.group(lm)
        vs1 = self.group(lm)
        vd = self.group(lm, allow_v0=False)
        if kind == "arith":
            op = self.ch(["vfadd", "vfsub", "vfmul", "vfdiv", "vfmin", "vfmax", "vfsgnj",
                           "vfsgnjn", "vfsgnjx"])
            self.emit(f"{op}.vv v{vd}, v{vs2}, v{vs1}{self.mask_suffix(vd)}")
        elif kind == "fma":
            op = r.choice(["vfmacc", "vfnmacc", "vfmsac", "vfnmsac", "vfmadd", "vfnmadd",
                           "vfmsub", "vfnmsub"])
            self.emit(f"{op}.vv v{vd}, v{vs1}, v{vs2}{self.mask_suffix(vd)}")
        elif kind == "cmp":
            op = r.choice(["vmfeq", "vmfne", "vmflt", "vmfle"])
            n = max(1, int(lm))
            vdm = self.group(1, avoid=[(vs2, n), (vs1, n)])
            self.emit(f"{op}.vv v{vdm}, v{vs2}, v{vs1}{self.mask_suffix(vdm)}")
        elif kind == "unary":
            op = self.ch(["vfsqrt.v", "vfrsqrt7.v", "vfrec7.v", "vfclass.v", "vfcvt.xu.f.v",
                           "vfcvt.x.f.v", "vfcvt.f.xu.v", "vfcvt.f.x.v", "vfcvt.rtz.xu.f.v",
                           "vfcvt.rtz.x.f.v"])
            self.emit(f"{op} v{vd}, v{vs2}{self.mask_suffix(vd)}")
        elif kind == "red":
            op = r.choice(["vfredusum", "vfredosum", "vfredmin", "vfredmax"])
            self.emit(f"{op}.vs v{r.randrange(32)}, v{vs2}, v{r.randrange(32)}{self.mask_suffix(1)}")
        else:
            # int16 <-> f32 conversions run at SEW=16
            return None

    def gen_sfp(self, sew, lm):
        r = self.r
        dbl = r.random() < 0.5
        sfx = "d" if dbl else "s"
        # operands: random data words (as singles / doubles) from vdata
        for i, fr in enumerate(("fa0", "fa1", "fa2")):
            off = r.randrange(0, 4096 - 8, 8)
            self.emit("la t3, vdata")
            self.emit(f"li t4, {off}")
            self.emit("add t3, t3, t4")
            self.emit(f"fl{'d' if dbl else 'w'} {fr}, 0(t3)")
        rm = r.choice(["", ", rne", ", rtz", ", rdn", ", rup", ", rmm"])
        op = self.ch(["fadd", "fsub", "fmul", "fdiv", "fsqrt", "fmin", "fmax", "fsgnj", "fsgnjn",
                       "fsgnjx", "fmadd", "fmsub", "fnmsub", "fnmadd", "feq", "flt", "fle", "fclass",
                       "fcvt.w", "fcvt.wu", "fcvt.from_w", "fcvt.from_wu", "fcvt.sd", "fmv"])
        if op in ("fadd", "fsub", "fmul", "fdiv"):
            self.emit(f"{op}.{sfx} fa3, fa0, fa1{rm}")
        elif op == "fsqrt":
            self.emit(f"fsqrt.{sfx} fa3, fa0{rm}")
        elif op in ("fmin", "fmax", "fsgnj", "fsgnjn", "fsgnjx"):
            self.emit(f"{op}.{sfx} fa3, fa0, fa1")
        elif op in ("fmadd", "fmsub", "fnmsub", "fnmadd"):
            self.emit(f"{op}.{sfx} fa3, fa0, fa1, fa2{rm}")
        elif op in ("feq", "flt", "fle"):
            self.emit(f"{op}.{sfx} a2, fa0, fa1")
            self.store_scalar("a2")
        elif op == "fclass":
            self.emit(f"fclass.{sfx} a2, fa0")
            self.store_scalar("a2")
        elif op in ("fcvt.w", "fcvt.wu"):
            self.emit(f"{op}.{sfx} a2, fa0{rm}")
            self.store_scalar("a2")
        elif op in ("fcvt.from_w", "fcvt.from_wu"):
            self.rand_x("a1")
            # int -> double is exact and takes no rounding mode
            self.emit(f"fcvt.{sfx}.{'w' if op == 'fcvt.from_w' else 'wu'} fa3, a1{'' if dbl else rm}")
        elif op == "fcvt.sd":
            if dbl:
                self.emit(f"fcvt.s.d fa3, fa0{rm}")
            else:
                self.emit("fcvt.d.s fa3, fa0")
        else:
            self.rand_x("a1")
            self.emit("fmv.w.x fa3, a1")
            self.emit("fmv.x.w a2, fa0")
            self.store_scalar("a2")
        # results to the signature: the whole 64-bit register, then the flags
        self.emit("addi sp, s2, 1024")
        self.emit(f"fsd fa3, {8 * (self.nscalar % 64)}(sp)")
        self.emit("frflags a2")
        self.store_scalar("a2")
        self.emit("fsflags zero")

    def gen_smem(self, sew, lm):
        # Scalar memory stress: a 32 KB window (4x the D-cache), same-set aliases 4 KB apart,
        # store->load pairs, all sizes, AMOs, byte copies like memcpy/gzip do.
        r = self.r
        self.emit("la s6, bigbuf")
        for _ in range(24):
            k = r.random()
            set_off = r.randrange(0, 4096, 4)
            way = r.randrange(8) * 4096           # 8 aliases of the same set
            off = (set_off + way) % 32768
            self.emit(f"li t5, {off}")
            self.emit("add t5, s6, t5")
            if k < 0.25:
                op = r.choice(["sb", "sh", "sw"])
                o = {"sb": r.randrange(4), "sh": r.choice([0, 2]), "sw": 0}[op]
                self.rand_x("t6")
                self.emit(f"{op} t6, {o}(t5)")
                ld = r.choice(["lb", "lbu", "lh", "lhu", "lw"])
                lo = {"lb": r.randrange(4), "lbu": r.randrange(4), "lh": r.choice([0, 2]),
                      "lhu": r.choice([0, 2]), "lw": 0}[ld]
                self.emit(f"{ld} a2, {lo}(t5)")       # store->load, often the same word
                self.store_scalar("a2")
            elif k < 0.45:
                ld = r.choice(["lb", "lbu", "lh", "lhu", "lw"])
                lo = {"lb": r.randrange(4), "lbu": r.randrange(4), "lh": r.choice([0, 2]),
                      "lhu": r.choice([0, 2]), "lw": 0}[ld]
                self.emit(f"{ld} a2, {lo}(t5)")
                self.store_scalar("a2")
            elif k < 0.6:
                # byte copy loop (memcpy-like), 1..40 bytes between aliasing addresses
                n = r.randint(1, 40)
                d = (off + r.randrange(8) * 4096 + r.randrange(64)) % (32768 - 64)
                self.emit(f"li t4, {d}")
                self.emit("add t4, s6, t4")
                self.emit(f"li t3, {n}")
                self.emit("1: lbu t6, 0(t5)")
                self.emit("sb t6, 0(t4)")
                self.emit("addi t5, t5, 1")
                self.emit("addi t4, t4, 1")
                self.emit("addi t3, t3, -1")
                self.emit("bnez t3, 1b")
            elif k < 0.75:
                self.rand_x("t6")
                op = r.choice(["amoadd.w", "amoswap.w", "amoxor.w", "amomax.w", "amominu.w"])
                self.emit(f"{op} a2, t6, (t5)")
                self.store_scalar("a2")
            elif k < 0.85:
                # Retry loop: SC may fail spuriously (Spike drops reservations every 5000 instrs)
                self.rand_x("t6")
                self.emit("3: lr.w a2, (t5)")
                self.emit("sc.w a3, t6, (t5)")
                self.emit("bnez a3, 3b")
                self.store_scalar("a2")
            else:
                # word fill (memset-like) over a few lines
                n = r.randint(1, 24)
                self.rand_x("t6")
                self.emit(f"li t3, {n}")
                self.emit("2: sw t6, 0(t5)")
                self.emit("addi t5, t5, 4")
                self.emit("addi t3, t3, -1")
                self.emit("bnez t3, 2b")

    def gen_mem(self, sew, lm):
        r = self.r
        kind = r.choice(["unit", "unit", "strided", "indexed", "seg", "mask", "whole", "ff"])
        eew = r.choice([8, 16, 32])
        el = eew / sew * lm
        if el > 8 or el < 0.125:
            eew, el = sew, lm
        ne = max(1, int(el))
        self.emit("la a3, scratch")
        if kind in ("unit", "ff"):
            vd = self.group(el, allow_v0=False)
            op = "vle" if kind == "unit" else "vle"
            if r.random() < 0.5 or kind == "ff":
                suf = "ff" if kind == "ff" else ""
                self.emit(f"{op}{eew}{suf}.v v{vd}, (a3){self.mask_suffix(vd)}")
            else:
                self.emit(f"vse{eew}.v v{vd}, (a3){self.mask_suffix(vd)}")
                self.emit(f"vle{eew}.v v{self.group(el, allow_v0=False)}, (a3)")
        elif kind == "strided":
            vd = self.group(el, allow_v0=False)
            self.emit(f"li a4, {r.choice([0, eew // 8, 2 * eew // 8, -eew // 8, 12])}")
            self.emit("li t2, 8192")
            if r.random() < 0.5:
                self.emit("add a3, a3, t2")
                self.emit(f"vlse{eew}.v v{vd}, (a3), a4{self.mask_suffix(vd)}")
            else:
                self.emit("add a3, a3, t2")
                self.emit(f"vsse{eew}.v v{vd}, (a3), a4{self.mask_suffix(vd)}")
        elif kind == "indexed":
            # index values: small multiples of the data size, built with vid + shift
            ie = r.choice([8, 16, 32])
            iel = ie / sew * lm
            if iel > 8 or iel < 0.125:
                ie, iel = sew, lm
            ni = max(1, int(iel))
            vi_ = self.group(iel)
            vd = self.group(lm, avoid=[(vi_, ni)], allow_v0=False)
            self.emit(f"vsetvli t0, zero, e{ie}, {self.lmul_str(iel)}, tu, mu")
            self.emit(f"vid.v v{vi_}")
            self.emit(f"vand.vi v{vi_}, v{vi_}, 15")
            self.emit(f"vsll.vi v{vi_}, v{vi_}, {(sew // 8).bit_length() - 1}")
            self.restore_vtype()
            ld = r.random() < 0.6
            o = r.choice(["u", "o"])
            if ld:
                self.emit(f"vl{o}xei{ie}.v v{vd}, (a3), v{vi_}{self.mask_suffix(vd)}")
            else:
                self.emit(f"vs{o}xei{ie}.v v{vd}, (a3), v{vi_}{self.mask_suffix(vd)}")
        elif kind == "seg":
            nf = r.randint(2, 4)
            if nf * ne > 8:
                nf = 2
                if nf * ne > 8:
                    return self.gen_int(sew, lm)
            vd = r.choice([b for b in range(ne, 32 - nf * ne + 1, ne)])
            if r.random() < 0.5:
                self.emit(f"vlseg{nf}e{eew}.v v{vd}, (a3){self.mask_suffix(vd)}")
            else:
                self.emit(f"vsseg{nf}e{eew}.v v{vd}, (a3){self.mask_suffix(vd)}")
                self.emit(f"vle{eew}.v v{self.group(el, allow_v0=False)}, (a3)")
        elif kind == "mask":
            vd = r.randrange(32)
            if r.random() < 0.5:
                self.emit(f"vlm.v v{vd}, (a3)")
            else:
                self.emit(f"vsm.v v{vd}, (a3)")
        else:
            k = r.choice([1, 2, 4, 8])
            v = r.randrange(0, 32, k)
            if r.random() < 0.5:
                self.emit(f"vl{k}re{r.choice([8, 16, 32])}.v v{v}, (a3)")
            else:
                self.emit(f"vs{k}r.v v{v}, (a3)")
        # make the scratch contents part of the signature
        self.emit("la a3, scratch")
        self.emit("addi a5, s2, 0")

    @staticmethod
    def lmul_str(l):
        return {8: "m8", 4: "m4", 2: "m2", 1: "m1", 0.5: "mf2", 0.25: "mf4", 0.125: "mf8"}[l]

    def restore_vtype(self):
        self.emit("vsetvl t0, s4, s5")

    # ---------------- program ----------------
    def block(self, bi):
        r = self.r
        cls = r.choice(self.classes)
        sew, lmul, lm, vlmax, vl = self.pick_vtype(need_widen=(cls == "widen"), fp=(cls == "fp"))
        self.emit(f"# block {bi}: {cls} e{sew} {lmul} vl={vl}")
        # load all registers from the random data area (with a block-dependent offset)
        self.emit(f"li t0, {(bi * 197) % 2048 & ~63}")
        self.emit("la t1, vdata")
        self.emit("add t1, t1, t0")
        for b in (0, 8, 16, 24):
            self.emit(f"vl8re32.v v{b}, (t1)")
            self.emit("addi t1, t1, 512")
        self.emit(f"li s4, {vl}")
        self.emit(f"vsetvli s4, s4, e{sew}, {lmul}, tu, mu")
        self.emit("csrr s5, vtype")
        self.emit(f"csrwi vxrm, {r.randint(0, 3)}")
        if cls in ("fp", "sfp"):
            self.emit(f"csrwi frm, {r.choice([0, 1, 2, 3, 4])}")
        for _ in range(self.ops):
            getattr(self, {"int": "gen_int", "mask": "gen_mask", "perm": "gen_perm",
                           "red": "gen_red", "widen": "gen_widen", "mem": "gen_mem",
                           "fp": "gen_fp", "sfp": "gen_sfp", "smem": "gen_smem"}[cls])(sew, lm)
        self.emit("csrr a2, vl")
        self.store_scalar("a2")
        self.emit("csrr a2, vxsat")
        self.store_scalar("a2")
        self.emit("csrwi vxsat, 0")
        self.emit("frflags a2")
        self.store_scalar("a2")
        self.emit("fsflags zero")
        # dump all registers and the scratch area
        self.emit(f"li t0, {bi * 2048}")
        self.emit("la t1, vsig")
        self.emit("add t1, t1, t0")
        for b in (0, 8, 16, 24):
            self.emit(f"vs8r.v v{b}, (t1)")
            self.emit("addi t1, t1, 512")
        if cls == "mem":
            self.emit(f"li t0, {bi * 2048}")
            self.emit("la t1, msig")
            self.emit("add t1, t1, t0")
            self.emit("la a3, scratch")
            for b in range(4):
                self.emit("vl8re32.v v8, (a3)")
                self.emit("vs8r.v v8, (t1)")
                self.emit("addi a3, a3, 512")
                self.emit("addi t1, t1, 512")

    def program(self):
        r = self.r
        out = [".section .text.init", ".globl _start", "_start:"]
        self.lines = []
        self.emit("li t0, 0x2200")         # mstatus.VS = initial, FS = initial
        self.emit("csrs mstatus, t0")
        self.emit("fsflags zero")
        self.emit("csrs mstatus, t0")
        self.emit("la s2, ssig")
        if self.irq:
            # Frequent machine timer interrupts. The handler leaves no architectural trace,
            # so the signature must match Spike whatever instruction each interrupt lands on.
            self.emit("la t0, irq_handler")
            self.emit("csrw mtvec, t0")
            self.emit("la t0, irq_area")
            self.emit("csrw mscratch, t0")
            self.emit("li t0, 0x2004000")
            self.emit("li t1, 50")
            self.emit("sw t1, 0(t0)")
            self.emit("sw zero, 4(t0)")
            self.emit("li t0, 0x80")
            self.emit("csrs mie, t0")
            self.emit("csrsi mstatus, 8")
        for bi in range(self.blocks):
            self.block(bi)
        if self.irq:
            self.emit("csrci mstatus, 8")
        self.emit("li t0, 1")
        self.emit("la t1, tohost")
        self.emit("sw t0, 0(t1)")
        self.emit("1: j 1b")
        out += self.lines
        if self.irq:
            out += [".align 4", "irq_handler:",
                    "  csrrw t0, mscratch, t0",        # t0 = save area
                    "  sw t1, 4(t0)", "  sw t2, 8(t0)",
                    "  csrr t1, mscratch", "  sw t1, 0(t0)",
                    "  csrr t1, mcause", "  bgez t1, irq_bad",
                    "  lw t1, 12(t0)",                  # xorshift32 interval
                    "  slli t2, t1, 13", "  xor t1, t1, t2", "  srli t2, t1, 17", "  xor t1, t1, t2",
                    "  slli t2, t1, 5", "  xor t1, t1, t2", "  sw t1, 12(t0)",
                    "  andi t1, t1, 255", "  addi t1, t1, 20",
                    "  li t2, 0x200BFF8", "  lw t2, 0(t2)", "  add t1, t1, t2",
                    "  li t2, 0x2004000", "  sw t1, 0(t2)", "  sw zero, 4(t2)",
                    "  lw t1, 16(t0)", "  addi t1, t1, 1", "  sw t1, 16(t0)",
                    "  csrw mscratch, t0",
                    "  lw t1, 4(t0)", "  lw t2, 8(t0)", "  lw t0, 0(t0)",
                    "  mret",
                    "irq_bad:", "  li t1, 3", "  la t2, tohost", "  sw t1, 0(t2)", "2: j 2b"]

        def fp_word():
            k = r.random()
            if k < 0.15:
                return r.choice([0x00000000, 0x80000000, 0x7F800000, 0xFF800000, 0x7FC00000,
                                 0x7F800001, 0x00000001, 0x807FFFFF, 0x3F800000, 0xBF800000,
                                 0x7F7FFFFF, 0x00800000])
            if k < 0.6:
                return struct.unpack("<I", struct.pack("<f", r.uniform(-1000, 1000)))[0]
            if k < 0.75:
                return struct.unpack("<I", struct.pack("<f", float(r.randint(-70000, 70000))))[0]
            return r.getrandbits(32)

        data = [fp_word() if r.random() < 0.5 else r.getrandbits(32) for _ in range(1024)]
        out += [".section .tohost,\"aw\",@progbits", ".align 6", ".globl tohost", "tohost: .dword 0",
                ".globl fromhost", "fromhost: .dword 0", ".data", ".align 6", "vdata:"]
        out += [f"  .word 0x{w:08x}" for w in data]
        out += [".align 4", "irq_area: .word 0, 0, 0, 0x%08x, 0" % (r.getrandbits(31) | 1)]
        out += [".align 6", "scratch:"] + [f"  .word 0x{r.getrandbits(32):08x}" for _ in range(4096)]
        out += [".align 6", ".globl begin_signature", "begin_signature:",
                "ssig: .fill 2048, 4, 0",
                "bigbuf:"] + [f"  .word 0x{r.getrandbits(32):08x}" for _ in range(8192)] + [
                f"vsig: .fill {self.blocks * 512}, 4, 0",
                f"msig: .fill {self.blocks * 512}, 4, 0",
                ".globl end_signature", "end_signature:"]
        return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--classes", default=",".join(CLASSES))
    ap.add_argument("--blocks", type=int, default=12)
    ap.add_argument("--ops", type=int, default=6)
    ap.add_argument("--irq", action="store_true")
    ap.add_argument("--exact-div", action="store_true", help="include divide/sqrt (hardware built with FAST_DIV=0)")
    a = ap.parse_args()
    g = Gen(a.seed, a.classes.split(","), a.blocks, a.ops, a.irq, a.exact_div)
    with open(a.out, "w") as f:
        f.write(g.program())


if __name__ == "__main__":
    main()
