// fp_rcpdiv accuracy test: random operands against the host's correctly rounded IEEE results.
// Reports the ulp error distribution per format / operation, exactness of special values,
// and the latency in cycles.
#include "Vfp_rcpdiv_tb.h"
#include "verilated.h"
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>

static Vfp_rcpdiv_tb *t;
static void tick() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }
static const int RM_HOST[5] = {FE_TONEAREST, FE_TOWARDZERO, FE_DOWNWARD, FE_UPWARD, FE_TONEAREST};

template <typename U> static int64_t ord(U v, int bits) {   // monotone integer order of a float
    int64_t s = (int64_t)(v >> (bits - 1)) & 1, m = (int64_t)(v & ((U(1) << (bits - 1)) - 1));
    return s ? -m : m;
}
struct Stat { long n = 0, exact = 0, ulp1 = 0, ulp2 = 0, more = 0, special_bad = 0, nvdz_bad = 0; int64_t maxu = 0; int lat = 0; };

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    t = new Vfp_rcpdiv_tb;
    long iters = argc > 1 ? atol(argv[1]) : 200000;
    std::mt19937_64 g(12345);
    t->rst = 1; tick(); tick(); t->rst = 0;
    Stat st[4];   // 0 div32 1 sqrt32 2 div64 3 sqrt64
    for (long i = 0; i < iters; i++) {
        int sq = (i >> 1) & 1, d64 = i & 1, rm = (i >> 2) % 5;
        uint64_t rx = g(), ry = g();
        int kind = (i >> 4) % 8;            // mostly ordinary values, some raw bit patterns
        if (!d64) {
            uint32_t x = rx, y = ry;
            if (kind < 6) { float fx = std::ldexp(1.0f + (rx % 1000000) / 1e6f, (int)(rx >> 40) % 60 - 30);
                            float fy = std::ldexp(1.0f + (ry % 1000000) / 1e6f, (int)(ry >> 40) % 60 - 30);
                            memcpy(&x, &fx, 4); memcpy(&y, &fy, 4); x ^= (uint32_t)(rx >> 20) & 0x7FFF; if (rx & (1ull<<63)) x |= 0x80000000u; }
            if (sq && kind < 6) x &= 0x7FFFFFFF;
            t->x32 = x; t->y32 = y; t->is_sqrt = sq; t->rm = rm; t->start32 = 1; t->start64 = 0;
            tick(); t->start32 = 0; int c = 1;
            while (!t->done32) { tick(); c++; }
            float fx, fy, fr; memcpy(&fx, &x, 4); memcpy(&fy, &y, 4);
            fesetround(RM_HOST[rm]); std::feclearexcept(FE_ALL_EXCEPT);
            volatile float a = fx, b = fy; fr = sq ? std::sqrt(a) : a / b;
            int nv = std::fetestexcept(FE_INVALID) != 0, dz = std::fetestexcept(FE_DIVBYZERO) != 0;
            fesetround(FE_TONEAREST);
            uint32_t ref; memcpy(&ref, &fr, 4); if (std::isnan(fr)) ref = 0x7FC00000u;
            Stat &s = st[sq]; s.n++; s.lat = c;
            uint32_t got = t->res32;
            bool special = std::isnan(fr) || std::isinf(fr) || fr == 0 || std::isnan(fx) || std::isinf(fx) || (!sq && (std::isnan(fy) || std::isinf(fy) || fy == 0));
            if (((t->fl32 >> 4) & 1) != nv || ((t->fl32 >> 3) & 1) != dz) s.nvdz_bad++;
            if (special && rm != 4) { if (got != ref) s.special_bad++; else s.exact++; continue; }
            int64_t u = std::llabs(ord<uint64_t>(got, 32) - ord<uint64_t>(ref, 32));
            if (rm == 4 && u <= 1) u = 0;  // RMM vs host RNE: ties may differ
            if (u == 0) s.exact++; else if (u == 1) s.ulp1++; else if (u == 2) s.ulp2++; else s.more++;
            if (u > s.maxu) s.maxu = u;
        } else {
            uint64_t x = rx, y = ry;
            if (kind < 6) { double fx = std::ldexp(1.0 + (rx % 1000000007) / 1e9, (int)(rx >> 40) % 600 - 300);
                            double fy = std::ldexp(1.0 + (ry % 1000000007) / 1e9, (int)(ry >> 40) % 600 - 300);
                            memcpy(&x, &fx, 8); memcpy(&y, &fy, 8); x ^= (rx >> 12) & 0xFFFFFFFFull; if (rx & (1ull<<63)) x |= 1ull << 63; }
            if (sq && kind < 6) x &= ~(1ull << 63);
            t->x64 = x; t->y64 = y; t->is_sqrt = sq; t->rm = rm; t->start64 = 1; t->start32 = 0;
            tick(); t->start64 = 0; int c = 1;
            while (!t->done64) { tick(); c++; }
            double fx, fy, fr; memcpy(&fx, &x, 8); memcpy(&fy, &y, 8);
            fesetround(RM_HOST[rm]); std::feclearexcept(FE_ALL_EXCEPT);
            volatile double a = fx, b = fy; fr = sq ? std::sqrt(a) : a / b;
            int nv = std::fetestexcept(FE_INVALID) != 0, dz = std::fetestexcept(FE_DIVBYZERO) != 0;
            fesetround(FE_TONEAREST);
            uint64_t ref; memcpy(&ref, &fr, 8); if (std::isnan(fr)) ref = 0x7FF8000000000000ull;
            Stat &s = st[2 + sq]; s.n++; s.lat = c;
            uint64_t got = t->res64;
            bool special = std::isnan(fr) || std::isinf(fr) || fr == 0 || std::isnan(fx) || std::isinf(fx) || (!sq && (std::isnan(fy) || std::isinf(fy) || fy == 0));
            if (((t->fl64 >> 4) & 1) != nv || ((t->fl64 >> 3) & 1) != dz) s.nvdz_bad++;
            if (special && rm != 4) { if (got != ref) s.special_bad++; else s.exact++; continue; }
            int64_t u = std::llabs(ord<uint64_t>(got, 64) - ord<uint64_t>(ref, 64));
            if (rm == 4 && u <= 1) u = 0;
            if (u == 0) s.exact++; else if (u == 1) s.ulp1++; else if (u == 2) s.ulp2++; else s.more++;
            if (u > s.maxu) s.maxu = u;
        }
    }
    const char *nm[4] = {"FP32 div ", "FP32 sqrt", "FP64 div ", "FP64 sqrt"};
    printf("op         count    exact   1ulp    2ulp   >2ulp  max_ulp  special_bad  NV/DZ_bad  cycles\n");
    for (int k = 0; k < 4; k++)
        printf("%s %7ld  %6.2f%% %6.2f%% %6.2f%% %6.3f%% %6lld %10ld %10ld %7d\n", nm[k], st[k].n,
               100.0 * st[k].exact / st[k].n, 100.0 * st[k].ulp1 / st[k].n, 100.0 * st[k].ulp2 / st[k].n,
               100.0 * st[k].more / st[k].n, (long long)st[k].maxu, st[k].special_bad, st[k].nvdz_bad, st[k].lat);
    delete t;
    return 0;
}
