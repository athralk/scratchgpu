// Synapse-32 + TinyGPU v2 demo: the same matrix multiplies on the scalar core and on the
// vector unit, results compared bit-for-bit, cycle counts printed over the UART.
#include "common.h"

#define N 32

void matmul_i32_scalar(int n, const int32_t *a, const int32_t *b, int32_t *c);
void matmul_i32_vector(int n, const int32_t *a, const int32_t *b, int32_t *c);
void matmul_f32_scalar(int n, const float *a, const float *b, float *c);
void matmul_f32_vector(int n, const float *a, const float *b, float *c);
void saxpy_scalar(int n, float alpha, const float *x, float *y);
void saxpy_auto(int n, float alpha, const float *x, float *y);

#define NS 4096
static float sx_[NS], sy_s[NS], sy_v[NS];

static int32_t ai[N * N], bi[N * N], ci_s[N * N], ci_v[N * N];
static float af[N * N], bf[N * N], cf_s[N * N], cf_v[N * N];

static uint32_t lcg = 12345;
static int32_t rnd(void) { lcg = lcg * 1103515245u + 12345u; return (int32_t)(lcg >> 16) % 17 - 8; }

static void report(const char *name, uint32_t cs, uint32_t cv, int bad) {
    puts_(name); puts_(": scalar "); putu(cs); puts_(" cycles, GPU "); putu(cv);
    puts_(" cycles, speedup "); putu(cs / (cv ? cv : 1)); putc_('.'); putu((cs * 10 / (cv ? cv : 1)) % 10);
    puts_("x, ");
    puts_(bad ? "MISMATCH\n" : "results identical\n");
}

int main(void) {
    for (int i = 0; i < N * N; i++) {
        ai[i] = rnd(); bi[i] = rnd();
        af[i] = (float)rnd(); bf[i] = (float)rnd();   // small integers: every sum is exact
    }
    puts_("Synapse-32 + TinyGPU v2: ");
    putu(N); puts_("x"); putu(N); puts_(" matmul\n");

    uint32_t t0 = rdcycle(); matmul_i32_scalar(N, ai, bi, ci_s);
    uint32_t t1 = rdcycle(); matmul_i32_vector(N, ai, bi, ci_v);
    uint32_t t2 = rdcycle();
    int bad = 0;
    for (int i = 0; i < N * N; i++) bad |= (ci_s[i] != ci_v[i]);
    report("int32", t1 - t0, t2 - t1, bad);
    int fails = bad;

    t0 = rdcycle(); matmul_f32_scalar(N, af, bf, cf_s);
    t1 = rdcycle(); matmul_f32_vector(N, af, bf, cf_v);
    t2 = rdcycle();
    bad = 0;
    for (int i = 0; i < N * N; i++) bad |= (((uint32_t *)cf_s)[i] != ((uint32_t *)cf_v)[i]);
    report("fp32 ", t1 - t0, t2 - t1, bad);
    fails |= bad;

    for (int i = 0; i < NS; i++) { sx_[i] = (float)rnd(); sy_s[i] = sy_v[i] = (float)rnd(); }
    t0 = rdcycle(); saxpy_scalar(NS, 3.0f, sx_, sy_s);
    t1 = rdcycle(); saxpy_auto(NS, 3.0f, sx_, sy_v);
    t2 = rdcycle();
    bad = 0;
    for (int i = 0; i < NS; i++) bad |= (((uint32_t *)sy_s)[i] != ((uint32_t *)sy_v)[i]);
    report("saxpy (plain C, auto-vectorized)", t1 - t0, t2 - t1, bad);
    return fails | bad;
}
