// tgc kernels on TinyGPU v2 vs plain scalar code, bit-exact.
#include "../demo/common.h"

void saxpy__tgc(int n, float a, const float *restrict x, float *restrict y);
void relu__tgc(int n, const float *restrict x, float *restrict y);
void mandel__tgc(int n, int width, float x0, float y0, float step, int16_t *restrict iters);
void blend__tgc(int w, int h, int stride, int16_t *restrict dst, const int16_t *restrict a,
                const int16_t *restrict b, int alpha);
#define launch(k, n, ...)        k##__tgc((int)(n), __VA_ARGS__)
#define launch2d(k, w, h, ...)   k##__tgc((int)(w), (int)(h), __VA_ARGS__)

#define N 2048
#define W 64
#define H 32
static float x[N], y[N], yr[N];
static int16_t it[W * H], itr[W * H];
static int16_t ia[W * H], ib[W * H], o[W * H], orf[W * H];
static uint32_t lcg = 99;
static int32_t rnd(void) { lcg = lcg * 1103515245u + 12345u; return (int32_t)(lcg >> 16) % 201 - 100; }

static int check(const char *name, const void *p, const void *q, int bytes, uint32_t cyc) {
    const uint8_t *a = p, *b = q;
    int bad = 0;
    for (int i = 0; i < bytes; i++) bad |= a[i] != b[i];
    puts_(name); puts_(": "); putu(cyc); puts_(" cycles on the GPU, ");
    puts_(bad ? "MISMATCH\n" : "matches scalar\n");
    return bad;
}

int main(void) {
    int fails = 0;
    puts_("tgc kernels on TinyGPU v2\n");
    for (int i = 0; i < N; i++) { x[i] = (float)rnd() * 0.25f; y[i] = yr[i] = (float)rnd(); }
    for (int i = 0; i < N; i++) yr[i] = 1.5f * x[i] + yr[i];
    uint32_t t0 = rdcycle(); launch(saxpy, N, 1.5f, x, y); uint32_t t1 = rdcycle();
    fails |= check("saxpy ", y, yr, sizeof y, t1 - t0);

    for (int i = 0; i < N; i++) yr[i] = x[i] > 0.0f ? x[i] : 0.0f;
    t0 = rdcycle(); launch(relu, N, x, y); t1 = rdcycle();
    fails |= check("relu  ", y, yr, sizeof y, t1 - t0);

    for (int p = 0; p < W * H; p++) {
        float cr = -2.0f + 0.046875f * (float)(p % W), ci = -0.75f + 0.046875f * (float)(p / W);
        float zr = 0.0f, zi = 0.0f;
        int16_t n = 0;
        for (int k = 0; k < 32; k++) {
            float zr2 = zr * zr, zi2 = zi * zi;
            n += (zr2 + zi2 < 4.0f);
            zi = 2.0f * zr * zi + ci;
            zr = zr2 - zi2 + cr;
        }
        itr[p] = n;
    }
    t0 = rdcycle(); launch(mandel, W * H, W, -2.0f, -0.75f, 0.046875f, it); t1 = rdcycle();
    fails |= check("mandel", it, itr, sizeof it, t1 - t0);

    for (int i = 0; i < W * H; i++) { ia[i] = (int16_t)(rnd() * 300); ib[i] = (int16_t)(rnd() * 300); }
    for (int i = 0; i < W * H; i++) orf[i] = (int16_t)((ia[i] * 77 + ib[i] * (256 - 77)) >> 8);
    t0 = rdcycle(); launch2d(blend, W, H, W, o, ia, ib, 77); t1 = rdcycle();
    fails |= check("blend ", o, orf, sizeof o, t1 - t0);

    // ASCII Mandelbrot from the GPU result
    for (int r = 0; r < H; r += 2) {
        for (int c = 0; c < W; c++) putc_(" .:-=+*#%@"[it[r * W + c] * 9 / 32]);
        putc_('\n');
    }
    return fails;
}
