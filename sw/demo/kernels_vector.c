// The same kernels offloaded to TinyGPU v2. Synapse runs the loops; every vector instruction
// executes on the GPU. The matmuls use RVV intrinsics; saxpy is plain C that GCC
// auto-vectorizes (-O3, zve32f, VLEN 512) with no source changes at all.
#include <riscv_vector.h>
#include "common.h"

void matmul_i32_vector(int n, const int32_t *a, const int32_t *b, int32_t *c) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n;) {
            size_t vl = __riscv_vsetvl_e32m4(n - j);
            vint32m4_t acc = __riscv_vmv_v_x_i32m4(0, vl);
            for (int k = 0; k < n; k++) {
                vint32m4_t row = __riscv_vle32_v_i32m4(&b[k * n + j], vl);
                acc = __riscv_vmacc_vx_i32m4(acc, a[i * n + k], row, vl);
            }
            __riscv_vse32_v_i32m4(&c[i * n + j], acc, vl);
            j += vl;
        }
    }
}

void matmul_f32_vector(int n, const float *a, const float *b, float *c) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n;) {
            size_t vl = __riscv_vsetvl_e32m4(n - j);
            vfloat32m4_t acc = __riscv_vreinterpret_v_i32m4_f32m4(__riscv_vmv_v_x_i32m4(0, vl));
            for (int k = 0; k < n; k++) {
                vfloat32m4_t row = __riscv_vle32_v_f32m4(&b[k * n + j], vl);
                acc = __riscv_vfmacc_vf_f32m4(acc, a[i * n + k], row, vl);   // scalar A[i][k] broadcast
            }
            __riscv_vse32_v_f32m4(&c[i * n + j], acc, vl);
            j += vl;
        }
    }
}

// Plain C: the compiler turns this loop into vector instructions for the GPU.
void saxpy_auto(int n, float alpha, const float *x, float *y) {
    for (int i = 0; i < n; i++) y[i] = alpha * x[i] + y[i];
}
