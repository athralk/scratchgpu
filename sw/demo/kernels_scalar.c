// Scalar reference kernels: Synapse-32 alone, with its hardware F/D floating point.
#include "common.h"

void matmul_i32_scalar(int n, const int32_t *a, const int32_t *b, int32_t *c) {
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            int32_t s = 0;
            for (int k = 0; k < n; k++) s += a[i * n + k] * b[k * n + j];
            c[i * n + j] = s;
        }
}

void matmul_f32_scalar(int n, const float *a, const float *b, float *c) {
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int k = 0; k < n; k++) s += a[i * n + k] * b[k * n + j];
            c[i * n + j] = s;
        }
}

void saxpy_scalar(int n, float alpha, const float *x, float *y) {
    for (int i = 0; i < n; i++) y[i] = alpha * x[i] + y[i];
}
