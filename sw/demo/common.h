#pragma once
#include <stdint.h>

#define UART_THR (*(volatile uint32_t *)0x20000000u)
#define UART_LSR (*(volatile uint32_t *)0x20000014u)

static inline void putc_(char c) {
    while (!(UART_LSR & 0x20)) { }
    UART_THR = (uint8_t)c;
}
static inline void puts_(const char *s) { while (*s) putc_(*s++); }
static inline void putu(uint32_t v) {
    char b[11]; int i = 10; b[i] = 0;
    do { b[--i] = '0' + v % 10; v /= 10; } while (v);
    puts_(&b[i]);
}
static inline uint32_t rdcycle(void) {
    uint32_t c; __asm__ volatile("csrr %0, mcycle" : "=r"(c)); return c;
}
