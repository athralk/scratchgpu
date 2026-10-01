// ZC702 ARM-side bridge for Synapse-32 (bare metal, no Xilinx BSP).
//
// UART1 (MIO, the board's USB-UART) is your terminal; UART0 is routed through EMIO to
// Synapse's UART. This releases Synapse from reset (control block on M_AXI_GP0) and then
// forwards bytes both ways, so the Synapse console appears on the USB-UART at 115200 8N1.
#include <stdint.h>

#define REG(a) (*(volatile uint32_t *)(a))
#define UART0 0xE0000000u              /* Synapse (EMIO) */
#define UART1 0xE0001000u              /* USB-UART */
#define U_CR 0x00
#define U_MR 0x04
#define U_BAUDGEN 0x18
#define U_SR 0x2C
#define U_FIFO 0x30
#define U_BAUDDIV 0x34
#define SR_RXEMPTY (1u << 1)
#define SR_TXFULL (1u << 4)

#define SYN_CTRL  0x40000000u           /* [0] = Synapse reset (1 = held) */
#define SYN_PC    0x40000004u
#define SYN_ID    0x40000008u
#define SYN_DDR   0x40000010u

static void uart_init(uint32_t u) {
    REG(u + U_CR) = 0x28 | 0x03;         // disable TX/RX, reset both
    REG(u + U_MR) = 0x20;                // 8 data bits, no parity, 1 stop
    REG(u + U_BAUDGEN) = 62;             // 50 MHz / (62 * (6 + 1)) = 115207 baud
    REG(u + U_BAUDDIV) = 6;
    REG(u + U_CR) = 0x14;                // enable TX and RX
}

static void uart_putc(uint32_t u, char c) {
    while (REG(u + U_SR) & SR_TXFULL) { }
    REG(u + U_FIFO) = (uint8_t)c;
}

static void puts1(const char *s) { while (*s) uart_putc(UART1, *s++); }

static void puthex(uint32_t v) {
    for (int i = 28; i >= 0; i -= 4) uart_putc(UART1, "0123456789abcdef"[(v >> i) & 15]);
}

int main(void) {
    uart_init(UART1);
    uart_init(UART0);
    puts1("\r\n[bridge] ZC702 ARM bridge for Synapse-32\r\n[bridge] control block ID ");
    uint32_t id = REG(SYN_ID);
    puthex(id);
    puts1(id == 0x53594E35u ? " (SYN5 ok)" : " (unexpected: is the bitstream loaded?)");
    puts1(", Synapse DRAM at DDR 0x");
    puthex(REG(SYN_DDR));
    puts1("\r\n[bridge] releasing Synapse reset; its console follows\r\n\r\n");
    REG(SYN_CTRL) = 0;

    for (;;) {
        if (!(REG(UART0 + U_SR) & SR_RXEMPTY)) uart_putc(UART1, (char)REG(UART0 + U_FIFO));
        if (!(REG(UART1 + U_SR) & SR_RXEMPTY)) uart_putc(UART0, (char)REG(UART1 + U_FIFO));
    }
}
