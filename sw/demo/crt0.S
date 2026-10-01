# Bare-metal start for Synapse-32 + TinyGPU v2 programs (simulation and board).
# Enables the vector unit (mstatus.VS), sets the stack, runs main, reports to tohost.
.section .text.init
.globl _start
_start:
    li   t0, 0x2200            # mstatus.VS = Initial (FS too, for Spike)
    csrs mstatus, t0
    la   sp, _stack_top
    la   t0, _bss_start
    la   t1, _bss_end
1:  bgeu t0, t1, 2f
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    1b
2:  call main
    slli a0, a0, 1             # riscv-tests convention: (code << 1) | 1, 1 = pass
    ori  a0, a0, 1
    la   t0, tohost
    sw   a0, 0(t0)
3:  j    3b

.section .tohost,"aw",@progbits
.align 6
.globl tohost
tohost: .dword 0
.globl fromhost
fromhost: .dword 0
