ENTRY(_start)
SECTIONS {
  . = 0x00100000;                 /* DDR above the OCM alias; Synapse owns 0x2000_0000+ */
  .text : { *(.text.start) *(.text*) *(.rodata*) }
  .data : { *(.data*) }
  _bss_start = .;
  .bss : { *(.bss*) *(COMMON) }
  _bss_end = .;
  . = ALIGN(16) + 0x4000;
  _stack_top = .;
}
