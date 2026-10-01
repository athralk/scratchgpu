OUTPUT_ARCH("riscv")
ENTRY(_start)
SECTIONS {
  . = 0x80000000;
  .text.init : { *(.text.init) }
  . = ALIGN(0x1000);
  .tohost : { *(.tohost) }
  . = ALIGN(64);
  .text : { *(.text*) }
  .rodata : { *(.rodata*) *(.srodata*) }
  . = ALIGN(64);
  .data : { *(.data*) *(.sdata*) }
  . = ALIGN(64);
  _bss_start = .;
  .bss : { *(.bss*) *(.sbss*) *(COMMON) }
  _bss_end = .;
  . = ALIGN(16);
  . = . + 0x10000;
  _stack_top = .;
  _end = .;
}
