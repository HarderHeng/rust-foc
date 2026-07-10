MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 24K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

PROVIDE(_ram_start = ORIGIN(RAM));
PROVIDE(_ram_end = ORIGIN(RAM) + LENGTH(RAM));
PROVIDE(_stack_start = _ram_end);

SECTIONS
{
  .vector_table ORIGIN(FLASH) :
  {
    LONG(_stack_start);
    KEEP(*(.vector_table.reset_vector));
    __exceptions = .;
    KEEP(*(.vector_table.exceptions));
    __eexceptions = .;
    KEEP(*(.vector_table.interrupts));
  } > FLASH

  PROVIDE(_stext = ADDR(.vector_table) + SIZEOF(.vector_table));

  .text _stext :
  {
    __stext = .;
    *(.text .text.*);
    *(.rodata .rodata.*);
  } > FLASH

  .data : AT(LOADADDR(.text) + SIZEOF(.text))
  {
    __sdata = .;
    *(.data .data.*);
    __edata = .;
  } > RAM

  .bss (NOLOAD) :
  {
    __sbss = .;
    *(.bss .bss.*);
    __ebss = .;
  } > RAM

  __uninit_start = .;
  .uninit (NOLOAD) : { *(.uninit .uninit.*) } > RAM
  __uninit_end = .;

  PROVIDE(_sheap = .);
  PROVIDE(_eheap = ORIGIN(RAM) + LENGTH(RAM));
}
