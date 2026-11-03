/* CH572 memory: 240 KB of user flash at 0 (the core starts executing at
   address 0) and 12 KB of SRAM (datasheet tables 2-2 and 2-4).  The 8 KB
   bootloader and 8 KB InfoFlash above 0x3C000 are not ours. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 240K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 12K
}

ENTRY(_start)

/* RAM the stack is guaranteed after .data and .bss. */
_stack_size = 2K;

SECTIONS
{
    .init ORIGIN(FLASH) :
    {
        KEEP(*(.init))
    } > FLASH

    .text : ALIGN(4)
    {
        *(.text .text.*)
    } > FLASH

    .rodata : ALIGN(4)
    {
        *(.srodata .srodata.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > FLASH

    /* Code that must not execute from flash while it runs: the clock
       setup changes the flash timing.  Copied to RAM by the startup code. */
    .highcode : ALIGN(4)
    {
        _highcode_start = .;
        *(.highcode .highcode.*)
        . = ALIGN(4);
        _highcode_end = .;
    } > RAM AT > FLASH
    _highcode_lma = LOADADDR(.highcode);

    .data : ALIGN(4)
    {
        _data_start = .;
        __global_pointer$ = . + 0x800;
        *(.sdata .sdata.* .sdata2 .sdata2.*)
        *(.data .data.*)
        . = ALIGN(4);
        _data_end = .;
    } > RAM AT > FLASH
    _data_lma = LOADADDR(.data);

    .bss (NOLOAD) : ALIGN(4)
    {
        _bss_start = .;
        *(.sbss .sbss.* .bss .bss.*)
        . = ALIGN(4);
        _bss_end = .;
    } > RAM

    _stack_top = ORIGIN(RAM) + LENGTH(RAM);
    ASSERT(_stack_top - _bss_end >= _stack_size, "less than _stack_size of RAM left for the stack")

    .got (INFO) : { KEEP(*(.got .got.*)) }
    /DISCARD/ : { *(.eh_frame .eh_frame_hdr) }
}
