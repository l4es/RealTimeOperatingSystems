/******************************************************************************
 *
 *  sam3u_flash.ld - linker script for nutos on cotrex-m3 sam3u-ek
 *
 *****************************************************************************/

ENTRY(ResetISR)
STARTUP(crtsam3u4_flash.o)
SEARCH_DIR(.)
OUTPUT_ARCH(arm)

"bootloader_offset" = 0x00;

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x00080000, LENGTH = 128K
    FLASH1 (rx) : ORIGIN = 0x00100000, LENGTH = 128K
    SRAM0 (rwx) : ORIGIN = 0x20000100, LENGTH = 0x00007F00
    SRAM1 (rwx) : ORIGIN = 0x20080000, LENGTH = 16K
}

SECTIONS
{
    .text :
    {
	. = ALIGN(4);
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
	. = ALIGN(4);
        _etext = .;
    } > FLASH0

    .data : AT (ADDR(.text) + SIZEOF(.text))
    {
	. = ALIGN(4);
        _data = .;
        *(vtable)
        *(.data*)
	. = ALIGN(4);
        _edata = .;
    } > SRAM0

    .bss :
    {
	 PROVIDE (_bss = .);
	 *(.bss)
	 *(COMMON)
	 . = ALIGN(4);
	 PROVIDE (_ebss = .);
	 . += 0x400;

	 PROVIDE (__exp_stack = .);
	 . += 0x400;

	 PROVIDE (__stack = .);
	 PROVIDE (__heap_start = .);
    } > SRAM0
}
end = .;
