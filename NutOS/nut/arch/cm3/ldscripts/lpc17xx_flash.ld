/************************************************************************************
 *
 * lpc17xx_flash.ld - linker script include for nutos on cortex-m3 NXP LPC17xx series
 *
 ************************************************************************************/

/* PROVIDE (__heap_start = _sheap); */

_ramend = ORIGIN(SRAM0) + LENGTH(SRAM0);

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        _svect = .;
        KEEP(*(.isr_vector))
        _evect = .;
        *(.nutinit)
        /* CRP section */
        . = 0x000002fc;
        KEEP(*(.crp))
        . = ALIGN(4);
        *(.text .text.* .gnu.linkonce.t.*)
        *(.rodata .rodata* .gnu.linkonce.r.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc_except_table)
        . = ALIGN(4);
    } > FLASH0

    /* .ARM.exidx is sorted, so has to go in its own output section. */
    .exidx : {
        __exidx_start = .;
        *(..exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH0
    _etext = .;

    .vtable (NOLOAD):
    {
        . = ALIGN(4);
        _vtable = .;
        *(.vtable*)
        . = ALIGN(4);
        _evtable = .;
    } > SRAM0

    .data : AT (_etext)
    {
        _sdata = .;
        *(.data .data.* .gnu.linkonce.d.*)
        . = ALIGN(4);
        *(.ramfunc);
        . = ALIGN(4);
        _edata = .;
    } > SRAM0

    .bss (NOLOAD):
    {
        _sbss = .;
        *(.bss .bss.* .gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM0

    .stack (NOLOAD):
    {
        . = ALIGN(8);
        _stack_start = .;
        KEEP(*(.stack .stack.*));
        _stack_end = .;
    } > SRAM0

    .psp_stack (NOLOAD):
    {
        . = ALIGN(8);
        _pspstack_start = .;
        KEEP(*(.psp_stack .psp_stack.*));
        _pspstack_end = .;
        PROVIDE (__heap_start = .);
    } > SRAM0
/*
    .heap (NOLOAD):
    {
        . = ALIGN(4);
        _sheap = .;
        . = _ramend - .;
        _eheap = .;
    } > SRAM0
*/

end = .;
/*
    DISCARD :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
*/
    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }

    /* DWARF debug sections.
       Symbols in the DWARF debugging sections are relative to the beginning
       of the section so we begin them at 0.  */

    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }

    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }

    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }

    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }

    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}
