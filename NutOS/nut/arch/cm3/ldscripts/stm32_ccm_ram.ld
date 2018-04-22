/*****************************************************************************
 *
 * stm32_ccm_ram - linker script  for Nut/OS on Cortex-M STM32, running in RAM
 *
 * Use for F3 and L4. F4 does not provide ICODE access to CCM!
 *
 * Ram needs to be remapped!
 * Text remapped at 0
 * CCM: Ram Vectors
 *      MSP/PSP Stack
 *      Ramfunc
 *      Initialization data for RW variables
 * SRAM: Variables to allow Bitband access.
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)
SRAM1_START = DEFINED(ram1_start) ? ram1_start : 0x20000000  + ram0_length;
SRAM2_START = SRAM1_START + ram1_length;

MEMORY
{
    REMAP (rwx) : ORIGIN = 0x00000000,  LENGTH = ram0_length
    FLASH0 (rx) : ORIGIN = 0x08000000,  LENGTH = flash0_length
    CCM   (rwx) : ORIGIN = 0x10000000,  LENGTH = ccm_length
    SRAM0 (rwx) : ORIGIN = 0x20000000,  LENGTH = ram0_length
    SRAM1 (rwx) : ORIGIN = SRAM1_START, LENGTH = ram1_length
    SRAM2 (rwx) : ORIGIN = SRAM2_START, LENGTH = ram2_length
}

_ramend = ORIGIN(SRAM0) + LENGTH(SRAM0);
_rodata_start  = 0;
_rodata_end    = 0;
_rodata_load   = 0;
_ramfunc_start = 0;
_ramfunc_end   = 0;
_ramfunc_load  = 0;
_ccm_used      = 1;
__end_rom = DEFINED(bootloader_offset) ? bootloader_offset : 0;

SECTIONS{
    .svect :
    {
         _svect = .;
        KEEP(*(.isr_vector))
        _evect = .;
     } > REMAP AT > SRAM0

   .text : ALIGN_WITH_INPUT
    {
        *(.nutinit)
        *(.text .text.* .gnu.linkonce.t.*)
        *(.gnu.linkonce.r.*)
        *(.glue_7t)
        *(.glue_7)
	*(.gcc_except_table)
	. = ALIGN(4);
    } > REMAP AT > SRAM0

    /* .ARM.exidx is sorted, so has to go in its own output section. */
    .exidx : ALIGN_WITH_INPUT
    {
        __exidx_start = .;
        *(..exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > REMAP AT > SRAM0

    .preinit_array : ALIGN_WITH_INPUT
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } > REMAP AT > SRAM0

    .init_array : ALIGN_WITH_INPUT
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } > REMAP AT > SRAM0

    .fini_array : ALIGN_WITH_INPUT
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } > REMAP AT > SRAM0

    /* VTABLE at start of CCM fullfills alignment requirements! */
   .vtable (NOLOAD):
    {
	. = ALIGN(4);
	    _vtable = .;
	    *(.vtable*)
	. = ALIGN(4);
	    _evtable = .;
    } > CCM

    .stack (NOLOAD):
    {
        . = ALIGN(8);
        _stack_start = .;
        KEEP(*(.stack .stack.*));
        _stack_end = .;
    } > CCM

    .psp_stack (NOLOAD):
    {
        . = ALIGN(8);
        _pspstack_start = .;
        KEEP(*(.psp_stack .psp_stack.*));
        _pspstack_end = .;
    } > CCM

    .ramfunc :
    {
        *(.ramfunc);
        . = ALIGN(4);
    } > CCM

    _etext = .;

    .rodata :
    {
        *(.rodata .rodata.*)
	. = ALIGN(4);
    } > SRAM0

    .data : AT (_etext)
    {
        _sdata = .;
        *(.data .data.* .gnu.linkonce.d.*)
	. = ALIGN(4);
        _edata = .;
    } > SRAM0

    _eccm = _etext + SIZEOF(.data);

    .bss (NOLOAD):
    {
        _sbss = .;
        *(.bss .bss.* .gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
	PROVIDE (__heap_start = .);
    } > SRAM0

end = .;
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
