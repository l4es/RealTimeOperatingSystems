ENTRY(_startup)
SEARCH_DIR(.)

/*
 * Define stack size here
 */
FIQ_STACK_SIZE = 0x0000;
IRQ_STACK_SIZE = 0x0080;
ABT_STACK_SIZE = 0x0080;
UND_STACK_SIZE = 0x0080;
SVC_STACK_SIZE = 0x0400;
SYS_STACK_SIZE = 0x1000;
HEAP_SIZE      = 0x5000;

/* specify the AT91SAM7S64 memory areas  */
MEMORY 
{
   flash : ORIGIN = 0x00000000, LENGTH = 256K   /* FLASH EPROM    */ 
   ram   : ORIGIN = 0x00200000, LENGTH = 32K    /* static RAM area   */
}

/* now define the output sections  */
SECTIONS 
{
   .text :                       /* collect all sections that should go into FLASH after startup  */ 
   {
    KEEP(*(.vectors));
    . = ALIGN(4);
    *(.init);
    . = ALIGN(4);
    *(.text);
    . = ALIGN(4);
    *(.rodata);
    . = ALIGN(4);
    *(.rodata*);
    . = ALIGN(4);
    *(.glue_7t);
    . = ALIGN(4);
    *(.glue_7);
    . = ALIGN(4);
    _etext = .;
   } >flash                   /* put all the above into FLASH */

   .data :                       /* collect all initialized .data sections that go into RAM  */ 
   {
    PROVIDE (__data_start = .);
    *(.data)
    . = ALIGN(4);
    edata = .;
    _edata = .;
    PROVIDE (__data_end = .);
   } >ram AT >flash                 /* put all the above into RAM (but load the LMA initializer copy into FLASH)  */

   .bss :                        /* collect all uninitialized .bss sections that go into RAM  */
   {
    PROVIDE (__bss_start = .);
    *(.bss)
    *(COMMON)
    . = ALIGN(4);
    PROVIDE (__bss_end = .);
    
    . = ALIGN(256);

    PROVIDE (__stack_start = .);
    PROVIDE (__stack_fiq_start = .);
    . += FIQ_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_fiq_end = .);

    PROVIDE (__stack_irq_start = .);
    . += IRQ_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_irq_end = .);

    PROVIDE (__stack_abt_start = .);
    . += ABT_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_abt_end = .);

    PROVIDE (__stack_und_start = .);
    . += UND_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_und_end = .);

    PROVIDE (__stack_svc_start = .);
    . += SVC_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_svc_end = .);

    PROVIDE (__stack_sys_start = .);
    . += SYS_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_sys_end = .);
    PROVIDE (__stack_end = .);

   _end = .;                        /* define a global symbol marking the end of application RAM */
   PROVIDE (end = .);

    PROVIDE (__heap_start = .);
    . += HEAP_SIZE;
    . = ALIGN(4);
    PROVIDE (__heap_end = .);
   } >ram                        /* put all the above in RAM (it will be cleared in the startup code */

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