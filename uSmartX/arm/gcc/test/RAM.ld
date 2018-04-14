/***********************************************************************/
/*                                                                     */
/*  RAM.ld:  Linker Script File                                        */
/*                                                                     */
/***********************************************************************/
ENTRY(_start)
STACK_SIZE = 0x100;

/* Memory Definitions */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x80000			/* 512 Kb of internal FLASH */
	SRAM (rwx) : ORIGIN = 0x0, LENGTH = 0x1000		/* 4Kb of internal SRAM (after remap) */
	XSRAM (rwx) : ORIGIN = 0x2000000, LENGTH = 0x80000		/* 512Kb of external SRAM (after remap) */
	SDRAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x4000000 /* 64 Mb of external SDRAM */
}

/* Section Definitions */
SECTIONS
{
	.startup :
	{
		*crt0.o (.text)

	} > XSRAM
	
	. = ALIGN(4);
	_estartup = . ;

  	/* first section is .text which is used for code */
  	.text :
  	{    		
		_stext_load = LOADADDR(.text);
    	*(EXCLUDE_FILE(*crt0.o).text)         /* remaining code */
    	*(.rodata)               			  /* read-only data (constants) */
    	*(.rodata*)
    	*(.glue_7)
    	*(.glue_7t)
  	} > XSRAM

	. = ALIGN(4);	
  	_etext_load = LOADADDR(.text) + SIZEOF(.text);

	__etext = . ;
  	PROVIDE (etext = .);

  /* .data section which is used for initialized data */
  	.data  :
  	{
    	_data = .;
    	*(.data)
  	} > XSRAM

  	. = ALIGN(4);
  	_edata = . ;
  	PROVIDE (edata = .);

  	/* .bss section which is used for uninitialized data */
  	.bss (NOLOAD) :
  	{
    	__bss_start = . ;
    	__bss_start__ = . ;
    	*(.bss)
    	*(COMMON)
    	. = ALIGN(4);
  	} > XSRAM

  	. = ALIGN(4);
  	__bss_end__ = . ;
  	PROVIDE (__bss_end = .);

  	.stack ALIGN(512) :
  	{
	    . += STACK_SIZE;
	    PROVIDE (_stack = .);
  	} > XSRAM

	  _end = . ;
  	PROVIDE (end = .);

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
