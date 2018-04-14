OUTPUT_FORMAT("elf32-i386")
ENTRY(kstart)
SECTIONS
{
  . = 0x100000;
  kernel = .;
  cosinit = .;
  .cosinit  : {
    *(.cosinit)
  }
  end_cosinit = .;
  . = ALIGN(4096);
  code = .;
  .text  : {
    *(.text)
  }
  end_code = .;
  . = ALIGN(4096);
  data = .;
  .data  : {
    *(.data)
  }
  end_data = .;
  . = ALIGN(4096);
  rodata = .;
  .rodata  :
  {
    *(.rodata*)
  }
  end_rodata = .;
  . = ALIGN(4096);
  bss = .;
  .bss  :
  {
    *(.bss)
  }
  end_bss = .;
  . = ALIGN(4096);
  end_kernel = .;
}
