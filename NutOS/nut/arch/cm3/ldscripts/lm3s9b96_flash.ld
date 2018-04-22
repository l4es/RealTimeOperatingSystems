/******************************************************************************
 *
 *  lm3s9b96_flash.ld - linker script for nutos on cotrex-m3 LM3S9B96
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

"bootloader_offset" = 0x00;

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x00000000, LENGTH = 256K
    SRAM0 (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

INCLUDE lm3x_flash.ld
