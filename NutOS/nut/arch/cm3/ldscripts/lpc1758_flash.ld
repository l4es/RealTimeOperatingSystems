/******************************************************************************
 *
 *  lpc1758_flash.ld - linker script for nutos on cortex-m3 NXP LPC1758
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

"bootloader_offset" = 0x00;

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x00000000, LENGTH = 512K
    SRAM0 (rwx) : ORIGIN = 0x10000000, LENGTH = (64K - 32)

    AHB_SRAM_BANK0 : ORIGIN = 0x2007C000, LENGTH = 16K
    AHB_SRAM_BANK1 : ORIGIN = 0x20080000, LENGTH = 16K
}

INCLUDE lpc17xx_flash.ld
