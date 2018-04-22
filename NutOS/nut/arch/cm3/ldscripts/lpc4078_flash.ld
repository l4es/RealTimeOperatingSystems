/******************************************************************************
 *
 *  lpc4078_flash.ld - linker script for nutos on cortex-m4 NXP LPC4078/88
 *
 *****************************************************************************/

ENTRY(NutInit)
SEARCH_DIR(.)

"bootloader_offset" = 0x00;

MEMORY
{
    FLASH0 (rx) : ORIGIN = 0x00000000, LENGTH = 512K
    SRAM0 (rwx) : ORIGIN = 0x10000000, LENGTH = (64K - 32)

    AHB_SRAM_BANK0A : ORIGIN = 0x20000000, LENGTH = 8K
    AHB_SRAM_BANK0B : ORIGIN = 0x20002000, LENGTH = 8K
    AHB_SRAM_BANK1  : ORIGIN = 0x20004000, LENGTH = 16K
}

INCLUDE lpc17xx_flash.ld
