#ifndef _STM32_SPI_H_
#define _STM32_SPI_H_

#define IRQ_MODE     0
#define DMA_MODE     1
#define POLLING_MODE 2

#include <arch/cm3/stm/stm32_clk.h>

static int Stm32SpiSetup(NUTSPINODE * node);
static int Stm32SpiBusNodeInit(NUTSPINODE * node);
static int Stm32SpiBusSelect(NUTSPINODE * node, uint32_t tmo);
static int Stm32SpiBusDeselect(NUTSPINODE * node);
static int Stm32SpiBusTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen);

#endif
