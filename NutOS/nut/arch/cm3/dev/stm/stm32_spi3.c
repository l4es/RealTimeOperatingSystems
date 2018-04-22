/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
 * Copyright (C) 2012-1016 by Uwe Bonnes
 *                               (bon@elektron.ikp.physik.tu-darmstadt.de)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*
 * \verbatim
 * $Id: stm32_spi3.c 6414 2016-03-22 13:28:35Z u_bonnes $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <sys/timer.h>
#include <cfg/spi.h>
#include <cfg/arch/gpio.h>
#include <dev/spibus.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32_spi_pinmux.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_dma.h>
#include <arch/cm3/stm/stm32_spi.h>
#include <dev/irqreg.h>
#include <sys/event.h>
#include <sys/nutdebug.h>

#include <stdlib.h>
#include <errno.h>

#define SPI_SPEED SPI3_SPEED

#define SPI_CS0 SPI3_CS0
#define SPI_CS1 SPI3_CS1
#define SPI_CS2 SPI3_CS2
#define SPI_CS3 SPI3_CS3

#if defined(MCU_STM32F1) && defined(AFIO_MAPR_SPI3_REMAP)
void Stm32F1SpiRemap(void)
{
    AFIO->MAPR &= ~AFIO_MAPR_SPI3_REMAP;
    AFIO->MAPR |= SPI3_REMAP * AFIO_MAPR_SPI3_REMAP;
}
#else
#define Stm32F1SpiRemap()
#endif

#define SPI_SCK     SPI3_SCK
#define SPI_MISO    SPI3_MISO
#define SPI_MOSI    SPI3_MOSI

#define SPI_SCK_AF  SPI3_SCK_AF
#define SPI_MISO_AF SPI3_MISO_AF
#define SPI_MOSI_AF SPI3_MOSI_AF

#define SPI_DEV 3
#define SPI_ENABLE_CLK_SET() CM3BBSET(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_SPI3EN))
#define SPI_ENABLE_CLK_GET() CM3BBGET(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_SPI3EN))
#define sig_SPI             sig_SPI3
#define SPI_BASE            SPI3_BASE

#if !defined(SPI3_MODE)
#define SPIBUS_MODE IRQ_MODE
#else
#define SPIBUS_MODE SPI3_MODE
#endif

#define SPI_DMA_TX_CHANNEL SPI3_TX_DMA
#define SPI_DMA_RX_CHANNEL SPI3_RX_DMA

#if SPIBUS_MODE == DMA_MODE && defined(HW_DMA_CSELR_STM32)
static void SpiDmaChannelSelection(void)
{
    DmaChannelSelection(SPI3_TX_DMA, SPI3_DMA_TX_SEL(SPI3_TX_DMA));
    DmaChannelSelection(SPI3_RX_DMA, SPI3_DMA_RX_SEL(SPI3_RX_DMA));
}
#endif

NUTSPIBUS spiBus3Stm32 = {
    NULL,                       /*!< Bus mutex semaphore (bus_mutex). */
    NULL,                       /*!< Bus ready signal (bus_ready). */
    SPI3_BASE,                  /*!< Bus base address (bus_base). */
    NULL,                       /*!< Bus interrupt handler (bus_sig). */
    Stm32SpiBusNodeInit,         /*!< Initialize the bus (bus_initnode). */
    Stm32SpiBusSelect,          /*!< Select the specified device (bus_alloc). */
    Stm32SpiBusDeselect,        /*!< Deselect the specified device (bus_release). */
    Stm32SpiBusTransfer,
    NutSpiBusWait,
    NutSpiBusSetMode,           /*!< Set SPI mode of a specified device (bus_set_mode). */
    NutSpiBusSetRate,           /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits,           /*!< Set number of data bits of a specified device (bus_set_bits). */
    NULL,                       /*!< Private data of the hardware specific implementation. */
    NULL,                       /*!< Pointer to the bus driver's device control block. */
};

#include "stm32_spi.c"
