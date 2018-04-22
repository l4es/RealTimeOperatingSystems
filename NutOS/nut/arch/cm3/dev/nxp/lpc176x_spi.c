/*
 * Copyright (C) 2012 by Simon Budig (simon@budig.de)
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
 * $Id$
 * \endverbatim
 */

#include <sys/timer.h>
#include <sys/nutdebug.h>
#include <dev/spibus.h>
#include <dev/gpio.h>

#include <arch/cm3.h>
#include <arch/cm3/nxp/mach/lpc_sc.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <arch/cm3/nxp/lpc176x_spi.h>

#include <errno.h>
#include <stdlib.h>

#define SPIBUS_SCK_PORT  NUTGPIO_PORT0
#define SPIBUS_MISO_PORT NUTGPIO_PORT0
#define SPIBUS_MOSI_PORT NUTGPIO_PORT0

#define SPIBUS_SCK_PIN  15
#define SPIBUS_MISO_PIN 17
#define SPIBUS_MOSI_PIN 18

#define SPIBUS_SCK_PIN_CFG  (GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHERAL3)
#define SPIBUS_MISO_PIN_CFG (GPIO_CFG_PERIPHERAL3)
#define SPIBUS_MOSI_PIN_CFG (GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHERAL3)

static int  Lpc17xxSpiSetup       (NUTSPINODE * node);
static int  Lpc17xxSpiBusNodeInit (NUTSPINODE * node);
static int  Lpc17xxSpiBusSelect   (NUTSPINODE * node, uint32_t tmo);
static int  Lpc17xxSpiBusDeselect (NUTSPINODE * node);
static int  Lpc17xxSpiBusTransfer (NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen);

// TODO: Add a mutex for the bus access

NUTSPIBUS spiBus0Lpc17xx = {
    NULL,                       /*!< Bus mutex semaphore (bus_mutex). */
    NULL,                       /*!< Bus ready signal (bus_ready). */
    LPC_SPI_BASE,               /*!< Bus base address (bus_base). */
    NULL,                       /*!< Bus interrupt handler (bus_sig). */
    Lpc17xxSpiBusNodeInit,      /*!< Initialize the bus (bus_initnode). */
    Lpc17xxSpiBusSelect,        /*!< Select the specified device (bus_alloc). */
    Lpc17xxSpiBusDeselect,      /*!< Deselect the specified device (bus_release). */
    Lpc17xxSpiBusTransfer,
    NutSpiBusWait,
    NutSpiBusSetMode,           /*!< Set SPI mode of a specified device (bus_set_mode). */
    NutSpiBusSetRate,           /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits,           /*!< Set number of data bits of a specified device (bus_set_bits). */
    NULL,                       /*!< Private data of the hardware specific implementation. */
    NULL,                       /*!< Pointer to the bus driver's device control block. */
};


/*!
 * \brief Set the specified chip select to a given level.
 */
static int Lpc17xxSpiChipSelect(uint_fast8_t cs, uint_fast8_t hi)
{
    GpioPinConfigSet(cs / 32, cs % 32, GPIO_CFG_OUTPUT);
    GpioPinSet(cs / 32, cs % 32, hi);
    return 0;
}

/*! \brief Deselect a device on the SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int Lpc17xxSpiBusDeselect(NUTSPINODE * node)
{
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    NutSpiBusWait(node, NUT_WAIT_INFINITE);
    /* Deactivate the node's chip select. */
    Lpc17xxSpiChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);

    /* Release the bus. */
    NutEventPostAsync(&node->node_bus->bus_mutex);

    return 0;
}

/*! \brief Select a device on the SPI bus.
 *
 * Locks and activates the bus for the specified node.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo  Timeout in milliseconds. To disable timeout, set this
 *             parameter to NUT_WAIT_INFINITE.
 *
 * \return 0 on success. In case of an error, -1 is returned and the bus
 *         is not locked.
 */
static int Lpc17xxSpiBusSelect(NUTSPINODE * node, uint32_t tmo)
{
    int rc;
    LPC_SPI_TypeDef* base;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_stat != NULL);

    base=(LPC_SPI_TypeDef*)(node->node_bus->bus_base);

    /* Allocate the bus. */
    rc = NutEventWait(&node->node_bus->bus_mutex, tmo);
    if (rc) {
        errno = EIO;
    } else {
        LPC_SPI_TypeDef *spireg = node->node_stat;

        /* Dectivate the IO Pins to avoid glitches*/
        GpioPinConfigSet(SPIBUS_SCK_PORT,  SPIBUS_SCK_PIN,  GPIO_CFG_DISABLED);  //SCK
        GpioPinConfigSet(SPIBUS_MISO_PORT, SPIBUS_MISO_PIN, GPIO_CFG_DISABLED ); //MISO
        GpioPinConfigSet(SPIBUS_MOSI_PORT, SPIBUS_MOSI_PIN, GPIO_CFG_DISABLED);  //MOSI

        /* If the mode update bit is set, then update our shadow registers. */
        if (node->node_mode & SPI_MODE_UPDATE) {
            Lpc17xxSpiSetup(node);
        }

        /* Set SPI mode. */
        base->SPCR = spireg->SPCR;
        base->SPCCR = spireg->SPCCR;

        GpioPinConfigSet(SPIBUS_SCK_PORT,  SPIBUS_SCK_PIN,  SPIBUS_SCK_PIN_CFG);  //SCK
        GpioPinConfigSet(SPIBUS_MISO_PORT, SPIBUS_MISO_PIN, SPIBUS_MISO_PIN_CFG); //MISO
        GpioPinConfigSet(SPIBUS_MOSI_PORT, SPIBUS_MOSI_PIN, SPIBUS_MOSI_PIN_CFG); //MOSI
        /* Finally activate the node's chip select. */
        rc = Lpc17xxSpiChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) != 0);
        if (rc) {
            /* Release the bus in case of an error. */
            NutEventPost(&node->node_bus->bus_mutex);
        }
    }
    return rc;
}


/*!
 * \brief Update SPI shadow registers.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int Lpc17xxSpiSetup(NUTSPINODE * node)
{
    uint32_t clk;
    uint32_t clkdiv;
    LPC_SPI_TypeDef *spireg;

    NUTASSERT(node != NULL);
    NUTASSERT(node->node_stat != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != 0);
    spireg = node->node_stat;

    spireg->SPCR &= ~( SPI_CR_LSBF | SPI_CR_BITENABLE | SPI_CR_BITS_MASK | SPI_CR_CPOL | SPI_CR_CPHA);
    if (node->node_bits != 8) {
        spireg->SPCR |= (SPI_CR_BITENABLE | SPI_CR_BITS(node->node_bits));
    }
    if (node->node_mode & SPI_MODE_CPOL) {
        spireg->SPCR |= SPI_CR_CPOL;
    }
    if (node->node_mode & SPI_MODE_CPHA) {
        spireg->SPCR |= SPI_CR_CPHA;
    }
    if (node->node_mode & SPI_MODE_LSB) {
        spireg->SPCR |= SPI_CR_LSBF;
    }
    spireg->SPCR |= SPI_CR_MSTR;  /* master only for now */

    clk = NutClockGet(NUT_HWCLK_PCLK);

    clk /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_SPI);

    /* Calculate the SPI clock divider. Avoid rounding errors. */
    clkdiv = (clk + node->node_rate - 1) / node->node_rate;

    /* The divider value minimum is 8. */
    if (clkdiv < 8) {
        clkdiv = 8;
    }
    /* The divider value maximum is 255. */
    else if (clkdiv > 255) {
        clkdiv = 255;
    }

    /* must be even */
    clkdiv &= 0xfe;

    spireg->SPCCR = clkdiv;

    /* Update interface parameters. */
    node->node_rate = clk / clkdiv;
    node->node_mode &= ~SPI_MODE_UPDATE;

    return 0;
}


/*!
 * \brief Initialize an SPI bus node.
 *
 * This routine is called for each SPI node, which is registered via
 * NutRegisterSpiDevice().
 *
 * \param node Specifies the SPI bus node.
 *
 * \return 0 on success or -1 if there is no valid chip select.
 */
static int Lpc17xxSpiBusNodeInit(NUTSPINODE * node)
{
    int rc = -1;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    /* Try to deactivate the node's chip select. */
    rc = Lpc17xxSpiChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);
    /* It should not hurt us being called more than once. Thus, we
       ** check wether any initialization had been taken place already. */
    if (rc == 0 && node->node_stat == NULL)
    {
        /* Allocate and set our shadow registers. */
        LPC_SPI_TypeDef *spireg = malloc(sizeof(LPC_SPI_TypeDef));

        if (spireg) {
            /* Update with node's defaults. */
            node->node_stat = (void *)spireg;
            Lpc17xxSpiSetup(node);
        }
        else {
            /* Out of memory? */
            rc = -1;
        }
    }

    return rc;
}

/*!
 * \brief Transfer data on the SPI bus using single buffered interrupt mode.
 *
 * A device must have been selected by calling At91SpiSelect().
 *
 * \param node  Specifies the SPI bus node.
 * \param txbuf Pointer to the transmit buffer. If NULL, undetermined
 *              byte values are transmitted.
 * \param rxbuf Pointer to the receive buffer. If NULL, then incoming
 *              data is discarded.
 * \param xlen  Number of bytes to transfer.
 *
 * \return Always 0.
 */
static int Lpc17xxSpiBusTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    LPC_SPI_TypeDef* base;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != 0);
    base = (LPC_SPI_TypeDef*) node->node_bus->bus_base;

    unsigned char *tx = (unsigned char*) txbuf;
    unsigned char *rx = (unsigned char*) rxbuf;

    while (xlen-- > 0) {
        unsigned char b = tx ? (*tx++) : 0xff;

        base->SPDR = b;

        /* wait until receive buffer no longer empty */
        while ((base->SPSR & SPI_SR_SPIF) == 0)
          ;

        b = base->SPDR;

        if (rx) {
          *rx++ = b;
        }
    }

    return 0;
}

