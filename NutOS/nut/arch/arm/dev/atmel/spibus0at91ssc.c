/*
 * Copyright (C) 2009, 2011 by egnite GmbH
 *
 * All rights reserved.
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
 */

/*!
 * \file arch/arm/dev/atmel/spibus0at91ssc.c
 * \brief SSC SPI bus 0 driver.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/spi.h>
#include <cfg/arch/gpio.h>

#include <dev/gpio.h>
#include <dev/spibus_ssc.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/nutdebug.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#ifdef ELEKTOR_IR1
#ifndef SSC0SPI_CS0_PIO_BIT
#define SSC0SPI_CS0_PIO_BIT 15
#endif
#ifndef SSC0SPI_CS0_PIO_ID
#define SSC0SPI_CS0_PIO_ID  PIOA_ID
#endif
#endif

#if defined(SSC0SPI_CS0_PIO_BIT) && defined(SSC0SPI_CS0_PIO_ID)
#undef GPIO_ID
#define GPIO_ID SSC0SPI_CS0_PIO_ID
#include <cfg/arch/porttran.h>
static INLINE void SSC0SPI_CS0_LO(void)
{
    GPIO_SET_LO(SSC0SPI_CS0_PIO_BIT);
}

static INLINE void SSC0SPI_CS0_HI(void)
{
    GPIO_SET_HI(SSC0SPI_CS0_PIO_BIT);
}

static INLINE void SSC0SPI_CS0_SO(void)
{
    GPIO_ENABLE(SSC0SPI_CS0_PIO_BIT);
    GPIO_OUTPUT(SSC0SPI_CS0_PIO_BIT);
}
#else
#define SSC0SPI_CS0_LO()
#define SSC0SPI_CS0_HI()
#define SSC0SPI_CS0_SO()
#endif


static AT91SSCREG gspi_reg0;

static uint8_t * volatile spi0_txp;
static uint8_t * volatile spi0_rxp;
static volatile size_t spi0_xc;

static uint8_t ssc_pdc_txbuf[512] SECTION_BSS_IRAM;
static uint8_t ssc_pdc_rxbuf[512] SECTION_BSS_IRAM;

/*!
 * \brief Set the specified chip select to a given level.
 */
static AT91SSCREG *SscSpi0ChipSelect(uint_fast8_t cs, uint_fast8_t hi)
{
    if (cs == 0) {
        if (hi) {
            SSC0SPI_CS0_HI();
        } else {
            SSC0SPI_CS0_LO();
        }
        SSC0SPI_CS0_SO();

        return &gspi_reg0;
    }
    return NULL;
}

void SscSpiBus0Interrupt(void *arg)
{
    if (spi0_xc) {
        if (spi0_rxp) {
            *spi0_rxp++ = (uint8_t) inr(SSC_RHR);
        }
        spi0_xc--;
    }
    if (spi0_xc) {
        if (spi0_txp) {
            outr(SSC_THR, *spi0_txp);
            spi0_txp++;
        } else {
            outr(SSC_THR, 0xFFFFFFFF);
        }
    } else {
        NutEventPostFromIrq((void **)arg);
    }
}

void SscSpiBus0PdcIrq(void *arg)
{
    NutEventPostFromIrq((void **)arg);
}

static void SscSpiBus0Clock(int xlen)
{
    while (xlen--) {
        outr(SSC_THR, 0xFF);
        while ((inr(SSC_SR) & SSC_RXRDY) == 0);
        inr(SSC_RHR);
    }
}

static void SscSpiBus0Read(uint8_t *rxbuf, int xlen)
{
    while (xlen--) {
        outr(SSC_THR, 0xFF);
        while ((inr(SSC_SR) & SSC_RXRDY) == 0);
        *rxbuf++ = (uint8_t) inr(SSC_RHR);
    }
}

static void SscSpiBus0Write(const uint8_t *txbuf, int xlen)
{
    while (xlen--) {
        outr(SSC_THR, *txbuf);
        txbuf++;
        while ((inr(SSC_SR) & SSC_RXRDY) == 0);
        inr(SSC_RHR);
    }
}

static void SscSpiBus0WriteRead(const uint8_t *txbuf, uint8_t *rxbuf, int xlen)
{
    while (xlen--) {
        outr(SSC_THR, *txbuf);
        txbuf++;
        while ((inr(SSC_SR) & SSC_RXRDY) == 0);
        *rxbuf++ = (uint8_t) inr(SSC_RHR);
    }
}

/*!
 * \brief Transfer data on the SPI bus.
 *
 * A device must have been selected by calling SscSpi0Select().
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
static int SscSpiBus0Transfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    if (xlen < 32) {
        if (txbuf) {
            if (rxbuf) {
                SscSpiBus0WriteRead((const uint8_t *) txbuf, (uint8_t *) rxbuf, xlen);
            } else {
                SscSpiBus0Write((const uint8_t *) txbuf, xlen);
            }
        }
        else if (rxbuf) {
            SscSpiBus0Read((uint8_t *) rxbuf, xlen);
        }
        else {
            SscSpiBus0Clock(xlen);
        }
    } else {
#if 0
        spi0_txp = (uint8_t *) txbuf;
        spi0_rxp = (uint8_t *) rxbuf;
        spi0_xc = (size_t) xlen;

        /* Enable and kick interrupts. */
        if (rxbuf) {
            outr(SSC_IER, SSC_RXRDY);
        } else {
            outr(SSC_IER, SSC_TXRDY);
        }
        if (txbuf) {
            outr(SSC_THR, *spi0_txp);
            spi0_txp++;
        } else {
            outr(SSC_THR, 0xFFFFFFFF);
        }
        /* Wait until transfer has finished. */
        NutEventWait(&node->node_bus->bus_ready, NUT_WAIT_INFINITE);
        outr(SSC_IDR, SSC_RXRDY);
#else
        if (txbuf) {
            memcpy(ssc_pdc_txbuf, txbuf, xlen);
        } else {
            memset(ssc_pdc_txbuf, 0xFF, xlen);
        }
        if (rxbuf) {
            memset(ssc_pdc_rxbuf, 0xFF, 512);
        }
        /* Set first transmit pointer and counter. */
        outr(SSC_TPR, (unsigned int) ssc_pdc_txbuf);
        outr(SSC_TCR, (unsigned int) xlen);
        /* Set first receive pointer and counter. */
        outr(SSC_RPR, (unsigned int) ssc_pdc_rxbuf);
        outr(SSC_RCR, (unsigned int) xlen);

        outr(SSC_IER, SSC_RXBUFF);
        outr(SSC_PTCR, PDC_TXTEN | PDC_RXTEN);

        /* Wait until transfer has finished. */
        NutEventWait(&node->node_bus->bus_ready, NUT_WAIT_INFINITE);
        outr(SSC_PTCR, PDC_TXTDIS | PDC_RXTDIS);
        outr(SSC_IDR, SSC_RXBUFF);
        if (rxbuf) {
            memcpy(rxbuf, ssc_pdc_rxbuf, xlen);
        }
#endif
    }
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
 * \return Always 0.
 */
static int SscSpiBus0NodeInit(NUTSPINODE * node)
{
    NUTSPIBUS *bus = node->node_bus;
    AT91SSCREG *sscreg;

    /* Try to deactivate the chip select and retrieve the shadow regs. */
    sscreg = SscSpi0ChipSelect(node->node_cs, 1);
    if (sscreg == NULL) {
        /* Invalid chip select. */
        return -1;
    }
    /* Check if this is the first call. */
    if (node->node_stat == NULL) {
        /* Bind the shadow register to this node. */
        node->node_stat = sscreg;
        /* Perform software reset. */
        outr(SSC_CR, SSC_SWRST | SSC_TXDIS | SSC_RXDIS);
        /* Enable SSC clock. */
        outr(PMC_PCER, _BV(SSC_ID));
        /* Select SSC peripheral functions. */
        outr(PIOA_ASR, _BV(PA16_TK_A) | _BV(PA17_TD_A) | _BV(PA18_RD_A) | _BV(PA19_RK_A));
        /* Enable SSC peripheral pins. */
        outr(PIOA_PDR, _BV(PA16_TK_A) | _BV(PA17_TD_A) | _BV(PA18_RD_A) | _BV(PA19_RK_A));
        /* Update shadow registers. */
        SscSpiSetup(node);
        /* Set clock divider from shadow register. */
        outr(SSC_CMR, sscreg->at91ssc_cmr);
        /* Set receive clock mode. */
        outr(SSC_RCMR, SSC_CKS_PIN | SSC_CKI | SSC_START_TX);
        /* Set transmit clock mode. */
        outr(SSC_TCMR, SSC_CKS_DIV | SSC_CKO_TRAN);
        /* Set receive and transmit frame mode from shadow register. */
        outr(SSC_RFMR, sscreg->at91ssc_fmr);
        outr(SSC_TFMR, sscreg->at91ssc_fmr);
        /* Enable transmitter. */
        outr(SSC_CR, SSC_TXEN | SSC_RXEN);

#if 0
        NutRegisterIrqHandler(bus->bus_sig, SscSpiBus0Interrupt, &bus->bus_ready);
#else
        NutRegisterIrqHandler(bus->bus_sig, SscSpiBus0PdcIrq, &bus->bus_ready);
#endif
        outr(SSC_IDR, (unsigned int) - 1);
        NutIrqEnable(bus->bus_sig);
    }
    return 0;
}

/*! \brief Select a device on the SPI bus.
 *
 * Locks and activates the bus for the specified node.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo Timeout in milliseconds. To disable timeout, set this
 *            parameter to NUT_WAIT_INFINITE.
 *
 * \return 0 on success. In case of an error, -1 is returned and the bus
 *         is not locked.
 */
static int SscSpiBus0Select(NUTSPINODE * node, uint32_t tmo)
{
    int rc;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_stat != NULL);

    /* Allocate the bus. */
    rc = NutEventWait(&node->node_bus->bus_mutex, tmo);
    if (rc) {
        errno = EIO;
    } else {
        AT91SSCREG *sscreg = (AT91SSCREG *) node->node_stat;

        /* If the mode changed, update our shadow registers. */
        if (node->node_mode & SPI_MODE_UPDATE) {
            SscSpiSetup(node);
        }
        /* Set the clock rate for this node. */
        outr(SSC_CMR, sscreg->at91ssc_cmr);
        /* Set receive and transmit frame mode for this node. */
        outr(SSC_RFMR, sscreg->at91ssc_fmr);
        outr(SSC_TFMR, sscreg->at91ssc_fmr);
        /* Activate the node's chip select. */
        SscSpi0ChipSelect(node->node_cs, 0);
    }
    return rc;
}

/*! \brief Deselect a device on the SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int SscSpiBus0Deselect(NUTSPINODE * node)
{
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    /* Deactivate the node's chip select. */
    SscSpi0ChipSelect(node->node_cs, 1);

    /* Release the bus. */
    NutEventPost(&node->node_bus->bus_mutex);

    return 0;
}

/*!
 * \brief AT91 SSC SPI bus driver implementation structure.
 */
NUTSPIBUS spiBus0Ssc = {
    NULL,               /*!< Bus mutex semaphore (bus_mutex). */
    NULL,               /*!< Bus ready signal (bus_ready). */
    0,                  /*!< Unused bus base address (bus_base). */
    &sig_SSC,           /*!< Bus interrupt handler (bus_sig). */
    SscSpiBus0NodeInit, /*!< Initialize the bus (bus_initnode). */
    SscSpiBus0Select,   /*!< Select the specified device (bus_alloc). */
    SscSpiBus0Deselect, /*!< Deselect the specified device (bus_release). */
    SscSpiBus0Transfer, /*!< Transfer data to and from a specified device (bus_transfer). */
    NutSpiBusWait,      /*!< Wait for bus transfer ready (bus_wait). */
    NutSpiBusSetMode,   /*!< Set SPI mode of a specified device (bus_set_mode). */
    NutSpiBusSetRate,   /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits,   /*!< Set number of data bits of a specified device (bus_set_bits). */
    NULL,               /*!< Private data of the hardware specific implementation. */
    NULL,               /*!< Pointer to the bus driver's device control block. */
};
