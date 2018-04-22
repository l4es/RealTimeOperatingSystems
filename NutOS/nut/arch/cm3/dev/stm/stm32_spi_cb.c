/*
 *  Copyright (C) 2015-2016
 *               Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de).
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
 * \file arch/cm3/stm/stm32_spi_cb.c
 * \brief STM32 SPI handler configured by interface control block..
 *
 * The SPI IP has no interrupt for SPI getting idle, so it takes some effort
 * to find the right moment to switch SPI and CS off. To do so, we always
 * count the received byte even if we transmit only.
 * DMA_Mode handles this case per se, but needs TX _and_ RX DMA channels.
 *
 * ToDo:
 * - Use Polling transfer when transfer needs only few cycles.
 * - Allow to specify only either DMA/TX or DMA/RX in addition to both
 *   channels.
 * - Use only one interrupt in interrupt mode, not both TX and RX IRQs.
 * - Use 16-bit transfer when possible.
 */

#include <stdlib.h>
#include <errno.h>

#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <dev/gpio.h>
#include <dev/spibus.h>

#include <arch/cm3/stm/stm32_spi_pinmux.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_dma.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <arch/cm3/stm/stm32_spi_cb.h>

#include <cfg/devices.h>
#include <cfg/spi.h>

static int Stm32SpiCbClkEnable(const STM32_SPI_ICB *bus_icb, int state)
{
    int res;
    uint32_t reg;

    reg = *bus_icb->enable_reg;
    res = (reg & bus_icb->enable_mask);
    if (state) {
        reg |= bus_icb->enable_mask;
    } else {
        reg &= ~bus_icb->enable_mask;
    }
    *bus_icb->enable_reg = reg;
    return res;
}
/*!
 * \brief Stop SPI when in RX-Only mode
 *
 * Follow "Disabling the SPI"
 * SPE must be disabled after first sampling edge and before last clock.
 * Function is called with RXNE just asserted.
 * RXNE was asserted with active edge of last cycle of last transfer
 *
 * So wait until last cycle of last transfer ends.
 * And wait until active edge of first clock of current cycle happens.
 *
 * Things may happen fast, so edges may be missed. Waiting for
 * 3 edges would not need the mode distinction, but increase chance
 * of failure.
 *
 * \param bus   Specifies the SPI bus .
 * \return      None.
 */
static void Stm32SpiCbBusRxStop(NUTSPIBUS *bus)
{
    uint32_t mode;
    STM32_SPI_DCB *dcb;
    SPI_TypeDef *spi;
    uint32_t mask;
    volatile uint32_t *idr;

    dcb = bus->bus_dcb;
    spi = (SPI_TypeDef *)bus->bus_base;
    mask = dcb->sck_input_mask;
    idr  = &dcb->sck_gpio->IDR;

    mode = spi->CR1 & (SPI_CR1_CPOL | SPI_CR1_CPHA);
    switch (mode) {
    case 0:
    case 3:
        /* Sample at rising edge */
        while (  *idr & mask );
        while (!(*idr & mask));
        break;
    default:
        /* Sample at falling edge */
        while (!(*idr & mask));
        while (  *idr & mask );
    }
    spi->CR1 &= ~SPI_CR1_SPE;
}

static void Stm32SpiCbBusInterrupt(void *arg)
{
    NUTSPIBUS *bus;
    SPI_TypeDef *spi;
    STM32_SPI_DCB *dcb;

    bus = (NUTSPIBUS *)arg;
    dcb = bus->bus_dcb;
    spi= (SPI_TypeDef *)bus->bus_base;

    if (spi->SR & SPI_SR_RXNE) {
        uint8_t b;
        b = spi->DR;
        if (dcb->spi_rx_len) {
            if (dcb->spi_rxp) {
                *dcb->spi_rxp = b;
                dcb->spi_rxp++;
                dcb->spi_rx_len--;
                if (dcb->spi_rx_len == 1) {
                    if (spi->CR1 & (SPI_CR1_BIDIMODE| SPI_CR1_RXONLY)) {
                        Stm32SpiCbBusRxStop(bus);
                    }
                }
            }
        }
        /* Terminate when the requested number of bytes have been received
         * even so perhaps we didn't need to store them.
         * That way we can make sure we don't deassert CS while SCK
         * is still running.
         */
        dcb->spi_len --;
        if (dcb->spi_len == 0) {
            spi->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
            NutEventPostFromIrq(&bus->bus_ready);
        }
    }
    if (spi->SR & SPI_SR_TXE) {
        if (dcb->spi_tx_len) {
            /* Half word access via "spi->DR = b" shifts out two frames
             * on the F373! */
            *(uint8_t*)&spi->DR = *dcb->spi_txp;
            /* After sending the last byte we need to wait for the last
             * receive interrupt, but we are not interested in the transmitter
             * empty interrupt
             */
            dcb->spi_tx_len --;
            if (dcb->spi_tx_len == 0) {
                spi->CR2 &= ~SPI_CR2_TXEIE;
            }
            dcb->spi_txp++;
        }
    }
}
static void Stm32SpiCbDmaInterrupt(void *arg)
{
    NUTSPIBUS *bus;
    const STM32_SPI_ICB *bus_icb;

    bus = (NUTSPIBUS *) arg;
    bus_icb = (STM32_SPI_ICB *)bus->bus_icb;
    DMA_ClearFlag(bus_icb->dma_rx, DMA_TCIF);
    NutEventPostFromIrq(&bus->bus_ready);
}

/*!
 * \brief Set the specified chip select to a given level.
 */
static int Stm32SpiCbChipSelect(NUTSPINODE *node, int assert)
{
    int res;
    int hi;
    const STM32_SPI_ICB *bus_icb;
    nutgpio_t cs_pin;

    bus_icb = (STM32_SPI_ICB *)node->node_bus->bus_icb;
    cs_pin = bus_icb->cs[node->node_cs];
    res = -1;

    if (node->node_mode & SPI_MODE_CSHIGH) {
        hi = assert;
    } else {
        hi = !assert;
    }
    res = Stm32GpioSet(cs_pin, hi);
    return res;
}

/*! \brief Deselect a device on the SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int Stm32SpiCbDeselect(NUTSPINODE * node)
{
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    NutSpiBusWait(node, NUT_WAIT_INFINITE);
    /* Deactivate the node's chip select. */
    Stm32SpiCbChipSelect(node, 0);

    /* Release the bus. */
    NutEventPostAsync(&node->node_bus->bus_mutex);

    return 0;
}

/*! \brief Pull SCK to the node-dependant idle level.
 *
 * \param node Specifies the SPI bus node.
 */
static void SetNodeSckIdleLevel(NUTSPINODE * node)
{
    STM32_SPI_DCB *spi_dcb = (STM32_SPI_DCB *)(node->node_bus->bus_dcb);

#if defined (MCU_STM32F1)
    if (node->node_mode & SPI_MODE_CPOL) {
        spi_dcb->sck_gpio->BRR  = spi_dcb->sck_input_mask;
    } else {
        spi_dcb->sck_gpio->BSRR = spi_dcb->sck_input_mask;
    }
#else
    uint32_t pupdr;

    pupdr  = spi_dcb->sck_gpio->PUPDR;
    if (node->node_mode & SPI_MODE_CPOL) {
        pupdr |=  (1 * spi_dcb->sck_pupdr_mask);
        pupdr &= ~(2 * spi_dcb->sck_pupdr_mask);
    } else {
        pupdr &= ~(1 * spi_dcb->sck_pupdr_mask);
        pupdr |=  (2 * spi_dcb->sck_pupdr_mask);
    }
    spi_dcb->sck_gpio->PUPDR = pupdr;
#endif
}

/*!
 * \brief Update SPI shadow registers.
 *
 * Called on setup of the node or when node parameters change
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
static int Stm32SpiCbNodeSetup(NUTSPINODE * node)
{
    uint32_t clk;
    STM32_SPINODE_VALUES *node_values;
    uint32_t cr1;
    int i;

    node_values = node->node_stat;

    cr1 = node_values->node_CR1;
    cr1 &= ~(SPI_CR1_LSBFIRST | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR);
#if defined(SPI_CR2_DS)
    node_values->node_CR2 &= ~SPI_CR2_DS;
    if ((node->node_bits > 6) && (node->node_bits <= 16)) {
        node_values->node_CR2 |= (node->node_bits - 1) <<8;
    }
#else
    switch(node->node_bits){
        case 8:
            cr1 &= ~(SPI_CR1_DFF);
            break;
        case 16:
            cr1 |= SPI_CR1_DFF;
            break;
        default:
            break;
    };
#endif
    if (node->node_mode & SPI_MODE_CPOL) {
        cr1 |= SPI_CR1_CPOL;
    }
    if (node->node_mode & SPI_MODE_CPHA) {
        cr1 |= SPI_CR1_CPHA;
    }
    if (node->node_mode & SPI_MODE_LSB) {
        cr1 |= SPI_CR1_LSBFIRST;
    }
    /* Query peripheral clock. */
    clk = Stm32ClockGet(BASE2CLKSRC(node->node_bus->bus_base));
    for (i = 0; i < 7 ; i++) {
        clk >>= 1;
        if (node->node_rate >= clk) {
            break;
        }
    }
    node->node_rate = clk;
    cr1 |= i * SPI_CR1_BR_0;
    node_values->node_CR1 = cr1;
    /* Update interface parameters. */
    node->node_mode &= ~SPI_MODE_UPDATE;

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
static int Stm32SpiCbSelect(NUTSPINODE * node, uint32_t tmo)
{
    int rc;
    SPI_TypeDef *base = (SPI_TypeDef *)node->node_bus->bus_base;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_stat != NULL);

    /* Allocate the bus. */
    rc = NutEventWait(&node->node_bus->bus_mutex, tmo);
    if (rc) {
        errno = EIO;
    } else {
        STM32_SPINODE_VALUES *node_values = node->node_stat;

        /* If the mode update bit is set, then update our shadow registers. */
        if (node->node_mode & SPI_MODE_UPDATE) {
            Stm32SpiCbNodeSetup(node);
        }

        /* Set Idle level before enabling SPI.
         * See note e.g. in RM0316 28.5.5 Communication formats
         */
        SetNodeSckIdleLevel(node);

        /* Set SPI mode. */
        base->CR1 = node_values->node_CR1;
        base->CR2 = node_values->node_CR2;
        /* Finally activate the node's chip select. */
        rc = Stm32SpiCbChipSelect(node, 1);
        if (rc) {
            /* Release the bus in case of an error. */
            NutEventPost(&node->node_bus->bus_mutex);
        }
    }
    return rc;
}

/*!
 * \brief Initialize an SPI bus..
 *
 * This routine is called for the first SPI node, which is registered via
 * NutRegisterSpiDevice().
 *
 * \param bus      Specifies the SPIBUS.
 *
 * \return 0 on success or -1 if there is no valid chip select.
 */
static int Stm32SpiCbBusInit(NUTSPIBUS *bus)
{
    uint32_t flags;
    int pin_nr;
    GPIO_TypeDef *gpio;
    STM32_SPI_DCB *bus_dcb;
    const STM32_SPI_ICB *bus_icb;

    bus_icb = (STM32_SPI_ICB *)bus->bus_icb;
    bus_dcb = malloc(sizeof(STM32_SPI_DCB));
    if (!bus_dcb) {
        return -1;
    }
    /* Initialize Bus Hardware */
    flags = bus_icb->device_pin_speed;
    flags |= GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT | GPIO_CFG_INIT_LOW;
    Stm32GpioConfigSet(bus_icb->sck,  flags            , bus_icb->sck_af );
    Stm32GpioConfigSet(bus_icb->miso, GPIO_CFG_PERIPHAL, bus_icb->miso_af);
    Stm32GpioConfigSet(bus_icb->mosi, flags            , bus_icb->mosi_af);
    /* Calculate SCK/MOSI speed register pointer and offsets.
     * nutgpio_t enum defeats compile time compilation!
     */
    gpio = stm32_port_nr2gpio[bus_icb->sck >> 8];
    pin_nr = bus_icb->sck & 0xf;
    bus_dcb->sck_gpio = gpio;
    bus_dcb->sck_input_mask = 1 << pin_nr;
    bus_dcb->sck_pupdr_mask = 1 << (pin_nr << 1);
    bus->bus_dcb = bus_dcb;
    Stm32SpiCbClkEnable(bus_icb, 1);
    if (bus_icb->transfer_mode == IRQ_MODE) {
        NutRegisterIrqHandler(bus->bus_sig, Stm32SpiCbBusInterrupt, bus);
        NutIrqEnable(bus->bus_sig);
    } else if (bus_icb->transfer_mode == DMA_MODE) {
        DMA_SIGNAL *sig;
        DMA_Init();
# if defined(HW_DMA_CSELR_STM32)
        DmaChannelSelection(bus_icb->dma_rx, bus_icb->dma_rx_csel);
        DmaChannelSelection(bus_icb->dma_tx, bus_icb->dma_tx_csel);
#endif
        sig = DmaCreateHandler(bus_icb->dma_rx, bus_icb->dma_rx_irq);
        DmaRegisterHandler(sig, Stm32SpiCbDmaInterrupt, bus,
                           bus_icb->dma_rx, bus_icb->dma_rx_irq);
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
 * \return 0 on success or -1 if there is no valid chip select.
 */
static int Stm32SpiCbNodeInit(NUTSPINODE *node)
{
    int rc;
    uint32_t init_flag;
    NUTSPIBUS *bus;
    const STM32_SPI_ICB *bus_icb;
    nutgpio_t cs_pin

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != NULL);
    bus = node->node_bus;
    bus_icb = (STM32_SPI_ICB *)bus->bus_icb;
    /* Initialize CS pin */
    if ((node->node_mode & SPI_MODE_CSHIGH) == 0) {
        init_flag = GPIO_CFG_INIT_HIGH;
    } else {
        init_flag = GPIO_CFG_INIT_LOW;
    }
    cs_pin = bus_icb->cs[node->node_cs];
    rc = Stm32GpioConfigSet(cs_pin, GPIO_CFG_OUTPUT | init_flag, 0);
    if (rc) {
        return rc;
    }
    if (node->node_bits != 8) {
        return -1;
    }
    /* Test if SPI Device is already initialized*/
    if (!bus->bus_dcb) {
        rc = Stm32SpiCbBusInit(bus);
    }
    /* It should not hurt us being called more than once. Thus, we
     * check wether any initialization had been taken place already.
     */
    if ((rc == 0) && (node->node_stat == NULL)) {
        /* Allocate and set our shadow registers. */
        STM32_SPINODE_VALUES *node_values =
            malloc(sizeof(STM32_SPINODE_VALUES));

        if (!node_values) {
            /* Out of memory? */
            rc = -1;
        } else {
            /* Set interface defaults. */
            node_values->node_CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
            /* FIXME: Check values needed*/
#if defined(SPI_CR2_FRXTH)
            node_values->node_CR2 = SPI_CR2_FRXTH |
                ((node->node_bits - 1) * SPI_CR2_DS_0);
#else
            node_values->node_CR2 =  0;
#endif
            /* Update with node's defaults. */
            node->node_stat = (void *)node_values;
            Stm32SpiCbNodeSetup(node);
        }
    }
    return rc;
}

static void PollingTransfer(
    NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    SPI_TypeDef *base;
    STM32_SPI_DCB *bus_dcb;

    base = (SPI_TypeDef *)node->node_bus->bus_base;
    bus_dcb = (STM32_SPI_DCB *)node->node_bus->bus_dcb;

    if (!rxbuf) {
        base->CR1 |= SPI_CR1_SPE;
        while (xlen > 0) {
            /* Half word access via "spi->DR = b" shifts out two frames
             * on the F373! */
            *(uint8_t *)&base->DR = *(const uint8_t *)txbuf;
            xlen--;
            txbuf++;
            while ((base->SR & SPI_SR_TXE) == 0); /* Wait till TXE = 1*/
        }
        while (base->SR & SPI_SR_BSY);     /* Wait till BSY = 0 */
        /* Wait for SCK idle */
        if (node->node_mode & SPI_MODE_CPOL) {
            while (!(bus_dcb->sck_gpio->IDR & bus_dcb->sck_input_mask));
        } else {
            while ((bus_dcb->sck_gpio->IDR & bus_dcb->sck_input_mask));
        }
    } else if (!txbuf) {
        base->CR1 |= SPI_CR1_SPE;
        while (xlen > 0) {
            if (xlen < 2) {
                Stm32SpiCbBusRxStop(node->node_bus);
            }
            xlen--;
            while ((base->SR & SPI_SR_RXNE) == 0); /* Wait till RXNE = 1*/
            if (rxbuf) {
                *(uint8_t *)rxbuf = base->DR;
                rxbuf++;
            }
        }
    } else {
        base->CR1 |= SPI_CR1_SPE;
        *(uint8_t *)&base->DR = *(const uint8_t *)txbuf; /* Write first item */
        while (xlen > 0) {
            txbuf++;
            xlen --;
#if defined(SPI_CR2_FRXTH)
            /* F0/F3/F7/L4 has 4 byte Fifo.
               Write new data before reading last data.*/
            if (xlen > 0) {
                while ((base->SR & SPI_SR_TXE) == 0); /* Wait till TXE = 1*/
                *(uint8_t *)&base->DR = *(const uint8_t *)txbuf;
            }
            while ((base->SR & SPI_SR_RXNE) == 0);/* Wait till RXNE = 1*/
            *(uint8_t *)rxbuf = base->DR;
#else
            /* First version of SPI implementation has only "one-byte" Fifo.
               Read last data before writing next data as otherwise with
               fast transfer, received byte can get lost.*/
             while ((base->SR & SPI_SR_RXNE) == 0);/* Wait till RXNE = 1*/
            *(uint8_t *)rxbuf = base->DR;
            while ((base->SR & SPI_SR_TXE) == 0); /* Wait till TXE = 1*/
            if (xlen > 0) {
                while ((base->SR & SPI_SR_TXE) == 0); /* Wait till TXE = 1*/
                *(uint8_t *)&base->DR = *(const uint8_t *)txbuf;
            }
#endif
            rxbuf++;
        }
        /* Wait until SCK reaches idle level.*/
        if (node->node_mode & SPI_MODE_CPOL) {
            while (!(bus_dcb->sck_gpio->IDR & bus_dcb->sck_input_mask));
        } else {
            while (  bus_dcb->sck_gpio->IDR & bus_dcb->sck_input_mask );
        }
    }
}

/*!
 * \brief Transfer data on the SPI bus using single buffered interrupt mode.
 *
 * A device must have been selected by calling SpiSelect().
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
static int Stm32SpiCbTransfer
    (NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    NUTSPIBUS *bus;
    SPI_TypeDef *base;
    const STM32_SPI_ICB *bus_icb;

    /* Sanity check and query MISO. */
    if (xlen == 0) {
        const STM32_SPI_ICB *bus_icb;
        bus_icb = (STM32_SPI_ICB *)node->node_bus->bus_icb;
        return Stm32GpioGet(bus_icb->miso);
    }

    bus = node->node_bus;
    base = (SPI_TypeDef *)bus->bus_base;
    bus_icb = (STM32_SPI_ICB *)bus->bus_icb;

    /* Remove any remainders in DR */
    while (base->SR & SPI_SR_RXNE) {
        (void) base->DR; /* Empty DR */
    }
    if (!txbuf) {
        if (node->node_mode & SPI_MODE_HALFDUPLEX) {
            base->CR1 |= SPI_CR1_BIDIMODE;
        } else {
            base->CR1 |= SPI_CR1_RXONLY;
        }
    }

    if (bus_icb->transfer_mode == POLLING_MODE) {
        PollingTransfer(node, txbuf, rxbuf, xlen);
    } else if (bus_icb->transfer_mode == DMA_MODE) {
        if (!txbuf) {
            DMA_Setup(bus_icb->dma_rx, rxbuf, (void*)(&base->DR),
                      xlen , DMA_MINC);
            DMA_Enable(bus_icb->dma_rx);
        } else {
            DMA_Setup(bus_icb->dma_tx, (void*)(&base->DR), (void*) txbuf,
                      xlen, DMA_MINC);
            DMA_Enable(bus_icb->dma_tx);
            if (!rxbuf) {
                uint32_t dummy;
                DMA_Setup(bus_icb->dma_rx, &dummy, (void*)(&base->DR), xlen, 0);
            } else {
                DMA_Setup(bus_icb->dma_rx, rxbuf, (void*)(&base->DR), xlen,
                          DMA_MINC);
            }
            base->CR2 |= SPI_CR2_TXDMAEN;
        }
        DMA_Enable(bus_icb->dma_rx);
        DMA_IrqMask(bus_icb->dma_rx, DMA_TCIF, 1);
        base->CR2 |= SPI_CR2_RXDMAEN;
        base->CR1 |= SPI_CR1_SPE;
        NutEventWait(&node->node_bus->bus_ready, NUT_WAIT_INFINITE);
    } else {
        /* IRQ Transfer */
        STM32_SPI_DCB *dcb;

        dcb = bus->bus_dcb;
        dcb->spi_txp = txbuf;
        dcb->spi_rxp = rxbuf;
        dcb->spi_len = xlen;
        dcb->spi_rx_len = xlen;
        if (!txbuf) {
            dcb->spi_tx_len = 0;
            base->CR2 |= SPI_CR2_RXNEIE;
            base->CR1 |= SPI_CR1_SPE;
        } else {
            dcb->spi_tx_len = xlen;
            if (!rxbuf) {
                dcb->spi_rx_len = 0;
            }
            base->CR2 |= SPI_CR2_RXNEIE;
            base->CR1 |= SPI_CR1_SPE;
            base->CR2 |= SPI_CR2_TXEIE;
        }
        NutEventWait(&node->node_bus->bus_ready, NUT_WAIT_INFINITE);
    }
    base->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_RXONLY | SPI_CR1_BIDIMODE);
    base->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    return 0;
}

#if defined(HW_SPI1_STM32)
STM32_SPI_ICB Stm32Spi1Icb = {
    .transfer_mode = SPI1_MODE,
    .device_pin_speed = SPI1_SPEED,
#if defined(MCU_STM32F1)
    .remap_reg = &AFIO->MAPR,
    .remap_mask = AFIO_MAPR_SPI1_REMAP,
    .remap_value = SPI1_REMAP * AFIO_MAPR_SPI1_REMAP,
#endif
    .enable_reg = &RCC->APB2ENR,
    .enable_mask = RCC_APB2ENR_SPI1EN,
    .dma_tx_irq = DMA_CH2IRQ_P(SPI1_DMA_TX),
    .dma_rx_irq = DMA_CH2IRQ_P(SPI1_DMA_RX),
    .dma_tx = SPI1_DMA_TX,
    .dma_rx = SPI1_DMA_RX,
#if defined(HW_DMA_CSELR_STM32)
    .dma_rx_csel = SPI1_DMA_RX_SEL(SPI1_DMA_RX),
    .dma_tx_csel = SPI1_DMA_TX_SEL(SPI1_DMA_TX),
#endif
    .sck = SPI1_SCK,
    .sck_af = SPI1_SCK_AF,
    .mosi = SPI1_MOSI,
    .mosi_af = SPI1_MOSI_AF,
    .miso = SPI1_MISO,
    .miso_af = SPI1_MISO_AF,
    .cs[0] = SPI1_CS0,
    .cs[1] = SPI1_CS1,
    .cs[2] = SPI1_CS2,
    .cs[3] = SPI1_CS3
};

NUTSPIBUS spiBus1Stm32Cb = {
    .bus_sig      =  &sig_SPI1,
    .bus_base     = (uintptr_t)SPI1_BASE,
    .bus_initnode = Stm32SpiCbNodeInit,
    .bus_alloc    = Stm32SpiCbSelect,
    .bus_release  = Stm32SpiCbDeselect,
    .bus_transfer = Stm32SpiCbTransfer,
    .bus_wait     = NutSpiBusWait,
    .bus_set_mode = NutSpiBusSetMode,
    .bus_set_rate = NutSpiBusSetRate,
    .bus_set_bits = NutSpiBusSetBits,
    .bus_icb      = (void*)&Stm32Spi1Icb,
};
#endif

#if defined(HW_SPI2_STM32)
STM32_SPI_ICB Stm32Spi2Icb = {
    .transfer_mode = SPI2_MODE,
    .device_pin_speed = SPI2_SPEED,
#if defined(MCU_STM32F1)
    .remap_reg = &AFIO->MAPR,
    .remap_mask = 0,
    .remap_value = 0,
#endif
    .enable_reg = &RCC->APB1ENR,
    .enable_mask = RCC_APB1ENR_SPI2EN,
    .dma_tx_irq = DMA_CH2IRQ_P(SPI2_DMA_TX),
    .dma_rx_irq = DMA_CH2IRQ_P(SPI2_DMA_RX),
    .dma_tx = SPI2_DMA_TX,
    .dma_rx = SPI2_DMA_RX,
#if defined(HW_DMA_CSELR_STM32)
    .dma_rx_csel = SPI2_DMA_RX_SEL(SPI2_DMA_RX),
    .dma_tx_csel = SPI2_DMA_TX_SEL(SPI2_DMA_TX),
#endif
    .sck = SPI2_SCK,
    .sck_af = SPI2_SCK_AF,
    .mosi = SPI2_MOSI,
    .mosi_af = SPI2_MOSI_AF,
    .miso = SPI2_MISO,
    .miso_af = SPI2_MISO_AF,
    .cs[0] = SPI2_CS0,
    .cs[1] = SPI2_CS1,
    .cs[2] = SPI2_CS2,
    .cs[3] = SPI2_CS3
};

NUTSPIBUS spiBus2Stm32Cb = {
    .bus_sig      =  &sig_SPI2,
    .bus_base     = (uintptr_t)SPI2_BASE,
    .bus_initnode = Stm32SpiCbNodeInit,
    .bus_alloc    = Stm32SpiCbSelect,
    .bus_release  = Stm32SpiCbDeselect,
    .bus_transfer = Stm32SpiCbTransfer,
    .bus_wait     = NutSpiBusWait,
    .bus_set_mode = NutSpiBusSetMode,
    .bus_set_rate = NutSpiBusSetRate,
    .bus_set_bits = NutSpiBusSetBits,
    .bus_icb      = (void*)&Stm32Spi2Icb,
};
#endif

#if defined(HW_SPI3_STM32)
STM32_SPI_ICB Stm32Spi3Icb = {
     .transfer_mode = SPI3_MODE,
     .device_pin_speed = SPI3_SPEED,
#if defined(MCU_STM32F1) && defined(AFIO_MAPR_SPI3_REMAP)
     .remap_reg = &AFIO->MAPR,
     .remap_mask = AFIO_MAPR_SPI3_REMAP,
     .remap_value = SPI3_REMAP * AFIO_MAPR_SPI3_REMAP,
#endif
     .enable_reg = &RCC->APB1ENR,
     .enable_mask = RCC_APB1ENR_SPI3EN,
     .dma_tx_irq = DMA_CH2IRQ_P(SPI3_DMA_TX),
     .dma_rx_irq = DMA_CH2IRQ_P(SPI3_DMA_RX),
     .dma_tx = SPI3_DMA_TX,
     .dma_rx = SPI3_DMA_RX,
#if defined(HW_DMA_CSELR_STM32)
     .dma_rx_csel = SPI3_DMA_RX_SEL(SPI3_DMA_RX),
     .dma_tx_csel = SPI3_DMA_TX_SEL(SPI3_DMA_TX),
#endif
     .sck = SPI3_SCK,
     .sck_af = SPI3_SCK_AF,
     .mosi = SPI3_MOSI,
     .mosi_af = SPI3_MOSI_AF,
     .miso = SPI3_MISO,
     .miso_af = SPI3_MISO_AF,
     .cs[0] = SPI3_CS0,
     .cs[1] = SPI3_CS1,
     .cs[2] = SPI3_CS2,
     .cs[3] = SPI3_CS3
};

NUTSPIBUS spiBus3Stm32Cb = {
    .bus_sig      =  &sig_SPI3,
    .bus_base     = (uintptr_t)SPI3_BASE,
    .bus_initnode = Stm32SpiCbNodeInit,
    .bus_alloc    = Stm32SpiCbSelect,
    .bus_release  = Stm32SpiCbDeselect,
    .bus_transfer = Stm32SpiCbTransfer,
    .bus_wait     = NutSpiBusWait,
    .bus_set_mode = NutSpiBusSetMode,
    .bus_set_rate = NutSpiBusSetRate,
    .bus_set_bits = NutSpiBusSetBits,
    .bus_icb      = &Stm32Spi3Icb,
};
#endif

#if defined(HW_SPI4_STM32)
STM32_SPI_ICB Stm32Spi4Icb = {
     .transfer_mode = SPI4_MODE,
     .device_pin_speed = SPI4_SPEED,
    .enable_reg = &RCC->APB2ENR,
    .enable_mask = RCC_APB2ENR_SPI4EN,
    .dma_tx_irq = DMA_CH2IRQ_P(SPI4_DMA_TX),
    .dma_rx_irq = DMA_CH2IRQ_P(SPI4_DMA_RX),
    .dma_tx = SPI4_DMA_TX,
    .dma_rx = SPI4_DMA_RX,
    .sck = SPI4_SCK,
    .sck_af = SPI4_SCK_AF,
    .mosi = SPI4_MOSI,
    .mosi_af = SPI4_MOSI_AF,
    .miso = SPI4_MISO,
    .miso_af = SPI4_MISO_AF,
    .cs[0] = SPI4_CS0,
    .cs[1] = SPI4_CS1,
    .cs[2] = SPI4_CS2,
    .cs[3] = SPI4_CS3
};

NUTSPIBUS spiBus4Stm32Cb = {
    .bus_sig      =  &sig_SPI4,
    .bus_base     = (uintptr_t)SPI4_BASE,
    .bus_initnode = Stm32SpiCbNodeInit,
    .bus_alloc    = Stm32SpiCbSelect,
    .bus_release  = Stm32SpiCbDeselect,
    .bus_transfer = Stm32SpiCbTransfer,
    .bus_wait     = NutSpiBusWait,
    .bus_set_mode = NutSpiBusSetMode,
    .bus_set_rate = NutSpiBusSetRate,
    .bus_set_bits = NutSpiBusSetBits,
    .bus_icb      = &Stm32Spi4Icb,
};
#endif

#if defined(HW_SPI5_STM32)
STM32_SPI_ICB Stm32Spi5Icb = {
     .transfer_mode = SPI5_MODE,
     .device_pin_speed = SPI5_SPEED,
    .enable_reg = &RCC->APB2ENR,
    .enable_mask = RCC_APB2ENR_SPI5EN,
    .dma_tx_irq = DMA_CH2IRQ_P(SPI5_DMA_TX),
    .dma_rx_irq = DMA_CH2IRQ_P(SPI5_DMA_RX),
    .dma_tx = SPI5_DMA_TX,
    .dma_rx = SPI5_DMA_RX,
    .sck = SPI5_SCK,
    .sck_af = SPI5_SCK_AF,
    .mosi = SPI5_MOSI,
    .mosi_af = SPI5_MOSI_AF,
    .miso = SPI5_MISO,
    .miso_af = SPI5_MISO_AF,
    .cs[0] = SPI5_CS0,
    .cs[1] = SPI5_CS1,
    .cs[2] = SPI5_CS2,
    .cs[3] = SPI5_CS3
};

NUTSPIBUS spiBus5Stm32Cb = {
    .bus_sig      =  &sig_SPI5,
    .bus_base     = (uintptr_t)SPI5_BASE,
    .bus_initnode = Stm32SpiCbNodeInit,
    .bus_alloc    = Stm32SpiCbSelect,
    .bus_release  = Stm32SpiCbDeselect,
    .bus_transfer = Stm32SpiCbTransfer,
    .bus_wait     = NutSpiBusWait,
    .bus_set_mode = NutSpiBusSetMode,
    .bus_set_rate = NutSpiBusSetRate,
    .bus_set_bits = NutSpiBusSetBits,
    .bus_icb      = &Stm32Spi5Icb,
};
#endif

#if defined(HW_SPI6_STM32)
STM32_SPI_ICB Stm32Spi6Icb = {
     .transfer_mode = SPI6_MODE,
     .device_pin_speed = SPI6_SPEED,
    .enable_reg = &RCC->APB2ENR,
    .enable_mask = RCC_APB2ENR_SPI6EN,
    .dma_tx_irq = DMA_CH2IRQ_P(SPI6_DMA_TX),
    .dma_rx_irq = DMA_CH2IRQ_P(SPI6_DMA_RX),
    .dma_tx = SPI6_DMA_TX,
    .dma_rx = SPI6_DMA_RX,
    .sck = SPI6_SCK,
    .sck_af = SPI6_SCK_AF,
    .mosi = SPI6_MOSI,
    .mosi_af = SPI6_MOSI_AF,
    .miso = SPI6_MISO,
    .miso_af = SPI6_MISO_AF,
    .cs[0] = SPI6_CS0,
    .cs[1] = SPI6_CS1,
    .cs[2] = SPI6_CS2,
    .cs[3] = SPI6_CS3
};

NUTSPIBUS spiBus6Stm32Cb = {
    .bus_sig      =  &sig_SPI6,
    .bus_base     = (uintptr_t)SPI6_BASE,
    .bus_initnode = Stm32SpiCbNodeInit,
    .bus_alloc    = Stm32SpiCbSelect,
    .bus_release  = Stm32SpiCbDeselect,
    .bus_transfer = Stm32SpiCbTransfer,
    .bus_wait     = NutSpiBusWait,
    .bus_set_mode = NutSpiBusSetMode,
    .bus_set_rate = NutSpiBusSetRate,
    .bus_set_bits = NutSpiBusSetBits,
    .bus_icb      = &Stm32Spi6Icb,
};
#endif
