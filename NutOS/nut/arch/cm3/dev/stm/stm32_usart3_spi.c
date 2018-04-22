/*
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
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
 * $Id: stm32_usart3_spi.c 6367 2016-03-15 10:27:09Z u_bonnes $
 */

#include <arch/cm3.h>
#include <sys/timer.h>
#include <cfg/spi.h>
#include <cfg/arch/gpio.h>
#include <dev/spibus.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32f10x_rcc.h>
#include <arch/cm3/stm/stm32f10x_usart.h>
#include <dev/irqreg.h>
#include <sys/event.h>
#include <sys/nutdebug.h>

#include <stdlib.h>
#include <errno.h>


static int Stm32SpiBusWait(NUTSPINODE * node, uint32_t tmo);
//static HANDLE spi0_que;
static HANDLE usart3_que;

static uint8_t * volatile usart3_txp;
static uint8_t * volatile usart3_rxp;
static volatile size_t usart3_xc;


int Stm32UsartBusWait(NUTSPINODE * node, uint32_t tmo);
int Stm32UsartSpiSetup(NUTSPINODE * node);

IRQ_HANDLER sig_USART3 = {
#ifdef NUT_PERFMON
    0,                          /* Interrupt counter, ir_count. */
#endif
    NULL,                       /* Passed argument, ir_arg. */
    NULL,                       /* Handler subroutine, ir_handler. */
    NULL//SerialPeripheral2IrqCtl     /* Interrupt control, ir_ctl. */
};

/*!
 * \brief Set the specified chip select to a given level.
 */
int Stm32Usart3ChipSelect(uint_fast8_t cs, uint_fast8_t hi)
{
    int rc = 0;

    switch (cs) {
    case 0:
        if (hi) {
            GpioPinSetHigh(NUTGPIO_PORTD,13);
        } else {
            GpioPinSetLow(NUTGPIO_PORTD,13);
        }
        break;
    default:
        errno = EIO;
        rc = -1;
        break;
    }
    return rc;
}



/*! \brief Deselect a device on the first SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
int Stm32Usart3BusDeselect(NUTSPINODE * node)
{
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    Stm32UsartBusWait(node, NUT_WAIT_INFINITE);
    /* Deactivate the node's chip select. */
    Stm32Usart3ChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);

    /* Release the bus. */
    NutEventPost(&node->node_bus->bus_mutex);

    return 0;
}

/*! \brief Select a device on the first SPI bus.
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
int Stm32Usart3BusSelect(NUTSPINODE * node, uint32_t tmo)
{
    int rc;
    USART_TypeDef* base;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_stat != NULL);

    /* Allocate the bus. */
    rc = NutEventWait(&node->node_bus->bus_mutex, tmo);
    if (rc) {
        errno = EIO;
    } else {
        USART_TypeDef *spireg = node->node_stat;

        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->APB2ENR |= RCC_APB2ENR_GPIODEN | //USART RX,TX,CK
            RCC_APB2ENR_AFIOEN;

    //У нас USART3 висит на PD8,9,10
    AFIO->MAPR |= GPIO_FullRemap_USART3;

    //Ставим на альтернативную функцию - как раз на usart1
    //tx
    GpioPinConfigSet( NUTGPIO_PORTD, 8,
            GPIO_CFG_DISABLED|GPIO_CFG_OUTPUT);
    //rx
    GpioPinConfigSet( NUTGPIO_PORTD, 9,
            0);
    //ck
    GpioPinConfigSet( NUTGPIO_PORTD, 10,
            GPIO_CFG_DISABLED|GPIO_CFG_OUTPUT);
    //На выход - для Chipselect-а
    GpioPinConfigSet(NUTGPIO_PORTD, 13,
            GPIO_CFG_OUTPUT);//FIXME: check correct pin

        /* If the mode update bit is set, then update our shadow registers. */
        if (node->node_mode & SPI_MODE_UPDATE) {
            Stm32UsartSpiSetup(node);
        }

        /* Enable SPI. */
    base=node->node_bus->bus_base;
    //spireg->CR1|=(1<<6);//SPE -spi enable
        /* Set SPI mode. */
    base->CR2=spireg->CR2;
    base->BRR=spireg->BRR;
    base->CR1=spireg->CR1;

        /* Finally activate the node's chip select. */
        rc = Stm32Usart3ChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) != 0);
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
int Stm32UsartSpiSetup(NUTSPINODE * node)
{
    uint32_t clk;
    uint8_t i;
    uint32_t clkdiv;
    USART_TypeDef *spireg;

    NUTASSERT(node != NULL);
    NUTASSERT(node->node_stat != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != 0);
    spireg = node->node_stat;

    spireg->CR1 =USART_Mode_Rx|USART_Mode_Tx|(1<<13);//fixme: write real values
    spireg->CR2 = USART_Clock_Enable|USART_LastBit_Enable;
    spireg->BRR =16;
    switch(node->node_bits){
        case 8:
            spireg->CR1 &= ~(USART_WordLength_9b);
            spireg->CR1 &= ~(1<<10);
            break;
        case 9:
            spireg->CR1 |= USART_WordLength_9b;
            spireg->CR1 &= ~(1<<10);
            break;
        default:
            break;
    };
    if (node->node_mode & SPI_MODE_CPOL) {
        spireg->CR2 |= USART_CPOL_High;
    }
    if ((node->node_mode & SPI_MODE_CPHA) == 0) {
        spireg->CR2 |= USART_CPHA_2Edge;
    }

    /* Query peripheral clock. */
    clk = NutClockGet(HWCLK_APB1);
    /* Calculate the SPI clock divider. Avoid rounding errors. */
    clkdiv = clk/(node->node_rate);
    if (clkdiv < 16) {
        clkdiv=16;
    }
    spireg->BRR = clkdiv;

    /* Update interface parameters. */
    node->node_rate = clk / (clkdiv);
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
int Stm32UsartBusNodeInit(NUTSPINODE * node)
{
    int rc;
    NUTSPIBUS *bus;
    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    bus = node->node_bus;

    rc = Stm32Usart3ChipSelect(node->node_cs, (node->node_mode & SPI_MODE_CSHIGH) == 0);
    /* It should not hurt us being called more than once. Thus, we
       ** check wether any initialization had been taken place already. */
    if (rc == 0 && node->node_stat == NULL) {
        /* Allocate and set our shadow registers. */
        USART_TypeDef *spireg = malloc(sizeof(USART_TypeDef));
        if (spireg) {
            /* Set interface defaults. */
        spireg->CR1 =USART_Mode_Rx|USART_Mode_Tx|(1<<13);//fixme: write real values
            spireg->CR2 = USART_Clock_Enable|USART_LastBit_Enable;
        spireg->BRR = 16;
            /* Update with node's defaults. */
            node->node_stat = (void *)spireg;
            Stm32UsartSpiSetup(node);

            /*
             * Register and enable SPI interrupt handler.
             */
//FIXME:add interrupt/dma support
//            if (bus->bus_base == SPI3_BASE) {
//                NutRegisterIrqHandler(bus->bus_sig, Stm32SpiBus2Interrupt, &bus->bus_ready);
//                outr(bus->bus_base + SPI_IDR_OFF, (unsigned int) - 1);
//                NutIrqEnable(bus->bus_sig);
//            } else {
//                NutRegisterIrqHandler(bus->bus_sig, Stm32SpiBus0Interrupt, &bus->bus_ready);
//                outr(bus->bus_base + SPI_IDR_OFF, (unsigned int) - 1);
//                NutIrqEnable(bus->bus_sig);
//            }
        } else {
            /* Out of memory? */
            rc = -1;
        }
    }
    return rc;
}

/*!
 * \brief Wait until all SPI bus transfers are done.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo  Timeout in milliseconds. To disable timeout, set this
 *             parameter to NUT_WAIT_INFINITE.
 *
 * \return Always 0.
 */
int Stm32UsartBusWait(NUTSPINODE * node, uint32_t tmo)
{//FIXME: check this - do I really need it?
/*  USART_TypeDef* spi_bus;
    spi_bus=((USART_TypeDef *) node->node_bus->bus_base);
    while (spi_bus->SR & USART_FLAG_RXNE) {
        if (NutEventWait(&node->node_bus->bus_ready, tmo)) {
           return -1;
        }
    }
    while (!(spi_bus->SR & USART_FLAG_TXE) ){
            if (NutEventWait(&node->node_bus->bus_ready, tmo)) {
               return -1;
           }
    }*/
    //Not Needed - transfer does this
    //FIXME: add interrupt support
    return 0;
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
int Stm32UsartBusTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    USART_TypeDef* base;
    uint8_t b=0xff;
    uint8_t *txp = (uint8_t *) txbuf;
    uint8_t *rxp = (uint8_t *) rxbuf;


    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    NUTASSERT(node->node_bus->bus_base != 0);
    base = node->node_bus->bus_base;
    while (xlen--) {
        if (txp) {
            b = *txp++;
        }
        /* Transmission starts by writing the transmit data. */
    base->DR=b;
        /* Wait for receiver data register full. */
        while( (base->SR & USART_FLAG_RXNE) == 0) { ; }
        /* Read incoming data. */
        b = base->DR;
    //b=(uint8_t)SPI_I2S_ReceiveData(USART1);
        if (rxp) {
            *rxp++ = b;
        }
    }
    return 0;

}



NUTSPIBUS spiUsart3BusStm32 = {
    NULL,                       /*!< Bus mutex semaphore (bus_mutex). */
    NULL,                       /*!< Bus ready signal (bus_ready). */
    USART3_BASE,                  /*!< Bus base address (bus_base). */
    &sig_USART3,                  /*!< Bus interrupt handler (bus_sig). */
    Stm32UsartBusNodeInit,         /*!< Initialize the bus (bus_initnode). */
    Stm32Usart3BusSelect,          /*!< Select the specified device (bus_alloc). */
    Stm32Usart3BusDeselect,        /*!< Deselect the specified device (bus_release). */
    Stm32UsartBusTransfer,
    Stm32UsartBusWait,
    NutSpiBusSetMode,           /*!< Set SPI mode of a specified device (bus_set_mode). */
    NutSpiBusSetRate,           /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits,           /*!< Set number of data bits of a specified device (bus_set_bits). */
    NULL,                       /*!< Private data of the hardware specific implementation. */
    NULL,                       /*!< Pointer to the bus driver's device control block. */
};
