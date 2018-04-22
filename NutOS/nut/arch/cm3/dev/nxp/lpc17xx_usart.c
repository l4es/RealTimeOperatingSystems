/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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


/*
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <inttypes.h>
#include <cfg/clock.h>

#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/usart.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <arch/cm3/nxp/lpc176x_gpio.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc17xx_usart.h>

#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
#include <arch/cm3/nxp/lpc17xx_gpdma.h>
#endif

/*!
 * \addtogroup xgNutArchArmLpc17xxUsart
 */
/*@{*/

#if defined(UART_XONXOFF_CONTROL)

/* \brief ASCII code for software flow control, starts transmitter. */
#define ASCII_XON       0x11
/* \brief ASCII code for software flow control, stops transmitter. */
#define ASCII_XOFF      0x13

/* \brief XON transmit pending flag. */
#define XON_PENDING     0x10
/* \brief XOFF transmit pending flag. */
#define XOFF_PENDING    0x20
/* \brief XOFF sent flag. */
#define XOFF_SENT       0x40
/* \brief XOFF received flag. */
#define XOFF_RCVD       0x80
#endif

/* USART default speed if not preset by nutconfig */
#ifndef USART_INIT_BAUTRATE
#define USART_INIT_BAUTRATE USART_INITSPEED
#endif

#define NutUartIrqEnable    NutIrqEnable
#define NutUartIrqDisable   NutIrqDisable

/*!
 * \brief Receiver error flags.
 */
static uint16_t rx_errors = 0;

#if defined(US_MODE_HWHANDSHAKE)
/*!
 * \brief Enables RTS control if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to control the RTS signal.
 */
static uint_fast8_t rts_control = 0;

/*!
 * \brief Enables CTS sense if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to sense the CTS signal.
 */

static uint_fast8_t cts_sense  = 0;
#endif

#if defined(UART_XONXOFF_CONTROL)
/*!
 * \brief Handles local software flow control.
 *
 * This variable exists only if the hardware configuration defines a
 * software flow control support.
 */
static uint_fast8_t flow_control = 0;
#endif

#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
/*!
 * \brief Handles local DMA block read/write control.
 *
 */
static uint_fast8_t block_control;

#define BC_RD_EN 0x01
#define BC_WR_EN 0x02

#endif

#if 0 // TODO: DMA
#ifdef UART_DMA_TXCHANNEL
static void Lpc17xxUsartDmaTxIrq(void* arg)
{
    NutEventPost(arg);
    NutSelectWakeupFromIrq();
}
#endif

#ifdef UART_DMA_RXCHANNEL
static void Lpc17xxUsartDmaRxIrq(void* arg)
{
    NutEventPost(arg);
    NutSelectWakeupFromIrq();
}
#endif
#endif

/*
 * \brief USARTn transmitter ready interrupt handler.
 *
 * \param rbf Pointer to the transmitter ring buffer.
 */
static void Lpc17xxUsartTxReady(RINGBUF * rbf, uint32_t lsr)
{
    register uint8_t *cp = rbf->rbf_tail;

#ifdef UART_DMA_TXCHANNEL
    // TODO: Implement DMA support
#endif

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Process pending software flow controls first.
     */
    if (flow_control & (XON_PENDING | XOFF_PENDING)) {
        if (flow_control & XON_PENDING) {
            /* Send XOFF */
            USARTn->THR = ASCII_XOFF;
            flow_control |= XOFF_SENT;
        } else {
            /* Send XON */
            USARTn->THR = ASCII_XON;
            flow_control &= ~XOFF_SENT;
        }
        flow_control &= ~(XON_PENDING | XOFF_PENDING);
        return;
    }

    if (flow_control & XOFF_RCVD) {
        /*
         * If XOFF has been received, we disable the transmit interrupts
         * and return without sending anything.
         */

        /* Disable transmitter and transmitter interrupt */
        USARTn->TER = 0;
        CM3BBCLR(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_THREINT_EN_POS);

        return;
    }
#endif

    /*
     * Check if we have more bytes to transmit anf if there is still space in the TX FIFO
     */
    while ((rbf->rbf_cnt) && (USARTn->LSR & UART_LSR_THRE)) {
        /*
         * If CTS has been disabled, we disable the transmit interrupts,
         * enable CTS interrupts and return without sending anything.
         */
        // TODO: CTS handling in here

        /* Start transmission of the next character. */
        USARTn->THR = *cp;

        /* Decrement the number of available bytes in the buffer. */
        rbf->rbf_cnt--;

        /* Wrap around the buffer pointer if we reached its end. */
        if (++cp == rbf->rbf_last) {
            cp = rbf->rbf_start;
        }
        rbf->rbf_tail = cp;

        /* Send an event if we reached the low watermark. */
        if (rbf->rbf_cnt == rbf->rbf_lwm) {
            NutEventPostFromIrq(&rbf->rbf_que);
            NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
        }
    }

    if ( rbf->rbf_cnt==0) {
        /* Nothing left to transmit: Disable transmit interrupts. */
        CM3BBCLR(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_THREINT_EN_POS);
        NutEventPostFromIrq(&rbf->rbf_que);
        NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
    }
}


/*
 * \brief USARTn receiver ready interrupt handler.
 *
 *
 * \param rbf Pointer to the receiver ring buffer.
 */
static void Lpc17xxUsartRxReady(RINGBUF * rbf, uint32_t lsr)
{
    register size_t cnt;
    register uint8_t ch;

#ifdef UART_DMA_RXCHANNEL
    // TODO: Implement DMA support
#endif

    while (lsr & UART_LSR_RDR) {

        /*
         * We read the received character as early as possible to avoid overflows
         * caused by interrupt latency.
         */

        ch = USARTn->RBR & UART_RBR_MASK;

        /* Collect receiver errors. */
        rx_errors |= lsr & (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI);

#if defined(UART_XONXOFF_CONTROL)
        /*
         * Handle software handshake. We have to do this before checking the
         * buffer, because flow control must work in write-only mode, where
         * there is no receive buffer.
         */
        if (flow_control) {
            /* XOFF character disables transmit interrupts. */
            if (ch == ASCII_XOFF) {
                /* Disable transmitter and transmitter interrupt */
                USARTn->TER = 0;

                CM3BBCLR(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_THREINT_EN_POS);

                flow_control |= XOFF_RCVD;
                return;
            }
            /* XON enables transmit interrupts. */
            else if (ch == ASCII_XON) {
                /* Disable transmitter and transmitter interrupt */
                USARTn->TER = UART_TER_TXEN;
                CM3BBSET(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_THREINT_EN_POS);

                flow_control &= ~XOFF_RCVD;
                return;
            }
        }
#endif

        /*
         * Check buffer overflow.
         */
        cnt = rbf->rbf_cnt;
        if (cnt >= rbf->rbf_siz) {
            // TODO: We use the same flag like when we got a FIFO overrun error
            rx_errors |= UART_LSR_OE;
            return;
        }

        /* Wake up waiting threads if this is the first byte in the buffer. */
        if (cnt++ == 0) {
            NutEventPostFromIrq(&rbf->rbf_que);
            NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_READ);
        }

#if defined(UART_XONXOFF_CONTROL)
        /*
         * Check the high watermark for software handshake. If the number of
         * buffered bytes is equal or above this mark, then send XOFF.
         */
        else if (flow_control) {
            if(cnt >= rbf->rbf_hwm) {
                if((flow_control & XOFF_SENT) == 0) {
                    /* Check it TX on and space in the FIFO, then send xoff */
                    if ((CM3BBGET(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_THREINT_EN_POS)) &&
                        (lsr & UART_LSR_THRE)) {
                        USARTn->THR = ASCII_XOFF;
                        flow_control |= XOFF_SENT;
                        flow_control &= ~XOFF_PENDING;
                    } else {
                        flow_control |= XOFF_PENDING;
                    }
                }
            }
        }
#endif

#ifdef US_MODE_HWHANDSHAKE
        /*
         * Check the high watermark for hardware handshake. If the number of
         * buffered bytes is above this mark, then disable RTS.
         */
        else if (rts_control && cnt >= rbf->rbf_hwm) {
    // TODO: Disable RTS
    //        USARTn->CR3 |= USART_CR3_RTSE;
        }
#endif

        /*
         * Store the character and increment and the ring buffer pointer.
         */
        *rbf->rbf_head++ = ch;
        if (rbf->rbf_head == rbf->rbf_last) {
            rbf->rbf_head = rbf->rbf_start;
        }

        /* Update the ring buffer counter. */
        rbf->rbf_cnt = cnt;

        /* update the LSR shadow variable */
        lsr = USARTn->LSR;
    }
}

/*!
 * \brief USART interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Lpc17xxUsartInterrupt(void *arg)
{
    USARTDCB *dcb = (USARTDCB *) arg;

    /* Read line status and interrupt identification register */
    uint32_t iir   = USARTn->IIR & UART_IIR_INTID_MASK;
    uint32_t intid = iir & UART_IIR_INTID_MASK;
    uint32_t lsr   = USARTn->LSR;

    if (intid == UART_IIR_INTID_RLS) {
        // TODO: Implement Line Status
        //        Lpc17xxUsartRxLineStatus(&dcb->dcb_rx_rbf);
    } else
    /* Test for byte received or character timeout */
    if ((intid == UART_IIR_INTID_RDA) || (intid == UART_IIR_INTID_CTI)) {
        Lpc17xxUsartRxReady(&dcb->dcb_rx_rbf, lsr);
    } else
    /* Test for next byte can be transmitted (transmit holding is empty) */
    if (intid == UART_IIR_INTID_THRE) {
        Lpc17xxUsartTxReady(&dcb->dcb_tx_rbf, lsr);
    }
}

/*!
 * \brief Carefully enable USART hardware functions.
 *
 * Always enabale transmitter and receiver, even on read-only or
 * write-only mode. So we can support software flow control.
 */
static void Lpc17xxUsartEnable(void)
{

    /* Enable UART transmitter. The receiver can not be enabled seperately on
       the LPC architecture. We just could disable the RX FIFOs, is this a
       good idea?
     */

    USARTn->TER = UART_TER_TXEN;

    /* Enable Usart Interrupts */
    NutUartIrqEnable(&SigUSART);
}

/*!
 * \brief Carefully disable USART hardware functions.
 *
 * TODO: Beschreibung anpassen
 * This routine is called before changing cruical settings like
 * baudrate, frame format etc.
 *
 * The previous version uses a 10ms delay to make sure, that any
 * incoming or outgoing character is processed. However, this time
 * depends on the baudrate.
 *
 * In fact we do not need to take care of incoming characters,
 * when changing such settings.
 *
 * For outgoing characters however, settings may be changed on
 * the fly and we should wait, until the last character transmitted
 * with the old settings has left the shift register. While TXRDY
 * is set when the holding register is empty, TXEMPTY is set when the
 * shift register is empty. The bad news is, that both are zero, if
 * the transmitter is disabled. We are not able to determine this
 * state. So we check TXRDY first and, if set, wait for any character
 * currently in the shift register.
 */
static void Lpc17xxUsartDisable(void)
{
    /* Disable Usart Interrupts*/
    NutUartIrqDisable(&SigUSART);

    /* If the transmitter is enabled, wait until all bits had been shifted out. */
    if (USARTn->TER & UART_TER_TXEN) {
        while ((USARTn->LSR & UART_LSR_TEMT) == 0);
    }

    /* Disable Transmitter, receiver can not be disabled on the LPC
       architecture. We just could disable the RX FIFOs, is this a good idea?
     */
    USARTn->TER = 0;
}

/*!
 * \brief Query the USART hardware for the selected speed.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The currently selected baudrate.
 */
static uint32_t Lpc17xxUsartGetSpeed(void)
{
    uint32_t clk = 0;
    // TODO: Implement!
    return clk;
}

/*!
 * \brief Set the USART hardware bit rate.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param rate Number of bits per second.
 *
 * \return 0 on success, -1 otherwise.
 */

static int Lpc17xxUsartSetSpeed(uint32_t baudrate)
{
    uint32_t uart_clock;
    uint32_t calcBaudrate = 0;
    uint32_t temp = 0;

    uint32_t mulFracDiv, dividerAddFracDiv;
    uint32_t diviser = 0 ;
    uint32_t bestm = 1;
    uint32_t bestd = 0;
    uint32_t best_divisor = 0;

    uint32_t relativeError = 0;
    uint32_t best_error = 100000;

    /* get UART block clock */

    uart_clock = NutArchClockGet(NUT_HWCLK_PCLK);

#if defined(MCU_LPC176x)

    if ((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART0);
    } else
    if ((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART1);
    } else
    if ((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART2);
    } else
    if ((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART3);
    }

#endif

    uart_clock = uart_clock >> 4; /* div by 16 */

    /* Baudrate calculation is done according the following formula:
       BaudRate= uart_clock * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)

       To avoid floating point calculation the formulae is adjusted with the
       multiply and divide method.

       The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
       0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15
    */

    for (mulFracDiv = 1 ; mulFracDiv <= 15 ;mulFracDiv++)
    {
        for (dividerAddFracDiv = 0 ; dividerAddFracDiv <= 15 ;dividerAddFracDiv++)
        {
            temp = (mulFracDiv * uart_clock) / ((mulFracDiv + dividerAddFracDiv));

            diviser = temp / baudrate;
            if ((temp % baudrate) > (baudrate / 2))
                diviser++;

            if (diviser > 2 && diviser < 65536)
            {
                calcBaudrate = temp / diviser;

                if (calcBaudrate <= baudrate) {
                    relativeError = baudrate - calcBaudrate;
                } else {
                    relativeError = calcBaudrate - baudrate;
                }

                if ((relativeError < best_error))
                {
                    bestm = mulFracDiv ;
                    bestd = dividerAddFracDiv;
                    best_divisor = diviser;
                    best_error = relativeError;
                    if (relativeError == 0) {
                        break;
                    }
                }
            }
        }
        if (relativeError == 0) {
            break;
        }
    }

    Lpc17xxUsartDisable();

    if (best_error < ((baudrate * UART_ACCEPTED_BAUDRATE_ERROR) / 100)) {
        /* Set DLAB bit */
        CM3BBSET(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_DLAB_EN_POS);

        USARTn->DLM  = UART_LOAD_DLM(best_divisor);
        USARTn->DLL  = UART_LOAD_DLL(best_divisor);

        /* Reset DLAB bit */
        CM3BBCLR(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_DLAB_EN_POS);
        USARTn->FDR  = (UART_FDR_MULVAL(bestm) | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
    } else {
        return -1;
    }

    Lpc17xxUsartEnable();

    return 0;
}

/*!
 * \brief Query the USART hardware for the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of data bits set.
 */
static uint8_t Lpc17xxUsartGetDataBits(void)
{
    uint32_t lcr = USARTn->LCR;

    switch(lcr & UART_LCR_WLEN_BITMASK) {
        case UART_LCR_WLEN5: return 5;
        case UART_LCR_WLEN6: return 6;
        case UART_LCR_WLEN7: return 7;
        case UART_LCR_WLEN8: return 8;
    }

    return 0;
}

/*!
 * \brief Set the USART hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartSetDataBits(uint8_t bits)
{
    int rc = 0;
    uint32_t lcr;

    Lpc17xxUsartDisable();

    lcr = USARTn->LCR & ~UART_LCR_WLEN_BITMASK;
    switch (bits)
    {
        case 5:
            lcr |= UART_LCR_WLEN5;
            break;

        case 6:
            lcr |= UART_LCR_WLEN6;
            break;

        case 7:
            lcr |= UART_LCR_WLEN7;
            break;

        case 8:
            lcr |= UART_LCR_WLEN8;
            break;

        default:
            Lpc17xxUsartEnable();
            return -1;
    }
    USARTn->LCR = lcr & UART_LCR_BITMASK;

    Lpc17xxUsartEnable();

    return rc;
}

/*!
 * \brief Query the USART hardware for the parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return Parity mode, either 0 (disabled), 1 (odd), 2 (even), 3 (mark) or 4 (space)
 */
static uint8_t Lpc17xxUsartGetParity(void)
{
    uint32_t lcr = USARTn->LCR;

    switch(lcr & UART_LCR_PARITY_BITMASK) {
        case UART_LCR_PARITY_EN | UART_LCR_PARITY_ODD:  return 1;
        case UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN: return 2;
        case UART_LCR_PARITY_EN | UART_LCR_PARITY_F_1:  return 3;
        case UART_LCR_PARITY_EN | UART_LCR_PARITY_F_0:  return 4;
        default:
            return 0;
    }

    return 0;
}

/*!
 * \brief Set the USART hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (disabled), 1 (odd), 2 (even) 3 (mark) or 4(space)
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartSetParity(uint8_t mode)
{
    uint32_t lcr;

    Lpc17xxUsartDisable();

    lcr = USARTn->LCR & ~UART_LCR_PARITY_BITMASK;

    switch (mode) {
        case 0:
            /* Parity disabled, do nothing */
            break;
        case 1:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_ODD;
            break;
        case 2:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN;
            break;
        case 3:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_F_1;
            break;
        case 4:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_F_0;
            break;
        default:
            Lpc17xxUsartEnable();
            return -1;
    }

    USARTn->LCR = lcr & UART_LCR_BITMASK;

    Lpc17xxUsartEnable();

    return 0;
}

/*!
 * \brief Query the USART hardware for the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of stop bits set, either 1, 2 or 3 (1.5 bits).
 */
static uint8_t Lpc17xxUsartGetStopBits(void)
{
    uint32_t lcr = USARTn->LCR;

    if (lcr & UART_LCR_STOPBIT_SEL) {
        if ((lcr & UART_LCR_WLEN_BITMASK) == UART_LCR_WLEN5) {
            return 3;
        } else {
            return 2;
        }
    } else {
        return 1;
    }
}

/*!
 * \brief Set the USART hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param bits The number of stop bits set, either 1, 2 or 3 (1.5 bits).
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartSetStopBits(uint8_t bits)
{
    Lpc17xxUsartDisable();

    switch (bits) {
        case 1:
            CM3BBCLR(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_STOPBIT_SEL_POS);
            break;
        case 2:
            CM3BBSET(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_STOPBIT_SEL_POS);
            break;
        case 3:
            if ((USARTn->LCR & UART_LCR_WLEN_BITMASK) == UART_LCR_WLEN5) {
                CM3BBSET(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_STOPBIT_SEL_POS);
            } else {
                Lpc17xxUsartEnable();
                return -1;
            }
            break;
        default:
            Lpc17xxUsartEnable();
            return -1;
    }

    Lpc17xxUsartEnable();

    return 0;
}

/*!
 * \brief Query the USART hardware status.
 *
 * \return Status flags.
 */
static uint32_t Lpc17xxUsartGetStatus(void)
{
    uint32_t rc = 0;

    /*
     * Set receiver error and status flags.
     */
    if ((rx_errors & UART_LSR_FE) != 0) {
        rc |= UART_FRAMINGERROR;
    }
    if ((rx_errors & UART_LSR_OE) != 0) {
        rc |= UART_OVERRUNERROR;
    }
    if ((rx_errors & UART_LSR_PE) != 0) {
        rc |= UART_PARITYERROR;
    }
    if ((rx_errors & UART_LSR_BI) != 0) {
        rc |= UART_BREAKCONDITION;
    }

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Determine software handshake status. The flow control status may
     * change during interrupt, but this doesn't really hurt us.
     */
    if (flow_control) {
        if (flow_control & XOFF_SENT) {
            rc |= UART_RXDISABLED;
        }
        if (flow_control & XOFF_RCVD) {
            rc |= UART_TXDISABLED;
        }
    }
#endif

    /*
     * Determine hardware handshake control status.
     */
#if defined(US_MODE_HWHANDSHAKE)
    // TODO: reflect RTS state
#endif

    /*
     * Determine hardware handshake sense status.
     */
#if defined(US_MODE_HWHANDSHAKE)
    // TODO: reflect CTS state
#endif

    /*
     * If transmitter and receiver haven't been detected disabled by any
     * of the checks above, then they are probably enabled.
     */
    if ((rc & UART_RXDISABLED) == 0) {
        rc |= UART_RXENABLED;
    }
    if ((rc & UART_TXDISABLED) == 0) {
        rc |= UART_TXENABLED;
    }

    return rc;
}

/*!
 * \brief Set the USART hardware status.
 *
 * \param flags Status flags.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartSetStatus(uint32_t flags)
{
    int rc = 0;
    /*
     * Process software handshake control.
     */

#if defined(UART_XONXOFF_CONTROL)
    if (flow_control) {

        /* Access to the flow control status must be atomic. */
        NutUartIrqDisable(&SigUSART);

        /*
         * Enabling or disabling the receiver means to behave like
         * having sent a XON or XOFF character resp.
         */
        if (flags & UART_RXENABLED) {
            flow_control &= ~XOFF_SENT;
        } else if (flags & UART_RXDISABLED) {
            flow_control |= XOFF_SENT;
        }

        /*
         * Enabling or disabling the transmitter means to behave like
         * having received a XON or XOFF character resp.
         */
        if (flags & UART_TXENABLED) {
            flow_control &= ~XOFF_RCVD;
        } else if (flags & UART_TXDISABLED) {
            flow_control |= XOFF_RCVD;
        }
        NutUartIrqEnable(&SigUSART);
    }
#endif

    /*
     * Clear USART receive errors.
     */
    if (flags & UART_ERRORS) {
        /* Clear errors by reading the line status register */
        (volatile uint32_t)USARTn->LSR;
        rx_errors = 0;
    }

    /*
     * Verify the result.
     */
    if ((Lpc17xxUsartGetStatus() & ~UART_ERRORS) != flags) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for synchronous mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * Not implemented for the LPC17xx USART. Always returns 0.
 *
 * \return Or-ed combination of \ref UART_SYNC, \ref UART_MASTER,
 *         \ref UART_NCLOCK and \ref UART_HIGHSPEED.
 */
uint8_t Lpc17xxUsartGetClockMode(void)
{
    return 0;
}

/*!
 * \brief Set asynchronous or synchronous mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * Not implemented for the LPC17xx USART. Always returns -1.
 *
 * \param mode Must be an or-ed combination of USART_SYNC, USART_MASTER,
 *             USART_NCLOCK and USART_HIGHSPEED.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartSetClockMode(uint8_t mode)
{
    return -1;
}

/*!
 * \brief Query flow control mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return See UsartIOCtl().
 */
static uint32_t Lpc17xxUsartGetFlowControl(void)
{
    uint32_t rc = 0;
// TODO: Implement more flow control
#if defined(UART_XONXOFF_CONTROL)
    if (flow_control) {
        rc |= USART_MF_XONXOFF;
    } else {
        rc &= ~USART_MF_XONXOFF;
    }
#endif

#ifdef UART_DMA_RXCHANNEL
    if (block_control & BC_RD_EN)
        rc |= USART_MF_BLOCKREAD;
    else
        rc &= ~USART_MF_BLOCKREAD;
#endif

#ifdef UART_DMA_TXCHANNEL
    if (block_control & BC_WR_EN)
        rc |= USART_MF_BLOCKWRITE;
    else
        rc &= ~USART_MF_BLOCKWRITE;
#endif

    return rc;
}

/*!
 * \brief Set flow control mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param flags See UsartIOCtl().
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartSetFlowControl(uint32_t flags)
{
// TODO: Implement more flow control
#if defined(UART_XONXOFF_CONTROL)
    NutUartIrqDisable(&SigUSART);
    /*
     * Set software handshake mode.
     */
    if (flags & USART_MF_XONXOFF) {
        if(flow_control == 0) {
            flow_control = 1 | XOFF_SENT;  /* force XON to be sent on next read */
        }
    } else {
        flow_control = 0;
    }
    NutUartIrqEnable(&SigUSART);
#endif

#ifdef UART_DMA_RXCHANNEL
    /* Setup block mode */
    if( flags & USART_MF_BLOCKREAD) {
        block_control |= BC_RD_EN;
    }
    else {
        block_control &= ~BC_RD_EN;
    }
#endif

#ifdef UART_DMA_TXCHANNEL
    if( flags & USART_MF_BLOCKWRITE) {
        block_control |= BC_WR_EN;
    }
    else {
        block_control &= ~BC_WR_EN;
    }
#endif

    return 0;
}

/*!
 * \brief Start the USART transmitter hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it added one or more bytes to the
 * transmit buffer.
 */
static void Lpc17xxUsartTxStart(void)
{
    register uint8_t *cp;
    /* Enable transmit interrupts. */
    CM3BBSET(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_THREINT_EN_POS);
    USARTn->TER = UART_TER_TXEN;

    /*
     * Check if we have any bytes to transmit. Send out first byte if there is space in the FIFO
     */
    NutEnterCritical();
    cp = DcbUSART.dcb_tx_rbf.rbf_tail;
    if ((DcbUSART.dcb_tx_rbf.rbf_cnt) && (USARTn->LSR & UART_LSR_THRE)) {
        /*
         * If CTS has been disabled, we disable the transmit interrupts,
         * enable CTS interrupts and return without sending anything.
         */
        // TODO: CTS handling in here

        /* Start transmission of the next character. */
        USARTn->THR = *cp;

        /* Decrement the number of available bytes in the buffer. */
        DcbUSART.dcb_tx_rbf.rbf_cnt--;

        /* Wrap around the buffer pointer if we reached its end. */
        if (++cp == DcbUSART.dcb_tx_rbf.rbf_last) {
            cp = DcbUSART.dcb_tx_rbf.rbf_start;
        }
        DcbUSART.dcb_tx_rbf.rbf_tail = cp;

        /* Send an event if we reached the low watermark. */
        if (DcbUSART.dcb_tx_rbf.rbf_cnt == DcbUSART.dcb_tx_rbf.rbf_lwm) {
            NutExitCritical(); /* Exit critical section _before_ posting to the event queues */
            NutEventPostAsync(&DcbUSART.dcb_tx_rbf.rbf_que);
            NutSelectWakeup(DcbUSART.dcb_tx_rbf.wq_list, WQ_FLAG_WRITE);
        } else {
            NutExitCritical();
        }
    }
}

/*!
 * \brief Start the USART receiver hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it removed enough bytes from the
 * receive buffer. Enough means, that the number of bytes left in
 * the buffer is below the low watermark.
 */
static void Lpc17xxUsartRxStart(void)
{
#if defined(UART_XONXOFF_CONTROL)
    /*
     * Do any required software flow control.
     */
    if (flow_control && (flow_control & XOFF_SENT) != 0) {
        NutUartIrqDisable(&SigUSART);
        /* Check if the transmit holding register is empty */
        if (USARTn->LSR & UART_LSR_THRE) {
            USARTn->THR = ASCII_XON;
            flow_control &= ~XON_PENDING;
        } else {
            flow_control |= XON_PENDING;
        }
        flow_control &= ~(XOFF_SENT | XOFF_PENDING);
        NutUartIrqEnable(&SigUSART);
    }
#endif
    /* Enable receive interrupts. */
    CM3BBSET(USARTnBase, LPC_UART_TypeDef, IER, UART_IER_RBRINT_EN_POS);
}

/*
 * \brief Initialize the USART hardware driver.
 *
 * This function is called during device registration by the upper level
 * USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartInit(void)
{
    volatile uint32_t tmp;

    /*
     * Register receive and transmit interrupts.
     */
    if (NutRegisterIrqHandler(&SigUSART, Lpc17xxUsartInterrupt, &DcbUSART)) {
        return -1;
    }

    /* Enable UART clock and power */
#if defined(MCU_LPC176x)
    if((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART0);
    } else
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART1);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART2);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART3);
    }
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    if((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART0);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART0);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART0);
    } else
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART1);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART1);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART1);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART2);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART2);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART2);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART3);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART3);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART3);
    }
#endif
    /* Disable IRQs */
    USARTn->IER = 0;

    /* Clear FIFOs */
    USARTn->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV2;

    /* Dummy reading */
    while (USARTn->LSR & UART_LSR_RDR) {
        tmp = USARTn->RBR;
    }

    /* Enable transmitter */
    USARTn->TER = UART_TER_TXEN;

    /* Wait for current transmit complete */
    while (!(USARTn->LSR & UART_LSR_THRE));

    /* Disable transmitter */
    USARTn->TER = 0;

    /* Set LCR to default state */
    USARTn->LCR = 0;

    /* Set ACR to default state */
    USARTn->ACR = 0;

#if defined(MCU_LPC176x)
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        /* Set RS485 control to default state */
        ((LPC_UART1_TypeDef*)USARTn)->RS485CTRL = 0;

        /* Set RS485 delay timer to default state */
        ((LPC_UART1_TypeDef*)USARTn)->RS485DLY = 0;

        /* Set RS485 addr match to default state */
        ((LPC_UART1_TypeDef*)USARTn)->ADRMATCH = 0;
    }
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    /* Set RS485 control to default state */
    USARTn->RS485CTRL = 0;

    /* Set RS485 delay timer to default state */
    USARTn->RS485DLY = 0;

    /* Set RS485 addr match to default state */
    USARTn->ADRMATCH = 0;
#endif

    /* Dummy reading to clear bits */
    tmp = USARTn->LSR;

    if(((LPC_UART1_TypeDef *)USARTn) == LPC_UART1) {
        /* Set Modem Control to default state */
        ((LPC_UART1_TypeDef *)USARTn)->MCR = 0;

        /* Dummy Reading to Clear Status */
        tmp = ((LPC_UART1_TypeDef *)USARTn)->MSR;
    }
#if defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    if(((LPC_UART4_TypeDef *)USARTn) == LPC_UART4) {
        /* Set IrDA to default state for all UART other than UART1 */
        ((LPC_UART4_TypeDef *)USARTn)->ICR = 0;
    }
#endif
    /* Configure USART Tx as alternate function push-pull */
    GpioPinConfigSet( TX_GPIO_PORT, TX_GPIO_PIN, TX_GPIO_PIN_CFG);
    /* Configure USART Rx as input floating */
    GpioPinConfigSet( RX_GPIO_PORT, RX_GPIO_PIN, RX_GPIO_PIN_CFG);

#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
    /* Configure USART RTS as alternate function push-pull */
    GpioPinConfigSet( RTS_GPIO_PORT, RTS_GPIO_PIN, RTS_GPIO_PIN_CFG);
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
    /* Configure USART CTS as input floating */
    GpioPinConfigSet( CTS_GPIO_PORT, CTS_GPIO_PIN, CTS_GPIO_PIN_CFG);
#endif


    /* Configure UART communication parameters */

    Lpc17xxUsartSetSpeed(USART_INIT_BAUTRATE);
    Lpc17xxUsartSetDataBits(8);
    Lpc17xxUsartSetStopBits(1);
    Lpc17xxUsartSetParity(0);

    /* Enable additional features */
#ifdef USART_HWFLOWCTRL

    // TODO: Implement GPIO based hardware handshake for UARTS without full modem handshake */
    if ((LPC_UART1_TypeDef *) USARTn == LPC_UART1) {
        /* Enable hardware handshake options */
        tmp = 0;

#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
        tmp |= UART1_MCR_AUTO_RTS_EN;
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
        tmp |= UART1_MCR_AUTO_CTS_EN;
#endif
        ((LPC_UART1_TypeDef *)USARTn)->MCR = tmp;
    }
#endif

#ifdef USART_MODE_IRDA
    // TODO: Further IRDA feature configuration
    if(((LPC_UART4_TypeDef *)USARTn) == LPC_UART4) {
        /* Set IrDA to default state for all UART other than UART1 */
        USARTn->ICR = UART_ICR_IRDAEN;
    }
#endif

    NutIrqEnable(&SigUSART);
    return 0;
}

/*
 * \brief Deinitialize the USART hardware driver.
 *
 * This function is called during device deregistration by the upper
 * level USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxUsartDeinit(void)
{
    /* Disable IRQs */
    USARTn->IER = 0;

    /* Disable interrupts */
    NutIrqDisable(&SigUSART);

    /* Deregister receive and transmit interrupts. */
    NutRegisterIrqHandler(&SigUSART, 0, 0);

    /* Disable UART clock and power */

#if defined(MCU_LPC176x)
    if((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART0);
    } else
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART1);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART2);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART3);
    }
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    if((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART0);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART0);

        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART0);
    } else
    if((LPC_UART1_TypeDef *)USARTn == LPC_UART1) {
        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART1);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART1);

        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART1);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART2);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART2);

        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART2);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART3);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART3);

        SysCtlPeripheralClkDisable(CLKPWR_PCONP_PCUART3);
    }
#endif

    return 0;
}

/*@}*/
