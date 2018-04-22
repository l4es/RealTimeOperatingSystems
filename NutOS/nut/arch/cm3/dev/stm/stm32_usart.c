/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 * $Id: stm32_usart.c 6678 2017-10-08 13:43:22Z u_bonnes $
 * \endverbatim
 */

#include <cfg/clock.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/usart.h>
#include <arch/cm3/stm/stm32_dma.h>
#include <arch/cm3/stm/stm32_usart.h>

#if defined(USART_RDR_RDR)
#define USARTN_RDR (USARTn->RDR)
#define USARTN_TDR (USARTn->TDR)
#define USARTN_ISR (USARTn->ISR)
#if defined(USART_ICR_LBDCF)
# if defined(USART_ICR_WUCF)
#  define CLEAR_ERRS() USARTn->ICR = USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NCF|USART_ICR_ORECF| \
        USART_ICR_IDLECF|USART_ICR_TCCF|USART_ICR_LBDCF|USART_ICR_CTSCF|USART_ICR_RTOCF| \
        USART_ICR_EOBCF|USART_ICR_CMCF|USART_ICR_WUCF
# else
#  define CLEAR_ERRS() USARTn->ICR = USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NCF|USART_ICR_ORECF| \
        USART_ICR_IDLECF|USART_ICR_TCCF|USART_ICR_LBDCF|USART_ICR_CTSCF|USART_ICR_RTOCF| \
        USART_ICR_EOBCF|USART_ICR_CMCF
# endif
#else
/* STM32F030 is missing some UART functionality and so some defined */
# define CLEAR_ERRS() USARTn->ICR = USART_ICR_PECF | USART_ICR_FECF |\
        USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_IDLECF | USART_ICR_TCCF |\
        USART_ICR_CTSCF|USART_ICR_RTOCF|USART_ICR_CMCF
#endif
#define TXE_SET CM3BBGET(USARTnBase, USART_TypeDef, ISR, _BI32(USART_ISR_TXE))
#define CLEAR_TC() CM3BBSET(USARTnBase, USART_TypeDef, ICR, _BI32(USART_ICR_TCCF))
#else
#define USARTN_RDR (USARTn->DR)
#define USARTN_TDR (USARTn->DR)
#define USARTN_ISR (USARTn->SR)
#define CLEAR_ERRS() (USARTn->SR)
#define TXE_SET CM3BBGET(USARTnBase, USART_TypeDef, SR, _BI32(USART_SR_TXE))
#define CLEAR_TC() CM3BBCLR(USARTnBase, USART_TypeDef, SR, _BI32(USART_SR_TC))
#endif
#if !defined(USART_ISR_ORE)
#define USART_ISR_ORE USART_SR_ORE
#endif
#if !defined(USART_ISR_NE)
#define USART_ISR_NE USART_SR_NE
#endif
#if !defined(USART_ISR_FE)
#define USART_ISR_FE USART_SR_FE
#endif
#if !defined(USART_ISR_PE)
#define USART_ISR_PE USART_SR_PE
#endif
#if !defined(USART_ISR_TC)
#define USART_ISR_TC USART_SR_TC
#endif
#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE USART_SR_RXNE
#endif
#if !defined(USART_ISR_TXE)
#define USART_ISR_TXE USART_SR_TXE
#endif
#if !defined(USART_CR1_OVER8)
# define USART_CR1_OVER8 0
#endif
#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
#include <arch/cm3/stm/stm32_dma.h>
#endif

/* Some function renaming for IRQ handling on uarts
 * that do not have a separate IRQ instance but a
 * merged one with others like DBU on SYS-IRQ or
 * external uarts on GPIO-IRQs
 */
#define NutUartIrqEnable()  USARTn->CR1 |= (USART_CR1_TXEIE | USART_CR1_RXNEIE)
#define NutUartIrqDisable() USARTn->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_RXNEIE)

/*!
 * \addtogroup xgNutArchArmStm32Us
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

/*!
 * \brief Receiver error flags.
 */
static uint16_t rx_errors;

/*!
 * \brief Transmit address frame, if not zero.
 */
static uint_fast8_t tx_aframe;

#if defined(US_MODE_HWHANDSHAKE)
/*!
 * \brief Enables RTS control if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to control the RTS signal.
 */
static uint_fast8_t rts_control;

/*!
 * \brief Enables CTS sense if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to sense the CTS signal.
 */
// TODO: CTS Handling
// static uint_fast8_t cts_sense;
#endif

#if defined(UART_XONXOFF_CONTROL)
/*!
 * \brief Handles local software flow control.
 *
 * This variable exists only if the hardware configuration defines a
 * software flow control support.
 */
static uint_fast8_t flow_control;
#endif

/*!
 * \brief Handles local half-duplex flow control.
 *
 */
static uint_fast8_t hdpx_control;

#ifdef USART_485_CTRL
/*!
 * \brief Handles local echo for RS485.
 *
 */
static uint_fast8_t rs485_control;
#endif /* #ifdef USART_485_CTRL */


#if defined(DE_GPIO_PORT) && defined(DE_GPIO_PIN)
/*!
 * \brief Definition of RS485 DE switching functions.
 */
#ifdef USART_4485DE_INV
#define Rs485DE_L() GpioPinSetHigh(DE_GPIO_PORT, DE_GPIO_PIN)
#define Rs485DE_H() GpioPinSetLow(DE_GPIO_PORT, DE_GPIO_PIN)
#else
#define Rs485DE_L() GpioPinSetLow(DE_GPIO_PORT, DE_GPIO_PIN)
#define Rs485DE_H() GpioPinSetHigh(DE_GPIO_PORT, DE_GPIO_PIN)
#endif
#else
#define Rs485DE_L()
#define Rs485DE_H()
#endif

#if defined(NRE_GPIO_PORT) && defined(NRE_GPIO_PIN)
/*!
 * \brief Definition of RS485 NRE switching functions.
 */
// TODO: Figure out fastest way for switching signals
//#define Rs485NRE_L() NRE_GPIO_BASE->BRR=_BV(NRE_GPIO_PIN)
//#define Rs485NRE_H() NRE_GPIO_BASE->BSRR=_BV(NRE_GPIO_PIN)
#ifdef USART_4485RE_INV
#define Rs485NRE_L() GpioPinSetHigh(NRE_GPIO_PORT, NRE_GPIO_PIN)
#define Rs485NRE_H() if( rs485_control == 0) \
    GpioPinSetLow(NRE_GPIO_PORT, NRE_GPIO_PIN)
#else
#define Rs485NRE_L() GpioPinSetLow(NRE_GPIO_PORT, NRE_GPIO_PIN)
#define Rs485NRE_H() if( rs485_control == 0) \
    GpioPinSetHigh(NRE_GPIO_PORT, NRE_GPIO_PIN)
#endif
#else
#define Rs485NRE_L()
#define Rs485NRE_H()
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

#if 0
#ifdef UART_DMA_TXCHANNEL
static void Stm32UsartDmaTxIrq(void* arg)
{
    NutEventPost(arg);
}
#endif

#ifdef UART_DMA_RXCHANNEL
static void Stm32UsartDmaRxIrq(void* arg)
{
    NutEventPost(arg);
}
#endif
#endif

IRQ_HANDLER *usart_sig;

/*
 * \brief USARTn transmitter ready interrupt handler.
 *
 * \param rbf Pointer to the transmitter ring buffer.
 */
static void Stm32UsartTxReady(RINGBUF * rbf)
{
    register uint8_t *cp = rbf->rbf_tail;

#ifdef UART_DMA_TXCHANNEL
    if (block_control & BC_WR_EN) {
        /* Disable TX-Interrupt */
        CM3BBCLR(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE));

        /* Setup Transfer */
        DMA_Setup(UART_DMA_TXCHANNEL, UART_DR_PTR, rbf->rbf_blockptr, rbf->rbf_blockcnt,
            DMA_MINC);

        /* Enable TxComplete Interrupt */
        CM3BBSET(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAT));
        CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TCIE));

        /* Get DMA running */
        DMA_Enable(UART_DMA_TXCHANNEL);

        /* Now wait for the DMA to finish */
        return;
    }
#endif

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Process pending software flow controls first.
     */
    if (flow_control & (XON_PENDING | XOFF_PENDING)) {
        if (flow_control & XON_PENDING) {
            USARTN_TDR_=ASCII_XOFF;
            flow_control |= XOFF_SENT;
        } else {
            USARTN_TDR=ASCII_XON;
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
        CM3BBCLR(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE));
        CLEAR_ERRS();
        return;
    }
#endif

    /*
     * Check if we have more bytes to transmit.
     */
    if (rbf->rbf_cnt) {
        /*
         * If CTS has been disabled, we disable the transmit interrupts,
         * enable CTS interrupts and return without sending anything.
         */
        // TODO: CTS handling in here

        /* Start transmission of the next character. */
        USARTN_TDR=*cp;

        /* Decrement the number of available bytes in the buffer. */
        rbf->rbf_cnt--;

        /* Wrap around the buffer pointer if we reached its end. */
        if (++cp == rbf->rbf_last) {
            cp = rbf->rbf_start;
        }
        rbf->rbf_tail = cp;

        /* Send an event if we reached the low watermark. */
        if (!hdpx_control && (rbf->rbf_cnt == rbf->rbf_lwm)) {
            NutEventPostFromIrq(&rbf->rbf_que);
            NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
        }
    }

    if ( rbf->rbf_cnt==0) {
        /*
         * Nothing left to transmit:
         */

        /* Enable transmit complete interrupt */
        CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TCIE));
        /* Disable transmit interrupts. */
        CM3BBCLR(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE));
        if( !hdpx_control) {
            /* if half-duplex post the waiting thread after all bits are out.
             * Otherwise he might read back his own echo */
            NutEventPostFromIrq(&rbf->rbf_que);
            NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
        }

    }
}


/*
 * \brief USARTn receiver ready interrupt handler.
 *
 *
 * \param rbf Pointer to the receiver ring buffer.
 */
static void Stm32UsartRxReady(RINGBUF * rbf)
{
    register size_t cnt;
    register uint8_t ch;

#ifdef UART_DMA_RXCHANNEL
    if (block_control & BC_RD_EN) {
        /* Disable TX-Interrupt */
        CM3BBCLR(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RXNEIE));

        /* Setup Transfer */
        DMA_Setup(UART_DMA_RXCHANNEL, rbf->rbf_blockptr, UART_DR_PTR, rbf->rbf_blockcnt,
            DMA_MINC);

        /* Enable TxComplete Interrupt */
        CM3BBSET(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAR));

        /* Get DMA running */
        DMA_Enable(UART_DMA_RXCHANNEL);

        /* Now wait for the DMA to finish */
        return;
    }
#endif

    /*
     * We read the received character as early as possible to avoid overflows
     * caused by interrupt latency.
     */
    ch = USARTN_RDR;

    /* Collect receiver errors. */
    rx_errors |= USARTN_ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE);

#if defined(UART_XONXOFF_CONTROL)
    /*
     * Handle software handshake. We have to do this before checking the
     * buffer, because flow control must work in write-only mode, where
     * there is no receive buffer.
     */
    if (flow_control) {
        /* XOFF character disables transmit interrupts. */
        if (ch == ASCII_XOFF) {
            CM3BBCLR(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TE));
            flow_control |= XOFF_RCVD;
            return;
        }
        /* XON enables transmit interrupts. */
        else if (ch == ASCII_XON) {
            CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TE));
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
        rx_errors |= USART_ISR_ORE;
        return;
    }

    /* Wake up waiting threads if this is the first byte in the buffer. */
    if (cnt++ == 0){
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
                if (CM3BBGET(USARTnBase, USART_TypeDef, SR, _BI32(USART_CR1_TE))) {
                    USARTN_TDR= ASCII_XOFF;
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
        CM3BBSET(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_RTSE));
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
}

/*
 * \brief USARTn transmitter empty interrupt handler.
 *
 * Used with half duplex communication to switch from tranmit to receive
 * mode after the last character has been transmitted.
 *
 * This routine exists only if the hardware configuration defines
 * RS485 control.
 *
 * \param arg Pointer to the transmitter ring buffer.
 */
static void Stm32UsartTxComplete(RINGBUF * rbf)
{
    /*
     * Check if half duplex mode has been enabled and if all characters
     * had been sent out.
     */
#ifdef UART_DMA_TXCHANNEL
    if (block_control & BC_WR_EN) {
        /* Clear TX and TX-Complete interrupt */
        USARTn->CR1 &= ~(USART_CR1_TCIE|USART_CR1_TXEIE);
        /* Disable DMA transfer */
        CM3BBCLR(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAT));

        if( hdpx_control) {
            /* In case of half-duplex, enable receiver again */
            CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RE));
        }

        /* Reset Counter */
        rbf->rbf_blockcnt = 0;
        rbf->rbf_cnt = 0;

        /* Switch to receiver mode. */
        Rs485DE_L();    /* Disable Sender */
        Rs485NRE_L();   /* Enable Receiver */
        NutEventPostFromIrq(&rbf->rbf_que);
        NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
        return;
    }
#endif

    if( rbf->rbf_cnt == 0) {
        /* Clear TX and TX-Complete interrupt */
        USARTn->CR1 &= ~(USART_CR1_TCIE|USART_CR1_TXEIE);

        /* Switch to receiver mode. */
        Rs485DE_L();    /* Disable Sender */
        Rs485NRE_L();   /* Enable Receiver */

        /* Clear pending TX-Complete Status */
        CLEAR_TC();

        if( hdpx_control) {
            /* In case of half-duplex, enable receiver again */
            CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RE));
        }
        /* Send an event to inform the upper level. */
        NutEventPostFromIrq(&rbf->rbf_que);
        NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
    }
}

/*!
 * \brief USART interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Stm32UsartInterrupt(void *arg)
{
    USARTDCB *dcb = (USARTDCB *) arg;
    uint32_t csr = USARTN_ISR;

    /* Test for byte received */
    if (csr & USART_ISR_RXNE) {
        Stm32UsartRxReady(&dcb->dcb_rx_rbf);
    }
    /* Test for next byte can be transmitted
     * At end of DMA_TX Transfer, both TC and TXE are set.
     * Do not reinitialize transfer in that case!
     */
    if ((csr & USART_ISR_TXE) && !CM3BBGET(USARTnBase, USART_TypeDef, CR3, _BI32(USART_CR3_DMAT))) {
        Stm32UsartTxReady(&dcb->dcb_tx_rbf);
    }
    /* Last byte has been sent completely. */
    if (csr & USART_ISR_TC) {
        Stm32UsartTxComplete(&dcb->dcb_tx_rbf);
    }
}

/*!
 * \brief Carefully enable USART hardware functions.
 *
 * Always enabale transmitter and receiver, even on read-only or
 * write-only mode. So we can support software flow control.
 */
static void Stm32UsartEnable(void)
{
    /* Enable UART, receiver and transmitter. */
    USARTn->CR1 |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;

    /* Enable Usart Interrupts */
    NutUartIrqEnable();
}

/*!
 * \brief Carefully disable USART hardware functions.
 *
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
static void Stm32UsartDisable(void)
{
    /* Disable Usart Interrupts*/
    NutUartIrqDisable();
    /* Wait until all bits had been shifted out. */
    while(!TXE_SET);
    /* Disable USART. */
    USARTn->CR1 &= ~(USART_CR1_TE|USART_CR1_RE);
}

/*!
 * \brief Query the USART hardware for the selected speed.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The currently selected baudrate.
 */
static uint32_t Stm32UsartGetSpeed(void)
{
    uint32_t clk;
    uint32_t brr;

    clk = NutClockGet(USARTclk);
    brr = USARTn->BRR;
    if (USART_CR1_OVER8 && (USARTn->CR1 & USART_CR1_OVER8))
    {
        uint32_t frac;
        frac = brr & 7;
        brr >>= 1;
        brr &= ~0x7;
        brr |= frac;
    }
    return clk / brr;
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
static int Stm32UsartSetSpeed(uint32_t rate)
{
    uint32_t apbclock;
    uint32_t divider;
    uint32_t cr1;

    Stm32UsartDisable();

    apbclock = NutClockGet(USARTclk);

    divider = (2 * apbclock) / rate;
    cr1 = USARTn->CR1;

    if ((divider > 0x20) || (USART_CR1_OVER8 == 0)) {
        cr1 &= ~USART_CR1_OVER8;
        divider = divider >> 1;
    } else {
        cr1 |= USART_CR1_OVER8;
    }
    if (divider < 0x10) {
        divider = 0x10;
    }
    if (USART_CR1_OVER8 && (cr1 & USART_CR1_OVER8)) {
        uint32_t remainder;

        remainder = divider & 0xf;
        divider &= ~0xf;
        divider |= remainder >> 1;
    }
    USARTn->CR1 = cr1;
    /* Write to USART BRR */
    USARTn->BRR = divider;

    Stm32UsartEnable();
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
static uint8_t Stm32UsartGetDataBits(void)
{
    uint32_t val = USARTn->CR1 & USART_CR1_M;

    if (val) {
        val = 9;
    } else {
        val = 8;
    }
    return (uint8_t)val;
}

/*!
 * \brief Set the USART hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetDataBits(uint8_t bits)
{
    int rc = 0;
    uint32_t val = USARTn->CR1;

    switch( bits) {
        case 8:
            val &= ~USART_CR1_M;
            break;
        case 9:
            val |= USART_CR1_M;
            break;
        default:
            return -1;
    }
    Stm32UsartDisable();
    USARTn->CR1=val;
    Stm32UsartEnable();

    /*
     * Verify the result.
     */
    if (Stm32UsartGetDataBits() != bits) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for the parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return Parity mode, either 0 (disabled), 1 (odd), 2 (even) or 9 (multidrop).
 */
static uint8_t Stm32UsartGetParity(void)
{
    uint32_t val = USARTn->CR1 & (USART_CR1_PCE|USART_CR1_PS);

    if (val == (USART_CR1_PCE|USART_CR1_PS))
        val = 1;
    else if (val == USART_CR1_PCE)
        val = 2;
    else
        val = 0;

    return (uint8_t)val;
}

/*!
 * \brief Set the USART hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (disabled), 1 (odd) or 2 (even)
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetParity(uint8_t mode)
{
    int rc = 0;
    uint32_t val = USARTn->CR1 & ~(USART_CR1_PCE|USART_CR1_PS);

    switch (mode) {
        case 0:
            break;
        case 1:
            val |= USART_CR1_PCE|USART_CR1_PS;
            break;
        case 2:
            val |= USART_CR1_PCE;
            break;
        default:
            return -1;
            break;
    }
    Stm32UsartDisable();
    USARTn->CR1=val;
    Stm32UsartEnable();

    /*
     * Verify the result.
     */
    if (Stm32UsartGetParity() != mode) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of stop bits set, either 1, 2 or 3 (1.5 bits).
 */
static uint8_t Stm32UsartGetStopBits(void)
{
    uint32_t val = USARTn->CR2 & USART_CR2_STOP;

    if (val == 0) {
        val = 1;
    }
    else if (val == USART_CR2_STOP_1) {
        val = 2;
    }
    else if (val == USART_CR2_STOP){
        val = 3;
    }
    return (uint8_t)val;
}

/*!
 * \brief Set the USART hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetStopBits(uint8_t bits)
{
    int rc = 0;
    uint32_t val = USARTn->CR2 & ~USART_CR2_STOP;

    switch(bits) {
        case 1:
            break;
        case 2:
            val |= USART_CR2_STOP_1;
            break;
        case 3:
            val |= USART_CR2_STOP;
            break;
        default:
            return -1;
            break;
    }
    Stm32UsartDisable();
    USARTn->CR2 = val;
    Stm32UsartEnable();

    /*
     * Verify the result.
     */
    if (Stm32UsartGetStopBits() != bits) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware status.
 *
 * \return Status flags.
 */
static uint32_t Stm32UsartGetStatus(void)
{
    uint32_t rc = 0;
#if defined(US_MODE_HWHANDSHAKE)
    uint32_t csr = USARTN_ISR;
#endif

    /*
     * Set receiver error flags.
     */
    if ((rx_errors & USART_ISR_FE) != 0) {
        rc |= UART_FRAMINGERROR;
    }
    if ((rx_errors & USART_ISR_ORE) != 0) {
        rc |= UART_OVERRUNERROR;
    }
    if ((rx_errors & USART_ISR_PE) != 0) {
        rc |= UART_PARITYERROR;
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

    /*
     * Process multidrop setting.
     */
    if (tx_aframe) {
        rc |= UART_TXADDRFRAME;
    } else {
        rc |= UART_TXNORMFRAME;
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
static int Stm32UsartSetStatus(uint32_t flags)
{
    int rc = 0;
    /*
     * Process software handshake control.
     */

#if defined(UART_XONXOFF_CONTROL)
    if (flow_control) {

        /* Access to the flow control status must be atomic. */
        NutUartIrqDisable();

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
        NutUartIrqEnable();
    }
#endif

    /*
     * Process multidrop setting.
     */
    if (flags & UART_TXADDRFRAME) {
        tx_aframe = 1;
    }
    if (flags & UART_TXNORMFRAME) {
        tx_aframe = 0;
    }

    /*
     * Clear USART receive errors.
     */
    if (flags & UART_ERRORS) {
        CLEAR_ERRS();
        USARTN_RDR;
    }

    /*
     * Verify the result.
     */
    if ((Stm32UsartGetStatus() & ~UART_ERRORS) != flags) rc = -1;

    return rc;
}

/*!
 * \brief Query the USART hardware for synchronous mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * Not implemented for the STM32 USART. Always returns 0.
 *
 * \return Or-ed combination of \ref UART_SYNC, \ref UART_MASTER,
 *         \ref UART_NCLOCK and \ref UART_HIGHSPEED.
 */
static uint8_t Stm32UsartGetClockMode(void)
{
    return 0;
}

/*!
 * \brief Set asynchronous or synchronous mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * Not implemented for the STM32 USART. Always returns -1.
 *
 * \param mode Must be an or-ed combination of USART_SYNC, USART_MASTER,
 *             USART_NCLOCK and USART_HIGHSPEED.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartSetClockMode(uint8_t mode)
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
static uint32_t Stm32UsartGetFlowControl(void)
{
    uint32_t rc = 0;

#if defined(UART_XONXOFF_CONTROL)
    if (flow_control) {
        rc |= USART_MF_XONXOFF;
    } else {
        rc &= ~USART_MF_XONXOFF;
    }
#endif

    if( hdpx_control) {
        rc |= USART_MF_HALFDUPLEX;
    } else {
        rc &= ~USART_MF_HALFDUPLEX;
    }

#if defined( USART_485_CTRL)
    if (rs485_control) {
        rc |= USART_MF_LOCALECHO;
    } else {
        rc &= ~USART_MF_LOCALECHO;
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
    if (USARTn->CR3 & USART_CR3_HDSEL)
        rc |= USART_MF_OWIHALFDUPLEX;
    else
        rc &= ~USART_MF_OWIHALFDUPLEX;

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
static int Stm32UsartSetFlowControl(uint32_t flags)
{
#if defined(UART_XONXOFF_CONTROL)
    NutUartIrqDisable();
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
    NutUartIrqEnable();
#endif

    /* Setup half-duplex mode */
    if (flags & USART_MF_HALFDUPLEX) {
        hdpx_control = 1;
    }
    else if( hdpx_control == 1) {
        hdpx_control = 0;
    }

    /* Setup half-duplex mode */
    if (flags & USART_MF_OWIHALFDUPLEX) {
        Stm32UsartDisable();
        USARTn->CR1 &= ~USART_CR1_UE;
        USARTn->CR3 |= USART_CR3_HDSEL;
        /* Set Alternate function, open drain, Pull up */
#if defined(TX_GPIO_PORT)
#if defined(MCU_STM32F1)
        CM3BBSET(TX_GPIO_PORT, GPIO_TypeDef, CRL, _BI32(GPIO_CRL_CNF0_0) + TX_GPIO_PIN * 4);
#elif defined(MCU_STM32L0)
        CM3BBSET(TX_GPIO_PORT, GPIO_TypeDef, PUPDR, _BI32(GPIO_PUPDR_PUPD0_0) + TX_GPIO_PIN * 2);
#else
        CM3BBSET(TX_GPIO_PORT, GPIO_TypeDef, PUPDR, _BI32(GPIO_PUPDR_PUPDR0_0) + TX_GPIO_PIN * 2);
#endif
#endif
        Stm32UsartEnable();
    }
    else {
        Stm32UsartDisable();
        USARTn->CR1 &= ~USART_CR1_UE;
        USARTn->CR3 &= ~USART_CR3_HDSEL;
        /* Set Alternate function, push-pull */
#if defined(TX_GPIO_PORT)
#if defined(MCU_STM32F1)
        CM3BBCLR(TX_GPIO_PORT, GPIO_TypeDef, CRL, _BI32(GPIO_CRL_CNF0_0) + TX_GPIO_PIN * 4) ;
#elif defined(MCU_STM32L0)
        CM3BBCLR(TX_GPIO_PORT, GPIO_TypeDef, PUPDR, _BI32(GPIO_PUPDR_PUPD0_0) + TX_GPIO_PIN * 2);
#else
        CM3BBCLR(TX_GPIO_PORT, GPIO_TypeDef, PUPDR, _BI32(GPIO_PUPDR_PUPDR0_0) + TX_GPIO_PIN * 2);
#endif
#endif
        Stm32UsartEnable();
    }

#if defined( USART_485_CTRL)
    /* Control local echo mode:
     * Established by keeping /RE low while DE high. */
    if (flags & USART_MF_LOCALECHO) {
        rs485_control = 1;
        Rs485NRE_L();
    }
    else if (rs485_control) {
        rs485_control = 0;
    }
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

    /*
     * Verify the result.
     */
    if ((Stm32UsartGetFlowControl() & USART_MF_FLOWMASK) !=
        (flags & USART_MF_FLOWMASK)) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Start the USART transmitter hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it added one or more bytes to the
 * transmit buffer.
 */
static void Stm32UsartTxStart(void)
{
    Rs485DE_H();    /* Enable Sender */
    Rs485NRE_H();   /* Disable Receiver */

    if( hdpx_control) {
        /* Disable Receiver if half-duplex */
        CM3BBCLR(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RE));
    }

    /* Clear Transmit Complete flag */
    CLEAR_TC();
    /* Enable transmit interrupts. */
    CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_TXEIE));
}

/*!
 * \brief Start the USART receiver hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it removed enough bytes from the
 * receive buffer. Enough means, that the number of bytes left in
 * the buffer is below the low watermark.
 */
static void Stm32UsartRxStart(void)
{
#if defined(UART_XONXOFF_CONTROL)
    /*
     * Do any required software flow control.
     */
    if (flow_control && (flow_control & XOFF_SENT) != 0) {
        NutUartIrqDisable();
        if (TXE_SET) {
            USARTN_TDR=ASCII_XON;
            flow_control &= ~XON_PENDING;
        } else {
            flow_control |= XON_PENDING;
        }
        flow_control &= ~(XOFF_SENT | XOFF_PENDING);
        NutUartIrqEnable();
    }
#endif
    /* Enable receive interrupts. */
    CM3BBSET(USARTnBase, USART_TypeDef, CR1, _BI32(USART_CR1_RXNEIE));
}

/*
 * \brief Initialize the USART hardware driver.
 *
 * This function is called during device registration by the upper level
 * USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Stm32UsartInit(void)
{
    uint32_t cr2 = 0;
    uint32_t cr3 = 0;
    int res;

    /*
     * Register receive and transmit interrupts.
     */
    usart_sig = Stm32UsartInstallHandler(USARTidx, USARTnSIG);
    if (!usart_sig)
        return -1;

    res = NutRegisterIrqHandler(usart_sig, Stm32UsartInterrupt, &DcbUSART);
    if (res) {
        return -1;
    }

    /* Enable UART clock and reset device
     * We rely on the same value for RCC_APBxENR_USARTxEN and RCC_APBxRSTR_USARTxRST*/
    StmUsartClkEnable(1);
    Stm32F1UsartRemap();
    Stm32GpioConfigSet( USART_TX,  GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT, USART_TX_AF );
    Stm32GpioConfigSet( USART_RX,  GPIO_CFG_PERIPHAL,                   USART_RX_AF );
    Stm32GpioConfigSet( USART_CTS, GPIO_CFG_PERIPHAL,                   USART_CTS_AF);
    Stm32GpioConfigSet( USART_RTS, GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT, USART_RTS_AF);
    Stm32GpioConfigSet( USART_CK,  GPIO_CFG_PERIPHAL,                   USART_CK_AF );

    /*
     *   USART Communication Init
     */

    /* Enable additional features.
     * Most CR2/CR3 bit can only be set with CR1_UE == 0
     * E.g. Stm32UsartSetSpeed() sets CR1_UE
     */
#if defined(USART_TX_PIN_INV)
    cr2 |= USART_CR2_TXINV;
#endif
#if defined(USART_RX_PIN_INV)
    cr2 |= USART_CR2_RXINV;
#endif
#if defined(USART_SWAP)
    cr2 |= USART_CR2_SWAP;
#endif
    USARTn->CR2 = cr2;

#ifdef USART_HWFLOWCTRL
    /* Enable hardware handshake options */
#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
    cr3 |= USART_CR3_RTSE;
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
    cr3 |= USART_CR3_CTSE;
#endif
#endif

#ifdef USART_MODE_IRDA
    cr3 |= USART_CR3_IREN;
#endif

#ifdef USART_HARDWARE_HDX
    cr3 |= USART_CR3_HDSEL;
#endif
    USARTn->CR3 = cr3;

    /* USART configuration */
    Stm32UsartSetSpeed(USART_INIT_BAUTRATE);
    Stm32UsartSetDataBits(8);
    Stm32UsartSetStopBits(2);
    Stm32UsartSetParity(0);

    /* Disable transmitter interrupts as they will
     * be enabled by the TxStart routine */
    USARTn->CR1 &= ~(USART_CR1_TXEIE|USART_CR1_TCIE|USART_CR1_RXNEIE);
    /* Enable USART */
    USARTn->CR1 |= USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;

#if defined(UART_DMA_TXCHANNEL) || defined(UART_DMA_RXCHANNEL)
    DMA_Init();
#endif
//    NutIrqEnable(&SigUSART);
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
static int Stm32UsartDeinit(void)
{
    /* Disable interrupts */
//    NutIrqDisable(&SigUSART);
    USARTn->CR1 = 0;

    /* Deregister receive and transmit interrupts. */
    NutRegisterIrqHandler(usart_sig, 0, 0);

    /* Reset UART. */
    StmUsartClkEnable(0);

    return 0;
}

/*@}*/
