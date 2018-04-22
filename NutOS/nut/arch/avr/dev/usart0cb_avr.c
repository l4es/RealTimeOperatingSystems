/*
 * Copyright (C) 2012-2013 by egnite GmbH
 * Copyright (C) 2001-2003 by egnite Software GmbH
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
 * \file arch/avr/usart0cb_avr.c
 * \brief Low level routines for UART0 on AVR.
 *
 * Contains those functions, which are implemented individually for UART0.
 *
 * Note, that it is essential for most AVR chips to use constants as port
 * addresses. This will allow the compiler to generate bit operations.
 * Otherwise, it will result in duplicate code for multiple UARTs. But
 * this is OK, because most applications will not use more than one UART.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/arch/avr.h>
#include <stdint.h>

#include <dev/irqreg.h>
#include <dev/usart.h>
#include <sys/timer.h>
#include <sys/atom.h>

#include <arch/avr/usart_avrctl.h>
#include <arch/avr/usart_cb_avr.h>

#ifndef UART0_INIT_BAUDRATE
#define UART0_INIT_BAUDRATE 115200
#endif

/*!
 * \name UART0 CTS Handshake Sense
 *
 * UART0_CTS_IRQ must be defined to enable CTS sensing.
 */
#ifdef UART0_CTS_IRQ
#undef UART_CTS_IRQ
#define UART_CTS_IRQ    UART0_CTS_IRQ
#include <arch/avr/usart_avrcts.h>
#endif

/*!
 * \name UART0 RTS Handshake Control
 *
 * UART0_RTS_BIT and UART0_RTS_PORT must be defined to enable RTS control.
 */
#if defined(UART0_RTS_BIT) && defined(UART0_RTS_AVRPORT)

#if (UART0_RTS_AVRPORT == AVRPORTB)
#define UART0_RTS_PORT  PORTB
#define UART0_RTS_DDR   DDRB

#elif (UART0_RTS_AVRPORT == AVRPORTD)
#define UART0_RTS_PORT  PORTD
#define UART0_RTS_DDR   DDRD

#elif (UART0_RTS_AVRPORT == AVRPORTE)
#define UART0_RTS_PORT  PORTE
#define UART0_RTS_DDR   DDRE

#elif (UART0_RTS_AVRPORT == AVRPORTF)
#define UART0_RTS_PORT  PORTF
#define UART0_RTS_DDR   DDRF

#elif (UART0_RTS_AVRPORT == AVRPORTG)
#define UART0_RTS_PORT  PORTG
#define UART0_RTS_DDR   DDRG

#elif (UART0_RTS_AVRPORT == AVRPORTH)
#define UART0_RTS_PORT  PORTH
#define UART0_RTS_DDR   DDRH
#endif

#endif

static AVRUSART_IFC ifc_usart0 = { 0, 0 };

static void AvrUsart0RxData(void *arg)
{
    uint8_t ch;
    USARTCB_DCB *dcb;
    cb_size_t idx;
    USARTCB_RXBUFF *rxcb;
    uint8_t err;

    err = inb(UCSR0A) & (_BV(FE) | _BV(DOR) | _BV(UPE));
    ch = inb(UDR0);
    if (err == 0) {
        dcb = (USARTCB_DCB *) arg;
        rxcb = &dcb->usart_rx_buff;
        idx = (rxcb->rxb_wri + 1) & rxcb->rxb_siz;
        if (idx == rxcb->rxb_rdi) {
            /* Receive buffer overflow. */
            cbi(UCSR0B, RXCIE);
        } else {
            rxcb->rxb_buf[rxcb->rxb_wri] = ch;
            rxcb->rxb_wri = idx;
            if (rxcb->rxb_cnt++ == 0) {
                NutEventPostFromIrq(&rxcb->rxb_que);
            }
            /* If an RTS port has been defined and if RTS control has
               been enabled and if the number of bytes in the receive
               buffer reached the high watermark, then disable RTS to
               stop the remote transmitter. */
#if defined(UART0_RTS_PORT) && defined(UART0_RTS_BIT)
            if ((dcb->usart_mode & USART_MF_RTSCONTROL) != 0 && rxcb->rxb_cnt >= dcb->usart_rx_hiwm) {
                sbi(UART0_RTS_PORT, UART0_RTS_BIT);
            }
#endif
        }
    } else {
        ifc_usart0.uifc_errors |= err;
    }
}

#ifdef UART_CTS_BIT
/*!
 * \brief USART0 CTS sense interrupt handler.
 *
 * This interrupt routine is called when the CTS line level is low.
 * Typical line drivers negate the signal, thus driving our port
 * low when CTS is active.
 *
 * This routine exists only if the hardware configuration defines a
 * port bit to sense the CTS signal.
 */
static void AvrUsart0CtsActive(void *arg)
{
    /* Enable transmit interrupt. */
    sbi(UCSR0B, UDRIE);
    /* Disable CTS sense interrupt. */
    cbi(EIMSK, UART_CTS_BIT);
}
#endif

static void AvrUsart0TxData(void *arg)
{
    USARTCB_DCB *dcb = (USARTCB_DCB *) arg;
    USARTCB_TXBUFF *txcb = &dcb->usart_tx_buff;
    cb_size_t idx;

    idx = txcb->txb_rdi;
    if (idx == txcb->txb_wri) {
        /* Transmit buffer underrun. */
        cbi(UCSR0B, UDRIE);
        NutEventPostFromIrq(&txcb->txb_que);
    } else {
#if defined(UART_CTS_BIT) && defined(UART_CTS_PIN)
        /* If CTS has been disabled, we disable the transmit interrupts
           and return without sending anything. */
        if ((dcb->usart_mode & USART_MF_CTSSENSE) != 0 && bit_is_set(UART_CTS_PIN, UART_CTS_BIT)) {
            cbi(UCSR0B, UDRIE);
            sbi(EIMSK, UART_CTS_BIT);
            return;
        }
#endif
        outb(UDR0, txcb->txb_buf[idx]);
        txcb->txb_rdi = (idx + 1) & txcb->txb_siz;
        if (txcb->txb_cnt-- == txcb->txb_siz) {
            NutEventPostFromIrq(&txcb->txb_que);
        }
    }
}

/*!
 * \brief Enable AVR USART0.
 */
static int AvrUsart0Enable(USARTCB_DCB *dcb)
{
    if (NutRegisterIrqHandler(&sig_UART0_RECV, AvrUsart0RxData, dcb)) {
        return -1;
    }
    if (NutRegisterIrqHandler(&sig_UART0_DATA, AvrUsart0TxData, dcb)) {
        return -1;
    }

#if defined(UART_CTS_BIT)
#if defined(UART_CTS_PORT)
    sbi(UART_CTS_PORT, UART_CTS_BIT);
#endif
#if defined(UART_CTS_DDR)
    cbi(UART_CTS_DDR, UART_CTS_BIT);
#endif
#endif
#if defined(UART_CTS_SIGNAL)
    NutRegisterIrqHandler(&UART_CTS_SIGNAL, AvrUsart0CtsActive, 0);
    NutIrqSetMode(&UART_CTS_SIGNAL, NUT_IRQMODE_FALLINGEDGE);
#endif

#if UART0_INIT_BAUDRATE
    {
        uint32_t baud = UART0_INIT_BAUDRATE;
        AvrUsartControl(dcb, UART_SETSPEED, &baud);
    }
#endif
    sbi(UCSR0B, TXEN);
    sbi(UCSR0B, RXEN);

#if defined(UART0_RTS_DDR) && defined(UART0_RTS_BIT)
    sbi(UART0_RTS_DDR, UART0_RTS_BIT);
#endif

    return 0;
}

static int AvrUsart0Disable(USARTCB_DCB *dcb)
{
    return -1;
}

static void AvrUsart0TxStart(USARTCB_DCB *dcb)
{
    USARTCB_TXBUFF *txcb = &dcb->usart_tx_buff;
    cb_size_t idx;

#if defined(UART_CTS_BIT) && defined(UART_CTS_PIN)
    if ((dcb->usart_mode & USART_MF_CTSSENSE) != 0 && bit_is_set(UART_CTS_PIN, UART_CTS_BIT)) {
        sbi(EIMSK, UART_CTS_BIT);
        return;
    }
#endif
    if ((inb(UCSR0A) & _BV(TXC0)) == 0 && (inb(UCSR0A) & _BV(UDRE0)) != 0) {
        idx = txcb->txb_rdi;
        if (idx != txcb->txb_wri) {
            txcb->txb_rdi = (idx + 1) & txcb->txb_siz;
            outb(UDR0, txcb->txb_buf[idx]);
        }
    }
    sbi(UCSR0B, UDRIE);
}

static void AvrUsart0TxStop(USARTCB_DCB *dcb)
{
    /* Disable receive and transmit interrupts. */
    cbi(UCSR0B, TXCIE);
    cbi(UCSR0B, UDRIE);
}

static void AvrUsart0RxStart(USARTCB_DCB *dcb)
{
    sbi(UCSR0B, RXCIE);
#if defined(UART0_RTS_PORT) && defined(UART0_RTS_BIT)
    if (dcb->usart_rx_buff.rxb_cnt < dcb->usart_rx_hiwm) {
        /* Enable handshake. */
        cbi(UART0_RTS_PORT, UART0_RTS_BIT);
    }
#endif
}

static void AvrUsart0RxStop(USARTCB_DCB *dcb)
{
    /* Disable receive and transmit interrupts. */
    cbi(UCSR0B, RXCIE);
}

/*!
 * \brief Set and query the UART status.
 */
static uint32_t AvrUsart0Status(USARTCB_DCB *dcb, uint32_t stat)
{
    uint32_t rc = 0;

    /*
     * Set receiver error flags.
     */
    if (ifc_usart0.uifc_errors & _BV(FE0)) {
        rc |= UART_FRAMINGERROR;
    }
    if (ifc_usart0.uifc_errors & _BV(DOR0)) {
        rc |= UART_OVERRUNERROR;
    }
    if (ifc_usart0.uifc_errors & _BV(UPE0)) {
        rc |= UART_PARITYERROR;
    }
    ifc_usart0.uifc_errors = 0;

    return rc;
}


static USARTCB_DCB dcb_usart0 = {
    (uintptr_t) &ifc_usart0,/* usart_hwif. */
    AvrUsart0Enable,        /* usart_enable(). */
    AvrUsart0Disable,       /* usart_disable(). */
    AvrUsartControl,        /* usart_control(). */
    0,                      /* usart_mode. */
#if defined(UART0_RTS_BIT)
    USART_MF_RTSCONTROL |
#endif
#if defined(UART_CTS_BIT)
    USART_MF_CTSSENSE |
#endif
    USART_MF_COOKEDMODE,    /* usart_caps. */
    AvrUsart0Status,        /* usart_status(). */
    { NULL, 0, 0, 0, NULL, 0 }, /* usart_tx_buff. */
    0,                      /* usart_wr_tmo. */
    AvrUsart0TxStart,       /* usart_tx_start(). */
    AvrUsart0TxStop,        /* usart_tx_stop(). */
    { NULL, 0, 0, 0, NULL, 0 }, /* usart_rx_buff. */
    0,                      /* usart_rd_tmo. */
    0,                      /* usart_rx_cr. */
    0,                      /* usart_rx_lowm. */
    0,                      /* usart_rx_hiwm. */
    AvrUsart0RxStart,       /* usart_rx_start(). */
    AvrUsart0RxStop         /* usart_rx_stop(). */
};

NUTDEVICE devUsart0CbAvr = {
    NULL,           /* Pointer to next device, dev_next. */
    { 'u', 'a', 'r', 't', '0', 0, 0, 0, 0 }, /* Hardware device name, dev_name. */
    IFTYP_CHAR,     /* Type of device, dev_type. */
    0,              /* Base address, dev_base (not used). */
    0,              /* First interrupt number, dev_irq (not used). */
    NULL,           /* Interface control block, dev_icb (not used). */
    &dcb_usart0,    /* Driver control block, dev_dcb. */
    UsartCbInit,    /* Driver initialization routine, dev_init. */
    UsartCbIoCtrl,  /* Driver specific control function, dev_ioctl. */
    UsartCbRead,    /* Read from device, dev_read. */
    UsartCbWrite,   /* Write to device, dev_write. */
    UsartCbWrite_P, /* Write to device, dev_write_P. */
    UsartCbOpen,    /* Open a device or file, dev_open. */
    UsartCbClose,   /* Close a device or file, dev_close. */
    UsartCbSize,    /* Request file size, dev_size. */
    NULL            /* Select function, optional, not yet implemented */
};
