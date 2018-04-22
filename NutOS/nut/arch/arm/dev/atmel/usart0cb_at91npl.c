/*
 * Copyright (C) 2012 by egnite GmbH
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
 * \file arch/arm/atmel/usart0cb_at91npl.c
 * \brief Low level routines for UART0 on Ethernut 3.
 *
 * Contains those functions, which are implemented individually for UART0.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/irqreg.h>
#include <dev/npl.h>
#include <dev/usart.h>

#include <arch/arm/atmel/usart_at91ctl.h>
#include <arch/arm/atmel/usart_cb_at91npl.h>

/*
 * For maximum performance we may use native interrupts.
 */
#ifdef USART0_NATIVE_IRQ

static void Usart0IrqEntry(void) NUT_NAKED_FUNC;
void Usart0IrqEntry(void)
{
    IRQ_ENTRY();
    NplUsartCbInterrupt(devUsart0CbNpl.dev_dcb);
    IRQ_EXIT();
}

#endif

/*!
 * \brief Enable UART0 on Ethernut 3.
 */
static int Usart0Enable(USARTCB_DCB *dcb)
{
    uint8_t scr;

#ifdef USART0_NATIVE_IRQ
    /* Set the vector. */
    mem_wr32(AIC_SVR(US0_ID), (unsigned int)Usart0CbIrqEntry);
    /* Initialize to edge triggered with defined priority. */
    mem_wr32(AIC_SMR(US0_ID), AIC_SRCTYPE_INT_LEVEL_SENSITIVE | 4);
    /* Clear interrupt */
    mem_wr32(AIC_ICCR, _BV(US0_ID));
#else
    if (NutRegisterIrqHandler(&sig_UART0, NplUsartCbInterrupt, dcb)) {
        return -1;
    }
#endif
    if (NplRegisterIrqHandler(&sig_RSCTS, NplCtsInterrupt, dcb)) {
        return -1;
    }
    /* Enable receive and transmit. */
#if defined (P15_RXD0) && defined (P14_TXD0)
    outr(PIO_PDR, _BV(P15_RXD0) | _BV(P14_TXD0));
#elif defined (PA0_RXD0_A) && defined (PA1_TXD0_A)
    outr(PIOA_PDR, _BV(PA0_RXD0_A) | _BV(PA1_TXD0_A));
#elif defined (PA5_RXD0_A) && defined (PA6_TXD0_A)
    outr(PIOA_PDR, _BV(PA5_RXD0_A) | _BV(PA6_TXD0_A));
#elif defined (PB5_RXD0_A) && defined (PB4_TXD0_A)
    outr(PIOB_PDR, _BV(PB5_RXD0_A) | _BV(PB4_TXD0_A));
#elif defined (PB18_RXD0_A) && defined (PB19_TXD0_A)
    outr(PIOB_PDR, _BV(PB18_RXD0_A) | _BV(PB19_TXD0_A));
#else
#warning Undefined pin
#endif
    mem_wr32(dcb->usart_hwif + US_CR_OFF, US_RXEN | US_TXEN);
    NutIrqEnable(&sig_UART0);

    /* Enable handshake control. */
    scr = mem_rd8(NPL_RSCR);
    if ((scr & NPL_RSUS1P) == 0) {
        dcb->usart_mode |= USART_MF_RTSCONTROL;
        if ((scr & NPL_RSUS1E) == 0) {
            dcb->usart_mode |= USART_MF_DTRCONTROL;
        }
    }
    return 0;
}

/*!
 * \brief Disable UART0 on Ethernut 3.
 */
static int Usart0Disable(USARTCB_DCB *dcb)
{
    dcb->usart_rx_stop(dcb);
    dcb->usart_tx_stop(dcb);
    NutIrqDisable(&sig_UART0);
#if defined (P15_RXD0) && defined (P14_TXD0)
    mem_wr32(PIO_PER, _BV(P15_RXD0) | _BV(P14_TXD0));
#elif defined (PA0_RXD0_A) && defined (PA1_TXD0_A)
    mem_wr32(PIOA_PER, _BV(PA0_RXD0_A) | _BV(PA1_TXD0_A));
#elif defined (PA5_RXD0_A) && defined (PA6_TXD0_A)
    mem_wr32(PIOA_PER, _BV(PA5_RXD0_A) | _BV(PA6_TXD0_A));
#elif defined (PB5_RXD0_A) && defined (PB4_TXD0_A)
    mem_wr32(PIOB_PDR, _BV(PB5_RXD0_A) | _BV(PB4_TXD0_A));
#elif defined (PB18_RXD0_A) && defined (PB19_TXD0_A)
    mem_wr32(PIOB_PDR, _BV(PB18_RXD0_A) | _BV(PB19_TXD0_A));
#else
#warning Undefined pin
#endif

    return 0;
}

/*!
 * \brief Set and query the status of UART0 on Ethernut 3.
 */
static uint32_t Usart0Status(USARTCB_DCB *dcb, uint32_t stat)
{
    uint32_t rc = 0;
    uint8_t scr;
    uint16_t slr;

    scr = mem_rd8(NPL_RSCR);
    if ((scr & NPL_RSUS1P) == 0) {
        if (stat) {
            if (stat & USART_SF_RTSOFF) {
                scr &= ~NPL_RSRTS;
            } else {
                scr |= NPL_RSRTS;
            }
            if ((scr & NPL_RSUS1E) == 0 && (stat & USART_SF_DTROFF) == 0) {
                scr |= NPL_RSDTR;
            } else {
                scr &= ~NPL_RSDTR;
            }
            mem_wr8(NPL_RSCR, scr);
        }
        if ((scr & NPL_RSRTS) == 0) {
            rc |= USART_SF_RTSOFF;
        }
        if ((scr & NPL_RSDTR) == 0) {
            rc |= USART_SF_DTROFF;
        }

        mem_wr16_mb(NPL_SCR, NPL_RSDSR | NPL_RSDCD | NPL_RSCTS);
        slr = mem_rd16(NPL_SLR);
        if ((slr & NPL_RSCTS) == 0) {
            rc |= USART_SF_CTSOFF;
        }
        if ((slr & NPL_RSDCD) == 0) {
            rc |= USART_SF_DCDOFF;
        }
        if ((scr & NPL_RSUS1E) == 0 && (slr & NPL_RSDSR) == 0) {
            rc |= USART_SF_DSROFF;
        }
    }
    if ((mem_rd32(dcb->usart_hwif + US_IMR_OFF) & US_TXRDY) == 0) {
        rc |= USART_SF_TXDISABLED;
    }
    if ((mem_rd32(dcb->usart_hwif + US_IMR_OFF) & US_RXRDY) == 0) {
        rc |= USART_SF_RXDISABLED;
    }
    return rc;
}

/*!
 * \brief Set hardware flow control of UART0 on Ethernut 3.
 */
static int Usart0SetFlowControl(USARTCB_DCB *dcb, uint32_t mode)
{
    uint32_t mask = 0;
    uint8_t scr;

    scr = mem_rd8(NPL_RSCR);
    if ((scr & NPL_RSUS1P) == 0) {
        mask |= USART_MF_RTSCONTROL | USART_MF_CTSSENSE | USART_MF_DCDSENSE;
        if ((scr & NPL_RSUS1E) == 0) {
            mask |= USART_MF_DTRCONTROL | USART_MF_DSRSENSE;
        }
    }
    if ((mode & mask) != mode) {
        return -1;
    }
    dcb->usart_mode &= ~mask;
    dcb->usart_mode |= mode;

    return 0;
}

/*!
 * \brief Handle Ethernut 3 UART0 specific functions.
 *
 * We only handle hardware flow control. Anything else is passed to the
 * general Ethernut 3 UART handler.
 */
static int Usart0Control(USARTCB_DCB *dcb, int req, void *conf)
{
    uint32_t *u32vp = (uint32_t *) conf;

    if (req == UART_SETFLOWCONTROL) {
        return Usart0SetFlowControl(dcb, *u32vp);
    }
    return At91UsartHwControl(dcb, req, conf);
}

#define USART0_CAPS \
    USART_MF_RTSCONTROL | USART_MF_DTRCONTROL | \
    USART_MF_CTSSENSE | \
    USART_MF_COOKEDMODE

static USARTCB_DCB dcb_usart0 = {
    USART0_BASE,        /* usart_hwif. */
    Usart0Enable,       /* usart_enable(). */
    Usart0Disable,      /* usart_disable(). */
    Usart0Control,      /* usart_control(). */
    0,                  /* usart_mode. */
    USART0_CAPS,        /* usart_caps. */
    Usart0Status,       /* usart_status(). */
    { NULL, 0, 0, 0, NULL, 0 }, /* usart_tx_buff. */
    0,                  /* usart_wr_tmo. */
    NplUsartCbTxStart,  /* usart_tx_start(). */
    At91UsartHwTxStop,  /* usart_tx_stop(). */
    { NULL, 0, 0, 0, NULL, 0 }, /* usart_rx_buff. */
    0,                  /* usart_rd_tmo. */
    0,                  /* usart_rx_cr. */
    0,                  /* usart_rx_lowm. */
    0,                  /* usart_rx_hiwm. */
    NplUsartCbRxStart,  /* usart_rx_start(). */
    At91UsartHwRxStop   /* usart_rx_stop(). */
};

NUTDEVICE devUsart0CbNpl = {
    NULL,           /* Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '0', 0, 0, 0, 0},    /* Hardware device name, dev_name. */
    IFTYP_CHAR,     /* Type of device, dev_type. */
    0,              /* Base address, dev_base (not used). */
    0,              /* First interrupt number, dev_irq (not used). */
    NULL,           /* Interface control block, dev_icb (not used). */
    &dcb_usart0,    /* Driver control block, dev_dcb. */
    UsartCbInit,    /* Driver initialization routine, dev_init. */
    UsartCbIoCtrl,  /* Driver specific control function, dev_ioctl. */
    UsartCbRead,    /* Read from device, dev_read. */
    UsartCbWrite,   /* Write to device, dev_write. */
    UsartCbOpen,    /* Open a device or file, dev_open. */
    UsartCbClose,   /* Close a device or file, dev_close. */
    UsartCbSize,    /* Request file size, dev_size. */
    NULL,           /* Select function, optional, not yet implemented */
};
