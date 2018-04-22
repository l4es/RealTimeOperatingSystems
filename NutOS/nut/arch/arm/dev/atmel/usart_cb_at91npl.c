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

#include <dev/npl.h>
#include <dev/usart.h>

#include <arch/arm/atmel/usart_cb_at91npl.h>

/*!
 * \file arch/arm/atmel/usart0cb_at91npl.c
 * \brief Low level routines for Ethernut 3 UARTs.
 *
 * Contains those functions, which are common to all internal UART
 * devices on Ethernut 3.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

static INLINE int CtsSense(void)
{
    mem_wr16_mb(NPL_SCR, NPL_RSCTS);
    return (mem_rd16(NPL_SLR) & NPL_RSCTS) != 0;
}

void NplCtsInterrupt(void *arg)
{
    USARTCB_DCB *dcb = (USARTCB_DCB *) arg;

    if (CtsSense()) {
        /* Disable CTS sense interrupt. */
        NplIrqDisable(&sig_RSCTS);
        /* Enable transmit interrupt. */
        mem_wr32(dcb->usart_hwif + US_IER_OFF, US_TXRDY);
    }
}

void NplUsartCbInterrupt(void *arg)
{
    uint32_t sr;
    uint8_t ch;
    USARTCB_DCB *dcb = (USARTCB_DCB *) arg;
    cb_size_t idx;

    sr = mem_rd32(dcb->usart_hwif + US_CSR_OFF);
    sr &= mem_rd32(dcb->usart_hwif + US_IMR_OFF);

    if (sr & US_RXRDY) {
        USARTCB_RXBUFF *rxcb;

        ch = inb(dcb->usart_hwif + US_RHR_OFF);
#if 0
        if (ch > 31 && ch < 127)
            putchar(ch);
        else
            putchar('.');
#endif
        rxcb = &dcb->usart_rx_buff;
        idx = (rxcb->rxb_wri + 1) & rxcb->rxb_siz;
        if (idx == rxcb->rxb_rdi) {
            /* Receive buffer overflow. */
            mem_wr32(dcb->usart_hwif + US_IDR_OFF, US_RXRDY);
        } else {
            rxcb->rxb_buf[rxcb->rxb_wri] = ch;
            rxcb->rxb_wri = idx;
            if (rxcb->rxb_cnt++ == 0) {
                NutEventPostFromIrq(&rxcb->rxb_que);
            }
            if ((dcb->usart_mode & USART_MF_RTSCONTROL) != 0 && rxcb->rxb_cnt >= dcb->usart_rx_hiwm) {
                /* Disable handshake. */
                mem_wr8(NPL_RSCR, mem_rd8(NPL_RSCR) & ~NPL_RSRTS);
            }
        }
    }

    if (sr & US_TXRDY) {
        USARTCB_TXBUFF *txcb = &dcb->usart_tx_buff;

        idx = txcb->txb_rdi;
        if (idx == txcb->txb_wri) {
            /* Transmit buffer underrun. */
            mem_wr32(dcb->usart_hwif + US_IDR_OFF, US_TXRDY);
            NutEventPostFromIrq(&txcb->txb_que);
        }
        else if ((dcb->usart_mode & USART_MF_CTSSENSE) == 0 || CtsSense()) {
            //ch = txcb->txb_buf[idx];
            mem_wr32(dcb->usart_hwif + US_THR_OFF, txcb->txb_buf[idx]);
            txcb->txb_rdi = (idx + 1) & txcb->txb_siz;
            if (txcb->txb_cnt-- == txcb->txb_siz) {
                NutEventPostFromIrq(&txcb->txb_que);
            }
#if 0
            if (ch > 31 && ch < 127)
                putchar(ch);
            else
                putchar('.');
#endif
        }
        else {
            mem_wr32(dcb->usart_hwif + US_IDR_OFF, US_TXRDY);
            mem_wr16(NPL_IMR, mem_rd16(NPL_IMR) | NPL_RSCTS);
        }
    }
}

void NplUsartCbRxStart(USARTCB_DCB *dcb)
{
    /* Enable receive interrupt. */
    mem_wr32(dcb->usart_hwif + US_IER_OFF, US_RXRDY);

    if (dcb->usart_mode & USART_MF_RTSCONTROL) {
        if (dcb->usart_rx_buff.rxb_cnt <= dcb->usart_rx_lowm) {
            /* Enable RTS handshake output. */
            mem_wr8(NPL_RSCR, mem_rd8(NPL_RSCR) | NPL_RSRTS);
        }
    }
    if (dcb->usart_mode & USART_MF_DTRCONTROL) {
        /* Enable DTR handshake output. */
        mem_wr8(NPL_RSCR, mem_rd8(NPL_RSCR) | NPL_RSDTR);
    }
}

void NplUsartCbTxStart(USARTCB_DCB *dcb)
{
    if ((dcb->usart_mode & USART_MF_CTSSENSE) == 0 || CtsSense()) {
        /* Enable transmit interrupt. */
        mem_wr32(dcb->usart_hwif + US_IER_OFF, US_TXRDY);
    } else {
        /* Enable CTS interrupt. */
        NplIrqEnable(&sig_RSCTS);
    }
}
