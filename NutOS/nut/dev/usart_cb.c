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
 * \file dev/usart_cb.c
 * \brief Hardware independent part of a circular buffer based U(S)ART
 *        driver.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/usart.h>
#include <sys/nutdebug.h>

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <dev/usart_cb.h>

int UsartCbInit(NUTDEVICE *dev)
{
    USARTCB_DCB *dcb;

    /* Do sanity checks if NUTDEBUG_USE_ASSERT is defined. */
    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);

    dcb = (USARTCB_DCB *) dev->dev_dcb;

    return dcb->usart_enable(dcb);
}

NUTFILE *UsartCbOpen(NUTDEVICE *dev, const char *name, int mode, int acc)
{
    int rc = -1;
    NUTFILE *nfp;
    USARTCB_DCB *dcb;

    /* Do sanity checks if NUTDEBUG_USE_ASSERT is defined. */
    NUTASSERT(dev != NULL);
    NUTASSERT(dev->dev_dcb != NULL);

    dcb = (USARTCB_DCB *) dev->dev_dcb;
    nfp = calloc(1, sizeof(*nfp));
    if (nfp) {
        nfp->nf_dev = dev;
        /* We assume, that UART interrupts are inactive. */
        rc = CircBuffReset((CIRCBUFF *) &dcb->usart_rx_buff, 255);
        if (rc == 0) {
            rc = CircBuffReset((CIRCBUFF *) &dcb->usart_tx_buff, 255);
        }
    }
    /* Release resources on any error. */
    if (rc) {
        free (nfp);
        CircBuffReset((CIRCBUFF *) &dcb->usart_tx_buff, 0);
        CircBuffReset((CIRCBUFF *) &dcb->usart_rx_buff, 0);

        return NUTFILE_EOF;
    }
    /* Set proper device modes. */
    if ((mode & 0xC000) == _O_BINARY) {
        dcb->usart_mode &= ~USART_MF_COOKEDMODE;
    } else {
        dcb->usart_mode |= USART_MF_COOKEDMODE;
    }

    /* Set default watermarks to 1/4 and 3/4. */
    dcb->usart_rx_lowm = dcb->usart_rx_buff.rxb_siz / 4;
    dcb->usart_rx_hiwm = dcb->usart_rx_lowm * 3;
    dcb->usart_rx_start(dcb);

    return nfp;
}

int UsartCbClose(NUTFILE *nfp)
{
    int rc = 0;
    USARTCB_DCB *dcb;
    USARTCB_TXBUFF *txcb;

    /* Optional sanity checks if NUTDEBUG_USE_ASSERT is defined. */
    NUTASSERT(nfp != NULL && nfp != NUTFILE_EOF);
    NUTASSERT(nfp->nf_dev != NULL);
    NUTASSERT(nfp->nf_dev->dev_dcb != NULL);

    dcb = (USARTCB_DCB *) nfp->nf_dev->dev_dcb;
    txcb = &dcb->usart_tx_buff;

    /* Flush the complete output buffer. If this times out, let the
       caller know, that not all bytes had been transmitted. */
    while (txcb->txb_wri != txcb->txb_rdi) {
        dcb->usart_tx_start(dcb);
        if (NutEventWait(&txcb->txb_que, dcb->usart_wr_tmo)) {
            rc = -1;
        }
    }
    /* Disable interrupts and release buffers. */
    dcb->usart_rx_stop(dcb);
    CircBuffReset((CIRCBUFF *) &dcb->usart_rx_buff, 0);
    dcb->usart_tx_stop(dcb);
    CircBuffReset((CIRCBUFF *) &dcb->usart_tx_buff, 0);

    return rc;
}

int UsartCbRead(NUTFILE *nfp, void *buffer, int size)
{
    int rc;
    USARTCB_DCB *dcb;
    USARTCB_RXBUFF *rxcb;
    uint8_t *cp = (uint8_t *) buffer;

    /* Optional sanity checks if NUTDEBUG_USE_ASSERT is defined. */
    NUTASSERT(nfp != NULL && nfp != NUTFILE_EOF);
    NUTASSERT(nfp->nf_dev != NULL);
    NUTASSERT(nfp->nf_dev->dev_dcb != NULL);
    NUTASSERT(buffer || (buffer == NULL && size == 0));

    dcb = (USARTCB_DCB *) nfp->nf_dev->dev_dcb;
    rxcb = &dcb->usart_rx_buff;

    /* Call without data pointer discards the receive buffer. */
    if (buffer == NULL) {
        dcb->usart_rx_stop(dcb);
        rxcb->rxb_rdi = rxcb->rxb_wri;
        rxcb->rxb_cnt = 0;
        dcb->usart_rx_start(dcb);
        return 0;
    }
    /* Use a byte pointer to access the caller's buffer. */
    cp = (uint8_t *) buffer;

    /* Loop until the requested number of bytes had been copied to the
       caller's buffer. Note, the internal premature return in case
       of an empty buffer. */
    for (rc = 0; rc < (size_t) size;) {
        size_t chunk;

        /* Retrieve the number of consecutive bytes available in the
           circular buffer. Note, that we must disable the receiver
           to get exclusive access to the buffer. */
        dcb->usart_rx_stop(dcb);
        chunk = CircBuffReadSize((CIRCBUFF *) rxcb);
        dcb->usart_rx_start(dcb);

        if (chunk == 0) {
            /* If the receive buffer is empty, then wait until at least
               one character had been available or until a read timeout
               occurs. */
            if (rc || NutEventWait(&rxcb->rxb_que, dcb->usart_rd_tmo)) {
                /* Return the number of bytes we got so far. */
                return rc;
            }
        } else {
            /* Limit the read size to the caller's buffer size. */
            chunk = chunk > (size - rc) ? (size - rc) : chunk;
            /* In cooked mode we will translate carriage return linefeed
               sequences as well as single carriage returns to linefeeds. */
            if (dcb->usart_mode & USART_MF_COOKEDMODE) {
                int i;
                uint_fast8_t ch;

                for (i = 0; i < chunk; i++) {
                    ch = rxcb->rxb_buf[rxcb->rxb_rdi + i];
                    if (ch == '\r') {
                        ch = '\n';
                        dcb->usart_rx_cr = 1;
                    }
                    else if (dcb->usart_rx_cr) {
                        if (ch == '\n') {
                            ch = 0;
                        }
                        dcb->usart_rx_cr = 0;
                    }
                    if (ch) {
                        *cp++ = ch;
                        rc++;
                    }
                }
            } else {
                /* In binary mode we can copy the complete chunk in one go. */
                memcpy(cp, (void *)&rxcb->rxb_buf[rxcb->rxb_rdi], chunk);
                cp += chunk;
                rc += chunk;
            }
            /* Update the circular buffer after we removed some data.
               Again, that we must disable the receiver to get exclusive
               access. Note, that this will automatically re-enable the
               receiver in case it has been stopped on a buffer overflow. */
            dcb->usart_rx_stop(dcb);
            rxcb->rxb_rdi = (rxcb->rxb_rdi + chunk) & rxcb->rxb_siz;
            rxcb->rxb_cnt -= chunk;
            dcb->usart_rx_start(dcb);
        }
    }
    return rc;
}

int UsartCbWrite(NUTFILE *nfp, const void *buffer, int len)
{
    int rc;
    USARTCB_DCB *dcb;
    USARTCB_TXBUFF *txcb;
    size_t chunk;
    uint8_t *cp = (uint8_t *) buffer;
    uint_fast8_t eol = 0;

    /* Optional sanity checks if NUTDEBUG_USE_ASSERT is defined. */
    NUTASSERT(nfp != NULL && nfp != NUTFILE_EOF);
    NUTASSERT(nfp->nf_dev != NULL);
    NUTASSERT(nfp->nf_dev->dev_dcb != NULL);
    NUTASSERT(buffer || (buffer == NULL && size == 0));

    dcb = (USARTCB_DCB *) nfp->nf_dev->dev_dcb;
    txcb = &dcb->usart_tx_buff;

    /* Call without data pointer flushes the buffer. In this case a
       value not equal zero indicates write timeout. */
    if (buffer == NULL) {
        while (txcb->txb_wri != txcb->txb_rdi) {
            dcb->usart_tx_start(dcb);
            if (NutEventWait(&txcb->txb_que, dcb->usart_wr_tmo)) {
                return -1;
            }
        }
        return 0;
    }
    for (rc = 0; rc < len || eol; ) {
        /* Disable the low level transmitter. This is required to get
           exclusive access to the transmit buffer. */
        dcb->usart_tx_stop(dcb);
        chunk = CircBuffWriteSize((CIRCBUFF *) txcb);
        if (chunk == 0) {
            dcb->usart_tx_start(dcb);
            if (NutEventWait(&txcb->txb_que, dcb->usart_wr_tmo)) {
                /* Timeout. */
                return rc;
            }
            continue;
        }
        if (eol) {
            txcb->txb_buf[txcb->txb_wri++] = '\n';
            eol = 0;
            chunk--;
        }
        chunk = chunk > (len - rc) ? (len - rc) : chunk;
        if (dcb->usart_mode & USART_MF_COOKEDMODE) {
            int i;

            for (i = 0; i < chunk; i++) {
                eol = *cp == '\n';
                if (eol) {
                    txcb->txb_buf[txcb->txb_wri + i] = '\r';
                    cp++;
                    chunk = i + 1;
                    break;
                }
                txcb->txb_buf[txcb->txb_wri + i] = *cp++;
            }
        } else {
            memcpy(&txcb->txb_buf[txcb->txb_wri], cp, chunk);
            cp += chunk;
        }
        txcb->txb_wri = (txcb->txb_wri + chunk) & txcb->txb_siz;
        txcb->txb_cnt += chunk;
        /* Re-enable the low level transmitter. */
        dcb->usart_tx_start(dcb);
        rc += chunk;
    }
    return rc;
}

#ifdef __HARVARD_ARCH__
int UsartCbWrite_P(NUTFILE *nfp, PGM_P buffer, int len)
{
    return -1;
}
#endif

int UsartCbIoCtrl(NUTDEVICE *dev, int req, void *conf)
{
    int rc = 0;
    USARTCB_DCB *dcb;
    uint32_t *u32vp = (uint32_t *) conf;

    /* Debug sanity check. */
    NUTASSERT(dev != NULL);

    dcb = (USARTCB_DCB *) dev->dev_dcb;

    switch (req) {
    case UART_SETREADTIMEOUT:
        NUTASSERT(dev->dev_dcb != NULL);
        dcb->usart_rd_tmo = *u32vp;
        break;
    case UART_GETREADTIMEOUT:
        NUTASSERT(dev->dev_dcb != NULL);
        *u32vp = dcb->usart_rd_tmo;
        break;

    case UART_SETWRITETIMEOUT:
        NUTASSERT(dev->dev_dcb != NULL);
        dcb->usart_wr_tmo = *u32vp;
        break;
    case UART_GETWRITETIMEOUT:
        NUTASSERT(dev->dev_dcb != NULL);
        *u32vp = dcb->usart_wr_tmo;
        break;

    case UART_SETCOOKEDMODE:
        if (*u32vp) {
            dcb->usart_mode |= USART_MF_COOKEDMODE;
        } else {
            dcb->usart_mode &= ~USART_MF_COOKEDMODE;
        }
        break;
    case UART_GETCOOKEDMODE:
        if (dcb->usart_mode & USART_MF_COOKEDMODE) {
            *u32vp = 1;
        } else {
            *u32vp = 0;
        }
        break;

    case UART_SETTXBUFSIZ:
        dcb->usart_tx_stop(dcb);
        rc = CircBuffReset((CIRCBUFF *) &dcb->usart_tx_buff, *u32vp);
        if (rc == 0) {
            dcb->usart_tx_start(dcb);
        }
        break;
    case UART_GETTXBUFSIZ:
        *u32vp = dcb->usart_tx_buff.txb_siz;
        break;

    case UART_SETRXBUFSIZ:
        dcb->usart_rx_stop(dcb);
        rc = CircBuffReset((CIRCBUFF *) &dcb->usart_rx_buff, *u32vp);
        if (rc == 0) {
            dcb->usart_rx_start(dcb);
        }
        break;
    case UART_GETRXBUFSIZ:
        *u32vp = dcb->usart_rx_buff.rxb_siz;
        break;

    case UART_SETRXBUFLWMARK:
        dcb->usart_rx_stop(dcb);
        dcb->usart_rx_lowm = *u32vp;
        dcb->usart_rx_start(dcb);
        break;
    case UART_GETRXBUFLWMARK:
        *u32vp = dcb->usart_rx_lowm;
        break;

    case UART_SETRXBUFHWMARK:
        dcb->usart_rx_stop(dcb);
        dcb->usart_rx_hiwm = *u32vp;
        dcb->usart_rx_start(dcb);
        break;
    case UART_GETRXBUFHWMARK:
        *u32vp = dcb->usart_rx_hiwm;
        break;

    default:
        NUTASSERT(dcb->usart_control != NULL);
        rc = dcb->usart_control(dcb, req, conf);
        break;
    }
    return rc;
}

long UsartCbSize(NUTFILE *nfp)
{
    long rc;
    USARTCB_DCB *dcb;
    USARTCB_RXBUFF *rxcb;

    NUTASSERT(nfp != NULL && nfp != NUTFILE_EOF);
    NUTASSERT(nfp->nf_dev != NULL);
    NUTASSERT(nfp->nf_dev->dev_dcb != NULL);

    dcb = (USARTCB_DCB *) nfp->nf_dev->dev_dcb;
    rxcb = &dcb->usart_rx_buff;

    /* Retrieve the number of bytes currently available in the
       circular buffer. Note, that we must disable the receiver
       to get exclusive access to the buffer. */
    dcb->usart_rx_stop(dcb);
    rc = rxcb->rxb_wri - rxcb->rxb_rdi;
    if (rxcb->rxb_rdi > rxcb->rxb_wri) {
        rc += rxcb->rxb_siz + 1;
    }
    dcb->usart_rx_start(dcb);

    return rc;
}
