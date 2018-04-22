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

#include <dev/usart.h>
#include <sys/timer.h>

#include <arch/arm/atmel/usart_at91ctl.h>

static uint32_t UsartHwGetSpeed(USARTCB_DCB *dcb)
{
    unsigned int cs;
    uint32_t clk;

    clk = NutClockGet(NUT_HWCLK_PERIPHERAL);
    cs = mem_rd32(dcb->usart_hwif + US_MR_OFF) & US_CLKS;
    if (cs == US_CLKS_MCK8) {
        clk /= 8;
    }
    else if (cs != US_CLKS_MCK) {
        clk = 0;
    }
    return clk / (16UL * (mem_rd32(dcb->usart_hwif + US_BRGR_OFF) & 0xFFFF));
}

static int UsartHwSetSpeed(USARTCB_DCB *dcb, uint32_t rate)
{
    uint32_t clk;
    int rc;

    clk = NutClockGet(NUT_HWCLK_PERIPHERAL);
    mem_wr32(dcb->usart_hwif + US_BRGR_OFF, (clk / (8 * rate) + 1) / 2);

    rc = rate - UsartHwGetSpeed(dcb);
    rc = rc < 0 ? -rc : rc;

    return rc < rate / 100 ? 0 : -1;
}

static uint_fast8_t UsartHwGetDataBits(USARTCB_DCB *dcb)
{
    unsigned int val;

    val = mem_rd32(dcb->usart_hwif + US_MR_OFF);
    if ((val & US_PAR) == US_PAR_MULTIDROP) {
        val = 9;
    } else {
        val = 5 + ((val & US_CHRL) >> 6);
    }
    return (uint_fast8_t) val;
}

static int UsartHwSetDataBits(USARTCB_DCB *dcb, uint_fast8_t bits)
{
    unsigned int val = mem_rd32(dcb->usart_hwif + US_MR_OFF);

    if (bits == 9) {
        val &= ~US_PAR;
        val |= US_PAR_MULTIDROP;
    }
    else {
        val &= ~US_CHRL;
        val |= (bits - 5) << 6;
    }
    mem_wr32(dcb->usart_hwif + US_MR_OFF, val);

    return UsartHwGetDataBits(dcb) == bits ? 0 : -1;
}

static uint_fast8_t UsartHwGetParity(USARTCB_DCB *dcb)
{
    unsigned int val;

    val = mem_rd32(dcb->usart_hwif + US_MR_OFF) & US_PAR;
    if ((val & US_PAR) == US_PAR_MULTIDROP) {
        val = 9;
    }
    else {
        if (val == US_PAR_ODD) {
            val = 1;
        }
        else if (val == US_PAR_EVEN) {
            val = 2;
        }
        else {
            val = 0;
        }
    }
    return (uint_fast8_t) val;
}

static int UsartHwSetParity(USARTCB_DCB *dcb, uint_fast8_t mode)
{
    unsigned int val;

    val = mem_rd32(dcb->usart_hwif + US_MR_OFF) & ~US_PAR;
    switch (mode) {
    case 0:
        val |= US_PAR_NO;
        break;
    case 1:
        val |= US_PAR_ODD;
        break;
    case 2:
        val |= US_PAR_EVEN;
        break;
    }
    mem_wr32(dcb->usart_hwif + US_MR_OFF, val);

    return UsartHwGetParity(dcb) == mode ? 0 : -1;
}

static uint8_t UsartHwGetStopBits(USARTCB_DCB *dcb)
{
    unsigned int val = mem_rd32(dcb->usart_hwif + US_MR_OFF) & US_NBSTOP;
    if (val == US_NBSTOP_1) {
        val = 1;
    }
    else if (val == US_NBSTOP_2) {
        val = 2;
    }
    else {
        val = 3;
    }
    return (uint8_t)val;
}

static int UsartHwSetStopBits(USARTCB_DCB *dcb, uint8_t bits)
{
    unsigned int val = mem_rd32(dcb->usart_hwif + US_MR_OFF) & ~US_NBSTOP;

    switch(bits) {
    case 1:
        val |= US_NBSTOP_1;
        break;
    case 2:
        val |= US_NBSTOP_2;
        break;
    case 3:
        val |= US_NBSTOP_1_5;
        break;
    }
    mem_wr32(dcb->usart_hwif + US_MR_OFF, val);

    return UsartHwGetStopBits(dcb) == bits ? 0 : -1;
}

void At91UsartHwRxStop(USARTCB_DCB *dcb)
{
    /* Disable receive and transmit interrupts. */
    mem_wr32(dcb->usart_hwif + US_IDR_OFF, US_RXRDY);
}

void At91UsartHwTxStop(USARTCB_DCB *dcb)
{
    /* Disable receive and transmit interrupts. */
    mem_wr32(dcb->usart_hwif + US_IDR_OFF, US_TXRDY);
    /* Wait until all bits had been shifted out. */
    if (mem_rd32(dcb->usart_hwif + US_CSR_OFF) & US_TXRDY) {
        while((mem_rd32(dcb->usart_hwif + US_CSR_OFF) & US_TXEMPTY) == 0);
    }
}

int At91UsartHwControl(USARTCB_DCB *dcb, int req, void *conf)
{
    int rc = 0;
    uint32_t *u32vp = (uint32_t *) conf;

    switch (req) {
    case UART_GETSPEED:
        *u32vp = UsartHwGetSpeed(dcb);
        break;
    case UART_SETSPEED:
        rc = UsartHwSetSpeed(dcb, *u32vp);
        break;

    case UART_GETDATABITS:
        *u32vp = UsartHwGetDataBits(dcb);
        break;
    case UART_SETDATABITS:
        rc = UsartHwSetDataBits(dcb, (uint_fast8_t) *u32vp);
        break;

    case UART_GETPARITY:
        *u32vp = UsartHwGetParity(dcb);
        break;
    case UART_SETPARITY:
        rc = UsartHwSetParity(dcb, (uint_fast8_t) *u32vp);
        break;

    case UART_GETSTOPBITS:
        *u32vp = UsartHwGetStopBits(dcb);
        break;
    case UART_SETSTOPBITS:
        rc = UsartHwSetStopBits(dcb, (uint_fast8_t) *u32vp);
        break;

    case UART_GETSTATUS:
        *u32vp = dcb->usart_status(dcb, 0);
        break;
    case UART_SETSTATUS:
        dcb->usart_status(dcb, *u32vp | _BV(31));
        break;

    case UART_GETFLOWCONTROL:
        *u32vp = dcb->usart_mode & (USART_MF_SENSEMASK | USART_MF_CONTROLMASK | USART_MF_XONXOFF);
        break;

    case UART_GETRXBUFLWMARK:
        *u32vp = dcb->usart_rx_lowm;
        break;
    case UART_SETRXBUFLWMARK:
        dcb->usart_rx_lowm = (size_t) *u32vp;
        break;

    case UART_GETRXBUFHWMARK:
        *u32vp = dcb->usart_rx_hiwm;
        break;
    case UART_SETRXBUFHWMARK:
        dcb->usart_rx_hiwm = (size_t) *u32vp;
        break;

    default:
        rc = -1;
        break;
    }
    return rc;
}

