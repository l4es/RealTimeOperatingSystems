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
 * \file arch/avr/usart_cb_avr.c
 * \brief Low level routines for AVR UARTs.
 *
 * See comments in the UART0 driver about constant port addresses.
 * The routines in this file should work for all UARTs. This saves
 * code in applications using multiple UARTs, but adds more code
 * to applications using a single UART only. To avoid the latter,
 * NUT_USE_UART0_ONLY may be defined.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/usart.h>
#include <sys/timer.h>

#include <arch/avr/usart_avrctl.h>

#if defined(UBRR0H)
#define UBRR0_HIGH  ((uint32_t) inb(UBRR0H) << 8)
#else
#define UBRR0_HIGH  0
#endif

#if defined(U2X0)
#define U2X0_SET (bit_is_set(UCSR0A, U2X0) != 0)
#else
#define U2X0_SET 0
#endif

#if defined(UMSEL0)
#define UMSEL0_SET (bit_is_set(UCSR0C, UMSEL0) != 0)
#else
#define UMSEL0_SET 0
#endif

#if defined(UBRR1H)
#define UBRR1_HIGH  ((uint32_t) inb(UBRR1H) << 8)
#else
#define UBRR1_HIGH  0
#endif

#if defined(U2X1)
#define U2X1_SET (bit_is_set(UCSR1A, U2X1) != 0)
#else
#define U2X1_SET 0
#endif

#if defined(UMSEL1)
#define UMSEL1_SET (bit_is_set(UCSR1C, UMSEL1) != 0)
#else
#define UMSEL1_SET 0
#endif

#if defined(UBRR2H)
#define UBRR2_HIGH  ((uint32_t) inb(UBRR2H) << 8)
#else
#define UBRR2_HIGH  0
#endif

#if defined(U2X2)
#define U2X2_SET (bit_is_set(UCSR2A, U2X2) != 0)
#else
#define U2X2_SET 0
#endif

#if defined(UMSEL2)
#define UMSEL2_SET (bit_is_set(UCSR2C, UMSEL2) != 0)
#else
#define UMSEL2_SET 0
#endif

#if defined(UBRR3H)
#define UBRR3_HIGH  ((uint32_t) inb(UBRR3H) << 8)
#else
#define UBRR3_HIGH  0
#endif

#if defined(U2X3)
#define U2X3_SET (bit_is_set(UCSR3A, U2X3) != 0)
#else
#define U2X3_SET 0
#endif

#if defined(UMSEL3)
#define UMSEL3_SET (bit_is_set(UCSR3C, UMSEL3) != 0)
#else
#define UMSEL3_SET 0
#endif


static uint32_t UsartGetSpeed(uint8_t ifn)
{
    uint32_t div;

#if !defined(NUT_USE_UART0_ONLY)

    uint8_t shift;

    switch (ifn) {
    case 0:
        div = inb(UBRR0L) | UBRR0_HIGH;
        shift = (UMSEL0_SET ? 0 : 4 - U2X0_SET);
        break;
    case 1:
        div = inb(UBRR1L) | UBRR1_HIGH;
        shift = (UMSEL1_SET ? 0 : 4 - U2X1_SET);
        break;
#if defined(UBRR2L)
    /* USART2 --------------------- */
    case 2:
        div = inb(UBRR2L) | UBRR2_HIGH;
        shift = (UMSEL2_SET ? 0 : 4 - U2X2_SET);
        break;
#if defined(UBRR3L)
    /* USART3 --------------------- */
    case 3:
        div = inb(UBRR3L) | UBRR3_HIGH;
        shift = (UMSEL3_SET ? 0 : 4 - U2X3_SET);
        break;
#endif
#endif
    default:
        return 0;
    }
    div++;
    div <<= shift;

#elif defined(NUT_USE_UART0_ONLY)

    div = inb(UBRR0L);
    div |= UBRR0_HIGH;
    div++;
    div <<= (UMSEL0_SET ? 0 : 4 - U2X0_SET);

#endif

    return NutClockGet(NUT_HWCLK_PERIPHERAL) / div;
}

static int UsartSetSpeed(uint8_t ifn, uint32_t rate)
{
    uint16_t sv;

#if !defined(NUT_USE_UART0_ONLY)

    uint8_t shift;

    switch (ifn) {
    case 0:
        shift = (UMSEL0_SET ? 0 : 3 - U2X0_SET);
        break;
    case 1:
        shift = (UMSEL1_SET ? 0 : 3 - U2X1_SET);
        break;
#if defined(UBRR2L)
    case 2:
        shift = (UMSEL2_SET ? 0 : 3 - U2X2_SET);
        break;
#if defined(UBRR3L)
    case 3:
        shift = (UMSEL3_SET ? 0 : 3 - U2X3_SET);
        break;
#endif
#endif
    default:
        return -1;
    }
    rate <<= shift;
    sv = (uint16_t) ((NutClockGet(NUT_HWCLK_PERIPHERAL) / rate + 1UL) / 2UL) - 1;

    switch (ifn) {
    case 0:
        outb(UBRR0L, (uint8_t) sv);
#if defined(UBRR0H)
        outb(UBRR0H, (uint8_t) (sv >> 8));
#endif
        break;
    case 1:
        outb(UBRR1L, (uint8_t) sv);
#if defined(UBRR1H)
        outb(UBRR1H, (uint8_t) (sv >> 8));
#endif
        break;
#if defined(UBRR2L)
    case 2:
        outb(UBRR2L, (uint8_t) sv);
#if defined(UBRR2H)
        outb(UBRR2H, (uint8_t) (sv >> 8));
#endif
        break;
#if defined(UBRR3L)
    case 3:
        outb(UBRR3L, (uint8_t) sv);
#if defined(UBRR3H)
        outb(UBRR3H, (uint8_t) (sv >> 8));
#endif
        break;
#endif
#endif
    }

#else

    if (bit_is_clear(UCSR0C, UMSEL)) {
        if (bit_is_set(UCSR0A, U2X)) {
            rate <<= 2;
        } else {
            rate <<= 3;
        }
    }

#endif

    return 0;
}

static uint_fast8_t UsartGetDataBits(uint8_t ifn)
{
#if !defined(NUT_USE_UART0_ONLY)

    uint_fast8_t szn2 = 0;
    uint_fast8_t src = 0;

    switch (ifn) {
    case 0:
        szn2 = bit_is_set(UCSR0B, UCSZ02) != 0;
        src = inb(UCSR0C);
        break;
    case 1:
        szn2 = bit_is_set(UCSR1B, UCSZ12) != 0;
        src = inb(UCSR1C);
        break;
#ifdef UCSR2C
    case 2:
        szn2 = bit_is_set(UCSR2B, UCSZ22) != 0;
        src = inb(UCSR2C);
        break;
#ifdef UCSR3C
    case 3:
        szn2 = bit_is_set(UCSR3B, UCSZ32) != 0;
        src = inb(UCSR3C);
        break;
#endif
#endif
    }
    return szn2 ? 9 : ((src & 0x06) >> 1) + 5;

#else

    if (bit_is_set(UCSR0B, UCSZ02)) {
        return 9;
    }
    return ((inb(UCSR0C) & 0x06) >> 1) + 5;
#endif
}

static int UsartSetDataBits(uint8_t ifn, uint_fast8_t bits)
{
    if (bits == 9) {
        switch (ifn) {
        case 0:
            sbi(UCSR0B, UCSZ02);
            break;
        case 1:
            sbi(UCSR1B, UCSZ12);
            break;
#ifdef UCSZ22
        case 2:
            sbi(UCSR2B, UCSZ22);
            break;
#ifdef UCSZ32
        case 3:
            sbi(UCSR3B, UCSZ32);
            break;
#endif
#endif
        default:
            return -1;
        }
        bits = 5;
    }
    if (bits >= 5 && bits <= 8) {
        bits = (bits - 5) << 1;

        switch (ifn) {
        case 0:
            outb(UCSR0C, (inb(UCSR0C) & ~(_BV(UCSZ00) | _BV(UCSZ01))) | bits);
            break;
        case 1:
            outb(UCSR1C, (inb(UCSR1C) & ~(_BV(UCSZ10) | _BV(UCSZ11))) | bits);
            break;
#if defined(UCSZ20) && defined(UCSZ21)
        case 2:
            outb(UCSR2C, (inb(UCSR2C) & ~(_BV(UCSZ20) | _BV(UCSZ21))) | bits);
            break;
#if defined(UCSZ30) && defined(UCSZ31)
        case 3:
            outb(UCSR3C, (inb(UCSR3C) & ~(_BV(UCSZ30) | _BV(UCSZ31))) | bits);
            break;
#endif
#endif
        default:
            return -1;
        }
    }
    return 0;
}

static uint_fast8_t UsartGetParity(uint8_t ifn)
{
    uint_fast8_t rc = 0;

    switch (ifn) {
    case 0:
        rc = inb(UCSR0C) & (_BV(UPM00) | _BV(UPM01));
        break;
    case 1:
        rc = inb(UCSR1C) & (_BV(UPM10) | _BV(UPM11));
        break;
#if defined(UPM20) && defined(UPM21)
    case 2:
        rc = inb(UCSR2C) & (_BV(UPM20) | _BV(UPM21));
        break;
#if defined(UPM30) && defined(UPM31)
    case 3:
        rc = inb(UCSR3C) & (_BV(UPM30) | _BV(UPM31));
        break;
#endif
#endif
    }
    rc >>= 4;
    if (rc == 3) {
        rc = 1;
    }
    return rc;
}

static int UsartSetParity(uint8_t ifn, uint_fast8_t mode)
{
    if (mode <= 2) {
        if (mode == 1) {
            mode = 3;
        }
        mode <<= 4;
        switch (ifn) {
        case 0:
            outb(UCSR0C, (inb(UCSR0C) & ~(_BV(UPM00) | _BV(UPM01))) | mode);
            break;
        case 1:
            outb(UCSR1C, (inb(UCSR1C) & ~(_BV(UPM10) | _BV(UPM11))) | mode);
            break;
#if defined(UPM20) && defined(UPM21)
        case 2:
            outb(UCSR2C, (inb(UCSR2C) & ~(_BV(UPM20) | _BV(UPM21))) | mode);
            break;
#if defined(UPM30) && defined(UPM31)
        case 3:
            outb(UCSR3C, (inb(UCSR3C) & ~(_BV(UPM30) | _BV(UPM31))) | mode);
            break;
#endif
#endif
        }
        return 0;
    }
    return -1;
}

static uint8_t UsartGetStopBits(uint8_t ifn)
{
    uint8_t rc = 0;

    switch (ifn) {
    case 0:
        rc = bit_is_set(UCSR0C, USBS0) != 0;
        break;
    case 1:
        rc = bit_is_set(UCSR1C, USBS1) != 0;
        break;
#if defined(USBS2)
    case 2:
        rc = bit_is_set(UCSR2C, USBS2) != 0;
        break;
#if defined(USBS2)
    case 3:
        rc = bit_is_set(UCSR3C, USBS3) != 0;
        break;
#endif
#endif
    }
    return rc + 1;
}

static int UsartSetStopBits(uint8_t ifn, uint8_t bits)
{
    if (bits == 1) {
        switch (ifn) {
        case 0:
            cbi(UCSR0C, USBS0);
            break;
        case 1:
            cbi(UCSR1C, USBS1);
            break;
#if defined(USBS2)
        case 2:
            cbi(UCSR2C, USBS2);
            break;
#if defined(USBS3)
        case 3:
            cbi(UCSR3C, USBS3);
            break;
#endif
#endif
        default:
            return -1;
        }
    }
    else if (bits == 2) {
        switch (ifn) {
        case 0:
            sbi(UCSR0C, USBS0);
            break;
        case 1:
            sbi(UCSR1C, USBS1);
            break;
#if defined(USBS2)
        case 2:
            sbi(UCSR2C, USBS2);
            break;
#if defined(USBS3)
        case 3:
            sbi(UCSR3C, USBS3);
            break;
#endif
#endif
        default:
            return -1;
        }
    }
    else {
        return -1;
    }
    return 0;
}

int AvrUsartControl(USARTCB_DCB *dcb, int req, void *conf)
{
    int rc = 0;
    uint32_t *u32vp = (uint32_t *) conf;
    AVRUSART_IFC *uif = (AVRUSART_IFC *) dcb->usart_hwif;
    int8_t ifn = uif->uifc_num;

    switch (req) {
    case UART_GETSPEED:
        *u32vp = UsartGetSpeed(ifn);
        break;
    case UART_SETSPEED:
        rc = UsartSetSpeed(ifn, *u32vp);
        break;

    case UART_GETDATABITS:
        *u32vp = UsartGetDataBits(ifn);
        break;
    case UART_SETDATABITS:
        rc = UsartSetDataBits(ifn, (uint_fast8_t) *u32vp);
        break;

    case UART_GETPARITY:
        *u32vp = UsartGetParity(ifn);
        break;
    case UART_SETPARITY:
        rc = UsartSetParity(ifn, (uint_fast8_t) *u32vp);
        break;

    case UART_GETSTOPBITS:
        *u32vp = UsartGetStopBits(ifn);
        break;
    case UART_SETSTOPBITS:
        rc = UsartSetStopBits(ifn, (uint_fast8_t) *u32vp);
        break;

    case UART_GETSTATUS:
        //*u32vp = dcb->usart_status(dcb, 0);
        break;
    case UART_SETSTATUS:
        //dcb->usart_status(dcb, *u32vp | _BV(31));
        break;

    case UART_GETFLOWCONTROL:
        *u32vp = dcb->usart_mode & (USART_MF_SENSEMASK | USART_MF_CONTROLMASK | USART_MF_XONXOFF);
        break;
    case UART_SETFLOWCONTROL:
        if (*u32vp & ~(dcb->usart_caps & (USART_MF_SENSEMASK | USART_MF_CONTROLMASK | USART_MF_XONXOFF))) {
            rc = -1;
        } else {
            dcb->usart_mode &= ~dcb->usart_caps;
            dcb->usart_mode |= *u32vp;
        }
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

