/*
 * Copyright (c) 2005-2007 proconX Pty Ltd <www.proconx.com>
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
 * \file arch/avr/board/xnut_100.c
 * \brief XNUT-100 board initialization.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/memory.h>
#include <cfg/arch/avr.h>

#include <toolchain.h>

#if defined(__GNUC__)
static void XnutInit(void) NUT_LINKER_SECT(".init1") NUT_NAKED_FUNC NUT_USED_FUNC;
#endif

static void XnutInit(void)
{
    PORTB = 0x35;
    DDRB  = 0x3F;
    PORTD = 0xE8;
    DDRD  = 0xB0;
    PORTE = 0x0E;
    DDRE  = 0x02;
    PORTF = 0xF0;
    DDRF  = 0x0F;
    PORTG = 0x1F;
    DDRG  = 0x07;

    ACSR |= _BV(ACD); /* Switch off analog comparator to reduce power consumption */

    /* Init I2C bus w/ 100 kHz */
    TWSR = 0;
    TWBR = (NUT_CPU_FREQ / 100000UL - 16) / 2; /* 100 kHz I2C */

    /* Set default baudrate */
#if NUT_CPU_FREQ == 14745600
    UBRR0L = (NUT_CPU_FREQ / (16 * 9600UL)) - 1;
    UBRR1L = (NUT_CPU_FREQ / (16 * 9600UL)) - 1;
#else
    sbi(UCSR0A, U2X0);
    sbi(UCSR1A, U2X1);
    UBRR0L = (NUT_CPU_FREQ / (8 * 9600UL)) - 1;
    UBRR1L = (NUT_CPU_FREQ / (8 * 9600UL)) - 1;
#endif
}

/*!
 * \brief Early XNUT-100 hardware initialization.
 */
void NutBoardInit(void)
{
#ifndef __GNUC__
    XnutInit();
#endif
}
