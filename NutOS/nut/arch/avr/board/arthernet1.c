/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/avr/board/arthernet1.c
 * \brief Arthernet 1 board initialization.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <stdint.h>
#include <cfg/memory.h>
#include <cfg/arch/avr.h>

#include <toolchain.h>

#if defined(__GNUC__)
static void ArthernetInit(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init1") NUT_USED_FUNC;
#endif

/*
 * Arthernet hardware setup.
 *
 * Note: This overwrites the default settings of NutInitXRAM()!
 * 0x1100-0x14FF  CLPD area  -> use 3 Waitstates for 0x1100-0x1FFF (no Limit at 0x1500 available)
 * 0x1500-0xFFFF  Heap/Stack -> use 1 Waitstate  for 0x2000-0xFFFF
 */
static void ArthernetInit(void)
{
    MCUCR  = _BV(SRE); /* enable xmem-Interface */
    XMCRA |= _BV(SRL0) | _BV(SRW01) | _BV(SRW00); /* sep. at 0x2000, 3WS for lower Sector */
    XMCRB = 0;

    *((volatile uint8_t *)(ARTHERCPLDSTART)) = 0x10; // arthernet cpld init - Bank
    *((volatile uint8_t *)(ARTHERCPLDSPI)) = 0xFF; // arthernet cpld init - SPI

    /* Assume standard Arthernet1 with 16 MHz crystal, set to 38400 bps */
    outb(UBRR, 25);
    outb(UBRR1L, 25);
}

/*!
 * \brief Early Arthernet 1 hardware initialization.
 */
void NutBoardInit(void)
{
#ifndef __GNUC__
    ArthernetInit();
#endif
}
