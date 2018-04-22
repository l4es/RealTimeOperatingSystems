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
 * \file arch/avr/board/mmnet.c
 * \brief MMnet board initialization.
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
static void MmnetInit(void) NUT_LINKER_SECT(".init1") NUT_NAKED_FUNC NUT_USED_FUNC;
#endif

/*
* MMnet02..04 and MMnet102..104 CPLD initialization.
*/
static void MmnetInit(void)
{
    volatile uint8_t *breg = (uint8_t *)((size_t)-1 & ~0xFF);

    *(breg + 1) = 0x01; // Memory Mode 1, Banked Memory

    /* Assume 14.745600 MHz crystal, set to 115200bps */
    outb(UBRR, 7);
    outb(UBRR1L, 7);
}

/*!
 * \brief Early MMnet hardware initialization.
 */
void NutBoardInit(void)
{
#ifndef __GNUC__
    MmnetInit();
#endif
}
