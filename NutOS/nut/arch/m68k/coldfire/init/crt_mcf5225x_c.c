/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#include <stdio.h>
#include <stdint.h>
#include <arch/m68k.h>

void InitClock(void)
{
    // JS TODO .. configurable from Nut/OS Configurator or Board specific function
    // This fixed configuration is used for SM2-MU board

    /* Turn on external oscillator in external crystal mode. */
    MCF_CLOCK_OCLR = MCF_CLOCK_OCLR_REFS | MCF_CLOCK_OCLR_OSCEN;

    /* Select closk source. */
    MCF_CLOCK_CCLR = 0x00;

    /* Disable on-chip oscilator. */
    MCF_CLOCK_OCHR = 0x00;

    /*
     * Configure PLL dividers:
     *  - PLL pre-divider = 48MHz / (CCHR + 1) = 8MHz
     *  - MFD, RFD = x10 .. 8*10 = 80MHz (system clock)
     */
    MCF_CLOCK_CCHR = 0x05;
    MCF_CLOCK_SYNCR |= MCF_CLOCK_SYNCR_MFD(3) | MCF_CLOCK_SYNCR_RFD(0);

    /*
     * PLL output clock drives the system clock.
     * Set operating mode to 1.
     * Enable PLL.
     */
    MCF_CLOCK_SYNCR |= MCF_CLOCK_SYNCR_CLKSRC | MCF_CLOCK_SYNCR_PLLMODE | MCF_CLOCK_SYNCR_PLLEN;

    /* Wait until the PLL is locked. */
    while (!(MCF_CLOCK_SYNSR & MCF_CLOCK_SYNSR_LOCK))
        ;

    /* Enable Real-Time Clock Control Register to enable RTC oscillator. */
    // JS TODO: Move to RTC driver
    // MCF_CLOCK_RTCCR = MCF_CLOCK_RTCCR_EXTALEN | MCF_CLOCK_RTCCR_OSCEN | MCF_CLOCK_RTCCR_REFS | MCF_CLOCK_RTCCR_RTCSEL;
}

void InitIntramAccess(void)
{
    extern void *__rambar;

    /* Enable on-chip modules (DMA, FEC, USB) to access internal SRAM. */
    MCF_SCM_RAMBAR = (0 | MCF_SCM_RAMBAR_BA((uint32_t) &__rambar) | MCF_SCM_RAMBAR_BDE);
}

void InitExtram(void)
{
    /*
     * External memory may be connected using several ways.
     * Call board specific Extram initialization.
     */
    extern void BoardInitExtram(void);
    BoardInitExtram();
}

