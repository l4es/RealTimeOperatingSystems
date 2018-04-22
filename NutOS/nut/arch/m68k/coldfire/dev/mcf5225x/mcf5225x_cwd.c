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

#include <arch/m68k.h>
#include <dev/watchdog.h>
#include <dev/irqreg.h>
#include <sys/timer.h>

static ureg_t nested;

static void IrqHandler(void *arg)
{
    /* Reset */
    MCF_RCM_RCR |= MCF_RCM_RCR_SOFTRST;
}

uint32_t Mcf5225xWatchDogStart(uint32_t ms)
{
    int cwt;
    uint8_t shift[8] = { 9, 11, 13, 15, 19, 23, 27, 31 };

    /* Stop watchdog + Following steps must be taken to change CWT */
    MCF_SCM_CWCR &= ~MCF_SCM_CWCR_CWE;
    MCF_SCM_CWSR = 0x55;
    MCF_SCM_CWSR = 0xAA;

    /* Get Core Watchdog Timing */
    for (cwt = 0; cwt < 8; cwt++)
        if (((1 << shift[cwt]) / (NutGetCpuClock() / 1000)) > ms)
            break;

    /* Register interrupt handler */
    NutRegisterIrqHandler(&sig_CWD, IrqHandler, NULL);

    /* Erase pending interrupt if present */
    MCF_SCM_CWCR |= MCF_SCM_CWCR_CWTIF;

    /* Configure and enable watchdog */
    MCF_SCM_CWCR = MCF_SCM_CWCR_CWE | MCF_SCM_CWCR_CWT(cwt);
    nested = 1;

    return ((1 << shift[cwt]) / (NutGetCpuClock() / 1000));
}

void Mcf5225xWatchDogRestart(void)
{
    MCF_SCM_CWSR = 0x55;
    MCF_SCM_CWSR = 0xAA;
}

void Mcf5225xWatchDogDisable(void)
{
    if (nested)
        nested++;

    MCF_SCM_CWCR &= ~MCF_SCM_CWCR_CWE;
}

void Mcf5225xWatchDogEnable(void)
{
    if (nested > 1 && --nested == 1)
        MCF_SCM_CWCR |= MCF_SCM_CWCR_CWE;
}
