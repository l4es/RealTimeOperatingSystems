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
#include <dev/irqreg.h>

static int IrqCtl0(int cmd, void *param);
static int IrqCtl1(int cmd, void *param);

IRQ_HANDLER sig_PIT0 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl0
    };

IRQ_HANDLER sig_PIT1 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1
    };

static int IrqCtl0(int cmd, void *param)
{
    return IrqCtlCommon(&sig_PIT0, cmd, param, &MCF_INTC_IMRH(0), &MCF_INTC_ICR55(0), MCF_INTC_IMRH_INT_MASK55, IPL_PIT0);
}

static int IrqCtl1(int cmd, void *param)
{
    return IrqCtlCommon(&sig_PIT1, cmd, param, &MCF_INTC_IMRH(0), &MCF_INTC_ICR56(0), MCF_INTC_IMRH_INT_MASK56, IPL_PIT1);
}

SIGNAL(IH_PIT0)
{
    MCF_PIT_PCSR(0) |= MCF_PIT_PCSR_PIF;
    CallHandler(&sig_PIT0);
}

SIGNAL(IH_PIT1)
{
    MCF_PIT_PCSR(1) |= MCF_PIT_PCSR_PIF;
    CallHandler(&sig_PIT1);
}
