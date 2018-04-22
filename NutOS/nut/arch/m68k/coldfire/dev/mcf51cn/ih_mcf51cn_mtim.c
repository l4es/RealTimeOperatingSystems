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

static int IrqCtl1(int cmd, void *param);
static int IrqCtl2(int cmd, void *param);

IRQ_HANDLER sig_MTIM1 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1
    };

IRQ_HANDLER sig_MTIM2 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl2
    };

static int IrqCtl1(int cmd, void *param)
{
    return IrqCtlCommon(&sig_MTIM1, cmd, param, &MTIM1SC, MTIM1SC_TOIE_MASK);
}

static int IrqCtl2(int cmd, void *param)
{
    return IrqCtlCommon(&sig_MTIM2, cmd, param, &MTIM2SC, MTIM2SC_TOIE_MASK);
}

SIGNAL(IH_MTIM1)
{
    (void) MTIM1SC;
    CallHandler(&sig_MTIM1);
}

SIGNAL(IH_MTIM2)
{
    (void) MTIM2SC;
    CallHandler(&sig_MTIM2);
}
