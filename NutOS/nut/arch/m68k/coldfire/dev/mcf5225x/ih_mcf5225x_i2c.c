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

IRQ_HANDLER sig_I2C0 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl0
    };

IRQ_HANDLER sig_I2C1 = {
#ifdef NUT_PERFMON
        0,
#endif
        NULL,
        NULL,
        IrqCtl1
    };

static int IrqCtl0(int cmd, void *param)
{
    return IrqCtlCommon(&sig_I2C0, cmd, param, &MCF_INTC_IMRL(0), &MCF_INTC_ICR17(0), MCF_INTC_IMRL_INT_MASK17, IPL_I2C0);
}

static int IrqCtl1(int cmd, void *param)
{
    return IrqCtlCommon(&sig_I2C1, cmd, param, &MCF_INTC_IMRH(0), &MCF_INTC_ICR62(0), MCF_INTC_IMRH_INT_MASK62, IPL_I2C1);
}

SIGNAL(IH_I2C0)
{
    MCF_I2C_I2SR(0) &= ~MCF_I2C_I2SR_IIF;
    CallHandler(&sig_I2C0);
}

SIGNAL(IH_I2C1)
{
    MCF_I2C_I2SR(1) &= ~MCF_I2C_I2SR_IIF;
    CallHandler(&sig_I2C1);
}
