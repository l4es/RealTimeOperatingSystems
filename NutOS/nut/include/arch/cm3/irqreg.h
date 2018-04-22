
#ifndef _DEV_IRQREG_CM3_H_
#define _DEV_IRQREG_CM3_H_

/*
 * Copyright (C) 2001-2007 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2013 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de).
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
 * $Log$
 *
 * Revision 1.00  2010/08/10 16:48:28  ulrichprinz
 * Initial version
 *
 *
 */

#if defined(__CORTEX__)

#if defined(SAM3U)
#include <arch/cm3/atmel/sam3u_irqreg.h>

#elif defined(MCU_STM32)
#include <arch/cm3/stm/stm32_irqreg.h>

#elif defined(MCU_LPC176x) || defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc17xx_irqreg.h>

#else
#warning "Unknown CortexM3 MCU defined"
#endif

#else
#warning "No CortexM3 MCU defined"
#endif

#ifdef NUT_PERFMON

#define CREATE_HANDLER(IRQ_SIG, DEV, PRIORITY)                          \
                                                                        \
static int IRQ_SIG## IrqCtl(int cmd, void *param);                      \
                                                                        \
IRQ_HANDLER sig_ ##IRQ_SIG = {                                          \
    0,                          /* Interrupt counter, ir_count. */      \
    NULL,                       /* Passed argument, ir_arg. */          \
    NULL,                       /* Handler subroutine, ir_handler. */   \
    IRQ_SIG## IrqCtl                /* Interrupt control, ir_ctl. */    \
};                                                                      \
static void IRQ_SIG## IrqEntry(void *arg);                              \
void IRQ_SIG## IrqEntry(void *arg)                                      \
{                                                                       \
    sig_ ##IRQ_SIG.ir_count++;                                          \
    if (sig_ ##IRQ_SIG.ir_handler) {                                    \
        (sig_ ##IRQ_SIG.ir_handler) (sig_ ##IRQ_SIG.ir_arg);            \
    }                                                                   \
}                                                                       \
int IRQ_SIG## IrqCtl(int cmd, void* param)                              \
{                                                                       \
    return CM3_IrqCtl(cmd, param, DEV## _IRQn, & IRQ_SIG##IrqEntry, &sig_ ##IRQ_SIG, PRIORITY); \
}

#else /* NUT_PERFMON */

#define CREATE_HANDLER(IRQ_SIG, DEV, PRIORITY)                          \
                                                                        \
static int IRQ_SIG##IrqCtl(int cmd, void *param);                       \
                                                                        \
IRQ_HANDLER sig_ ##IRQ_SIG = {                                          \
    NULL,                       /* Passed argument, ir_arg. */          \
    NULL,                       /* Handler subroutine, ir_handler. */   \
    IRQ_SIG## IrqCtl                /* Interrupt control, ir_ctl. */    \
};                                                                      \
static void IRQ_SIG## IrqEntry(void *arg);                              \
void IRQ_SIG## IrqEntry(void *arg)                                      \
{                                                                       \
    if (sig_ ##IRQ_SIG.ir_handler) {                                    \
        (sig_ ##IRQ_SIG.ir_handler) (sig_ ##IRQ_SIG.ir_arg);            \
    }                                                                   \
}                                                                       \
int IRQ_SIG## IrqCtl(int cmd, void* param)                              \
{                                                                       \
    return CM3_IrqCtl(cmd, param, DEV## _IRQn, & IRQ_SIG##IrqEntry, &sig_ ##IRQ_SIG, PRIORITY); \
}

#endif

#endif
