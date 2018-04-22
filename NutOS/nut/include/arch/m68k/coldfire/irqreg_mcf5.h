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

#ifndef _DEV_IRQREG_H_
#error "Do not include this file directly. Use dev/irqreg.h instead!"
#endif

#if defined(MCU_MCF5225X)
#include <arch/m68k/coldfire/mcf5225x/irqreg_mcf5225x.h>
#elif defined(MCU_MCF51CN)
#include <arch/m68k/coldfire/mcf51cn/irqreg_mcf51cn.h>
#else
#warning "Unknown Coldfire MCU Family defined"
#endif

/*
 * MCF52259RM.pdf - 16.3.2 Interrupt Mask Registers (IMRHn, IMRLn)
 *
 * NOTE
 *
 * A spurious interrupt may occur if an interrupt source is being masked in the
 * interrupt controller mask register (IMR) or a module’s interrupt mask
 * register while the interrupt mask in the status register (SR[I]) is set to a value
 * lower than the interrupt’s level. This is because by the time the status
 * register acknowledges this interrupt, the interrupt has been masked. A
 * spurious interrupt is generated because the CPU cannot determine the
 * interrupt source.
 * To avoid this situation for interrupts sources with levels 1–6, first write a
 * higher level interrupt mask to the status register, before setting the mask in
 * the IMR or the module’s interrupt mask register. After the mask is set, return
 * the interrupt mask in the status register to its previous value. Because level
 * 7 interrupts cannot be disabled in the status register prior to masking, use of
 * the IMR or module interrupt mask registers to disable level 7 interrupts is
 * not recommended.
 */
#define PREVENT_SPURIOUS_INTERRUPT(code) {NutEnterCritical();{code;} NutExitCritical();}

