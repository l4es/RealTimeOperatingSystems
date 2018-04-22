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

/*
 * Interrupt level & priority setup
 *
 * IMPORTANT: Interrupt level and priority combination MUST be unique
 */
#define IPL_UART0   (MCF_INTC_ICR_IL(3) | MCF_INTC_ICR_IP(0))
#define IPL_UART1   (MCF_INTC_ICR_IL(3) | MCF_INTC_ICR_IP(1))
#define IPL_UART2   (MCF_INTC_ICR_IL(3) | MCF_INTC_ICR_IP(2))
#define IPL_I2C0    (MCF_INTC_ICR_IL(3) | MCF_INTC_ICR_IP(3))
#define IPL_I2C1    (MCF_INTC_ICR_IL(3) | MCF_INTC_ICR_IP(4))
#define IPL_PIT0    (MCF_INTC_ICR_IL(4) | MCF_INTC_ICR_IP(0))
#define IPL_PIT1    (MCF_INTC_ICR_IL(4) | MCF_INTC_ICR_IP(1))
#define IPL_CWD     (MCF_INTC_ICR_IL(7) | MCF_INTC_ICR_IP(7))

/*
 * Interrupt handlers
 */
extern IRQ_HANDLER sig_CWD;
extern IRQ_HANDLER sig_I2C0;
extern IRQ_HANDLER sig_I2C1;
extern IRQ_HANDLER sig_PIT0;
extern IRQ_HANDLER sig_PIT1;
extern IRQ_HANDLER sig_UART0;
extern IRQ_HANDLER sig_UART1;
extern IRQ_HANDLER sig_UART2;

/*
 * Common Interrupt control
 */
extern int IrqCtlCommon(IRQ_HANDLER *sig_handler, int cmd, void *param, volatile uint32_t *reg_imr, volatile uint8_t *reg_icr,
        uint32_t imr_mask, uint8_t ipl);

