#ifndef _DEV_IRQREG_ARCH_CM3_LPC17xx_H_
#define _DEV_IRQREG_ARCH_CM3_LPC17xx_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

/*!
 * \verbatim
 * $Id: $
 * \endverbatim
 */

extern IRQ_HANDLER sig_USART0;      // USART 0
extern IRQ_HANDLER sig_USART1;      // USART 1
extern IRQ_HANDLER sig_USART2;      // USART 2
extern IRQ_HANDLER sig_USART3;      // USART 3
extern IRQ_HANDLER sig_USART4;      // USART 4

extern IRQ_HANDLER sig_RTC;         // Real Time Clock
extern IRQ_HANDLER sig_WDT;         // Watchdog Timer
extern IRQ_HANDLER sig_MCI;         // Multimedia Card Interface / SDIO Interface
extern IRQ_HANDLER sig_EMAC;        // Ethernet MAC
extern IRQ_HANDLER sig_GPIO;        // GPIO
extern IRQ_HANDLER sig_DMA;         // General Purpose DMA controller
extern IRQ_HANDLER sig_EINT0;       // External interrupt 0
extern IRQ_HANDLER sig_EINT1;       // External interrupt 1
extern IRQ_HANDLER sig_EINT2;       // External interrupt 2
extern IRQ_HANDLER sig_EINT3;       // External interrupt 3

#define sig_INTERRUPT0 sig_EINT0
#define sig_INTERRUPT1 sig_EINT1
#define sig_INTERRUPT2 sig_EINT2
#define sig_INTERRUPT3 sig_EINT3

#endif
