#ifndef _DEV_IRQREG_ARCH_CM3_SAM3U_H_
#define _DEV_IRQREG_ARCH_CM3_SAM3U_H_

/*
 * Copyright (C) 2001-2007 by egnite Software GmbH. All rights reserved.
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

extern IRQ_HANDLER sig_SUPC;    // supply controller
extern IRQ_HANDLER sig_RSTC;    // reset controller
extern IRQ_HANDLER sig_RTC;     // real time clock
extern IRQ_HANDLER sig_RTT;     // real time timer
extern IRQ_HANDLER sig_WDT;     // watchdog timer
extern IRQ_HANDLER sig_PMC;     // power management controller
extern IRQ_HANDLER sig_EEFC0;   // enhanced flash controller 0
extern IRQ_HANDLER sig_EEFC1;   // enhanced flash controller 1
extern IRQ_HANDLER sig_UART;    // debug port/uart
extern IRQ_HANDLER sig_SMC;     // static memory controller
extern IRQ_HANDLER sig_PIOA;    // gpio a
extern IRQ_HANDLER sig_PIOB;    // gpio b
extern IRQ_HANDLER sig_PIOC;    // gpio c
extern IRQ_HANDLER sig_USART0;  // usart0
extern IRQ_HANDLER sig_USART1;  // usart1
extern IRQ_HANDLER sig_USART2;  // usart2
extern IRQ_HANDLER sig_USART3;  // usart3
extern IRQ_HANDLER sig_HSMCI;   // high speed multimedia card interface
extern IRQ_HANDLER sig_TWI0;    // i2c 0
extern IRQ_HANDLER sig_TWI1;    // i2c 1
extern IRQ_HANDLER sig_SPI;     // spi controller
extern IRQ_HANDLER sig_SSC;     // syncronous serial controller
extern IRQ_HANDLER sig_TC0;     // Timer 0
extern IRQ_HANDLER sig_TC1;     // Timer 1
extern IRQ_HANDLER sig_TC2;     // Timer 2
extern IRQ_HANDLER sig_PWM;     // PWM controller
extern IRQ_HANDLER sig_ADC12B;  // ADC 12bit
extern IRQ_HANDLER sig_ADC;     // ADC 10bit
extern IRQ_HANDLER sig_DMAC;    // DMA controller
extern IRQ_HANDLER sig_UDPHS;   // Usb Device High Speed

#endif
