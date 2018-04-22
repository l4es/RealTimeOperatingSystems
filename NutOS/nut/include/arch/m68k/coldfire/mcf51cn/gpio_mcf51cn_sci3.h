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

#ifndef _DEV_GPIO_H_
#error "Do not include this file directly. Use dev/gpio.h instead!"
#endif

#include <cfg/sci.h>

/*
 * Default Peripheral Configuration
 */
#ifndef SCI3_TXD_PORTPIN
#define SCI3_TXD_PORTPIN       PTE6
#endif

#ifndef SCI3_RXD_PORTPIN
#define SCI3_RXD_PORTPIN       PTE7
#endif

/*
 * Peripheral GPIO Configuration
 */
#if SCI3_TXD_PORTPIN == PTE6
#define SCI3_TXD_PORT            PORTE
#define SCI3_TXD_PIN             6
#define SCI3_TXD_PERIPHERAL      GPIO_CFG_ALT3
#elif SCI3_TXD_PORTPIN == PTA3
#define SCI3_TXD_PORT            PORTA
#define SCI3_TXD_PIN             3
#define SCI3_TXD_PERIPHERAL      GPIO_CFG_ALT2
#else
#warning "Illegal SCI3 TXD pin assignement"
#endif

#if SCI3_RXD_PORTPIN == PTE7
#define SCI3_RXD_PORT            PORTE
#define SCI3_RXD_PIN             7
#define SCI3_RXD_PERIPHERAL      GPIO_CFG_ALT3
#elif SCI3_RXD_PORTPIN == PTA4
#define SCI3_RXD_PORT            PORTA
#define SCI3_RXD_PIN             4
#define SCI3_RXD_PERIPHERAL      GPIO_CFG_ALT2
#else
#warning "Illegal SCI3 RXD pin assignement"
#endif
