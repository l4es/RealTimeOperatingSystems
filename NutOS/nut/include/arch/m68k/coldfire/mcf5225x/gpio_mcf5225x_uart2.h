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

#include <cfg/uart.h>

/*
 * Default Peripheral Configuration
 */
#ifndef UART2_TXD_PORTPIN
#define UART2_TXD_PORTPIN       PUC0
#endif

#ifndef UART2_RXD_PORTPIN
#define UART2_RXD_PORTPIN       PUC1
#endif

/*
 * Peripheral GPIO Configuration
 */
#if UART2_TXD_PORTPIN == PUC0
#define UART2_TXD_PORT            PORTUC
#define UART2_TXD_PIN             0
#define UART2_TXD_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#elif UART2_TXD_PORTPIN == PUB2
#define UART2_TXD_PORT            PORTUB
#define UART2_TXD_PIN             2
#define UART2_TXD_PERIPHERAL      GPIO_CFG_PERIPHERAL2
#elif UART2_TXD_PORTPIN == PAS0
#define UART2_TXD_PORT            PORTAS
#define UART2_TXD_PIN             0
#define UART2_TXD_PERIPHERAL      GPIO_CFG_PERIPHERAL2
#else
#warning "Illegal UART2 TXD pin assignement"
#endif

#if UART2_RXD_PORTPIN == PUC1
#define UART2_RXD_PORT            PORTUC
#define UART2_RXD_PIN             1
#define UART2_RXD_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#elif UART2_RXD_PORTPIN == PUB3
#define UART2_RXD_PORT            PORTUB
#define UART2_RXD_PIN             3
#define UART2_RXD_PERIPHERAL      GPIO_CFG_PERIPHERAL2
#elif UART2_RXD_PORTPIN == PAS1
#define UART2_RXD_PORT            PORTAS
#define UART2_RXD_PIN             1
#define UART2_RXD_PERIPHERAL      GPIO_CFG_PERIPHERAL2
#else
#warning "Illegal UART2 RXD pin assignement"
#endif

#ifdef UART2_RTS_PORTPIN
#if UART2_RTS_PORTPIN == PUC2
#define UART2_RTS_PORT            PORTUC
#define UART2_RTS_PIN             2
#define UART2_RTS_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#else
#warning "Illegal UART2 RTS pin assignement"
#endif
#endif

#ifdef UART2_CTS_PORTPIN
#if UART2_CTS_PORTPIN == PUC3
#define UART2_CTS_PORT            PORTUC
#define UART2_CTS_PIN             3
#define UART2_CTS_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#else
#warning "Illegal UART2 CTS pin assignement"
#endif
#endif
