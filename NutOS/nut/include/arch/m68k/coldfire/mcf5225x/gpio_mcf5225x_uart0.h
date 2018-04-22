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
#ifndef UART0_TXD_PORTPIN
#define UART0_TXD_PORTPIN       PUA0
#endif

#ifndef UART0_RXD_PORTPIN
#define UART0_RXD_PORTPIN       PUA1
#endif

/*
 * Peripheral GPIO Configuration
 */
#if UART0_TXD_PORTPIN == PUA0
#define UART0_TXD_PORT            PORTUA
#define UART0_TXD_PIN             0
#define UART0_TXD_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#else
#warning "Illegal UART0 TXD pin assignement"
#endif

#if UART0_RXD_PORTPIN == PUA1
#define UART0_RXD_PORT            PORTUA
#define UART0_RXD_PIN             1
#define UART0_RXD_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#else
#warning "Illegal UART0 RXD pin assignement"
#endif

#ifdef UART0_RTS_PORTPIN
#if UART0_RTS_PORTPIN == PUA2
#define UART0_RTS_PORT            PORTUA
#define UART0_RTS_PIN             2
#define UART0_RTS_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#else
#warning "Illegal UART0 RTS pin assignement"
#endif
#endif

#ifdef UART0_CTS_PORTPIN
#if UART0_CTS_PORTPIN == PUA3
#define UART0_CTS_PORT            PORTUA
#define UART0_CTS_PIN             3
#define UART0_CTS_PERIPHERAL      GPIO_CFG_PERIPHERAL0
#else
#warning "Illegal UART0 CTS pin assignement"
#endif
#endif
