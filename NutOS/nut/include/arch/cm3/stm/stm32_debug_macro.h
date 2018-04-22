#ifndef _STM32_DEBUG_MACRO_H_
#define _STM32_DEBUG_MACRO_H_

/*
 * Copyright (C) 2013 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

/*
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <cfg/cortex_debug.h>

#include <arch/cm3/stm/stm32xxxx.h>

#ifndef DEBUG_UART_NR
#define DEBUG_UART_NR USART1
#endif

#if defined(DEBUG_MACRO)

/*!
 * \brief Write out a character to the debug UART.
 *
 * \param ch    character to write
 */

#if defined(USART_RDR_RDR)
#define DebugPut(ch) {while (!(DEBUG_UART_NR->ISR & USART_ISR_TXE)); DEBUG_UART_NR->TDR = ((ch) & 0x1FF);}
#else
#define DebugPut(ch) {while (!(DEBUG_UART_NR->SR & USART_SR_TXE)); DEBUG_UART_NR->DR = ((ch) & 0x1FF);}
#endif

#else

#define DebugPut(ch) ;

#endif

#endif
