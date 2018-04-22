#ifndef _STM32_USART_H_
#define _STM32_USART_H_

/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 *
 */

/*
 * \verbatim
 * $Id: stm32_usart.h 6367 2016-03-15 10:27:09Z u_bonnes $
 * \endverbatim
 */

#include <arch/cm3/stm/stm32_clk.h>

/*
 * Nut/OS to STM32 Abstraction handler
 */

/*
 * Function prototypes.
 */
static uint32_t Stm32UsartGetSpeed(void);
static int Stm32UsartSetSpeed(uint32_t rate);
static uint8_t Stm32UsartGetDataBits(void);
static int Stm32UsartSetDataBits(uint8_t bits);
static uint8_t Stm32UsartGetParity(void);
static int Stm32UsartSetParity(uint8_t mode);
static uint8_t Stm32UsartGetStopBits(void);
static int Stm32UsartSetStopBits(uint8_t bits);
static uint32_t Stm32UsartGetFlowControl(void);
static int Stm32UsartSetFlowControl(uint32_t flags);
static uint32_t Stm32UsartGetStatus(void);
static int Stm32UsartSetStatus(uint32_t flags);
static uint8_t Stm32UsartGetClockMode(void);
static int Stm32UsartSetClockMode(uint8_t mode);
static void Stm32UsartTxStart(void);
static void Stm32UsartRxStart(void);
static int Stm32UsartInit(void);
static int Stm32UsartDeinit(void);

#endif
