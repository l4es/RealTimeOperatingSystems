/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/avr/board/at90usbkey.h
 * \brief  AT90USBKEY board specific settings.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#ifndef _DEV_BOARD_H_
#error "Do not include this file directly. Use dev/board.h instead!"
#endif

#ifndef DEV_UART
#define DEV_UART devUsartAvr1
#endif

#ifndef DEV_UART_NAME
#define DEV_UART_NAME devUsartAvr1.dev_name
#endif

#ifndef DEV_DEBUG
#define DEV_DEBUG devDebug1
#endif

#ifndef DEV_DEBUG_NAME
#define DEV_DEBUG_NAME devDebug1.dev_name
#endif

#ifndef LED1_PORT
#define LED1_PORT NUTGPIO_PORTD
#endif

#ifndef LED1_PIN
#define LED1_PIN 4
#endif

#ifndef LED2_PORT
#define LED2_PORT NUTGPIO_PORTD
#endif

#ifndef LED2_PIN
#define LED2_PIN 5
#endif

#ifndef LED3_PORT
#define LED3_PORT NUTGPIO_PORTD
#endif

#ifndef LED3_PIN
#define LED3_PIN 6
#endif

#ifndef LED4_PORT
#define LED4_PORT NUTGPIO_PORTD
#endif

#ifndef LED4_PIN
#define LED4_PIN 7
#endif


