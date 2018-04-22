/*
 * Copyright 2015 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/board/l4_discovery.h
 * \brief STM32L4_Discovery board specific settings.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#ifndef DEV_UART
#define DEV_UART devUsartStm32_2
#include <dev/usartstm32.h>
#endif

#ifndef DEV_UART_NAME
#define DEV_UART_NAME devUsartStm32_2.dev_name
#endif

#ifndef DEV_DISPLAY
# undef DEV_DISPLAY_NAME
# include <arch/cm3/stm/stm32_lcd16seg.h>
# define DEV_DISPLAY      devStm32Lcd16Seg
# define DEV_DISPLAY_NAME devStm32Lcd16Seg.dev_name
#endif

/* LD5/PE8: green*/
#ifndef LED1_PORT
# define LED1_PORT NUTGPIO_PORTE
#endif

#ifndef LED1_PIN
# define LED1_PIN 8
#endif

/* LD4/PB2: Red. */
#ifndef LED2_PORT
# define LED2_PORT NUTGPIO_PORTB
#endif

#ifndef LED2_PIN
# define LED2_PIN 2
#endif

#ifndef SW1_PORT
# define SW1_PORT NUTGPIO_PORTA
#endif

#ifndef SW1_PIN
# define SW1_PIN 0
#endif
