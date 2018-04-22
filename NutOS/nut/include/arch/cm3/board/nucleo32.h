/*
 * Copyright 2014, 2016 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/board/nucleo.h
 * \brief STM32 Nucleo board specific settings.
 *
 * \verbatim
 * $Id: nucleo32.h 6611 2017-02-15 15:37:00Z u_bonnes $
 * \endverbatim
 */

#include <cfg/arch.h>

#ifndef DEV_UART
#define DEV_UART devUsartStm32_2
#include <dev/usartstm32.h>
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME  devUsartStm32_2.dev_name
#endif

#ifndef DEV_DEBUG
#define DEV_DEBUG devUsartStm32_2
#endif
#ifndef DEV_DEBUG_NAME
#define DEV_DEBUG_NAME  devUsartStm32_2.dev_name
#endif

#ifndef DEV_CONSOLE
#define DEV_CONSOLE devUsartStm32_2
#endif

#ifndef DEV_CONSOLE_NAME
#define DEV_CONSOLE_NAME devUsartStm32_2.dev_name
#endif

#ifndef DEV_I2CBUS
#define DEV_I2CBUS i2cBus1Stm32
#include <dev/i2cbus_stm32.h>
#endif

#ifndef DEV_SPIBUS
#define DEV_SPIBUS spiBus1Stm32Cb
#include <dev/spibus_stm32.h>
#endif

/* LED3 is Arduino D12 PB3*/
#ifndef LED1_PORT
#  define LED1_PORT NUTGPIO_PORTB
#endif

#ifndef LED1_PIN
#  define LED1_PIN 3
#endif

#define HAS_ARDUINO_CONNECTOR
