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
 * $Id: nucleo.h 6636 2017-05-08 11:36:20Z u_bonnes $
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

#ifndef DEV_MMCARD
/* Assume an Ethernet_Shield_2 attached, with ICSP SPI
 * inputs handwired to Arduino SPI lines
 */
# include <dev/spi_mmc_gpio.h>
# define DEV_MMCARD devSpiMmcGpio
#endif

/* LED1 collides with Chip select for VL6180 X-Nucleo-IJP01A1 !*/
/* LED1 is PB13 on NUCLEO_F303R8 and PA5 else! */
#ifndef LED1_PORT
# if defined(STM32F302x8)
#  define LED1_PORT NUTGPIO_PORTB
# else
#  define LED1_PORT NUTGPIO_PORTA
# endif
#endif

#ifndef LED1_PIN
# if defined(STM32F302x8)
#  define LED1_PIN 13
# else
#  define LED1_PIN  5
# endif
#endif

/* Blue user button */
#ifndef SW1_PORT
# define SW1_PORT NUTGPIO_PORTC
#endif
#ifndef SW1_PIN
# define SW1_PIN  13
#endif

#ifndef SW1_ACTIVE_LOW
# define SW1_ACTIVE_LOW
#endif

#define HAS_ARDUINO_CONNECTOR
