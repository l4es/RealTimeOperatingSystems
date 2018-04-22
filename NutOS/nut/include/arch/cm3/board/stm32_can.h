/*
 * Copyright 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/board/stm32_can.h
 * \brief STM32_CAN board specific settings.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */
#ifndef LED1_PORT
#define LED1_PORT NUTGPIO_PORTA
#endif
#ifndef LED1_PIN
#define LED1_PIN 8
#endif

#ifndef LED2_PORT
#define LED2_PORT NUTGPIO_PORTC
#endif
#ifndef LED2_PIN
#define LED2_PIN 8
#endif

#ifndef LED3_PORT
#define LED3_PORT NUTGPIO_PORTC
#endif
#ifndef LED3_PIN
#define LED3_PIN 7
#endif

#ifndef LED4_PORT
#define LED4_PORT NUTGPIO_PORTB
#endif
#ifndef LED4_PIN
#define LED4_PIN 12
#endif

#ifndef OWI_PORT
#define OWI_PORT NUTGPIO_PORTA
#endif
#ifndef OWI_PIN
#define OWI_PIN  9
#endif

#ifndef OWI_UART
#define OWI_UART devUsartStm32_1
#endif

#ifndef DEV_UART
#define DEV_UART devUsartStm32_2
#include <dev/usartstm32.h>
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME  "uart2"
#endif

#ifndef DEV_TWIBUS
#define DEV_TWIBUS Stm32TwiBus_2
#endif

#if !defined(JTAG0_TDO_PIO_ID)
#define JTAG0_TDO_PIO_ID    NUTGPIO_PORTC
#endif

#if !defined(JTAG0_TDO_PIO_BIT)
#define JTAG0_TDO_PIO_BIT   11
#endif

#if !defined(JTAG0_TDI_PIO_ID)
#define JTAG0_TDI_PIO_ID    NUTGPIO_PORTC
#endif

#if !defined(JTAG0_TDI_PIO_BIT)
#define JTAG0_TDI_PIO_BIT   12
#endif

#if !defined(JTAG0_TMS_PIO_ID)
#define JTAG0_TMS_PIO_ID    NUTGPIO_PORTB
#endif

#if !defined(JTAG0_TMS_PIO_BIT)
#define JTAG0_TMS_PIO_BIT   15
#endif

#if !defined(JTAG0_TCK_PIO_ID)
#define JTAG0_TCK_PIO_ID    NUTGPIO_PORTC
#endif

#if !defined(JTAG0_TCK_PIO_BIT)
#define JTAG0_TCK_PIO_BIT   10
#endif

#if !defined(JTAG0_NTRST_PIO_ID)
#define JTAG0_NTRST_PIO_ID    NUTGPIO_PORTC
#endif

#if !defined(JTAG0_NTRST_PIO_BIT)
#define JTAG0_NTRST_PIO_BIT   9
#endif

#if !defined(JTAG0_CLOCK_RATE)
#define JTAG0_CLOCK_RATE    100000
#endif

#if !defined(DEF_JTAG_CABLE)
#define DEF_JTAG_CABLE jtag_gpio0
#include <dev/jtag_gpio.h>
#endif

#ifndef DEV_I2CBUS
#if 1
#define DEV_I2CBUS i2cBus2Stm32
#include <dev/i2cbus_stm32.h>
#else
#define DEV_I2CBUS i2cBus0Gpio
#include <dev/i2cbus_gpio.h>
#endif
#endif
