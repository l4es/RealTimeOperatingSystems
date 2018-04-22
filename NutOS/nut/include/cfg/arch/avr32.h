#ifndef _CFG_ARCH_AVR32_H_
#define _CFG_ARCH_AVR32_H_

/*
 * Copyright (C) 2004 by egnite Software GmbH. All rights reserved.
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
 * $Log: avr.h32,v $
 *
 */

/*!
 * \addtogroup xgConfigAvr32
 */
/*@{*/

/*!
 * \file include/cfg/arch/avr32.h
 * \brief AVR32 hardware configuration.
 */
#include <dev/gpio.h>

#define PIOA_ID     NUTGPIO_PORTA
#define PIOB_ID     NUTGPIO_PORTB
#define PIOC_ID     NUTGPIO_PORTC
#define PIOD_ID     NUTGPIO_PORTD
#define PIOE_ID     NUTGPIO_PORTE
#define PIOF_ID     NUTGPIO_PORTF
#define PIOG_ID     NUTGPIO_PORTG
#define PIOH_ID     NUTGPIO_PORTH
#define PIOI_ID     NUTGPIO_PORTI
#define PIOJ_ID     NUTGPIO_PORTJ
#define PIOK_ID     NUTGPIO_PORTK
#define PIOL_ID     NUTGPIO_PORTL

#define PIOA_BASE       AVR32_GPIO_ADDRESS                               /*!< \brief PIO A base address.->GPIO_ADDRESS */
#define PIOB_BASE       AVR32_GPIO_ADDRESS + AVR32_GPIO_PORT_LENGTH      /*!< \brief PIO B base address. */

/*@}*/

#include <cfg/arch/avr32pio.h>

#endif
