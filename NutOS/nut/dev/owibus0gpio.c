/*
 * Copyright (C) 2013, 2016 by Uwe Bonnes
 *                              (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/*!
 * \file dev/owibus0gpio.c
 * \brief OWI bus 0 for GPIO declaration file.
 *
 * \verbatim
 * $Id: owibus0gpio.c 6532 2016-09-07 10:21:31Z u_bonnes $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/timer.h>

#include <dev/gpio.h>
#include <dev/owibus.h>

#if defined(OWI0_PORT) && defined(OWI0_PIN)
#define OWI_HW
#undef GPIO_ID
#define GPIO_ID OWI0_PORT
#include <cfg/arch/piotran.h>
static INLINE void OWI_INIT(void) { GPIO_INIT(OWI0_PIN); GPIO_PULLUP_ON(OWI0_PIN); }
static INLINE void OWI_LO(void) { GPIO_SET_LO(OWI0_PIN); GPIO_OUTPUT(OWI0_PIN); }
static INLINE void OWI_HI(void) { GPIO_INPUT(OWI0_PIN); GPIO_SET_HI(OWI0_PIN); }
static INLINE int  OWI_GET(void) { return GPIO_GET(OWI0_PIN); }
#else
#define OWI_INIT()
#define OWI_LO()
#define OWI_HI()
#define OWI_GET() 0
#endif

static int Gpio_Setup(NUTOWIBUS *bus);
static int Gpio_OwiTouchReset(NUTOWIBUS *bus);
static int Gpio_OwiReadBlock(NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len);
static int Gpio_OwiWriteBlock(NUTOWIBUS *bus, const uint8_t *data,
                              uint_fast8_t len);

/*!
 * \brief Library compile time configured OWI bus driver for GPIO.
 *
 */
 NUTOWIBUS owiBus0Gpio = {
    0,                   /*!< \brief OWIBUSBUS::owibus_info */
    OWI_MODE_NORMAL,     /*!< \brief OWIBUSBUS::mode */
    Gpio_Setup,         /*!< \brief OWIBUSBUS::OwiSetup */
    Gpio_OwiTouchReset, /*!< \brief OWIBUSBUS::OwiTouchReset*/
    Gpio_OwiReadBlock,  /*!< \brief OWIBUSBUS::OwiReadBlock */
    Gpio_OwiWriteBlock  /*!< \brief OWIBUSBUS::OwiWriteBlock */
 };
/*@}*/
#include "owibus_gpio.c"
