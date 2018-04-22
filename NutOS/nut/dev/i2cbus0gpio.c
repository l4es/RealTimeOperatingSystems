/*
 * Copyright (C) 2013 Uwe Bonnes(bon@elelktron.ikp.physik.tu-darmstadt.de
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
 * \file dev/i2cbus0gpio.c
 * \brief I2C bus 0 for GPIO declaration file.
 *
 * \verbatim
 * $Id: i2cbus0gpio.c 5327 2013-09-18 09:33:55Z u_bonnes $
 * \endverbatim
 */

/*!
 * \brief I2C bus driver for GPIO hardware.
 *
 * This is an polling driver, which supports master mode only.
 */

#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <stdlib.h>

#include <dev/gpio.h>
#include <dev/i2cbus_gpio.h>
#include <cfg/twi.h>
/*!
 * \addtogroup xgI2cBusGPIO
 */
/*@{*/

#if defined(GPIO0_SDA_PORT) && defined(GPIO0_SDA_PIN)
#undef GPIO_ID
#define GPIO_ID GPIO0_SDA_PORT
#include <cfg/arch/piotran.h>
static INLINE void I2C_SDA_INIT(void) { GPIO_INIT(GPIO0_SDA_PIN); GPIO_PULLUP_ON(GPIO0_SDA_PIN); }
static INLINE void I2C_SDA_LO(void) { GPIO_SET_LO(GPIO0_SDA_PIN); GPIO_OUTPUT(GPIO0_SDA_PIN); }
static INLINE void I2C_SDA_HI(void) { GPIO_INPUT(GPIO0_SDA_PIN); GPIO_SET_HI(GPIO0_SDA_PIN); }
static INLINE int I2C_SDA_GET(void) { return GPIO_GET(GPIO0_SDA_PIN); }
#else
#define I2C_SDA_INIT()
#define I2C_SDA_LO()
#define I2C_SDA_HI()
#define I2C_SDA_GET() 0
#endif

#if defined(GPIO0_SCL_PORT) && defined(GPIO0_SCL_PIN)
#undef GPIO_ID
#define GPIO_ID GPIO0_SCL_PORT
#include <cfg/arch/piotran.h>
static INLINE void I2C_SCL_INIT(void) { GPIO_INIT(GPIO0_SCL_PIN); GPIO_PULLUP_ON(GPIO0_SCL_PIN); }
static INLINE void I2C_SCL_LO(void) { GPIO_SET_LO(GPIO0_SCL_PIN); GPIO_OUTPUT(GPIO0_SCL_PIN);}
static INLINE void I2C_SCL_HI(void) { GPIO_INPUT(GPIO0_SCL_PIN); GPIO_SET_HI(GPIO0_SCL_PIN); }
static INLINE int I2C_SCL_GET(void) { return GPIO_GET(GPIO0_SCL_PIN); }
#else
#define I2C_SCL_INIT()
#define I2C_SCL_LO()
#define I2C_SCL_HI()
#define I2C_SCL_GET() 0
#endif

static int TwiBusTran(NUTI2C_SLAVE *slave, NUTI2C_MSG *msg);

static int TwiBusConf(NUTI2C_BUS *bus);

static int TwiBusInit(NUTI2C_BUS *bus);

static int TwiBusProbe(NUTI2C_BUS *bus, int sla);

static GPIO_TWICB twi0cb = {
    0                 /* Delay unit in us*/
};


NUTI2C_BUS i2cBus0Gpio = {
    &twi0cb,    /* bus_icb */
    TwiBusInit, /* bus_init */
    TwiBusConf, /* bus_configure */
    TwiBusProbe,/* bus_probe */
    TwiBusTran, /* bus_transceive */
    100,        /* bus_timeout */
    0,          /* bus_rate */
    0,          /* bus_flags */
    NULL        /* bus_mutex */
};

#include "i2cbus_gpio.c"
