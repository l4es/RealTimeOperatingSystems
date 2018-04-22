/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#ifndef _DEV_GPIO_H_
#error "Do not include this file directly. Use dev/gpio.h instead!"
#endif

#include <cfg/twi.h>

/*
 * Default Peripheral Configuration
 */
#ifndef I2C0_SCL_PORTPIN
#define I2C0_SCL_PORTPIN        PAS0
#endif

#ifndef I2C0_SDA_PORTPIN
#define I2C0_SDA_PORTPIN     PAS1
#endif

/*
 * Peripheral GPIO Configuration
 */
#if I2C0_SCL_PORTPIN == PAS0
#define I2C0_SCL_PORT           PORTAS
#define I2C0_SCL_PIN            0
#define I2C0_SCL_PERIPHERAL     GPIO_CFG_PERIPHERAL0
#elif I2C0_SCL_PORTPIN == PQS2
#define I2C0_SCL_PORT           PORTQS
#define I2C0_SCL_PIN            2
#define I2C0_SCL_PERIPHERAL     GPIO_CFG_PERIPHERAL1
#else
#warning "Illegal I2C0 SCL pin assignement"
#endif

#if I2C0_SDA_PORTPIN == PAS1
#define I2C0_SDA_PORT           PORTAS
#define I2C0_SDA_PIN            1
#define I2C0_SDA_PERIPHERAL     GPIO_CFG_PERIPHERAL0
#elif I2C0_SDA_PORTPIN == PQS3
#define I2C0_SDA_PORT           PORTQS
#define I2C0_SDA_PIN            3
#define I2C0_SDA_PERIPHERAL     GPIO_CFG_PERIPHERAL1
#else
#warning "Illegal I2C0 SDA pin assignement"
#endif
