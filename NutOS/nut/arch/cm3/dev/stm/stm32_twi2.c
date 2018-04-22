/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 * \verbatim
 * $Id: stm32_twi2.c 6219 2015-10-07 08:42:51Z u_bonnes $
 * \endverbatim
 */

/*!
 * \file arch/cm3/dev/stm/stm32_twi2.c
 * \brief STM32F I2C bus 1 initialization
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/twi.h>
#include <cfg/arch/gpio.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/twif.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_i2c_pinmux.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_twi.h>

#ifdef I2CBUS2_DEFAULT_SPEED
#define I2C_DEFAULT_SPEED
#else
#endif

int Stm32I2cBus2Recover( void)
{
    uint_fast8_t i;

    GPIO_TypeDef * sda_port, *scl_port;
    uint8_t sda_pin, scl_pin;

    sda_port = stm32_port_nr2gpio[I2C2_SDA >> 8];
    scl_port = stm32_port_nr2gpio[I2C2_SCL >> 8];
    sda_pin = I2C2_SDA & 0xf;
    scl_pin = I2C2_SCL & 0xf;

    /* Handle pins as GPIOs, set SCL low */
    Stm32GpioConfigSet(I2C2_SDA, GPIO_CFG_OUTPUT|GPIO_CFG_MULTIDRIVE, 0);
    Stm32GpioConfigSet(I2C2_SCL, GPIO_CFG_OUTPUT|GPIO_CFG_MULTIDRIVE, 0);
    GpioPinSetLow((uint32_t)sda_port, sda_pin);
    NutMicroDelay(10);

    /* Run sequence of 8 SCL clocks */
    for( i=0; i<8; i++) {
        GpioPinSetLow((uint32_t)scl_port, scl_pin);
        NutMicroDelay(10);
        GpioPinSetHigh((uint32_t)scl_port, scl_pin);
        NutMicroDelay(10);
    }

    /* Issue Stop condition on the bus */
    GpioPinSetHigh((uint32_t)sda_port, sda_pin);
    NutMicroDelay(10);

    Stm32GpioConfigSet(I2C2_SDA,
                       GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_MULTIDRIVE,
                       I2C2_SDA_AF);
    Stm32GpioConfigSet(I2C2_SCL,
                       GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_MULTIDRIVE,
                       I2C2_SCL_AF);

    return 0;
}

/*!
 * \brief Processor specific Hardware Initiliaization
 *
 */
int Stm32I2cBus2Init(void)
{
    /* Reset I2C Bus 2 IP */
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C2RST;
    /* Enable I2C Bus 2 peripheral clock. */
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

    /* Setup Related GPIOs.
     * On non-F1 parts, pins may be mapped to different ports!
     */
     /* Setup Related GPIOs. */
    Stm32GpioConfigSet( I2C2_SDA,
                        GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL
                        | GPIO_CFG_MULTIDRIVE |GPIO_CFG_INIT_HIGH, I2C2_SDA_AF);
    Stm32GpioConfigSet( I2C2_SCL,
                        GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL
                        | GPIO_CFG_MULTIDRIVE |GPIO_CFG_INIT_HIGH, I2C2_SCL_AF);
    Stm32GpioConfigSet( I2C2_SMBA,
                        GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL
                        | GPIO_CFG_MULTIDRIVE |GPIO_CFG_INIT_HIGH, I2C2_SMBA_AF);
    NVIC_SetPriorityGrouping(4);
    NVIC_SetPriority( I2C2_EV_IRQn, 0);
    NVIC_SetPriority( I2C2_ER_IRQn, 1);

#if defined(I2CBUS2_USE_DMA)
    DMA_Init();
    DMA_Disable(I2C2_DMA_CHANNEL_TX);
    DMA_Disable(I2C2_DMA_CHANNEL_RX);
#endif

    return 0;
}

/*!
 * \brief TWI/I2C bus structure.
 */
NUTTWIBUS Stm32TwiBus_2 = {
  /*.bus_base =    */  I2C2_BASE,              /* Bus base address. */
  /*.bus_sig_ev =  */ &sig_TWI2_EV,            /* Bus data and event interrupt handler. */
  /*.bus_sig_er =  */ &sig_TWI2_ER,            /* Bus error interrupt handler. */
  /*.bus_mutex =   */  NULL,                   /* Bus lock queue. */
  /*.bus_icb   =   */  NULL,                   /* Bus Runtime Data Pointer */
#if defined(I2CBUS1_USE_DMA)
  /*.bus_dma_tx =  */  I2C2_DMA_CHANNEL_TX,    /* DMA channel for TX direction. */
  /*.bus_dma_rx =  */  I2C2_DMA_CHANNEL_RX,    /* DMA channel for RX direction. */
#else
  /*.bus_dma_tx =  */  0,
  /*.bus_dma_rx =  */  0,
#endif
  /*.bus_initbus = */  Stm32I2cBus2Init,       /* Initialize bus controller. */
  /*.bus_recover = */  Stm32I2cBus2Recover,    /* Recover bus in case a slave hangs with SCL low */
};

