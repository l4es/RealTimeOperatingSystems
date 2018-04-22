/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2011-2015 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: stm32_gpio_v2.c 6527 2016-09-07 10:21:10Z u_bonnes $
 * \endverbatim
 */

/* GPIO Configuration for the GPIO of L0/L1/L4/F0/F2/F3/F4/F7 */

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

#include <arch/cm3.h>
#include <dev/gpio.h>
#include <arch/cm3/stm/stm32_gpio.h>
#if defined (MCU_STM32L0)
const uint16_t ospeed_values[4] =
{2000000 >> 16,  2000000 >> 16, 10000000 >> 16,  50000000 >> 16};
#elif defined (MCU_STM32L1)
const uint16_t ospeed_values[4] =
{ 400000 >> 16,  2000000 >> 16, 10000000 >> 16,  50000000 >> 16};
#elif defined (MCU_STM32L4)
const uint16_t ospeed_values[4] =
{ 5000000 >> 16, 25000000 >> 16, 50000000 >> 16,  80000000 >> 16};
#elif defined (MCU_STM32F0)
const uint16_t ospeed_values[4] =
{2000000 >> 16,  2000000 >> 16, 10000000 >> 16,  50000000 >> 16};
#elif defined (MCU_STM32F2)
const uint16_t ospeed_values[4] =
{2000000 >> 16, 25000000 >> 16, 50000000 >> 16, 100000000 >> 16};
#elif defined (MCU_STM32F3)
const uint16_t ospeed_values[4] =
{2000000 >> 16,  2000000 >> 16, 10000000 >> 16,  50000000 >> 16};
#elif defined (MCU_STM32F4)
const uint16_t ospeed_values[4] =
{2000000 >> 16, 25000000 >> 16, 50000000 >> 16, 100000000 >> 16};
#elif defined (MCU_STM32F7)
const uint16_t ospeed_values[4] =
{2000000 >> 16, 25000000 >> 16, 50000000 >> 16, 100000000 >> 16};
#else
#warning "Unknown STM32 family"
#endif

/*!
 * \brief Internal function to set checked pin configuration
 *  and eventually alternate function
 *
 *
 * \param GPIO  Pin designator like PA1 as defined in dev/pins.h.
 * \param flags Flags for the pin
 * \param af    Number of alternate function to set.
 *              Use -1 to set pure GPIO.
 *
 * \return 0 on success, -1 on error.
 */
int Stm32GpioConfigSetDo(GPIO_TypeDef *gpio, int pin_nr, uint32_t flags, uint32_t af) {
    uint32_t moder;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t otyper;
    uint32_t bsrr;
    uint32_t afr = 0;
    int afr_index = 2;
    int shift;
    int speed;

    GpioClkEnable((uint32_t)gpio);
    shift = pin_nr << 1;

/* Work on local copy of registers and only write
 * out when successfull at end of function
 */
    moder   = gpio->MODER;
    ospeedr = gpio->OSPEEDR;
    pupdr   = gpio->PUPDR;
    otyper  = gpio->OTYPER;
    bsrr    = 0;
    /* Reset all two bit entries */
    moder   &= ~(0x3 << shift);
    ospeedr &= ~(0x3 << shift);
    pupdr   &= ~(0x3 << shift);
    otyper  &= ~(1 << pin_nr);
    if (flags & GPIO_CFG_DISABLED) {
        /* Set Analog Mode, ignore all other flags and reset all other bits */
        moder   |=  (3 << shift);
    } else {
        /* Set the inital value, if given
         *
         * Otherwise we may introduce unwanted transistions on the port
         */
        if (flags & GPIO_CFG_INIT_HIGH) {
            if (flags & GPIO_CFG_INIT_LOW) {
                return -1;
            } else {
                bsrr |= GPIO_BSRR_BS_0 << pin_nr ;
            }
        }
        if (flags & GPIO_CFG_INIT_LOW) {
            bsrr |= GPIO_BSRR_BR_0 << pin_nr ;
        }
        /* we can't check for these flags, so clear them */
        flags &= ~(GPIO_CFG_INIT_LOW |GPIO_CFG_INIT_HIGH);

        /* For non-output, multidrive is don't care*/
        if (flags & GPIO_CFG_MULTIDRIVE ) {
            otyper |= 1 << pin_nr;
        }
        /* For non-output, ospeedr is don't care*/
        switch (flags & GPIO_CFG_SPEED) {
        case GPIO_CFG_SPEED_HIGH:
            speed = 3;
            break;
        case GPIO_CFG_SPEED_FAST:
            speed = 2;
            break;
        case GPIO_CFG_SPEED_MED:
            speed = 1;
            break;
        default:
            speed = 0;
            break;
        }
#if defined(SYSCFG_CMPCR_CMP_PD)
        if (speed > 1 ) {
            /* On F4/F7, even if one pin needs high or fast speed, we need to
             * enable the SYSCFG clock and the IO compensation cell
             */
            CM3BBSET(SYSCFG_BASE, SYSCFG_TypeDef, CMPCR, _BI32(SYSCFG_CMPCR_CMP_PD));
            /* FIXME: Do we need to check SYSCFG_CMPCR_READY ? */
        }
#endif
        ospeedr |=  (speed << shift);
        /* Pull Up/Pull Down applies to all configurations*/
        if (flags & GPIO_CFG_PULLUP ) {
            pupdr |=  (1 << shift);
        }
        if (flags & GPIO_CFG_PULLDOWN ) {
            pupdr |=  (2 << shift );
        }
        if (flags & GPIO_CFG_PERIPHAL) {
            moder |=  2 << shift;
            if ((uint32_t)-1 != af) {
                int af_shift;
                if (pin_nr > 7) {
                    af_shift = (pin_nr -8) << 2;
                    afr_index = 1;
                } else {
                    af_shift = pin_nr << 2;
                    afr_index = 0;
                }
                afr  = gpio->AFR[afr_index];
                afr &= ~(0xf << af_shift);
                afr |=  ( af << af_shift);
            }
        } else if (flags & GPIO_CFG_OUTPUT) {
            moder |=  1 << shift;
        }
    }
    if (afr_index < 2) {
        gpio->AFR[afr_index] = afr;
    }
    gpio->OSPEEDR =  ospeedr;
    gpio->PUPDR   =  pupdr;
    gpio->OTYPER  =  otyper;
/* The families still differ in BSSR vs. BSSRL. Use the macro. */
    GpioPortSetHigh((uint32_t)gpio, bsrr);
    gpio->MODER   =  moder;
    return 0;
}

/*!
 * \brief Get pin configuration.
 *
 * Trying to set undefined ports must be avoided.
 * If NUTDEBUG is enabled an assertion will be rised.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 *
 * \return Attribute flags of the pin.
 */
uint32_t GpioPinConfigGet(int bank, int bit)
{
    uint32_t rc = 0;
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)bank;
    uint8_t mode = ((gpio->MODER) >> (bit *2)) & 0x3;
    uint8_t pull = ((gpio->PUPDR) >> (bit *2)) & 0x3;
    uint8_t dr_oc = ((gpio->OTYPER) >> bit ) & 0x1;
    uint8_t speed = ((gpio->OSPEEDR) >> (bit *2)) & 0x3;
    if (mode == GPIO_Mode_OUT)
    {
        rc = GPIO_CFG_OUTPUT;
        rc |= (dr_oc)? GPIO_CFG_MULTIDRIVE: 0;
        rc |= (pull == GPIO_PuPd_UP)?GPIO_CFG_PULLUP : 0;
    }
    else if (mode == GPIO_Mode_AF)
    {
        rc  = GPIO_CFG_PERIPHAL;
        rc |= (dr_oc)? GPIO_CFG_MULTIDRIVE: 0;
        rc |= (pull == GPIO_PuPd_UP)?GPIO_CFG_PULLUP : 0;
    }
    else if (mode == GPIO_Mode_AN)
    {
        rc  = GPIO_CFG_OUTPUT| GPIO_CFG_DISABLED;
    }
    else if(mode == GPIO_Mode_IN)
    {
        rc |= (pull == GPIO_PuPd_UP)?GPIO_CFG_PULLUP : 0;
    }
    switch (speed)
    {
    case 0:  rc |= GPIO_CFG_SPEED_SLOW; break;
    case 1:  rc |= GPIO_CFG_SPEED_MED;  break ;
    case 2:  rc |= GPIO_CFG_SPEED_HIGH; break;
    case 3:  rc |= GPIO_CFG_SPEED_FAST; break;
     }
    return rc;
}

/*!
 * \brief Set port wide pin configuration.
 *
 * \note This function does not check for undefined ports and pins or
 *       invalid attributes. If this is required, use GpioPinConfigSet().
 *       If NUTDEBUG is enabled accessing an undefined port will rise
 *       an assertion.
 *
 * \param bank  GPIO bank/port number.
 * \param mask  The given attributes are set for a specific pin, if the
 *              corresponding bit in this mask is 1.
 * \param flags Attribute flags to set.
 *
 * \return Always 0.
 */
int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags)
{
    int i;

    for( i=0; i<16; i++)
    {
        if (mask & 1)
            GpioPinConfigSet(bank, i, flags);
        mask >>= 1;
    }
    return 0;
}

int GpioPinConfigSet(int bank, int bit, uint32_t flags)
{
    NUTASSERT(IS_GPIO_ALL_PERIPH(bank));
    GPIO_TypeDef *gpio = (GPIO_TypeDef *) bank;
    if (bit > 15) {
        return -1;
    }
    if (Stm32GpioConfigSetDo(gpio, bit, flags, (uint32_t) -1))
        return -1;
    /* Check the result. */
    if (GpioPinConfigGet(bank, bit) != flags) {
        return -1;
    }
    return 0;
}

/* Copied from STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c*/
/**
  * @}
  */

/** @defgroup GPIO_Group3 GPIO Alternate functions configuration function
 *  @brief   GPIO Alternate functions configuration function
 *
 ===============================================================================
               GPIO Alternate functions configuration function
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *         This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @param  GPIO_AFSelection: selects the pin to used as Alternate function.
  *          This parameter can be one of the following values:
  *            @arg GPIO_AF_RTC_50Hz: Connect RTC_50Hz pin to AF0 (default after reset)
  *            @arg GPIO_AF_MCO: Connect MCO pin (MCO1 and MCO2) to AF0 (default after reset)
  *            @arg GPIO_AF_TAMPER: Connect TAMPER pins (TAMPER_1 and TAMPER_2) to AF0 (default after reset)
  *            @arg GPIO_AF_SWJ: Connect SWJ pins (SWD and JTAG)to AF0 (default after reset)
  *            @arg GPIO_AF_TRACE: Connect TRACE pins to AF0 (default after reset)
  *            @arg GPIO_AF_TIM1: Connect TIM1 pins to AF1
  *            @arg GPIO_AF_TIM2: Connect TIM2 pins to AF1
  *            @arg GPIO_AF_TIM3: Connect TIM3 pins to AF2
  *            @arg GPIO_AF_TIM4: Connect TIM4 pins to AF2
  *            @arg GPIO_AF_TIM5: Connect TIM5 pins to AF2
  *            @arg GPIO_AF_TIM8: Connect TIM8 pins to AF3
  *            @arg GPIO_AF_TIM9: Connect TIM9 pins to AF3
  *            @arg GPIO_AF_TIM10: Connect TIM10 pins to AF3
  *            @arg GPIO_AF_TIM11: Connect TIM11 pins to AF3
  *            @arg GPIO_AF_I2C1: Connect I2C1 pins to AF4
  *            @arg GPIO_AF_I2C2: Connect I2C2 pins to AF4
  *            @arg GPIO_AF_I2C3: Connect I2C3 pins to AF4
  *            @arg GPIO_AF_SPI1: Connect SPI1 pins to AF5
  *            @arg GPIO_AF_SPI2: Connect SPI2/I2S2 pins to AF5
  *            @arg GPIO_AF_SPI3: Connect SPI3/I2S3 pins to AF6
  *            @arg GPIO_AF_I2S3ext: Connect I2S3ext pins to AF7
  *            @arg GPIO_AF_USART1: Connect USART1 pins to AF7
  *            @arg GPIO_AF_USART2: Connect USART2 pins to AF7
  *            @arg GPIO_AF_USART3: Connect USART3 pins to AF7
  *            @arg GPIO_AF_UART4: Connect UART4 pins to AF8
  *            @arg GPIO_AF_UART5: Connect UART5 pins to AF8
  *            @arg GPIO_AF_USART6: Connect USART6 pins to AF8
  *            @arg GPIO_AF_CAN1: Connect CAN1 pins to AF9
  *            @arg GPIO_AF_CAN2: Connect CAN2 pins to AF9
  *            @arg GPIO_AF_TIM12: Connect TIM12 pins to AF9
  *            @arg GPIO_AF_TIM13: Connect TIM13 pins to AF9
  *            @arg GPIO_AF_TIM14: Connect TIM14 pins to AF9
  *            @arg GPIO_AF_OTG_FS: Connect OTG_FS pins to AF10
  *            @arg GPIO_AF_OTG_HS: Connect OTG_HS pins to AF10
  *            @arg GPIO_AF_ETH: Connect ETHERNET pins to AF11
  *            @arg GPIO_AF_FSMC: Connect FSMC pins to AF12
  *            @arg GPIO_AF_OTG_HS_FS: Connect OTG HS (configured in FS) pins to AF12
  *            @arg GPIO_AF_SDIO: Connect SDIO pins to AF12
  *            @arg GPIO_AF_DCMI: Connect DCMI pins to AF13
  *            @arg GPIO_AF_EVENTOUT: Connect EVENTOUT pins to AF15
  * @retval None
  */
void GPIO_PinAFConfig(nutgpio_port_t GPIOx, nutgpio_pin_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;
  GPIO_TypeDef * gpio = (GPIO_TypeDef *)GPIOx;
  volatile uint32_t *afr;

  /* Check the parameters */
  NUTASSERT(IS_GPIO_ALL_PERIPH(GPIOx));
  NUTASSERT(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  NUTASSERT(IS_GPIO_AF(GPIO_AF));

  afr = &gpio->AFR[GPIO_PinSource / 8 ];
  temp = 0xf << ((GPIO_PinSource % 8) * 4);
  temp_2 = *afr;
  temp_2 &= ~temp;
  temp_2 |= GPIO_AF << ((GPIO_PinSource % 8) * 4);
  *afr = temp_2;
}

/*!
 * \brief Set pin configuration and alternate function
 *
 * When code calls Stm32GpioConfigSet() for both STM32F1 and all
 * newer families, use the GPIO_CFG_OUTPUT flag together with the
 * GPIO_CFG_PERIPHAL flag to allow STM32F1 configuration.
 *
 * \param GPIO  Pin designator like PA1 as defined in dev/pins.h.
 * \param flags Flags for the pin.
 * \param af    Number of alternate function to set
 *
 * \return 0 on success, -1 on error.
 */
int Stm32GpioConfigSet(nutgpio_t GPIO, uint32_t flags, uint32_t af)
{
    int gpio_nr;
    int pin_nr;
    GPIO_TypeDef *gpio;

    if (af > 15)
        return -1;
    pin_nr = GPIO & 0xff;
    if (pin_nr > 15)
        return -1;
    gpio_nr = GPIO >> 8;
    if (gpio_nr >= STM32_NR_GPIO)
        return -1;
    gpio = stm32_port_nr2gpio[gpio_nr];
    if (!gpio)
        return -1;
    return Stm32GpioConfigSetDo(gpio, pin_nr, flags, af);
}

/*!
 * \brief Set pin to value
 *
 * \param GPIO  Pin designator like PA01 as defined in dev/pins.h.
 * \param value If value == 0, set pin to 0, otherwise to 1;
 *
 * \return 0 on success, -1 for PIN_NONE.
 */
int Stm32GpioSet(nutgpio_t gpio_pin, int value)
{
    if (gpio_pin == PIN_NONE) {
        return -1;
    } else {
        int gpio_nr;
        int pin_nr;
        GPIO_TypeDef *gpio;

        pin_nr = gpio_pin & 0xff;
        NUTASSERT(pin_nr > 16)
            gpio_nr = gpio_pin >> 8;
        gpio = stm32_port_nr2gpio[gpio_nr];
        NUTASSERT(IS_GPIO_ALL_INSTANCE(gpio));
#if defined(GPIO_BRR_BR_0)
/* F0/F1/F3/L0/L4 have explicit BRR register */
        if (value) {
            gpio->BSRR = 1 << pin_nr;
        } else {
            gpio->BRR  = 1 << pin_nr;
        }
#else
/* L1/F2/F4/F7 have only BSRR */
        if (value) {
            gpio->BSRR = 1 << pin_nr;
        } else {
            gpio->BSRR = 1 << (16 + pin_nr);
        }
#endif
    }
    return 0;
}

/*!
 * \brief Get pin value
 *
 * \param Gpio pin to query.
 *
 * \return 1 if pin is set, else 0, e.g. for non-existing pin.
 */
int Stm32GpioGet(nutgpio_t gpio_pin)
{
    if (gpio_pin != PIN_NONE) {
        int gpio_nr;
        int pin_nr;
        GPIO_TypeDef *gpio;

        pin_nr = gpio_pin & 0xff;
        NUTASSERT(pin_nr > 16)
            gpio_nr = gpio_pin >> 8;
        gpio = stm32_port_nr2gpio[gpio_nr];
        NUTASSERT(IS_GPIO_ALL_INSTANCE(gpio));
        return (gpio->IDR & (1 << pin_nr));
    }
    return 0;
}
