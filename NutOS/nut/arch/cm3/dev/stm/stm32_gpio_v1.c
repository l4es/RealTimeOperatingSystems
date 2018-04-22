/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * $Id: stm32_gpio_v1.c 6300 2016-03-09 11:26:34Z u_bonnes $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

#include <arch/cm3.h>
#include <dev/gpio.h>
#include <arch/cm3/stm/stm32_gpio.h>

#define NUTGPIOPORT_MAX NUTGPIO_PORTG+1

const uint16_t ospeed_values[4] =
{-1,  10000000 >> 16, 2000000 >> 16,  50000000 >> 16};

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
    uint8_t cnf; */
uint32_t GpioPinConfigGet(int bank, int bit)
{
    uint32_t rc = 0;
    uint8_t mode;
    uint8_t cnf;
    uint8_t speed;
    GPIO_TypeDef *GPIOx = ((GPIO_TypeDef *)bank);

    if( bit < 8 ) {
        rc = GPIOx->CRL;
    }
    else {
        rc = GPIOx->CRH;
        bit -=8;
    }

    mode = ( rc >> ( bit * 4 ) );
    speed = mode & 0x3;
    cnf = ( mode >> 2 ) & 0x3;
    mode &= 0x3;

    if( mode == 0 ) {
        rc = 0;
        /* Pin is input */
        switch ( cnf ) {
        case 0:
            /* Input Analog */
            rc |= GPIO_CFG_DISABLED;
            break;
        case 1:
            /* Input Floating */
            rc = 0;
            break;        case 2:
            /* Input with Pullup */
            if (GPIOx->ODR & _BV(bit))
                rc |= GPIO_CFG_PULLUP;
            else
                rc |= GPIO_CFG_PULLDOWN;
            break;
        }
    }
    else {
        rc = GPIO_CFG_OUTPUT;

        /* Pin is output */
        switch ( cnf ) {
        case 1:
            /* Output Open-Drain */
            rc |= GPIO_CFG_MULTIDRIVE;
            break;
        case 2:
            /* Output Push-Pull Alternative Function */
            rc |= (GPIO_CFG_PERIPHAL);
            break;
        case 3:
            /* Output Open-Drain Alternative Function */
            rc |= (GPIO_CFG_PERIPHAL | GPIO_CFG_MULTIDRIVE);
            break;
        }
        switch ( speed) {
        case 2 :
            rc |= GPIO_CFG_SPEED_SLOW;
            break;
        case 3 :
            rc |= GPIO_CFG_SPEED_FAST;
            break;
        default :
            rc |= GPIO_CFG_SPEED_MED;
        }
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
 * \return Corrected flags
 */
int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags)
{
    uint32_t cxmx = 0;      /* Configuration mask */
    uint32_t crl, crh, cln;
    uint8_t  msl, msh;
    int i;
    GPIO_TypeDef *GPIOx = ((GPIO_TypeDef *)bank);

    /* Important: Enable clock of port first
     * before trying to configure it!
     * Enable GPIO clock source */
    CM3BBSET(RCC_BASE, RCC_TypeDef, APB2ENR,
             (((bank-GPIOA_BASE)/0x400) + _BI32(RCC_APB2ENR_IOPAEN)));
    if (flags & GPIO_CFG_PERIPHAL )
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB2ENR,
                 _BI32(RCC_APB2ENR_AFIOEN));

    /* Set the inital value, if given
     *
     * Otherwise we may introduce unwanted transistions on the port
     */
    if (flags & GPIO_CFG_INIT_HIGH)
    {
        if (flags & GPIO_CFG_INIT_LOW)
            return -1;
        else
            GPIOx->BSRR = mask;
    }
    if (flags & GPIO_CFG_INIT_LOW)
            GPIOx->BRR = mask;

    /* we can't check for these flags, so clear them */
    flags &= ~(GPIO_CFG_INIT_LOW |GPIO_CFG_INIT_HIGH);

    /* Pull-up/down can only be configured for input mode*/
    if (flags & (GPIO_CFG_OUTPUT|GPIO_CFG_DISABLED))
        if(flags & (GPIO_CFG_PULLUP |GPIO_CFG_PULLDOWN))
            flags &= ~(GPIO_CFG_PULLUP |GPIO_CFG_PULLDOWN);


    /*
     * cnf  mode  Meaning
     * 00   00    Input Analog
     * 01   00    Input Floating
     * 10   00    Input PullUp/Down
     * 11   00    Reserved

     * 00   xx    Output Push-Pull
     * 01   xx    Output Open-Drain
     * 10   xx    Output Alternate Push-Pull
     * 11   xx    Output Alternate Open-Drain
     *
     *      01    Output mode, max speed 10MHz
     *      10    Output mode, max speed 2MHz
     *      11    Output mode, max speed 50MHz
     */

/* All newer STM32 GPIO needs to set peripheral mode for
 * peripheral input too, STM32F1 needs to set peripheral
 * mode only for peripheral outputs.
 * To keep compatibility between both version, we need to qualify the
 * peripheral flag with GPIO_CFG_OUTPUT for peripheral outputs.
 *
 * Ignore GPIO_CFG_PERIPHAL without GPIO_CFG_OUTPUT on STM32F1!
 * Ignore GPIO_CFG_OUTPUT with GPIO_CFG_PERIPHAL on other STM32 GPIO.
 */
    if( flags & GPIO_CFG_OUTPUT ) {
        switch (flags & GPIO_CFG_SPEED)
        {
        case GPIO_CFG_SPEED_SLOW: cxmx = 0x2; break;
        case GPIO_CFG_SPEED_FAST: cxmx = 0x3; break;
        default: cxmx = 0x1;
        }

        /* Configure Open-Drain */
        if( flags & GPIO_CFG_MULTIDRIVE )
            cxmx |= 0x4;

        /* Configure Alternate Function */
        if( flags & GPIO_CFG_DISABLED ) {
            cxmx |= 0x8;
        }

        /* Configure Alternate Function */
        if( flags & GPIO_CFG_PERIPHAL ) {
            cxmx |= 0x8;
        }
    }
    else {
        /* Configure pin as input floating */
        cxmx = 0x4;

        /* Configure pin as input PullUp/Down */
        if( flags & (GPIO_CFG_PULLUP |GPIO_CFG_PULLDOWN)) {
            cxmx = 0x8;
            if (flags & GPIO_CFG_PULLUP)
                GPIOx->ODR |= mask;
            else
                GPIOx->ODR &= ~mask;
        }
        /* Configure pin as analog input */
        if( flags & GPIO_CFG_DISABLED )
            cxmx = 0x0;
    }

    msl = (uint8_t)(mask&0xff);
    msh = (uint8_t)(mask>>8);

    /* get actual config */
    crl = GPIOx->CRL;
    crh = GPIOx->CRH;

    /* shift in modified values */
    for( i = 0; i < 8; i++ ) {
        /* mask out actual nibble */
        cln = ~(0xf<<(i*4));

        /* check if pins 0..7 are affected */
        if( msl & _BV(i)) {
            crl &= cln;     /* mask nibble */
            crl |= cxmx;    /* write nibble */
        }
        /* check if pins 8..15 are affected */
        if( msh & _BV(i)) {
            crh &= cln;     /* mask nibble */
            crh |= cxmx;    /* write nibble */
        }
        /* shift left configuration for next bit */
        cxmx <<= 4;
    }

    /* write back new config */
    GPIOx->CRL = crl;
    GPIOx->CRH = crh;

    return flags;
}

/*!
 * \brief Set pin configuration.
 *
 * Applications may also use this function to make sure, that a specific
 * attribute is available for a specific pin.
 *
 * \note GPIO pins are typically initialized to a safe state after power
 *       up. This routine is not able to determine the consequences of
 *       changing pin configurations. In the worst case you may permanently
 *       damage your hardware by bad pin settings.
 *
 * \param bank  GPIO bank/port number.
 * \param bit   Bit number of the specified bank/port.
 * \param flags Attribute flags.
 *
 * \return 0 if all attributes had been set, -1 otherwise.
 */
int GpioPinConfigSet(int bank, int bit, uint32_t flags)
{
    NUTASSERT(bank<NUTGPIOPORT_MAX);

    if (GpioPortConfigSet( bank, _BV( bit ), flags ) !=
        GpioPinConfigGet( bank, bit )) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Set pin configuration
 *
 * STM32F1 has only a function global pin remapping, not an indivudual
 * remap as newer families. To keep common code possible, the value of
 * the af parameter is ignored for STM32F1.
 *
 *
 * \param GPIO  Pin designator like PA1 as defined in dev/pins.h.
 * \param flags Flags for the pin.
 * \param af    Ignored.
 *
 * \return 0 on success, -1 on error.
 */
int Stm32GpioConfigSet(nutgpio_t GPIO, uint32_t flags, uint32_t af)
{
    int gpio_nr;
    int pin_nr;
    GPIO_TypeDef *gpio;

    (void) af;
    pin_nr = GPIO & 0xff;
    if (pin_nr > 15)
        return -1;
    gpio_nr = GPIO >> 8;
    if (gpio_nr >= STM32_NR_GPIO)
        return -1;
    gpio = stm32_port_nr2gpio[gpio_nr];
    if (!gpio)
        return -1;
    GpioPortConfigSet((uint32_t)gpio, _BV(pin_nr), flags);
    return 0;
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
        NUTASSERT(pin_nr < 16);
        gpio_nr = gpio_pin >> 8;
        gpio = stm32_port_nr2gpio[gpio_nr];
        NUTASSERT(IS_GPIO_ALL_INSTANCE(gpio));
        if (value) {
            gpio->BSRR = 1 << pin_nr;
        } else {
            gpio->BRR  = 1 << pin_nr;
        }
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
        NUTASSERT(pin_nr > 16);
        gpio_nr = gpio_pin >> 8;
        gpio = stm32_port_nr2gpio[gpio_nr];
        NUTASSERT(IS_GPIO_ALL_INSTANCE(gpio));
        return (gpio->IDR & (1 << pin_nr));
    }
    return 0;
}
