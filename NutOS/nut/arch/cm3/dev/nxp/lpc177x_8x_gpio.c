/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 * $Id: lpc177x_8x_gpio.c $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

#include <arch/cm3.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#include <dev/gpio.h>

#include <string.h>

/*!
 * \addtogroup xgNutArchCm3Lpc177x_8xGpio
 */
/*@{*/

#define NUTGPIOPORT_MAX NUTGPIO_PORT5+1

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
    uint32_t mode;
    __IO uint32_t *IOCON;

    NUTASSERT(bank < NUTGPIOPORT_MAX);
    NUTASSERT((bit < 32) && ((bank != NUTGPIOPORT_MAX - 1) || (bit < 5)));

    /* Calculate the address of the desired IOCON register */
    IOCON = (uint32_t *)(LPC_IOCON_BASE + ((bank * 32 + bit) * sizeof(uint32_t)));

    /*
     * See register description of the IOCON registers of each I/O pin.
     */

    mode = *IOCON;

    /* Query pin direction */
    if (CM3BBGET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit)) {
        rc |= GPIO_CFG_OUTPUT;
    }

    /* Query currently set alternate function */
    rc |= ((mode >> IOCON_FUNC_POS) & IOCON_FUNC_BITMASK) << GPIO_CFG_PERIPHERAL_POS;
    if (rc & GPIO_CFG_PERIPHERAL_MASK) {
        rc |= GPIO_CFG_DISABLED;
    }

    switch (mode & IOCON_MODE_BITMASK) {
        case IOCON_MODE_PLAIN:
            /* Nothing special */
            break;
        case IOCON_MODE_PULLDOWN:
            rc |= GPIO_CFG_PULLDOWN;
            break;
        case IOCON_MODE_PULLUP:
            rc |= GPIO_CFG_PULLUP;
            break;
        case IOCON_MODE_REPEATER:
            rc |= GPIO_CFG_REPEATER;
            break;
    }

    if ((mode & IOCON_HYSTERESIS_BITMASK) == IOCON_HYSTERESIS) {
        rc |= GPIO_CFG_HYSTERESIS;
    }

    if ((mode & IOCON_INVERT_BITMASK) == IOCON_INVERTED) {
        rc |= GPIO_CFG_INVERT;
    }

    if ((mode & IOCON_ADMODE_BITMASK) == IOCON_ADMODE) {
        rc |= GPIO_CFG_ADMODE;
    }

    if ((mode & IOCON_GLITCH_FILTER_BITMASK) == IOCON_GLITCH_FILTER) {
        rc |= GPIO_CFG_DEBOUNCE;
    }

    if ((mode & IOCON_SLEW_BITMASK) == IOCON_SLEW) {
        rc |= GPIO_CFG_SLEWCTRL;
    }

    if ((mode & IOCON_ODMODE_BITMASK) == IOCON_ODMODE) {
        rc |= GPIO_CFG_MULTIDRIVE;
    }

    if ((mode & IOCON_DACEN_BITMASK) == IOCON_DACEN) {
        rc |= GPIO_CFG_DAC_ENABLE;
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
 * \note Port P0-7 .. P0-9 pins (W-Mode pins) are a little bit special and need
         bit 7 always be set for normal operation
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
    uint32_t mode = 0;
    int      i;

    __IO uint32_t *IOCON;

    NUTASSERT(bank < NUTGPIOPORT_MAX);

    /*
     * See register description of the IOCON registers of each I/O pin.
     */

    mode |= ((flags & GPIO_CFG_PERIPHERAL_MASK) >> GPIO_CFG_PERIPHERAL_POS) & IOCON_FUNC_BITMASK;

    if (flags & GPIO_CFG_PULLDOWN) {
        mode |= IOCON_MODE_PULLDOWN;
    } else
    if (flags & GPIO_CFG_PULLUP) {
        mode |= IOCON_MODE_PULLUP;
    } else
    if (flags & GPIO_CFG_REPEATER) {
        mode |= IOCON_MODE_REPEATER;
    }

    if (flags & GPIO_CFG_HYSTERESIS) {
        mode |= IOCON_HYSTERESIS;
    }

    if (flags & GPIO_CFG_INVERT) {
        mode |= IOCON_INVERTED;
    }

    if (flags & GPIO_CFG_ADMODE) {
        mode |= IOCON_ADMODE;
    }

    if (flags & GPIO_CFG_DEBOUNCE) {
        mode |= IOCON_GLITCH_FILTER;
    }

    if (flags & GPIO_CFG_SLEWCTRL) {
        mode |= IOCON_SLEW;
    }

    if (flags & GPIO_CFG_MULTIDRIVE) {
        mode |= IOCON_ODMODE;
    }

    if (flags & GPIO_CFG_DAC_ENABLE) {
        mode |= IOCON_DACEN;
    }

    IOCON = (uint32_t *)(LPC_IOCON_BASE + ((bank * 32) * sizeof(uint32_t)));

    for (i = 0; i < (bank != NUTGPIO_PORT5 ? 32 : 5); i ++) {
        if (mask & _BV(i)) {
            if ((bank == NUTGPIO_PORT0) && (i >= 7) && (i <= 9)) {
                /* Port P0-7 .. P0-9 pins (W-Mode pins) are a little bit special and need
                   bit 7 always be set for normal operation
                */
                mode |= _BV(7);
            }
            *(IOCON ++) = mode;
        }
    }

    if (GPIO_CFG_OUTPUT) {
        CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR) |= mask;
    } else {
        CM3REG(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR) &= ~mask;
    }

    return 0;
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
    uint32_t rc = 0;
    uint32_t mode;
    __IO uint32_t *IOCON;

    NUTASSERT(bank < NUTGPIOPORT_MAX);
    NUTASSERT((bit < 32) && ((bank != NUTGPIOPORT_MAX - 1) || (bit < 5)));


    /* Set the inital value, if given
     *
     * Otherwise we may introduce unwanted transistions on the port
     */
    if (flags & GPIO_CFG_INIT_HIGH)
    {
        if (flags & GPIO_CFG_INIT_LOW) {
            return -1;
        } else {
            GpioPinSetHigh(bank, bit);
        }
    }
    if (flags & GPIO_CFG_INIT_LOW) {
        GpioPinSetLow(bank, bit);
    }    

    /* we can't check for these flags, so clear them */
    flags &= ~(GPIO_CFG_INIT_LOW |GPIO_CFG_INIT_HIGH);


    /* Calculate the address of the desired IOCON register */
    IOCON = (uint32_t *)(LPC_IOCON_BASE + ((bank * 32 + bit) * sizeof(uint32_t)));

    /*
     * See register description of the IOCON registers of each I/O pin.
     */

    mode = 0;

    mode |= ((flags & GPIO_CFG_PERIPHERAL_MASK) >> GPIO_CFG_PERIPHERAL_POS) & IOCON_FUNC_BITMASK;

    if (flags & GPIO_CFG_PULLDOWN) {
        mode |= IOCON_MODE_PULLDOWN;
    } else
    if (flags & GPIO_CFG_PULLUP) {
        mode |= IOCON_MODE_PULLUP;
    } else
    if (flags & GPIO_CFG_REPEATER) {
        mode |= IOCON_MODE_REPEATER;
    }

    if (flags & GPIO_CFG_HYSTERESIS) {
        mode |= IOCON_HYSTERESIS;
    }

    if (flags & GPIO_CFG_INVERT) {
        mode |= IOCON_INVERTED;
    }

    if (flags & GPIO_CFG_ADMODE) {
        mode |= IOCON_ADMODE;
    }

    if (flags & GPIO_CFG_DEBOUNCE) {
        mode |= IOCON_GLITCH_FILTER;
    }

    if (flags & GPIO_CFG_SLEWCTRL) {
        mode |= IOCON_SLEW;
    }

    if (flags & GPIO_CFG_MULTIDRIVE) {
        mode |= IOCON_ODMODE;
    }

    if (flags & GPIO_CFG_DAC_ENABLE) {
        mode |= IOCON_DACEN;
    }

    if ((bank == NUTGPIO_PORT0) && (bit >= 7) && (bit <= 9)) {
        /* Port P0-7 .. P0-9 pins (W-Mode pins) are a little bit special and need
           bit 7 always be set for normal operation
        */
        mode |= _BV(7);
    }
#if defined(MCU_LPC407x_8x)
    if ((bank == NUTGPIO_PORT1) && (((bit >= 5) && (bit <= 7)) || (bit == 14) || (bit == 16) || (bit == 17))) {
        /* Port P1-5 .. P1-7, P1-14, P1-16, P1-17 pins (W-Mode pins) are a little bit special and need
           bit 7 always be set for normal operation
        */
        mode |= _BV(7);
    }
#endif

    *IOCON = mode;

    if (flags & GPIO_CFG_OUTPUT) {
        CM3BBSET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit);
    } else {
        CM3BBCLR(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit);
    }

    if (mode != (*IOCON & (IOCON_FUNC_BITMASK | IOCON_MODE_BITMASK |
                           IOCON_HYSTERESIS_BITMASK | IOCON_INVERT_BITMASK |
                           IOCON_ADMODE_BITMASK | IOCON_GLITCH_FILTER_BITMASK |
                           IOCON_I2C_MODE_BITMASK | IOCON_SLEW_BITMASK |
                           IOCON_ODMODE_BITMASK | IOCON_DACEN_BITMASK))) {
        rc = -1;
    }

    if (CM3BBGET(GPIO_BANKID2BASE(bank), LPC_GPIO_TypeDef, FIODIR, bit) != (flags & GPIO_CFG_OUTPUT) ? 1 : 0) {
        rc = -1;
    }


    return rc;
}
/*@}*/
