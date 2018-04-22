/*!
 * Copyright (C) 2007 by egnite Software GmbH. All rights reserved.
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
 * $Id: gpio_sam3u.c 4590 2012-09-10 10:26:01Z olereinhardt $
 */

#include <arch/cm3.h>

#include <stdlib.h>
#include <string.h>

#include <dev/gpio.h>

/*!
 * \brief Get pin level.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 *
 * \return 1 if the pin level is high. 0 is returned if the pin level
 *         is low or if the pin is undefined.
 */
int GpioPinGet(int bank, int bit)
{
    int rc = 0;

    switch(bank) {
    case NUTGPIO_PORTA:
        rc = (inr(AT91C_PIOA_PDSR) & _BV(bit)) != 0;
        break;

    case NUTGPIO_PORTB:
        rc = (inr(AT91C_PIOB_PDSR) & _BV(bit)) != 0;
        break;

    case NUTGPIO_PORTC:
        rc = (inr(AT91C_PIOC_PDSR) & _BV(bit)) != 0;
        break;
    }
    return rc;
}

/*!
 * \brief Set pin level to low.
 *
 * Trying to set undefined pins is silently ignored.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 */
void GpioPinSetLow(int bank, int bit)
{
    switch(bank) {
    case NUTGPIO_PORTA:
        outr(AT91C_PIOA_CODR, _BV(bit));
        break;

    case NUTGPIO_PORTB:
        outr(AT91C_PIOB_CODR, _BV(bit));
        break;

    case NUTGPIO_PORTC:
        outr(AT91C_PIOC_CODR, _BV(bit));
        break;
    }
}

/*!
 * \brief Set pin level to high.
 *
 * Trying to set undefined pins is silently ignored.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 */
void GpioPinSetHigh(int bank, int bit)
{
    switch(bank) {

    case NUTGPIO_PORTA:
        outr(AT91C_PIOA_SODR, _BV(bit));
        break;

    case NUTGPIO_PORTB:
        outr(AT91C_PIOB_SODR, _BV(bit));
        break;

    case NUTGPIO_PORTC:
        outr(AT91C_PIOC_SODR, _BV(bit));
        break;
    }
}

/*!
 * \brief Set pin level.
 *
 * Trying to set undefined pins is silently ignored.
 *
 * \param bank  GPIO bank/port number.
 * \param bit   Bit number of the specified bank/port.
 * \param value Level, 0 for low or any other value for high.
 */
void GpioPinSet(int bank, int bit, int value)
{
    if (value) {
        GpioPinSetHigh(bank, bit);
    }
    else {
        GpioPinSetLow(bank, bit);
    }
}

/*!
 * \brief Get all pin levels of a specified bank/port.
 *
 * \param bank GPIO bank/port number.
 *
 * \return Pin levels. 0 is returned for unknown banks and pins.
 */
unsigned int GpioPortGet(int bank)
{
    unsigned int rc = 0;

    switch(bank) {
    case NUTGPIO_PORTA:
        rc = inr(AT91C_PIOA_PDSR);
        break;

    case NUTGPIO_PORTB:
        rc = inr(AT91C_PIOB_PDSR);
        break;

    case NUTGPIO_PORTC:
        rc = inr(AT91C_PIOC_PDSR);
        break;
    }
    return rc;
}

/*!
 * \brief Set multiple pin levels of a bank/port to low.
 *
 * \param bank GPIO bank/port number.
 * \param mask Pin levels are set to low, if the corresponding
 *             bit in this mask is 1.
 *
 * \return Levels.
 */
void GpioPortSetLow(int bank, unsigned int mask)
{
    switch(bank) {

    case NUTGPIO_PORTA:
        outr(AT91C_PIOA_CODR, mask);
        break;

    case NUTGPIO_PORTB:
        outr(AT91C_PIOB_CODR, mask);
        break;

    case NUTGPIO_PORTC:
        outr(AT91C_PIOC_CODR, mask);
        break;
    }
}

/*!
 * \brief Set multiple pin levels of a bank/port to high.
 *
 * Trying to set undefined ports is silently ignored.
 *
 * \param bank GPIO bank/port number.
 * \param mask Pin levels are set to high, if the corresponding
 *             bit in this mask is 1.
 */
void GpioPortSetHigh(int bank, unsigned int mask)
{
    switch(bank) {

    case NUTGPIO_PORTA:
        outr(AT91C_PIOA_SODR, mask);
        break;

    case NUTGPIO_PORTB:
        outr(AT91C_PIOB_SODR, mask);
        break;

    case NUTGPIO_PORTC:
        outr(AT91C_PIOC_SODR, mask);
        break;
    }
}

/*!
 * \brief Set all pin levels of a bank/port.
 *
 * This routine can be used to simultaneously set all pins of a specific
 * port. However, in some implementations the high bit values will be
 * set before the low bit values. If this is a problem, you should use
 * GpioPortSetHigh() and GpioPortSetLow().
 *
 * \param bank  GPIO bank/port number.
 * \param value Pin levels are set to high, if the corresponding
 *              bit in this mask is 1. All other pin levels are
 *              set to low.
 */
void GpioPortSet(int bank, unsigned int value)
{
    if (value) {
        GpioPortSetHigh(bank, value);
    }
    value = ~value;
    if (value) {
        GpioPortSetLow(bank, value);
    }
}

/*!
 * \brief Get pin configuration.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 *
 * \return Attribute flags of the pin.
 */
uint32_t GpioPinConfigGet(int bank, int bit)
{
    uint32_t rc = 0;

    switch(bank) {

    case NUTGPIO_PORTA:
        if ((inr(AT91C_PIOA_PSR) & _BV(bit)) == 0) {
            rc |= GPIO_CFG_DISABLED;
        }

        if (inr(AT91C_PIOA_OSR) & _BV(bit)) {
            rc |= GPIO_CFG_OUTPUT;
        }

        if (inr(AT91C_PIOA_IFSR) & _BV(bit)) {
            rc |= GPIO_CFG_DEBOUNCE;
        }

        if (inr(AT91C_PIOA_MDSR) & _BV(bit)) {
            rc |= GPIO_CFG_MULTIDRIVE;
        }

        if ((inr(AT91C_PIOA_PPUSR) & _BV(bit)) == 0) {
            rc |= GPIO_CFG_PULLUP;
        }
        break;

    case NUTGPIO_PORTB:
        if ((inr(AT91C_PIOB_PSR) & _BV(bit)) == 0) {
            rc |= GPIO_CFG_DISABLED;
        }

        if (inr(AT91C_PIOB_OSR) & _BV(bit)) {
            rc |= GPIO_CFG_OUTPUT;
        }

        if (inr(AT91C_PIOB_IFSR) & _BV(bit)) {
            rc |= GPIO_CFG_DEBOUNCE;
        }

        if (inr(AT91C_PIOB_MDSR) & _BV(bit)) {
            rc |= GPIO_CFG_MULTIDRIVE;
        }

        if ((inr(AT91C_PIOB_PPUSR) & _BV(bit)) == 0) {
            rc |= GPIO_CFG_PULLUP;
        }
        break;

    case NUTGPIO_PORTC:
        if ((inr(AT91C_PIOC_PSR) & _BV(bit)) == 0) {
            rc |= GPIO_CFG_DISABLED;
        }

        if (inr(AT91C_PIOC_OSR) & _BV(bit)) {
            rc |= GPIO_CFG_OUTPUT;
        }

        if (inr(AT91C_PIOC_IFSR) & _BV(bit)) {
            rc |= GPIO_CFG_DEBOUNCE;
        }

        if (inr(AT91C_PIOC_MDSR) & _BV(bit)) {
            rc |= GPIO_CFG_MULTIDRIVE;
        }

        if ((inr(AT91C_PIOC_PPUSR) & _BV(bit)) == 0) {
            rc |= GPIO_CFG_PULLUP;
        }
        break;

    }
    return rc;
}

/*!
 * \brief Set port wide pin configuration.
 *
 * \note This function does not check for undefined ports and pins or
 *       invalid attributes. If this is required, use GpioPinConfigSet().
 *
 * \param bank  GPIO bank/port number.
 * \param mask  The given attributes are set for a specific pin, if the
 *              corresponding bit in this mask is 1.
 * \param flags Attribute flags to set.
 *
 * \return Always 0.
 */
int GpioPortConfigSet(int bank, unsigned int mask, uint32_t flags)
{
    switch(bank) {

    case NUTGPIO_PORTA:
        if (flags & GPIO_CFG_DISABLED) {
            outr(AT91C_PIOA_PDR, mask);
        }
        else {
            outr(AT91C_PIOA_PER, mask);
            outr(AT91C_PMC_PCER, _BV(AT91C_ID_PIOA));
        }

        if (flags & GPIO_CFG_PULLUP) {
            outr(AT91C_PIOA_PPUER, mask);
        }
        else {
            outr(AT91C_PIOA_PPUDR, mask);
        }

        if (flags & GPIO_CFG_DEBOUNCE) {
            outr(AT91C_PIOA_IFER, mask);
        }
        else {
            outr(AT91C_PIOA_IFDR, mask);
        }

        if ((flags & GPIO_CFG_OUTPUT) == 0) {
            outr(AT91C_PIOA_ODR, mask);
        }

        if (flags & GPIO_CFG_MULTIDRIVE) {
            outr(AT91C_PIOA_MDER, mask);
        }
        else {
            outr(AT91C_PIOA_MDDR, mask);
        }

        if (flags & GPIO_CFG_OUTPUT) {
            outr(AT91C_PIOA_OER, mask);
        }
        break;

    case NUTGPIO_PORTB:
        if (flags & GPIO_CFG_DISABLED) {
            outr(AT91C_PIOB_PDR, mask);
        }
        else {
            outr(AT91C_PIOB_PER, mask);
            outr(AT91C_PMC_PCER, _BV(AT91C_ID_PIOB));
        }

        if (flags & GPIO_CFG_PULLUP) {
            outr(AT91C_PIOB_PPUER, mask);
        }
        else {
            outr(AT91C_PIOB_PPUDR, mask);
        }

        if (flags & GPIO_CFG_DEBOUNCE) {
            outr(AT91C_PIOB_IFER, mask);
        }
        else {
            outr(AT91C_PIOB_IFDR, mask);
        }

        if ((flags & GPIO_CFG_OUTPUT) == 0) {
            outr(AT91C_PIOB_ODR, mask);
        }

        if (flags & GPIO_CFG_MULTIDRIVE) {
            outr(AT91C_PIOB_MDER, mask);
        }
        else {
            outr(AT91C_PIOB_MDDR, mask);
        }

        if (flags & GPIO_CFG_OUTPUT) {
            outr(AT91C_PIOB_OER, mask);
        }
        break;

    case NUTGPIO_PORTC:
        if (flags & GPIO_CFG_DISABLED) {
            outr(AT91C_PIOC_PDR, mask);
        }
        else {
            outr(AT91C_PIOC_PER, mask);
            outr(AT91C_PMC_PCER, _BV(AT91C_ID_PIOC));
        }

        if (flags & GPIO_CFG_PULLUP) {
            outr(AT91C_PIOC_PPUER, mask);
        }
        else {
            outr(AT91C_PIOC_PPUDR, mask);
        }

        if (flags & GPIO_CFG_DEBOUNCE) {
            outr(AT91C_PIOC_IFER, mask);
        }
        else {
            outr(AT91C_PIOC_IFDR, mask);
        }

        if ((flags & GPIO_CFG_OUTPUT) == 0) {
            outr(AT91C_PIOC_ODR, mask);
        }

        if (flags & GPIO_CFG_MULTIDRIVE) {
            outr(AT91C_PIOC_MDER, mask);
        }
        else {
            outr(AT91C_PIOC_MDDR, mask);
        }

        if (flags & GPIO_CFG_OUTPUT) {
            outr(AT91C_PIOC_OER, mask);
        }
        break;
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
    GpioPortConfigSet(bank, _BV(bit), flags);

    /* Check the result. */
    if (GpioPinConfigGet(bank, bit) != flags) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Register a GPIO pin interrupt handler.
 *
 * Generating interrupts on GPIO pin changes is not supported on all
 * platforms. In this case dedicated external interrupt pins may
 * be used with NutRegisterIrqHandler().
 *
 * Interrupts are triggered on rising and falling edges. Level triggering
 * or triggering on specific edges is not supported.
 *
 * After registering, interrupts are disabled. Calling GpioIrqEnable()
 * is required to activate the interrupt.
 *
 * The following code fragment registers an interrupt handler which is
 * called on each change of bit 4 of the first GPIO port:
 * \code
 * #include <dev/gpio.h>
 *
 * static void PinChange(void *arg)
 * {
 *     ...
 * }
 *
 * {
 *     ...
 *     GpioPinConfigSet(0, 4, GPIO_CFG_PULLUP);
 *     GpioRegisterIrqHandler(&sig_GPIO, 4, PinChange, NULL);
 *     GpioIrqEnable(&sig_GPIO, 4);
 *     ...
 * }
 * \endcode
 *
 * \param sig     Bank/port interrupt to be associated with this handler.
 * \param bit     Bit number of the specified bank/port.
 * \param handler This routine will be called by Nut/OS, when the specified
 *                pin changes its state.
 * \param arg     Argument to be passed to the interrupt handler routine.
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioRegisterIrqHandler(GPIO_SIGNAL * sig, int bit, void (*handler) (void *), void *arg)
{
    int rc = 0;

    if (sig->ios_vector == 0) {
        /* This is the first call. Allocate the vector table. */
        sig->ios_vector = malloc(sizeof(GPIO_VECTOR) * 32);
        if (sig->ios_vector) {
            memset(sig->ios_vector, 0, sizeof(GPIO_VECTOR) * 32);
            /* Register our internal PIO interrupt service. */
            rc = NutRegisterIrqHandler(sig->ios_sig, sig->ios_handler, sig->ios_vector);
            if (rc == 0) {
                rc = NutIrqEnable(sig->ios_sig);
            }
        }
        else {
            rc = -1;
        }
    }
    sig->ios_vector[bit].iov_handler = handler;
    sig->ios_vector[bit].iov_arg = arg;

    return rc;
}

/*!
 * \brief Enable a specified GPIO interrupt.
 *
 * A related interrupt handler must have been registered before calling
 * this function. See GpioRegisterIrqHandler().
 *
 * \param sig Interrupt to enable.
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqEnable(GPIO_SIGNAL * sig, int bit)
{
    return (sig->ios_ctl) (NUT_IRQCTL_ENABLE, NULL, bit);
}

/*!
 * \brief Disable a specified GPIO interrupt.
 *
 * \param sig Interrupt to disable.
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqDisable(GPIO_SIGNAL * sig, int bit)
{
    return (sig->ios_ctl) (NUT_IRQCTL_DISABLE, NULL, bit);
}
