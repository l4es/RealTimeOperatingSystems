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

#include <stdlib.h>
#include <arch/cm3.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#else
#warning "Unknown LPC family"
#endif

#include <dev/gpio.h>

/*!
 * \brief PIO interrupt service.
 */
static void Lpc17xxGpioIsr(void *arg)
{
    GPIO_VECTOR *vct;

    uint32_t isr = LPC_GPIOINT->IntStatus;
    uint32_t port_status;

    if (isr & _BV(NUTGPIO_PORT0)) {
        port_status = LPC_GPIOINT->IO0IntStatR | LPC_GPIOINT->IO0IntStatF;

        vct = sig_GPIO0.ios_vector;
        while (port_status) {
            if ((port_status & 1) != 0 && vct->iov_handler) {
                (vct->iov_handler) (vct->iov_arg);
            }
            port_status >>= 1;
            vct++;
        }

        LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
    }
    if (isr & _BV(NUTGPIO_PORT2)) {
        port_status = LPC_GPIOINT->IO2IntStatR | LPC_GPIOINT->IO2IntStatF;

        vct = sig_GPIO2.ios_vector;
        while (port_status) {
            if ((port_status & 1) != 0 && vct->iov_handler) {
                (vct->iov_handler) (vct->iov_arg);
            }
            port_status >>= 1;
            vct++;
        }

        LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
    }
}

/*!
 * \brief PIO interrupt control.
 */
static int Lpc17xxGpioCtrl(GPIO_SIGNAL * sig, int cmd, void *param, int bit)
{
    int rc = 0;
    uint32_t *ival = (uint32_t *) param;

    switch (cmd) {
        case NUT_IRQCTL_STATUS:
            if (sig->enabled & _BV(bit)) {
                *ival = 1;
            } else {
                *ival = 0;
            }
            break;

        case NUT_IRQCTL_ENABLE:
            sig->enabled |= _BV(bit);
            break;

        case NUT_IRQCTL_DISABLE:
            sig->enabled &= ~_BV(bit);
            break;

        case NUT_IRQCTL_GETMODE:
            if ((sig->mode_rising_enabled & _BV(bit)) && ((sig->mode_falling_enabled & _BV(bit)) == 0)) {
                *ival = NUT_IRQMODE_RISINGEDGE;
            } else
            if (((sig->mode_rising_enabled & _BV(bit)) == 0) && (sig->mode_falling_enabled & _BV(bit))) {
                *ival = NUT_IRQMODE_FALLINGEDGE;
            } else
            if ((sig->mode_rising_enabled & _BV(bit)) && (sig->mode_falling_enabled & _BV(bit))) {
                *ival = NUT_IRQMODE_BOTHEDGE;
            } else {
                *ival = NUT_IRQMODE_NONE;
            }
            break;

        case NUT_IRQCTL_SETMODE:
            switch (*ival) {
                case NUT_IRQMODE_RISINGEDGE:
                    sig->mode_rising_enabled  |= _BV(bit);
                    sig->mode_falling_enabled &= ~_BV(bit);
                    break;
                case NUT_IRQMODE_FALLINGEDGE:
                    sig->mode_rising_enabled  &= ~_BV(bit);
                    sig->mode_falling_enabled |= _BV(bit);
                    break;
                case NUT_IRQMODE_BOTHEDGE:
                    sig->mode_rising_enabled  |= _BV(bit);
                    sig->mode_falling_enabled |= _BV(bit);
                    break;
                case NUT_IRQMODE_NONE:
                    sig->mode_rising_enabled  &= ~_BV(bit);
                    sig->mode_falling_enabled &= ~_BV(bit);
                    break;
                default:
                    rc = -1;
            }
            break;


        default:
            rc = -1;
            break;
    }

    /* Enable / disable interrupt and set mode */

    switch (sig->ios_port) {
        case NUTGPIO_PORT0:
            LPC_GPIOINT->IO0IntEnR = sig->mode_rising_enabled & sig->enabled;
            LPC_GPIOINT->IO0IntEnF = sig->mode_falling_enabled & sig->enabled;
            break;

        case NUTGPIO_PORT2:
            LPC_GPIOINT->IO2IntEnR = sig->mode_rising_enabled & sig->enabled;
            LPC_GPIOINT->IO2IntEnF = sig->mode_falling_enabled & sig->enabled;
            break;
        default:
            rc = -1;
            break;
    }

    return rc;
}

GPIO_SIGNAL sig_GPIO0 = {
    NUTGPIO_PORT0,   /* ios_port */
    Lpc17xxGpioIsr,  /* ios_handler */
    Lpc17xxGpioCtrl, /* ios_ctl */
    NULL,            /* ios_vector */
    0,               /* mode_rising_enabled */
    0                /* mode_falling_enabled */
};

GPIO_SIGNAL sig_GPIO2 = {
    NUTGPIO_PORT2,   /* ios_port */
    Lpc17xxGpioIsr,  /* ios_handler */
    Lpc17xxGpioCtrl, /* ios_ctl */
    NULL,            /* ios_vector */
    0,               /* mode_rising_enabled */
    0                /* mode_falling_enabled */

};

/*!
 * \brief Create a GPIO signal.
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
 *     GPIO_SIGNAL *sig;
 *     ...
 *     GpioPinConfigSet(NUTGPIO_PORTA, 4, GPIO_CFG_PULLUP);
 *     sig = GpioCreateIrqHandler(NUTGPIO_PORTA, 4, PinChange, NULL);
 *     if (sig) {
 *         GpioIrqEnable(sig, 4);
 *         GpioIrqSetMode(sig, 4, NUT_IRQMODE_RISINGEDGE);
 *     ...
 * }
 * \endcode
 *
 * \param port    Port number of the specified bank/port.
 * \param bit     Bit number of the specified bank/port.
 * \param handler This routine will be called by Nut/OS, when the specified
 *                pin changes its state.
 * \param arg     Argument to be passed to the interrupt handler routine.
 *
 * \return
 */
GPIO_SIGNAL *GpioCreateIrqHandler(nutgpio_port_t port, nutgpio_pin_t bit, void (*handler) (void *), void *arg)
{
    int rc;
    GPIO_SIGNAL *sig = NULL;
    if(port == NUTGPIO_PORT0)
        sig = &sig_GPIO0;
    else if (port == NUTGPIO_PORT2)
        sig = &sig_GPIO2;
    else
        return 0;
    rc = GpioRegisterIrqHandler(sig, bit, handler, arg);
    if (rc)
        return NULL;
    return sig;
}

/*!
 * \brief Register a GPIO pin interrupt handler.
 *
 * Generating interrupts on GPIO pin changes is not supported on all
 * platforms. In this case dedicated external interrupt pins may
 * be used with NutRegisterIrqHandler().
 *
 * On the LPC17xx interrupts are triggered on rising, falling or both
 * edges. Level triggering is not supported.
 *
 * After registering, interrupts are disabled. Calling GpioIrqEnable()
 * is required to activate the interrupt.
 *
 * The following code fragment registers an interrupt handler which is
 * called on a rising edge of bit 4 of the first GPIO port:
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
 *     GpioRegisterIrqHandler(&sig_GPIO0, 4, PinChange, NULL);
 *     GpioIrqSetMode(&sig_GPIO0, 4, NUT_IRQMODE_RISINGEDGE);
 *     GpioIrqEnable(&sig_GPIO0, 4);
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
            if (sig_GPIO.ir_handler == NULL) {
                rc = NutRegisterIrqHandler(&sig_GPIO, sig->ios_handler, NULL);
                if (rc == 0) {
                    /* Clear any pending interrupts */
                    LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
                    LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;

                    rc = NutIrqEnable(&sig_GPIO);
                }
            }
        }
        else {
            return -1;
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
    return (sig->ios_ctl) (sig, NUT_IRQCTL_ENABLE, NULL, bit);
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
    return (sig->ios_ctl) (sig, NUT_IRQCTL_DISABLE, NULL, bit);
}

/*!
 * \brief Query the status of a specified GPIO interrupt.
 *
 * A related interrupt handler must have been registered before calling
 * this function. See GpioRegisterIrqHandler().
 *
 * \param sig Interrupt to query
 * \param bit Bit number of the specified bank/port.
 *
 * \return 0 if interrupt is disabled, 1 of enabled
 */

int GpioIrqStatus(GPIO_SIGNAL * sig, int bit)
{
    uint32_t status;
    (sig->ios_ctl) (sig, NUT_IRQCTL_STATUS, &status, bit);

    return status;
}

/*!
 * \brief Set the GPIO interrupt mode for a pin
 *
 * \param sig Interrupt to configure.
 * \param bit Bit number of the specified bank/port.
 * \param mode one of the following modes:
 *          NUT_IRQMODE_RISINGEDGE,
 *          NUT_IRQMODE_FALLINGEDGE,
 *          NUT_IRQMODE_BOTHEDGE,
 *          NUT_IRQMODE_NONE,
 *
 * \return 0 on success, -1 otherwise.
 */
int GpioIrqSetMode(GPIO_SIGNAL * sig, int bit, int mode)
{
    return (sig->ios_ctl) (sig, NUT_IRQCTL_SETMODE, &mode, bit);
}

