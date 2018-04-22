/*
 * Copyright (C) 2014-2017 by Uwe Bonnes
 *                                (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: stm32_gpioirq.c 6591 2017-02-15 15:35:32Z u_bonnes $
 * \endverbatim
 */

/* GPIO Configuration for the GPIO of L1/F2/F4 */

#include <stdlib.h>

#include <cfg/os.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/nutdebug.h>

/*!
 * \brief Common interrupt routine dedicated EXTI interrupts.
 *
 * \param arg  *GPIO_SIGNALS
 *
 */
static void ExtiIsr(void *arg) {
    GPIO_SIGNAL *signal = (GPIO_SIGNAL *) arg;
    uint32_t pending = EXTI_PR;
    if(pending & (1 << signal->ios_pin)) {
        GPIO_VECTOR *vector = &signal->ios_vector;
        EXTI_PR = (1 << signal->ios_pin);
        if (vector->iov_handler)
            (vector->iov_handler)(vector->iov_arg);
    }
}

/*!
 * \brief Common interrupt routine for shared EXTI interrupts.
 *
 * \param arg  Linked list of *GPIO_SIGNALS
 *
 * The linked list is followed and the iov_handler called
 * if the appropriate pending bit is set.
 */
static void ExtiSharedIsr(void *arg) {
    GPIO_SIGNAL *signal;
    uint32_t pending = EXTI_PR;
    for (signal = (GPIO_SIGNAL *) arg; signal; signal = signal->sig_next) {
        /* It seems, "pending" is only set for an unmasked IRQ, so no need
           to check here. */
        if (pending & (1 << signal->ios_pin)) {
            GPIO_VECTOR *vector = &signal->ios_vector;
            EXTI_PR = (1 << signal->ios_pin);
            if (vector->iov_handler)
                (vector->iov_handler)(vector->iov_arg);
        }
    }
    /* Reset Pending bit */
}

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
    GPIO_SIGNAL *sig;
    int port_nr = (port - GPIOA_BASE) >> 10;
    sig = malloc(sizeof(GPIO_SIGNAL));
    if (sig) {
        int rc;
        sig->ios_port = port_nr;
        sig->ios_pin  = bit;
        sig->sig_next = NULL;
        rc = GpioRegisterIrqHandler(sig, bit, handler, arg);
        if (rc) {
            free(sig);
            return 0;
        }
    }
    return sig;
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
    IRQ_HANDLER *isrhandler;
    GPIO_VECTOR *vector;
    int shared;
    int rc = -1;

#if defined(MCU_STM32F0) || defined(MCU_STM32L0)
    shared = 1;
    if (bit < 2)
        isrhandler = &sig_INTERRUPT0_1;
    else if (bit < 4)
        isrhandler = &sig_INTERRUPT2_3;
    else
        isrhandler = &sig_INTERRUPT4_15;
#else
    shared = 0;
    switch (bit) {
    case 0:
        isrhandler = &sig_INTERRUPT0;
        break;
    case 1:
        isrhandler = &sig_INTERRUPT1;
        break;
    case 2:
        isrhandler = &sig_INTERRUPT2;
        break;
    case 3:
        isrhandler = &sig_INTERRUPT3;
        break;
    case 4:
        isrhandler = &sig_INTERRUPT4;
        break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
        shared = 1;
        isrhandler = &sig_INTERRUPT9_5;
        break;
    default:
        shared = 1;
        isrhandler = &sig_INTERRUPT15_10;
    }
#endif
    vector = &sig->ios_vector;
    vector->iov_handler = handler;
    vector->iov_arg = arg;
    /* The ISR is not yet installed. Add first GPIO Vector */
    if (0 == isrhandler->ir_handler) {
        if (shared)
            rc = NutRegisterIrqHandler(isrhandler, ExtiSharedIsr, sig);
        else
            rc = NutRegisterIrqHandler(isrhandler, ExtiIsr, sig);
        if (0 == rc ) {
            /* Clear any pending interrupts and mask */
            EXTI_PR   =  (1 << bit);
            EXTI_IMR &= ~(1 << bit);
            rc = NutIrqEnable(isrhandler);
        }
    }
    else if (shared) {
        /* Interrupt already installed, check chain */
        GPIO_SIGNAL *sig_chain;
        sig_chain = isrhandler->ir_arg;
        while (sig_chain) {
            if(bit == sig_chain->ios_pin ) {
                /* Pin is already mapped */
                break;
            }
            if (NULL == sig_chain->sig_next) {
                /* We found last slot in chain */
                /* Clear any pending interrupts and mask */
                EXTI_PR   =  (1 << bit);
                EXTI_IMR &= ~(1 << bit);
                sig_chain->sig_next = sig;
                rc = 0;
                break;
            }
            sig_chain = sig_chain->sig_next;
        }
    }
    if (0 == rc) {
        /* Route  the signal */
        uint32_t *cr;
        int offset;
        uint32_t crx;

#if defined(AFIO_BASE)
        /* Any GpioPortConfigSet already enabled the AFIO Clock */
        cr = (uint32_t *)AFIO->EXTICR;
#else
        cr = (uint32_t *)SYSCFG->EXTICR;
#endif
        offset = (bit % 4) * 4;

        crx =  cr[bit / 4];
        crx &= ~(0xf << offset);
        crx |=  (sig->ios_port << offset);
        cr[bit / 4] = crx;
    }
    return rc;
}

int GpioIrqSetMode(GPIO_SIGNAL *sig, int bit, int mode)
{
    int rc = 0;

    switch (mode) {
    case NUT_IRQMODE_RISINGEDGE:
        EXTI_RTSR |=  (1 << bit);
        EXTI_FTSR &= ~(1 << bit);
        break;
    case NUT_IRQMODE_FALLINGEDGE:
        EXTI_RTSR &= ~(1 << bit);
        EXTI_FTSR |=  (1 << bit);
        break;
    case NUT_IRQMODE_BOTHEDGE:
        EXTI_RTSR |=  (1 << bit);
        EXTI_FTSR |=  (1 << bit);
        break;
    case NUT_IRQMODE_NONE:
        EXTI_RTSR &= ~(1 << bit);
        EXTI_FTSR &= ~(1 << bit);
        break;
    default:
        rc = -1;
    }
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
    EXTI_IMR |=  (1 << bit);
    return 0;
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
    EXTI_IMR &= ~(1 << bit);
    return 0;
}
