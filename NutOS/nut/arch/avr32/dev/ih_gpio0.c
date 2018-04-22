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

#include <arch/avr32.h>
#include <dev/irqreg.h>
#include <avr32/io.h>
#include <dev/gpio.h>

#include <sys/atom.h>

#include <arch/avr32/ihndlr.h>

#ifndef NUT_IRQPRI_GPIO
#define NUT_IRQPRI_GPIO  0
#endif

static int const GPIOpinNumber[] = {
#if defined(AVR32_PIN_PA00)
AVR32_PIN_PA00,
#else
-1,
#endif
#if defined(AVR32_PIN_PA01)
AVR32_PIN_PA01,
#else
-1,
#endif
#if defined(AVR32_PIN_PA02)
AVR32_PIN_PA02,
#else
-1,
#endif
#if defined(AVR32_PIN_PA03)
AVR32_PIN_PA03,
#else
-1,
#endif
#if defined(AVR32_PIN_PA04)
AVR32_PIN_PA04,
#else
-1,
#endif
#if defined(AVR32_PIN_PA05)
AVR32_PIN_PA05,
#else
-1,
#endif
#if defined(AVR32_PIN_PA06)
AVR32_PIN_PA06,
#else
-1,
#endif
#if defined(AVR32_PIN_PA07)
AVR32_PIN_PA07,
#else
-1,
#endif
#if defined(AVR32_PIN_PA08)
AVR32_PIN_PA08,
#else
-1,
#endif
#if defined(AVR32_PIN_PA09)
AVR32_PIN_PA09,
#else
-1,
#endif
#if defined(AVR32_PIN_PA10)
AVR32_PIN_PA10,
#else
-1,
#endif
#if defined(AVR32_PIN_PA11)
AVR32_PIN_PA11,
#else
-1,
#endif
#if defined(AVR32_PIN_PA12)
AVR32_PIN_PA12,
#else
-1,
#endif
#if defined(AVR32_PIN_PA13)
AVR32_PIN_PA13,
#else
-1,
#endif
#if defined(AVR32_PIN_PA14)
AVR32_PIN_PA14,
#else
-1,
#endif
#if defined(AVR32_PIN_PA15)
AVR32_PIN_PA15,
#else
-1,
#endif
#if defined(AVR32_PIN_PA16)
AVR32_PIN_PA16,
#else
-1,
#endif
#if defined(AVR32_PIN_PA17)
AVR32_PIN_PA17,
#else
-1,
#endif
#if defined(AVR32_PIN_PA18)
AVR32_PIN_PA18,
#else
-1,
#endif
#if defined(AVR32_PIN_PA19)
AVR32_PIN_PA19,
#else
-1,
#endif
#if defined(AVR32_PIN_PA20)
AVR32_PIN_PA20,
#else
-1,
#endif
#if defined(AVR32_PIN_PA21)
AVR32_PIN_PA21,
#else
-1,
#endif
#if defined(AVR32_PIN_PA22)
AVR32_PIN_PA22,
#else
-1,
#endif
#if defined(AVR32_PIN_PA23)
AVR32_PIN_PA23,
#else
-1,
#endif
#if defined(AVR32_PIN_PA24)
AVR32_PIN_PA24,
#else
-1,
#endif
#if defined(AVR32_PIN_PA25)
AVR32_PIN_PA25,
#else
-1,
#endif
#if defined(AVR32_PIN_PA26)
AVR32_PIN_PA26,
#else
-1,
#endif
#if defined(AVR32_PIN_PA27)
AVR32_PIN_PA27,
#else
-1,
#endif
#if defined(AVR32_PIN_PA28)
AVR32_PIN_PA28,
#else
-1,
#endif
#if defined(AVR32_PIN_PA29)
AVR32_PIN_PA29,
#else
-1,
#endif
#if defined(AVR32_PIN_PA30)
AVR32_PIN_PA30,
#else
-1,
#endif
#if defined(AVR32_PIN_PA31)
AVR32_PIN_PA31,
#else
-1,
#endif
};

static int GpioIrqCtl(int cmd, void *param, int bit);

GPIO_SIGNAL sig_GPIO = {
	NULL,       /* ios_sig */
	NULL,       /* ios_handler */
	GpioIrqCtl, /* ios_ctl */
	NULL,       /* ios_vector */
};


/*!
 * \brief GPIO bank 0 interrupt entry.
 */
SIGNAL(GPIO0IrqEntry)
{
    IRQ_ENTRY();
#ifdef NUT_PERFMON
    sig_GPIO.ir_count++;
#endif

	GPIO_VECTOR *vct;
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[0];
	uint32_t const ier = gpio_port->ier;
	uint32_t port_status = gpio_port->ifr & ier;

	vct = sig_GPIO.ios_vector;
	while (port_status) {
		if ((port_status & 1) != 0 && vct->iov_handler) {
			(vct->iov_handler) (vct->iov_arg);
		}
		port_status >>= 1;
		vct++;
	}

#if (AVR32_GPIO_H_VERSION == 211)
	// Clear interrupt
	// Workaround errata bug in UC3L, simply setting IRFC with interrupts enabled won't work.
	gpio_port->ierc = ier;
	gpio_port->ifrc = ier;
	gpio_port->pvr;
	gpio_port->iers = ier;
#else
	gpio_port->ifrc = ier;
	gpio_port->pvr;
#endif

    IRQ_EXIT();
}

/*!
* \brief Timer/Counter 0 interrupt control.
*
* \param cmd   Control command.
*              - NUT_IRQCTL_INIT Initialize and disable interrupt.
*              - NUT_IRQCTL_STATUS Query interrupt status.
*              - NUT_IRQCTL_ENABLE Enable interrupt.
*              - NUT_IRQCTL_DISABLE Disable interrupt.
*              - NUT_IRQCTL_GETMODE Query interrupt mode.
*              - NUT_IRQCTL_SETMODE Set interrupt mode (NUT_IRQMODE_LEVEL or NUT_IRQMODE_EDGE).
*              - NUT_IRQCTL_GETPRIO Query interrupt priority.
*              - NUT_IRQCTL_SETPRIO Set interrupt priority.
*              - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
* \param param Pointer to optional parameter.
*
* \return 0 on success, -1 otherwise.
*/
static int GpioIrqCtl(int cmd, void *param, int bit)
{
    int rc = 0;
    uint32_t *ival = (uint32_t *) param;
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[0];
	ureg_t ier = gpio_port->ier;
	int_fast8_t enabled = ier;

	if ( bit >= sizeof(GPIOpinNumber) || GPIOpinNumber[bit] == -1 )
		return -1;

	/* Disable interrupt. */
	if (enabled) {
		gpio_port->ierc = 0xFFFFFFFF;
	}

    switch (cmd) {
		case NUT_IRQCTL_INIT:
		break;

        case NUT_IRQCTL_STATUS:
            if (ier & _BV(bit)) {
                *ival = 1;
            } else {
                *ival = 0;
            }
            break;

        case NUT_IRQCTL_ENABLE:
            ier |= _BV(bit);
			enabled = 1;
			gpio_port->gfers = (1 << bit);
			register_interrupt(GPIO0IrqEntry, AVR32_GPIO_IRQ_0 + (GPIOpinNumber[bit]/AVR32_GPIO_IRQS_PER_GROUP), NUT_IRQPRI_GPIO);
            break;

        case NUT_IRQCTL_DISABLE:
			ier &= ~_BV(bit);
			enabled = 0;
            break;

        case NUT_IRQCTL_GETMODE:
            if (((gpio_port->imr1 & _BV(bit)) == 0) && ((gpio_port->imr0 & _BV(bit)) == 1)) {
                *ival = NUT_IRQMODE_RISINGEDGE;
            } else if (((gpio_port->imr1 & _BV(bit)) == 1) && ((gpio_port->imr0 & _BV(bit)) == 0)) {
                *ival = NUT_IRQMODE_FALLINGEDGE;
            } else if (((gpio_port->imr1 & _BV(bit)) == 0) && ((gpio_port->imr0 & _BV(bit)) == 0)) {
                *ival = NUT_IRQMODE_BOTHEDGE;
            } else {
                *ival = NUT_IRQMODE_NONE;
            }
            break;

        case NUT_IRQCTL_SETMODE:
            switch (*ival) {
                case NUT_IRQMODE_RISINGEDGE:
					gpio_port->imr1 &= ~_BV(bit);
					gpio_port->imr0 |= _BV(bit);
                    break;
                case NUT_IRQMODE_FALLINGEDGE:
					gpio_port->imr1 |= _BV(bit);
					gpio_port->imr0 &= ~_BV(bit);
                    break;
                case NUT_IRQMODE_BOTHEDGE:
					gpio_port->imr1 &= ~_BV(bit);
					gpio_port->imr0 &= ~_BV(bit);
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
	if (enabled) {
		gpio_port->iers = ier;
	}

    return rc;
}
