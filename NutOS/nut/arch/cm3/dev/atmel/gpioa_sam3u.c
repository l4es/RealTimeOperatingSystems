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
 * $Id: gpioa_sam3u.c 4590 2012-09-10 10:26:01Z olereinhardt $
 */

#include <arch/cm3.h>

#include <stdlib.h>
#include <string.h>

#include <dev/gpio.h>

/*!
 * \brief PIO interrupt service.
 */
static void PioIsrA(void *arg)
{
    GPIO_VECTOR *vct = (GPIO_VECTOR *)arg;
    unsigned int isr = inr(AT91C_PIOA_ISR);

    while (isr) {
        if ((isr & 1) != 0 && vct->iov_handler) {
            (vct->iov_handler) (vct->iov_arg);
        }
        isr >>= 1;
        vct++;
    }
}

/*!
 * \brief PIO interrupt control.
 */
static int PioCtlA(int cmd, void *param, int bit)
{
    int rc = 0;
    unsigned int *ival = (unsigned int *) param;
    uint32_t enabled = inr(AT91C_PIOA_IMR) & _BV(bit);

    /* Disable interrupt. */
    if (enabled) {
        outr(AT91C_PIOA_IDR, _BV(bit));
    }
    switch (cmd) {
    case NUT_IRQCTL_STATUS:
        if (enabled) {
            *ival |= 1;
        } else {
            *ival &= ~1;
        }
        break;
    case NUT_IRQCTL_ENABLE:
        enabled = 1;
        break;
    case NUT_IRQCTL_DISABLE:
        enabled = 0;
        break;
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        outr(AT91C_PIOA_IER, _BV(bit));
    }
    return rc;
}

GPIO_SIGNAL sig_GPIO1 = {
    &sig_PIOA,  /* ios_sig */
    PioIsrA,    /* ios_handler */
    PioCtlA,    /* ios_ctl */
    NULL        /* ios_vector */
};
