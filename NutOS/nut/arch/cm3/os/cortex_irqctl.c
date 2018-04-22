/*
 * Copyright (C) 2013 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: cortex_irqctl.c 6035 2015-03-12 17:15:58Z u_bonnes $
 *
 * Common function to control the interrupt settings of IRQs
 * of cortex CPUs. External interrupts with level control still
 * need special handling.
 * \endverbatim
 */

/*!
 * \brief CM3 common interrupt control.
 *
 * \param interrupt    Interrupt number.
 * \param sig          Interrupt control block
 * \param cmd          Control command.
 *                     - NUT_IRQCTL_INIT Initialize and disable interrupt.
 *                     - NUT_IRQCTL_STATUS Query interrupt status.
 *                     - NUT_IRQCTL_ENABLE Enable interrupt.
 *                     - NUT_IRQCTL_DISABLE Disable interrupt.
 *                     - NUT_IRQCTL_GETMODE Query interrupt mode.
 *                     - NUT_IRQCTL_SETMODE Set interrupt mode (NUT_IRQMODE_LEVEL or NUT_IRQMODE_EDGE).
 *                       Always treated as NUT_IRQMODE_LEVEL.
 *                     - NUT_IRQCTL_GETPRIO Query interrupt priority.
 *                     - NUT_IRQCTL_SETPRIO Set interrupt priority.
 *                     - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
 * \param param        Pointer to optional parameter.
 * \param def_priority Priority to use on init
 *
 * \return 0 on success, -1 otherwise.
 */

#include <dev/irqreg.h>

int CM3_IrqCtl(int cmd, void *param, IRQn_Type interrupt,
               void (*pfnHandler)(void*),
               IRQ_HANDLER* sig, int def_priority)
{
    int rc = 0;
    unsigned int *ival = (unsigned int *)param;
    int enabled;
#if       (__CORTEX_M >= 0x03)
    enabled = (NVIC->ISER[((uint32_t)(interrupt) >> 5)] &
               (1 << ((uint32_t)(interrupt) & 0x1F)));
#else
    enabled = (NVIC->ISER[0] & (1 << ((uint32_t)(interrupt) & 0x1F)));
#endif
    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(interrupt);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(interrupt, pfnHandler);
        /* Initialize with defined priority. */
        NVIC_SetPriority(interrupt, def_priority);
        /* Clear interrupt */
        NVIC_ClearPendingIRQ(interrupt);
        break;
    case NUT_IRQCTL_STATUS:
        if (enabled) {
            *ival |= 1;
        }
        else {
            *ival &= ~1;
        }
        break;
    case NUT_IRQCTL_ENABLE:
        enabled = 1;
        break;
    case NUT_IRQCTL_DISABLE:
        enabled = 0;
        break;
    case NUT_IRQCTL_GETMODE:
        *ival = NUT_IRQMODE_LEVEL;
        break;
    case NUT_IRQCTL_SETMODE:
        break;
    case NUT_IRQCTL_GETPRIO:
        *ival = NVIC_GetPriority(interrupt);
        break;
    case NUT_IRQCTL_SETPRIO:
        NVIC_SetPriority(interrupt, *ival);
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (unsigned int)sig->ir_count;
        sig->ir_count = 0;
        break;
#endif
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        NVIC_EnableIRQ(interrupt);
    }
    return rc;
}
