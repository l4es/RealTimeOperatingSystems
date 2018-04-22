/*!
 * Copyright (C) 2001-2010 by egnite Software GmbH
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
 * $Log: ostimer.c,v $
 *
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <arch/avr32.h>
#include <dev/irqreg.h>
#include <sys/timer.h>

#include <arch/avr32/ihndlr.h>

#include <avr32/io.h>


/*!
 * \addtogroup xgNutArchAvr32OsTimer
 */
/*@{*/

#ifndef NUT_TICK_FREQ
#define NUT_TICK_FREQ   1000UL
#endif

IRQ_HANDLER sig_sysCompare = {
#ifdef NUT_PERFMON
    0,                          /* Interrupt counter, ir_count. */
#endif
    NULL,                       /* Passed argument, ir_arg. */
    NULL,                       /* Handler subroutine, ir_handler. */
    NULL                        /* Interrupt control, ir_ctl. */
};



/*!
* \brief System Compare interrupt entry.
*/
static SIGNAL(SystemCompareIrqEntry)
{
    IRQ_ENTRY();
    uint32_t compare;
    compare = Get_system_register(AVR32_COMPARE);
    Set_system_register(AVR32_COMPARE, 0);
    if (sig_sysCompare.ir_handler) {
        (sig_sysCompare.ir_handler) (sig_sysCompare.ir_arg);
    }
#if __AVR32_AP7000__ || __AT32AP7000__ || __AVR32_UC3A0512ES__ || __AVR32_UC3A1512ES__
    /* AP7000 and UC3 prior to rev H doesn't clear COUNT on compare match, so we need to
       offset COMPARE */
    compare += NutGetCpuClock() / NUT_TICK_FREQ;
    if (!compare)               // Avoid disabling compare.
        ++compare;
#endif
    Set_system_register(AVR32_COMPARE, compare);
    IRQ_EXIT();
}

/*!
 * \brief Initialize system timer.
 *
 * This function is automatically called by Nut/OS
 * during system initialization.
 *
 * Nut/OS uses on-chip tick counter for its timer services.
 * Applications should not modify any registers of this
 * timer, but make use of the Nut/OS timer API. Timer 1
 * and timer 2 are available to applications.
 */
void NutRegisterTimer(void (*handler) (void *))
{
    /* Set compare value for the specified tick frequency. */
    Set_system_register(AVR32_COMPARE, NutGetCpuClock() / NUT_TICK_FREQ + Get_system_register(AVR32_COUNT));

    sig_sysCompare.ir_handler = handler;

    register_interrupt(SystemCompareIrqEntry, AVR32_CORE_COMPARE_IRQ, AVR32_INTC_INT0);
}

/*!
 * \brief Return the CPU clock in Hertz.
 *
 * On AVR32 CPUs the processor clock may differ from the clock
 * driving the peripherals. In this case NutArchClockGet() will
 * provide the correct clock depending on it's argument.
 *
 * \return CPU clock frequency in Hertz.
 */
uint32_t NutArchClockGet(int idx);


/*!
 * \brief Return the number of system ticks per second.
 *
 * \return System tick frequency in Hertz.
 */
uint32_t NutGetTickClock(void)
{
    return NUT_TICK_FREQ;
}

/*!
 * \brief Calculate system ticks for a given number of milliseconds.
 */
uint32_t NutTimerMillisToTicks(uint32_t ms)
{
#if (NUT_TICK_FREQ % 1000)
    if (ms >= 0x3E8000UL)
        return (ms / 1000UL) * NUT_TICK_FREQ;
    return (ms * NUT_TICK_FREQ + 999UL) / 1000UL;
#else
    return ms * (NUT_TICK_FREQ / 1000UL);
#endif
}

/*@}*/

#if defined ( MCU_AVR32UC3L064 )
#include "ostimer_uc3l.c"
#else
#include "ostimer_uc3a.c"
#endif
