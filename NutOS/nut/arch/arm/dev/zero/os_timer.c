/*
 * Copyright (C) 2011 by egnite GmbH
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

/*!
 * \file arch/arm/dev/zero/os_timer.c
 * \brief System timer for Zero CPU.
 *
 * This code implements a system timer for an imaginary CPU.
 * It may serve as a template when porting Nut/OS to a new target.
 * Implementers should look out for 'TODO' comments.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/* The Configurator will create header files with configuration
   parameters in nutbld/include/cfg/ (build tree). If nothing specific
   has been configured, the compiler will include the original files
   from nut/include/cfg/ (source tree). */
#include <cfg/os.h>
#include <cfg/clock.h>

/*
 * TODO: Include architecture specific header files, if required.
 *
 * To implement your timer hardware, you need to access several
 * hardware registers and therefore need to include the header files,
 * where they are defined. However, if properly configured, this is usually
 * done automatically via compiler.h, which in turn is included almost
 * anywhere.
 *
 * Sometimes the deep level of included headers may become tricky to
 * follow or something may get broken by later changes. It is legitimate
 * to include essential header files, even if they are already included
 * indirectly by other header files.
 */

#include <sys/timer.h>

/*!
 * \addtogroup xgNutArchArmZeroOsTimer
 */
/*@{*/

#ifndef NUT_TICK_FREQ
/*!
 * \brief System timer interrupt frequency.
 *
 * Specifies the number of interrupts per second, typically 1000.
 * In order to reduce overhead, you may choose lower values. Note,
 * that Nut/OS API timer values are given in milliseconds. Thus,
 * lower values will reduce the available resolution, while
 * larger values may not provide any benefit.
 */
#define NUT_TICK_FREQ   1000UL
#endif

/*!
 * \brief Initialize system timer.
 *
 * Applications must not call this function.
 *
 * It is automatically called by Nut/OS during initialization to register
 * the system timer interrupt handler. It is an essential part of the
 * hardware dependant code and must be available for any platform that is
 * running Nut/OS.
 *
 * The number of system timer interrupts is define by \ref NUT_TICK_FREQ.
 *
 * Timer interrupts are enabled when this function returns.
 *
 * \param handler This routine should be called each time, when a
 *                system timer interrupt occurs.
 */
void NutRegisterTimer(void (*handler) (void *))
{
    /*
     * TODO: Select the timer hardware.
     *
     * Note, that this timer will no longer be available for
     * applications.
     */

    /*
     * TODO: Initialize the timer hardware.
     *
     * It is recommended to use NutClockGet() to determine the input
     * clock frequency.
     */

    /*
     * TODO: Enable timer interrupts.
     *
     * You may, but do not need to use NutRegisterIrqHandler() and
     * NutIrqEnable(). Implementing Nut/OS interrupt handler
     * registration is not always trivial, therefore you may hard code
     * the handler call for a first port. However, many platform
     * independent drivers use it as well and sooner or later you need
     * to implement interrupt handling anyway.
     */
}

#ifndef NUT_CPU_FREQ
/*!
 * \brief Return the specified clock frequency.
 *
 * Applications must not call this function, but use NutClockGet()
 * instead.
 *
 * Simple implementations may not provide this function, in which case
 * \ref NUT_CPU_FREQ must define the CPU frequency in Hertz. This can
 * be done in the Configurator.
 *
 * \param idx This zero based index specifies the clock to retrieve. The
 *            number of available hardware clocks depends on the target
 *            harware and is specified by NUT_HWCLK_MAX + 1. Typically
 *            \ref NUT_HWCLK_CPU is used to retrieve the current CPU
 *            clock. Additional indices may be available to retrieve one
 *            or more peripheral clocks or a special slow clock.
 *
 * \return Clock frequency in Hertz.
 */
uint32_t NutArchClockGet(int idx)
{
    /* TODO: Calculate the specified clock from current register settings. */
    return 100000000;
}
#endif

/*!
 * \brief Return the number of system ticks per second.
 *
 * This routine is used by Nut/OS to convert tick counts into
 * milliseconds.
 *
 * Applications typically do not deal with system ticks. Instead,
 * they use milliseconds to specify timeouts or call NutGetMillis()
 * and NutGetSeconds() to retrieve an elapsed time.
 *
 * \return System tick frequency in Hertz, typically the value of
 *         \ref NUT_TICK_FREQ.
 */
uint32_t NutGetTickClock(void)
{
    return NUT_TICK_FREQ;
}

/*!
 * \brief Calculate system ticks for a given number of milliseconds.
 *
 * This routine is used by Nut/OS to retrieve the number of system
 * ticks for a given timeout.
 *
 * \param ms Number of milliseconds.
 *
 * \return Number of system ticks. The resolution is limited to the
 *         granularity of the system timer.
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
