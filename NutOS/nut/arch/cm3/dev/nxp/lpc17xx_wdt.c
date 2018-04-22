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
 * $Id:$
 * \endverbatim
 */


#include <sys/timer.h>
#include <dev/watchdog.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <arch/cm3/nxp/lpc176x_wdt.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_wwdt.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_wwdt.h>
#else
#warning "Unknown LPC familiy"
#endif

/*!
 * \addtogroup xgNutArchCm3Lpc17xxDevWatchDog
 */
/*@{*/

//static ureg_t nested;

/*!
 * \brief   Start the LPC17xx hardware watch dog timer.
 *
 * \param   timeout  watchdog timeout in µs
 * \return  0 on success or -1 in case of an error.
 */

static int Lpc17xxWatchDogSetTimeOut(uint32_t timeout)
{
    uint32_t val = WDT_GET_FROM_USEC(timeout);
    int      rc  = 0;

    if (val < WDT_TIMEOUT_MIN) {
        val = WDT_TIMEOUT_MIN;
        rc = -1;
    } else
    if (val > WDT_TIMEOUT_MAX) {
        val = WDT_TIMEOUT_MAX;
        rc = -1;
    }

    LPC_WDT->TC = val;

    return rc;
}



/*!
 * \brief Start the LPC17xx hardware watch dog timer.
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
uint32_t Lpc17xxWatchDogStart(uint32_t ms, uint32_t xmode)
{
#if defined (MCU_LPC176x)
    /* Select Watchdog clock source to IRC Clock */
    LPC_WDT->CLKSEL = WDT_WDCLKSEL_RC;
#endif

    /* Clear the Watchdog timeout flag */
    LPC_WDT->MOD &= ~WDT_WDMOD_WDTOF;

    Lpc17xxWatchDogDisable();

    Lpc17xxWatchDogSetTimeOut(ms * 1000);

    /* Set reset mode to be compatible with other implementations too */
    LPC_WDT->MOD |= WDT_WDMOD_WDRESET;

#if defined (MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    /* Disable watchdog protect mode */
    LPC_WDT->MOD &= ~WDT_WDMOD_WDPROTECT;
#endif

    /* Enable the watchdog */
    LPC_WDT->MOD |= WDT_WDMOD_WDEN;
    /* We have to feed the watchdog once after enabling it */
    Lpc17xxWatchDogRestart();

    //nested = 1;

    return ms;
}

/*!
 * \brief Re-start the LPC17xx hardware watch dog timer.
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
void Lpc17xxWatchDogRestart(void)
{
    /* Standard watchdog feed sequence */

    __disable_irq();

    LPC_WDT->FEED = 0xAA;

    LPC_WDT->FEED = 0x55;

    __enable_irq();
}

/*!
 * \brief Disable the LPC17xx hardware watch dog timer.
 *
 * Att: Disabling the watchdog is not supported by hardware
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
void Lpc17xxWatchDogDisable(void)
{
    /* Clear the Watchdog timeout flag */
    LPC_WDT->MOD &= ~WDT_WDMOD_WDTOF;

    /* Disabling the watchdog is not supported by hardware
        if (nested) {
            nested++;
        }
        LPC_WDT->MOD &= ~WDT_WDMOD_WDEN;
    */
    return;
}

/*!
 * \brief Enable the LPC17xx hardware watch dog timer.
 *
 * Att: Disabling and re-enabling the watchdog is not supported by hardware
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
void Lpc17xxWatchDogEnable(void)
{
    /* Disabling and re-enabling the watchdog is not supported by hardware
    if (nested > 1 && --nested == 1) {

        LPC_WDT->MOD |= WDT_WDMOD_WDEN;
        // We have to feed the watchdog once after enabling it
        Lpc17xxWatchDogRestart();
    }
    */
}

/*@}*/

