/*
 * Copyright (C) 2013 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 * \file arch/cm3/board/mbed_nxp_lpc1768.c
 * \brief MBED eval board initialization.
 *
 *
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <inttypes.h>

#include <sys/timer.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <dev/gpio.h>
#include <dev/sdram.h>

#include <sys/heap.h>

/*!
 * \brief Delay loop.
 *
 * We are running prior to Nut/OS timer initialization and cannot use
 * NutSleep, not even NutDelay.
 *
 * \param Number of loops to execute.
 */

static void Delay(int n)
{
    int l;

    for (l = 0; l < n; l++) {
        _NOP();
    }
}


/*!
 * \brief   Early MBED LPC1768 hardware initialization.
 *
 * This routine is called during system initalization.
 */
void NutBoardInit(void)
{
    /* Configure Ethernet PHY */
    /* Reset the PHY and configure the bootstrap pins using the pullup / pulldown resistors */

    GpioPinConfigSet(NUTGPIO_PORT1, 28, GPIO_CFG_OUTPUT);       /* PHY Reset */
    GpioPinSetLow(NUTGPIO_PORT1, 28);                           /* Put PHY into reset */

    GpioPinConfigSet(NUTGPIO_PORT1, 9,  GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL0);    /* PHY_AD1    -- ETH_RXD0 */
    GpioPinConfigSet(NUTGPIO_PORT1, 10, GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL0);    /* PHY_AD1    -- ETH_RXD1 */
    GpioPinConfigSet(NUTGPIO_PORT1, 8,  GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL0);    /* PHY_LEDCFG -- ETH_CRS  */
    GpioPinConfigSet(NUTGPIO_PORT1, 14, GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL0);    /* PHY_MDIXEN -- ETH_RXER */

    GpioPinConfigSet(NUTGPIO_PORT1, 25, GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL0);    /* PHY_AN0 */
    GpioPinConfigSet(NUTGPIO_PORT1, 26, GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL0);    /* PHY_AN1 */

    Delay(25000);
    GpioPinSetHigh(NUTGPIO_PORT1, 28);                           /* Put PHY into reset */
    Delay(1000);
}

/*!
 * \brief   Extended system initialisation.
 *
 * This routine is called during system initialisation right before the idle
 * thread is created.
 */
void NutIdleInit(void)
{
}
