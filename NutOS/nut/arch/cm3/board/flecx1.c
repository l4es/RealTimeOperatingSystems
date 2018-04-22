/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/cm3/board/flecx1.c
 * \brief FLECX 1 board initialization.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <arch/cm3.h>

/********** TODO: Clock setup is done in arch/cm3/dev/nxp/lpc176x_clk.c
                  Correct config values should be made configurable in the
                  configurator... Left this code as reference...
          eventually pin setup has to be integrated here later
*/

__attribute__ ((section(".crp"))) const uint32_t CRP_WORD = 0xFFFFFFFF;

/*!
 * \brief Early hardware initialization for FLECX 1 board.
 *
 * This routine is called during system initialization, if NUT_INIT_BOARD
 * has been enabled in the architecture configuration.
 *
 * It will mainly set up the basic clocks for the CPU to run at 99.6MHz.
 */
void NutBoardInit(void)
{
#if 0
    /* Set flash for 100MHz CPU. */
    outr(SC_FLASHCFG, (inr(SC_FLASHCFG) & ~SC_FLASHTIM) | (5 << SC_FLASHTIM_LSB));

    /* Divide all peripheral clocks by 4. */
    outr(SC_PCLKSEL0, 0);
    outr(SC_PCLKSEL1, 0);

    /* Enable main oscillator. */
    outr(SC_SCS, SC_OSCEN);
    while ((inr(SC_SCS) & SC_OSCSTAT) == 0);
    /* Set CPU clock divider to 4. */
    outr(SC_CCLKCFG, (4 - 1) << SC_CCLKSEL_LSB);

    /* Select main clock as clock source for PLL0. */
    outr(SC_CLKSRCSEL, SC_CLKSRC_MCLK);

    /* Configure PLL0 divider and multiplier to 99.6 MHz. */
    outr(SC_PLL0CFG, ((5 - 1) << SC_NSEL_LSB) | ((83 - 1) << SC_MSEL_LSB));
    outr(SC_PLL0FEED, PLLFEED_FEED1);
    outr(SC_PLL0FEED, PLLFEED_FEED2);

    /* Enable PLL0. */
    outr(SC_PLL0CON, SC_PLLE);
    outr(SC_PLL0FEED, PLLFEED_FEED1);
    outr(SC_PLL0FEED, PLLFEED_FEED2);
    /* Wait for PLL0 locked. */
    while ((inr(SC_PLL0STAT) & SC_PLOCK) == 0);

    /* Enable and connect PLL0. */
    outr(SC_PLL0CON, SC_PLLE | SC_PLLC);
    outr(SC_PLL0FEED, PLLFEED_FEED1);
    outr(SC_PLL0FEED, PLLFEED_FEED2);
    /* Wait for PLL0 ready. */
    while ((inr(SC_PLL0STAT) & (SC_PLLE_STAT | SC_PLLC_STAT)) != (SC_PLLE_STAT | SC_PLLC_STAT));

    /* Enable peripheral clocks. */
    outr(SC_PCONP,
        SC_PCTIM0 | SC_PCTIM1 | SC_PCUART0 | SC_PCPWM1 |
        SC_PCI2C0 | SC_PCSPI | SC_PCRTC | SC_PCSSP1 |
        SC_PCGPIO | SC_PCI2C1 | SC_PCTIM2 | SC_PCI2C2);
    /* Clock output pin configuration. */
    outr(SC_CLKOUTCFG, SC_CLKOUT_EN | ((2 - 1) << SC_CLKOUTDIV_LSB));
    outr(PINSEL(3), inr(PINSEL(3)) | PS3_P1_27_CLKOUT);
#endif
}

