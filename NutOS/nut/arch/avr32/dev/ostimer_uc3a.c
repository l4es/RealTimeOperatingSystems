/*
* Copyright (C) 2001-2007 by egnite Software GmbH. All rights reserved.
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
* THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
* ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
* SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*
* For additional information see http://www.ethernut.de/
*
*/

// This is the source implementation for model specific
// routines for UC3 family
// Functions here are documented in ostimer.c

/*!
 * \brief Determine the PLL output clock frequency.
 *
 * \param pll Specifies the PLL, 0 for PLL0, 1 for PLL1.
 *
 * \return Frequency of the selected PLL in Hertz.
 */
static uint32_t AVR32GetPllClock(int pll)
{
    uint32_t rc;
    uint32_t osc = 0;

    if ( AVR32_PM.PLL[pll].pllosc )
        osc = 0;
    else
        osc = OSC0_VAL;

    /*
    * The main oscillator clock frequency is specified by the
    * configuration. It's usually equal to the on-board crystal.
    */
    rc = osc;

    if ( AVR32_PM.PLL[pll].pllen ) {
        uint32_t divider = AVR32_PM.PLL[pll].plldiv;
        uint32_t multiplier = AVR32_PM.PLL[pll].pllmul;

        if ( divider )
            rc *= (multiplier +1)/divider;
        else
            rc *= 2*(multiplier+1);

        if ( AVR32_PM.PLL[pll].pllopt )
            rc /= 2;
    }
    return rc;
}

/*!
 * \brief Determine the processor clock frequency.
 *
 * \return CPU clock frequency in Hertz.
 */
static uint32_t Avr32GetProcessorClock(void)
{
    uint32_t rc = 0;
    uint32_t mckr = AVR32_PM.mcctrl;

    /* Determine the clock source. */
    switch(mckr & AVR32_PM_MCCTRL_MCSEL_MASK) {
    case AVR32_PM_MCCTRL_MCSEL_OSC0:
        /* OSC0 clock selected */
        rc = OSC0_VAL;
        break;
    case AVR32_PM_MCCTRL_MCSEL_SLOW:
        /* Slow clock selected. */
        rc = AVR32_PM_RCOSC_FREQUENCY;
        break;
    case AVR32_PM_MCCTRL_MCSEL_PLL0:
        /* PLL0 clock selected. */
        rc = AVR32GetPllClock(0);
        break;
    }

    /* Handle pre-scaling. */
    if (AVR32_PM.cksel & AVR32_PM_CKSEL_CPUDIV_MASK) {
        int cpusel = ( mckr & AVR32_PM_CKSEL_CPUSEL_MASK ) >> AVR32_PM_CKSEL_CPUSEL_OFFSET;
        /* CPUDIV = 1: CPU Clock equals main clock divided by 2^(CPUSEL+1). */
        rc /= _BV(cpusel + 1);
    }

    return rc;
}

uint32_t NutArchClockGet(int idx)
{
    uint32_t rc = AVR32_PM_PBA_MAX_FREQ;

    if (idx == NUT_HWCLK_CPU || idx == NUT_HWCLK_PERIPHERAL_HSB) {
        rc = Avr32GetProcessorClock();
    } else if (idx == NUT_HWCLK_PERIPHERAL_A) {
        /* Get PBA Clock */
        rc = Avr32GetProcessorClock();

        if (AVR32_PM.CKSEL.pbadiv) {
            rc /= _BV(AVR32_PM.CKSEL.pbasel + 1);
        }
    } else if (idx == NUT_HWCLK_PERIPHERAL_B) {
        /* Get PBB Clock */
        rc = Avr32GetProcessorClock();

        if (AVR32_PM.CKSEL.pbbdiv) {
            rc /= _BV(AVR32_PM.CKSEL.pbbsel + 1);
        }
    } else if (idx == NUT_HWCLK_SLOW_CLOCK) {
        /* Can be changed using the RCCR register
           but there is no information on the datasheet yet
           on how to do so. Therefore we don't know how to calculate
           non-default values yet. */
#if defined( __AVR32_AP7000__ )
        rc = 32768;             // AP7000 has no constant for slow clock, but this is the same as XIN32, which should be 32768hz
#else
        rc = AVR32_PM_RCOSC_FREQUENCY;
#endif
    }

    return rc;
}
