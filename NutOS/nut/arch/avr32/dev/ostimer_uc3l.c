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

/*!
 * \brief Determine the processor clock frequency.
 *
 * \return CPU clock frequency in Hertz.
 */

uint32_t NutArchClockGet(int idx)
{
    uint32_t rc = AVR32_PM_PBA_MAX_FREQ;

    if (idx == NUT_HWCLK_CPU || idx == NUT_HWCLK_PERIPHERAL_HSB) {
        rc = 50000000;
    } else if (idx == NUT_HWCLK_PERIPHERAL_A) {
        /* Get PBA Clock */
        rc = 50000000 / 2;

    } else if (idx == NUT_HWCLK_PERIPHERAL_B) {
        /* Get PBB Clock */
        rc = 50000000 / 2;
    } else if (idx == NUT_HWCLK_SLOW_CLOCK) {
        /* Can be changed using the RCCR register
           but there is no information on the datasheet yet
           on how to do so. Therefore we don't know how to calculate
           non-default values yet. */
#if defined( __AVR32_AP7000__ )
        rc = 32768;             // AP7000 has no constant for slow clock, but this is the same as XIN32, which should be 32768hz
#else
        rc = 0;
#endif
    }

    return rc;
}
