/*
 * Copyright (C) 2009, 2011 by egnite GmbH
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
 * \file arch/arm/dev/atmel/spibus_at91ssc.c
 * \brief General SSC SPI bus driver routines.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/spibus_ssc.h>
#include <sys/timer.h>

/*!
 * \brief Update SPI settings.
 */
int SscSpiSetup(NUTSPINODE * node)
{
    uint32_t mck;
    AT91SSCREG *sscreg = (AT91SSCREG *) node->node_stat;

    /* The SSC can transfer 8, 16 or 32 bits only. */
    if (node->node_bits != 8 && node->node_bits != 16 && node->node_bits != 32) {
        return -1;
    }
    sscreg->at91ssc_fmr = ((node->node_bits - 1) << SSC_DATLEN_LSB) | SSC_MSBF;

    /* Query peripheral clock. */
    mck = NutClockGet(NUT_HWCLK_PERIPHERAL);
    /* Calculate the SPI clock divider: n=mck/(2*rate). Avoid rounding errors. */
    sscreg->at91ssc_cmr = mck + node->node_rate - 1;
    sscreg->at91ssc_cmr /= node->node_rate;
    sscreg->at91ssc_cmr /= 2;
    /* Honor minimum and maximum value. */
    if (sscreg->at91ssc_cmr < 3) {
        /* Theoretically 1, but any value below 3 fails on the EIR.
           No other boards tested so far. */
        sscreg->at91ssc_cmr = 3;
    }
    else if (sscreg->at91ssc_cmr > 4095) {
        /* SSC maximum divider on the SAM7SE. No other CPUs had been tested. */
        sscreg->at91ssc_cmr = 4095;
    }
    /* Calculate the actual rate. */
    node->node_rate = mck / sscreg->at91ssc_cmr / 2;

    /* Update done. */
    node->node_mode &= ~SPI_MODE_UPDATE;

    return 0;
}
