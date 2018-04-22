/*
 * Copyright 2010-2011 by egnite GmbH
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
 * \file arch/arm/board/eir1.c
 * \brief Elektor Internet Radio board initialization.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <toolchain.h>

/*!
 * \brief Delay loop.
 *
 * In this early stage of NutBoardInit() we have to roll our own delay
 * loop.
 *
 * \param Number of loops to execute. Each loop takes at least 150ns
 *        on the EIR running at 48 MHz.
 */
static void Delay(int n)
{
    int l;

    for (l = 0; l < n; l++) {
        _NOP();
    }
}

/*!
 * \brief Early hardware initialization.
 */
void NutBoardInit(void)
{
    /*
     * Reset the NIC.
     *
     * The low active PW_RST line of the DM9000E is connected to PC17.
     * Unfortunately the datasheet does not specify the minimum active
     * time. It only tells us, that the NIC will be available 5us
     * after de-asserting PW_RST. We keep the line low for 75us and
     * add a delay of 75us after putting it high again. This seems
     * to work reliable.
     */
    outr(PIOC_PER,  _BV(17));
    outr(PIOC_OER,  _BV(17));
    outr(PIOC_CODR, _BV(17));
    Delay(50);
    outr(PIOC_SODR, _BV(17));
    Delay(50);

    /* Enable Ethernet controller chip select. */
    outr(PIOA_BSR, _BV(PA20_NCS2_B));
    outr(PIOA_PDR, _BV(PA20_NCS2_B));
    /* Enable external memory read, write and wait signals. */
    outr(PIOC_BSR, _BV(PC16_NWAIT_B) | _BV(PC21_NWR0_B) | _BV(PC22_NRD_B));
    outr(PIOC_PDR, _BV(PC16_NWAIT_B) | _BV(PC21_NWR0_B) | _BV(PC22_NRD_B));
    /* Configure Ethernet controller chip select. */
    outr(SMC_CSR(2)
        , (1 << SMC_NWS_LSB)
        | SMC_WSEN
        | (2 << SMC_TDF_LSB)
        | SMC_BAT
        | SMC_DBW_16
        | (1 << SMC_RWSETUP_LSB)
        | (1 << SMC_RWHOLD_LSB)
        );
}
