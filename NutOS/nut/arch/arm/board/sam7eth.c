/*
 * Copyright 2011 by Thermotemp GmbH
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
 * \file arch/arm/board/enet_sam7x.c
 * \brief eNet-sam7X board initialization.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <toolchain.h>
#include <inttypes.h>

#define PHY_STRAP_AD0       _BV(PB7_ERXER_A)
#define PHY_STRAP_MODE0     _BV(PB5_ERX0_A) //RXD0
#define PHY_STRAP_MODE1     _BV(PB6_ERX1_A) //RXD1
#define PHY_STRAP_MODE2     _BV(PB15_ERXDV_ECRSDV_A)  //CRS
//#define PHY_STRAP_REGOFF (LED1)
//#define PHY_STRAP_INTSEL (LED2)
#define PHY_RESET           _BV(4)

#define PHY_STRAP   (PHY_STRAP_AD0 | PHY_STRAP_MODE0 | PHY_STRAP_MODE1 | PHY_STRAP_MODE2)

/*!
 * \brief Delay loop.
 *
 * \param Number of loops to execute.
 */

static void Sam7EthDelay(int n)
{
    int l;

    for (l = 0; l < n; l++) {
        _NOP();
    }
}

/*!
 * \brief Enable peripheral clocks.
 */
static void Sam7EthClockInit(void)
{
    outr(PMC_PCER, _BV(PIOA_ID));
    outr(PMC_PCER, _BV(PIOB_ID));
    outr(PMC_PCER, _BV(EMAC_ID));
}

/*!
 * \brief Toggle external hardware reset line.
 */
static void Sam7EthReset(void)
{
    /* Invoke external reset. */
    outr(PIOB_PER, PHY_RESET);
    outr(PIOB_OER, PHY_RESET);
    outr(PIOB_CODR, PHY_RESET);
    Sam7EthDelay(250);
    outr(PIOB_SODR, PHY_RESET);
    Sam7EthDelay(25000);
}


/*!
 * \brief Initialize the PHY hardware.
 *
 * On startup all GPIO pull-ups on the SAM7X are enabled by default
 * and may fight against internal pull-downs of the LAN8710. Here
 * we switch off all pull-ups connected to LAN8710 strap pins and
 * issue an external reset.
 */
static void Sam7EthPhyInit(void)
{
#if 1
    /* Allow the PHY to power up (25ms) */
    Sam7EthDelay(250000);
    /* Disable pull-ups. */
    outr(PIOB_PUDR, PHY_STRAP);
    outr(PIOB_ODR,  PHY_STRAP);
    outr(PIOB_PER,  PHY_STRAP);
#endif
    /* Toggle reset line. */
    Sam7EthReset();
#if 1
    /* Re-enable the pull-ups. */
    outr(PIOB_PUER, PHY_STRAP);
#endif
}

/*!
 * \brief Early eNet-sam7X hardware initialization.
 *
 * This routine is called during system initalization.
 */
void NutBoardInit(void)
{
    Sam7EthClockInit();
    Sam7EthPhyInit();
}
