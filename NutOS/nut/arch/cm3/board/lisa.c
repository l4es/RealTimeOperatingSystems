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
 * \file arch/cm3/board/lisa.c
 * \brief Lisa board initialization.
 *
 *
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <inttypes.h>

#include <sys/timer.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_emc.h>
#include <dev/gpio.h>
#include <dev/sdram.h>

#include <sys/heap.h>

/* SDRAM configuration for Lisa board */
/* configuration for IS42S16100E sdram.                      */

#define SDRAM_BASE_ADDR     0xA0000000
#define SDRAM_SIZE          0x00200000

/* Samsung IS42S16100E SDRAMs */
static SDRAM sdram_is42s16100e = {
    .base_addr     = SDRAM_BASE_ADDR,
    .size          = SDRAM_SIZE,
    .bus_width     = 16,
    .rows          = 11,
    .cols          = 8,
    .ras_latency   = 3,
    .cas_latency   = 3,
    .tRP           = 21,
    .tRAS          = 42,
    .tSREX         = 70, /* same as .tXSR */
    .tAPR          = 1,
    .tDAL          = 5,
    .tWR           = 3,
    .tRC           = 63,
    .tRFC          = 66,
    .tXSR          = 70,
    .tRRD          = 15,
    .tMRD          = 2,
    .refresh       = 7000,
//    .refresh       = 32000,
};


/*!
 * \brief Delay loop.
 *
 * We are running prior to Nut/OS timer initialization and cannot use
 * NutSleep, not even NutDelay.
 *
 * \param Number of loops to execute.
 */

static void Lisa_Delay(int n)
{
    int l;

    for (l = 0; l < n; l++) {
        _NOP();
    }
}

/*!
 * \brief   Early LISA hardware initialization. Especialy SD-RAM
 *
 * This routine is called during system initalization.
 */
void NutBoardInit(void)
{
    int i;
    /* Configure SDRAM interface GPIO Pins  */

    /* Pin configuration:
     * P2.14 - /EMC_CS2   - not used
     * P2.15 - /EMC_CS3   - not used
     *
     * P2.16 - /EMC_CAS
     * P2.17 - /EMC_RAS
     * P2.18 - EMC_CLK[0]
     * P2.19 - EMC_CLK[1] - not used
     *
     * P2.20 - EMC_DYCS0
     * P2.21 - EMC_DYCS1  - not used
     * P2.22 - EMC_DYCS2  - not used
     * P2.23 - EMC_DYCS3  - not used
     *
     * P2.24 - EMC_CKE0
     * P2.25 - EMC_CKE1   - not used
     * P2.26 - EMC_CKE2   - not used
     * P2.27 - EMC_CKE3   - not used
     *
     * P2.28 - EMC_DQM0
     * P2.29 - EMC_DQM1
     * P2.30 - EMC_DQM2   - not used
     * P2.31 - EMC_DQM3   - not used
     *
     * P3.0-P3.31 - EMC_D[0-31]    - only D0 .. D15 used
     * P4.0-P4.23 - EMC_A[0-23]    - only A0 .. A11 used
     *
     * P4.24 - /EMC_OE   - not used
     * P4.25 - /EMC_WE
     *
     * P4.30 - /EMC_CS0  - not used
     * P4.31 - /EMC_CS1  - not used
     */

    GpioPinConfigSet(NUTGPIO_PORT2, 16, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);
    GpioPinConfigSet(NUTGPIO_PORT2, 17, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);
    GpioPinConfigSet(NUTGPIO_PORT2, 18, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    GpioPinConfigSet(NUTGPIO_PORT2, 20, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    GpioPinConfigSet(NUTGPIO_PORT2, 24, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    GpioPinConfigSet(NUTGPIO_PORT2, 28, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);
    GpioPinConfigSet(NUTGPIO_PORT2, 29, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    /* Configure D0 .. D15 */
    for(i = 0; i < 16; i++) {
        GpioPinConfigSet(NUTGPIO_PORT3, i, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL | GPIO_CFG_REPEATER);
    }

    /* Configure A0 .. A10 */
    for(i = 0; i < 11; i++) {
        GpioPinConfigSet(NUTGPIO_PORT4, i, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);
    }

    /* Configure A13 (bank select) */
    GpioPinConfigSet(NUTGPIO_PORT4, 13, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);


    GpioPinConfigSet(NUTGPIO_PORT4, 25, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    /* Initialize the external memory controller */
    Lpc177x_8x_EmcInit();

    Lpc177x_8x_EmcSDRAMInit(sdram_is42s16100e, 0x00001080 /* bank-row-column mode: 11 rows, 8 cols, 1Mx16, 16MBit */);


    /* Configure Ethernet PHY */
    /* Reset the PHY and configure the bootstrap pins using the pullup / pulldown resistors */

    GpioPinConfigSet(NUTGPIO_PORT1, 13, GPIO_CFG_OUTPUT);       /* PHY Reset */
    GpioPinSetLow(NUTGPIO_PORT1, 13);                           /* Put PHY into reset */

    GpioPinConfigSet(NUTGPIO_PORT1, 9,  GPIO_CFG_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_PERIPHERAL0);    /* MODE0   -- ETH_RXD0 */
    GpioPinConfigSet(NUTGPIO_PORT1, 10, GPIO_CFG_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_PERIPHERAL0);    /* MODE1   -- ETH_RXD1 */
    GpioPinConfigSet(NUTGPIO_PORT1, 8,  GPIO_CFG_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_PERIPHERAL0);    /* MODE2   -- ETH_CRS  */
    GpioPinConfigSet(NUTGPIO_PORT1, 14, GPIO_CFG_INPUT | GPIO_CFG_PULLDOWN | GPIO_CFG_PERIPHERAL0);  /* PHY_AD0 -- ETH_RXER */

    Lisa_Delay(25000);
    GpioPinSetHigh(NUTGPIO_PORT1, 13);                           /* Put PHY into reset */
    Lisa_Delay(1000);
}


/*!
 * \brief   Adjust the sdram timings to compensate temperature drifts
 *
 * This function is called as timer callback which was initialised at NutIdleInit()
 */
static void adjust_sdram_timing(HANDLE this, void* arg)
{
    Lpc177x_8x_EmcSDRAMAdjustTiming();
}

/*!
 * \brief   Extended system initialisation. Add sdram memory to the available memory
 *
 * This routine is called during system initialisation right before the idle
 * thread is created. We will use it to add the SDRAM memory space to out heap.
 */
void NutIdleInit(void)
{
    /* Sanity check if the sdram is working. If all is fine add the sdram to
       heap space.
     */
    if (Lpc177x_8x_EmcSDRAMCheck(sdram_is42s16100e, 0xaa55) == 0) {
        NutHeapAdd((void*)sdram_is42s16100e.base_addr, sdram_is42s16100e.size);
    }

    /* Initialise a timer that re-calibrates the delay loops once a minute to
       compensate temperature drift of the sdram controller
     */
    NutTimerStart(60000, adjust_sdram_timing, NULL, 0);
}
