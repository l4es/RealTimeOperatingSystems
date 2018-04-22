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
 * \file arch/cm3/board/ksk_lpc1788_sk.c
 * \brief IAR KSK LPC1788 SK eval board initialization.
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

/* SDRAM configuration for IAR KSK LPC1788 evaluation board */
/* configuration for K4S561632J sdram.                      */

#define SDRAM_BASE_ADDR     0xA0000000
#define SDRAM_SIZE          0x04000000

/* Samsung K4S561632J SDRAMs */

static SDRAM sdram_k4s561632j = {
    .base_addr     = SDRAM_BASE_ADDR,
    .size          = SDRAM_SIZE,
    .bus_width     = 32,
    .rows          = 13,
    .cols          = 9,
    .ras_latency   = 3,
    .cas_latency   = 3,
    .tRP           = 20,
    .tRAS          = 45,
    .tSREX         = 67, /* same as .tXSR */
    .tAPR          = 1,
    .tDAL          = 3,
    .tWR           = 3,
    .tRC           = 65,
    .tRFC          = 66,
    .tXSR          = 67,
    .tRRD          = 15,
    .tMRD          = 3,
    .refresh       = 7813,
};



/*!
 * \brief   Early KSK LPC1788 SK hardware initialization. Especialy SD-RAM
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
     * P2.19 - EMC_CLK[1]
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
     * P2.30 - EMC_DQM2
     * P2.31 - EMC_DQM3
     *
     * P3.0-P3.31 - EMC_D[0-31]
     * P4.0-P4.23 - EMC_A[0-23]    - only A0 .. A14 used
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
    GpioPinConfigSet(NUTGPIO_PORT2, 30, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);
    GpioPinConfigSet(NUTGPIO_PORT2, 31, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    /* Configure D0 .. D31 */
    for(i = 0; i < 32; i++) {
        GpioPinConfigSet(NUTGPIO_PORT3, i, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL | GPIO_CFG_REPEATER);
    }

    /* Configure A0 .. A12, A13 and A14 used as bank select */
    for(i = 0; i < 15; i++) {
        GpioPinConfigSet(NUTGPIO_PORT4, i, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);
    }

    GpioPinConfigSet(NUTGPIO_PORT4, 25, GPIO_CFG_PERIPHERAL1 | GPIO_CFG_SLEWCTRL);

    /* Initialize the external memory controller */
    Lpc177x_8x_EmcInit();

    Lpc177x_8x_EmcSDRAMInit(sdram_k4s561632j, 0x00004680 /* 13 rows, 9 cols, 16Mx32, 64MB */);
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
    if (Lpc177x_8x_EmcSDRAMCheck(sdram_k4s561632j, 0xaa55) == 0) {
        NutHeapAdd((void*)sdram_k4s561632j.base_addr, sdram_k4s561632j.size);
    }

    /* Initialise a timer that re-calibrates the delay loops once a minute to
       compensate temperature drift of the sdram controller
     */
    NutTimerStart(60000, adjust_sdram_timing, NULL, 0);
}
