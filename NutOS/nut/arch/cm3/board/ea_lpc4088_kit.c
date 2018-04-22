/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
 * Copyright (c) 2016 by Michael Fischer (www.emb4fun.de)
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
 * \file arch/cm3/board/ea_lpc4088_kit.c
 * \brief Embedded Artists LPC4088 Developer's Kit eval board initialization.
 *
 *
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <inttypes.h>

#include <sys/timer.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_emc.h>
#include <dev/gpio.h>
#include <dev/sdram.h>

#include <sys/heap.h>


/* 
 * SDRAM configuration for Embedded Artists LPC4088 Developer's Kit
 * evaluation board configuration for IS42S32800B sdram.
 */

#define SDRAM_BASE_ADDR    0xA0000000
#define SDRAM_SIZE         0x02000000

/* Samsung IS42S32800B SDRAMs */

/*
 * The Embedded Artists values looks like:
 *
 *    LPC_EMC->DynamicConfig0    = 0x00004480; 256MB, 8Mx32, 4 banks, row=12, column=9
 *    LPC_EMC->DynamicRasCas0    = 0x00000202; 2 RAS, 2 CAS latency
 *    LPC_EMC->DynamicReadConfig = 0x00000001; Command delayed strategy, using EMCCLKDELAY
 *    LPC_EMC->DynamicRP         = 0x00000001;    ( n + 1 ) -> 2 clock cycles
 *    LPC_EMC->DynamicRAS        = 0x00000003;    ( n + 1 ) -> 4 clock cycles
 *    LPC_EMC->DynamicSREX       = 0x00000005;    ( n + 1 ) -> 6 clock cycles
 *    LPC_EMC->DynamicAPR        = 0x00000002;    ( n + 1 ) -> 3 clock cycles
 *    LPC_EMC->DynamicDAL        = 0x00000003;    ( n ) -> 3 clock cycles
 *    LPC_EMC->DynamicWR         = 0x00000001;    ( n + 1 ) -> 2 clock cycles
 *    LPC_EMC->DynamicRC         = 0x00000004;    ( n + 1 ) -> 5 clock cycles
 *    LPC_EMC->DynamicRFC        = 0x00000004;    ( n + 1 ) -> 5 clock cycles
 *    LPC_EMC->DynamicXSR        = 0x00000005;    ( n + 1 ) -> 6 clock cycles
 *    LPC_EMC->DynamicRRD        = 0x00000001;    ( n + 1 ) -> 2 clock cycles
 *    LPC_EMC->DynamicMRD        = 0x00000001;    ( n + 1 ) -> 2 clock cycles
 *
 * Hack alert:
 * The values which are needed in ns instead of clocks are multiplied by 16,
 * 16 is here the EMC frequency in ns (1/60MHz). 
 */

static SDRAM sdram_is42s32800b =
{
   .base_addr     = SDRAM_BASE_ADDR,
   .size          = SDRAM_SIZE,
   .bus_width     = 32,
   .rows          = 12,
   .cols          = 9,
   .ras_latency   = 2,
   .cas_latency   = 2,
   .tRP           = (1*16),
   .tRAS          = (3*16),
   .tSREX         = (5*16), /* same as .tXSR */
   .tAPR          = 2,      
   .tDAL          = 2,      /* Hack, 2 is correct here */
   .tWR           = 1,
   .tRC           = (4*16),
   .tRFC          = (4*16),
   .tXSR          = (5*16),
   .tRRD          = (1*16),
   .tMRD          = 1,
   .refresh       = 15625,
};


/*!
 * \brief   Early Embedded Artists LPC4088 Developer's Kit hardware initialization.
 *          Especialy SD-RAM
 *
 * This routine is called during system initalization.
 */
void NutBoardInit(void)
{
   /* 
    * Configure SDRAM interface GPIO Pins
    */

   LPC_IOCON->P3_0  = 1; /* D0 */
   LPC_IOCON->P3_1  = 1; /* D1 */
   LPC_IOCON->P3_2  = 1; /* D2 */
   LPC_IOCON->P3_3  = 1; /* D3 */
   LPC_IOCON->P3_4  = 1; /* D4 */
   LPC_IOCON->P3_5  = 1; /* D5 */
   LPC_IOCON->P3_6  = 1; /* D6 */
   LPC_IOCON->P3_7  = 1; /* D7 */
   LPC_IOCON->P3_8  = 1; /* D8 */
   LPC_IOCON->P3_9  = 1; /* D9 */
   LPC_IOCON->P3_10 = 1; /* D10 */
   LPC_IOCON->P3_11 = 1; /* D11 */
   LPC_IOCON->P3_12 = 1; /* D12 */
   LPC_IOCON->P3_13 = 1; /* D13 */
   LPC_IOCON->P3_14 = 1; /* D14 */
   LPC_IOCON->P3_15 = 1; /* D15 */
   LPC_IOCON->P3_16 = 1; /* D16 */
   LPC_IOCON->P3_17 = 1; /* D17 */
   LPC_IOCON->P3_18 = 1; /* D18 */
   LPC_IOCON->P3_19 = 1; /* D19 */
   LPC_IOCON->P3_20 = 1; /* D20 */
   LPC_IOCON->P3_21 = 1; /* D21 */
   LPC_IOCON->P3_22 = 1; /* D22 */
   LPC_IOCON->P3_23 = 1; /* D23 */
   LPC_IOCON->P3_24 = 1; /* D24 */
   LPC_IOCON->P3_25 = 1; /* D25 */
   LPC_IOCON->P3_26 = 1; /* D26 */
   LPC_IOCON->P3_27 = 1; /* D27 */
   LPC_IOCON->P3_28 = 1; /* D28 */
   LPC_IOCON->P3_29 = 1; /* D29 */
   LPC_IOCON->P3_30 = 1; /* D30 */
   LPC_IOCON->P3_31 = 1; /* D31 */

   LPC_IOCON->P4_0  = 1; /* A0 */
   LPC_IOCON->P4_1  = 1; /* A1 */
   LPC_IOCON->P4_2  = 1; /* A2 */
   LPC_IOCON->P4_3  = 1; /* A3 */
   LPC_IOCON->P4_4  = 1; /* A4 */
   LPC_IOCON->P4_5  = 1; /* A5 */
   LPC_IOCON->P4_6  = 1; /* A6 */
   LPC_IOCON->P4_7  = 1; /* A7 */
   LPC_IOCON->P4_8  = 1; /* A8 */
   LPC_IOCON->P4_9  = 1; /* A9 */
   LPC_IOCON->P4_10 = 1; /* A10 */
   LPC_IOCON->P4_11 = 1; /* A11 */
   LPC_IOCON->P4_12 = 1; /* A12 */
   LPC_IOCON->P4_13 = 1; /* A13 */
   LPC_IOCON->P4_14 = 1; /* A14 */
   LPC_IOCON->P4_15 = 1; /* A15 */
   LPC_IOCON->P4_16 = 1; /* A16 */
   LPC_IOCON->P4_17 = 1; /* A17 */
   LPC_IOCON->P4_18 = 1; /* A18 */
   LPC_IOCON->P4_19 = 1; /* A19 */
   LPC_IOCON->P4_20 = 1; /* A20 */
   LPC_IOCON->P4_21 = 1; /* A21 */
   LPC_IOCON->P4_22 = 1; /* A22 */
   LPC_IOCON->P4_23 = 1; /* A23 */

   LPC_IOCON->P4_24 = 1; /* OEN */
   LPC_IOCON->P4_25 = 1; /* WEN */
   LPC_IOCON->P4_26 = 1; /* BLSN[0] */
   LPC_IOCON->P4_27 = 1; /* BLSN[1] */
   LPC_IOCON->P4_28 = 1; /* BLSN[2] */
   LPC_IOCON->P4_29 = 1; /* BLSN[3] */
   LPC_IOCON->P4_30 = 1; /* CSN[0] */
   LPC_IOCON->P4_31 = 1; /* CSN[1] */
   LPC_IOCON->P2_14 = 1; /* CSN[2] */
   LPC_IOCON->P2_15 = 1; /* CSN[3] */
   LPC_IOCON->P2_16 = 1; /* CASN */
   LPC_IOCON->P2_17 = 1; /* RASN */
   LPC_IOCON->P2_18 = 1; /* CLK[0] */
   LPC_IOCON->P2_19 = 1; /* CLK[1] */
   LPC_IOCON->P2_20 = 1; /* DYCSN[0] */
   LPC_IOCON->P2_21 = 1; /* DYCSN[1] */
   LPC_IOCON->P2_22 = 1; /* DYCSN[2] */
   LPC_IOCON->P2_23 = 1; /* DYCSN[3] */
   LPC_IOCON->P2_24 = 1; /* CKE[0] */
   LPC_IOCON->P2_25 = 1; /* CKE[1] */
   
   LPC_IOCON->P2_28 = 1; /* DQM[0] */
   LPC_IOCON->P2_29 = 1; /* DQM[1] */
   LPC_IOCON->P2_30 = 1; /* DQM[2] */
   LPC_IOCON->P2_31 = 1; /* DQM[3] */


   /* 
    * Initialize the external memory controller 
    */
   Lpc177x_8x_EmcInit();
   Lpc177x_8x_EmcSDRAMInit(sdram_is42s32800b, 0x00004480 /* 12 rows, 9 cols, 32MB */);
   

   /* 
    * Init LED controlled by the memory bus 
    */
   LPC_EMC->StaticConfig2   = 0x00000081;
   LPC_EMC->StaticWaitWen2  = 0x00000003; /* ( n + 1 ) -> 4 clock cycles */
   LPC_EMC->StaticWaitOen2  = 0x00000003; /* ( n )     -> 3 clock cycles */
   LPC_EMC->StaticWaitRd2   = 0x00000006; /* ( n + 1 ) -> 7 clock cycles */
   LPC_EMC->StaticWaitPage2 = 0x00000003; /* ( n + 1 ) -> 4 clock cycles */
   LPC_EMC->StaticWaitWr2   = 0x00000005; /* ( n + 2 ) -> 7 clock cycles */
   LPC_EMC->StaticWaitTurn2 = 0x00000003; /* ( n + 1 ) -> 4 clock cycles */

   /* Clear all LEDs controlled by the memory bus */
   *((uint16_t*)0x98000000) = 0;
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
   if (Lpc177x_8x_EmcSDRAMCheck(sdram_is42s32800b, 0xaa55) == 0) 
   {
      NutHeapAdd((void*)sdram_is42s32800b.base_addr, sdram_is42s32800b.size);
   }

   /* Initialise a timer that re-calibrates the delay loops once a minute to
      compensate temperature drift of the sdram controller
    */
   NutTimerStart(60000, adjust_sdram_timing, NULL, 0);
}

/*** EOF ***/

