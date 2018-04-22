/*
 * Copyright (C) 2013 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

#include <stdint.h>
#include <arch/cm3/nxp/lpc407x_8x.h>

/** @addtogroup LPC407x_8x_System_Defines
  * @{
  */


/**
 * Note: LPC40xx Code Read Protection field field must be present in
 *       LPC40xx flash image and ocupy a word at 0x000002FC
 *
 */

const uint32_t LPC40XX_CRP __attribute__((section(".crp"), used)) = 0xFFFFFFFF;

uint32_t Lpc407x_8xGetCrpValue(void)
{
    return *(&LPC40XX_CRP);
}


#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
/**
 * Initialize the FPU
 *
 * @param  none
 * @return none
 *
 * @brief  Bring up the FPU
 */

static void FpuInit(void)
{
    /* From ARM trm manual: 
       ; CPACR is located at address 0xE000ED88
       LDR.W R0, =0xE000ED88
       ; Read CPACR
       LDR R1, [R0]
       ; Set bits 20-23 to enable CP10 and CP11 coprocessors
       ORR R1, R1, #(0xF << 20)
       ; Write back the modified value to the CPACR
       STR R1, [R0]
    */

    volatile uint32_t* regCpacr = (uint32_t*) LPC_CPACR;
    volatile uint32_t* regMvfr0 = (uint32_t*) SCB_MVFR0;
    volatile uint32_t* regMvfr1 = (uint32_t*) SCB_MVFR1;
    volatile uint32_t Cpacr;
    volatile uint32_t Mvfr0;
    volatile uint32_t Mvfr1;
    char vfpPresent = 0;

    Mvfr0 = *regMvfr0;
    Mvfr1 = *regMvfr1;

    vfpPresent = ((SCB_MVFR0_RESET == Mvfr0) && (SCB_MVFR1_RESET == Mvfr1));

    if(vfpPresent)
    {
        Cpacr = *regCpacr;
        Cpacr |= (0xF << 20);
        *regCpacr = Cpacr;   // enable CP10 and CP11 for full access
    }

}
#endif

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
void SystemInit (void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    FpuInit();
#endif
}

/**
 * @}
 */

