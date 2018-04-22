/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#include <cfg/arch.h>
#include <arch/m68k.h>
#include <dev/gpio.h>

void NutBoardInit(void)
{
#if PLATFORM_SUB == REV_C
    /* Set GPIO function for enable Usart2 (RS-232). Set pin FORCEON. */
    GpioPinConfigSet(PORTAN, 7, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(PORTAN, 7);

#elif PLATFORM_SUB == REV_D
    /* Set GPIO function for enable Usart2 (RS-232). Set pins \EN and SD (shutdown)*/
    GpioPinConfigSet(PORTAN, 3, GPIO_CFG_OUTPUT);
    GpioPinConfigSet(PORTAN, 4, GPIO_CFG_OUTPUT);
    GpioPinSetLow(PORTAN, 3);
    GpioPinSetLow(PORTAN, 4);
#else
    #error "Please define User Platform Macro PLATFORM_SUB in Nut/OS Configurator."
#endif
}

void BoardInitExtram(void)
{
    extern void *__extram_start;
    extern void *__extram_size;

    GpioPortConfigSet(PORTTH, 0xFF, GPIO_CFG_PERIPHERAL0);  /* Enable Data Lines D0-D7 */
    GpioPortConfigSet(PORTTE, 0xFF, GPIO_CFG_PERIPHERAL0);  /* Enable Address Lines A0-A7 */
    GpioPortConfigSet(PORTTF, 0xFF, GPIO_CFG_PERIPHERAL0);  /* Enable Address Lines A8-A15 */
    GpioPortConfigSet(PORTTG, 0xEF, GPIO_CFG_PERIPHERAL0);

    MCF_FBCS_CSAR(0) = MCF_FBCS_CSAR_BA(((uint32_t) &__extram_start));
    MCF_FBCS_CSMR(0) = ((((uint32_t) &__extram_size) - 1) & 0xFFFF0000) | MCF_FBCS_CSMR_V;
    MCF_FBCS_CSCR(0) = MCF_FBCS_CSCR_AA | MCF_FBCS_CSCR_PS_8;
}
