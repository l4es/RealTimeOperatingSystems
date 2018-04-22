/*
 * Copyright (C) 2016 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/dev/stm/stm32_reset.c
 *
 * \brief  Find cause of last reset.
 *
 */
#include <cfg/arch.h>
#include <dev/reset.h>

/*!
 * \brief Find cause of Reset.
 */
int Stm32ResetCause(void)
{
    uint32_t csr;
    int flags;

    csr = RCC->CSR;
    flags = 0;
    if        (csr & RCC_CSR_WWDGRSTF)  {
        flags |= NUT_RSTTYP_WWATCHDOG;
    } else  if (csr & RCC_CSR_SFTRSTF)  {
        flags |= NUT_RSTTYP_SOFTWARE;
    } else if (csr & RCC_CSR_LPWRRSTF) {
        flags |= NUT_RSTTYP_LOWPOWER;
#if defined(RCC_CSR_WDGRSTF)
    } else if (csr & RCC_CSR_WDGRSTF)  {
        flags |= NUT_RSTTYP_WATCHDOG;
#endif
#if defined(RCC_CSR_IWDGRSTF)
    } else if (csr & RCC_CSR_IWDGRSTF)  {
        flags |= NUT_RSTTYP_WATCHDOG;
#endif
#if defined(RCC_CSR_PINRSTF)
    } else if (csr & RCC_CSR_PINRSTF)   {
        flags |= NUT_RSTTYP_EXTERNAL;
#endif
#if defined(RCC_CSR_PADRSTF)
    } else if (csr & RCC_CSR_PADRSTF)   {
        flags |= NUT_RSTTYP_EXTERNAL;
#endif
#if defined(RCC_CSR_FWRSTF)
    } else if (csr & RCC_CSR_FWRSTF)    {
        flags |= NUT_RSTTYP_FIREWALL;
#endif
#if defined(RCC_CSR_OBLRST)
    } else if (csr & RCC_CSR_OBLRSTF)   {
        flags |= NUT_RSTTYP_OPTIONLOADER;
#endif
#if defined(RCC_CSR_V18PWRSTF)
    } else if (csr & RCC_CSR_V18PWRSTF) {
        flags |= NUT_RSTTYP_COREPOWER;
#endif
#if defined(RCC_CSR_BORRSTF)
    } else if (csr & RCC_CSR_BORRSTF)   {
        flags |= NUT_RSTTYP_BROWNOUT;
#endif
#if defined(RCC_CSR_PORRSTF)
    } else if (csr & RCC_CSR_PORRSTF)   {
        flags |= NUT_RSTTYP_POWERUP;
#endif
    }
    RCC->CSR |= RCC_CSR_RMVF;
    return (flags) ? flags : NUT_RSTTYP_UNKNOWN;
}
