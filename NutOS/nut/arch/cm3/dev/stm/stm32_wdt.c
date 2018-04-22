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

/*
 * LSI Frequency
 * Type  Min Typ Max
 * F0    30  40  50
 * F1    30  40  60
 * F2    17  32  47
 * F30   30  40  50
 * F37   30  40  60
 * F4    17  32  47
 * F7    17  32  47
 * L0    26  38  56
 * L1    26  38  56
 * L4    29      34
 *
 * Code uses typical values. Expect up to 50 percent deviation!
 * Todo: Eventual measure LSI Frequency during startup!
 */
#include <sys/timer.h>
#include <dev/watchdog.h>

#include <cfg/arch.h>

#if  defined(MCU_STM32F0) || defined(MCU_STM32F1) || defined(MCU_STM32F3)
# define LSI_VALUE 40000
#elif defined(MCU_STM32L0) || defined (MCU_STM32L1)
# define LSI_VALUE 38000
#else
# define LSI_VALUE 32000
#endif
/*!
 * \brief Start the STM32 hardware independent watch dog timer.
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 *
 * \param ms    Watch dog time out in milliseconds.
 * \param xmode Hardware specific mode. If 0, the default mode is used.
 *              In this mode, the watch dog will reset the CPU if not
 *              restarted within the specified time out period.
 *
 * \return The actual time out value, which may differ from the
 *         specified value due to hardware limitations. The watch
 *         dog timer will be automatically enabled on return.
 *
 */
uint32_t Stm32WatchDogStart(uint32_t ms, uint32_t xmode)
{
    int i;
    uint16_t reload;
    uint32_t actual_ms;

    while(!(RCC->CSR & RCC_CSR_LSIRDY)) {
        RCC->CSR |= RCC_CSR_LSION;
    }
    i = 8;
    /* Check for maximum value achievable:
     * 0xfff: Maximum reload
     * (1 << 8) : Maximum divisor
     */
    if ( ms >= ((0xfff << 8)/ (LSI_VALUE / 1000))) {
        reload = 0xfff;
        i = 8;
    } else {
        reload = ms * (LSI_VALUE / 1000) >> 2;
        for (i = 2; reload > 0xfff; i++) {
            reload = reload >> 1;
        }
    }
    actual_ms = (reload << i) / (LSI_VALUE / 1000);
    IWDG->KR = 0x0000cccc;
    IWDG->KR = 0x00005555;
    IWDG->PR =  i - 2;
    IWDG->RLR = reload;
    while(IWDG->SR) {}
    IWDG->KR = 0x0000aaaa;
    return actual_ms;
}
/*!
 * \brief Re-start the STM32 hardware independent watch dog timer.
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
void Stm32WatchDogRestart(void)
{
     IWDG->KR = 0x0000aaaa;
}

/*!
 * \brief Re-start the STM32 hardware independent watch dog timer.
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
void Stm32WatchDogDisable(void)
{
    /* Disabling the watchdog is not supported by hardware */
}

/*!
 * \brief Enable the .
 *
 * Att: Disabling and re-enabling the watchdog is not supported by hardware
 *
 * For portability, applications should use the platform independent
 * \ref xgWatchDog "Watchdog Driver API".
 */
void Stm32WatchDogEnable(void)
{
    /* Dis/Reenabling the watchdog is not supported by hardware */
}
