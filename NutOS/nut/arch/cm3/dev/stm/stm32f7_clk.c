
/*
 * Copyright (C) 2015 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/dev/stm/stm32f7_clk.c
 * \brief Clock handling for F2, F4 and F7
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>

#include <arch/cm3/timer.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>

#include <arch/cm3/stm/stm32xxxx.h>

#define HSI_VALUE 16000000

/* Define PLL Input Clock */
#if (PLLCLK_SOURCE == PLLCLK_HSE)
# define PLLCLK_IN HSE_VALUE
#elif (PLLCLK_SOURCE == PLLCLK_HSI)
# define PLLCLK_IN HSI_VALUE
#endif

/* VCO limits for F2/F4/F7*/
# define PLLVCO_MAX      432000000
# define PLLVCO_MIN      192000000

/* Prepare system linits */

/* FLASH_BASE_FREQ is dependant on supply voltage */
#if   (STM32_SUPPLY_MINIMUM >= 2700)
# define FLASH_BASE_FREQ  30000000
#elif (STM32_SUPPLY_MINIMUM >= 2400)
# define FLASH_BASE_FREQ  24000000
#elif (STM32_SUPPLY_MINIMUM >= 2100)
# if defined(MCU_STM32F2)
#  define FLASH_BASE_FREQ  18000000
# else
#  define FLASH_BASE_FREQ  22000000
# endif
#else
# if defined(MCU_STM32F2)
#  define FLASH_BASE_FREQ  16000000
# else
#  define FLASH_BASE_FREQ  20000000
# endif
#endif

#if defined(MCU_STM32F2)
# define HSE_MAX           26000000
#else
# define HSE_MAX           50000000
#endif

/* System Speed on powerscale and overdrive*/
#if defined(MCU_STM32F2)
# define SYSCLK_MAX      120000000
# define APB1_MAX         30000000
# define APB2_MAX         60000000
#endif

#if defined(MCU_STM32F40)
# if   (STM32_POWERSCALE == 1)
#  define SYSCLK_MAX     168000000
# elif (STM32_POWERSCALE == 0)
#  define SYSCLK_MAX     144000000
# else
#  warning Wrong POWERSCALE for F40x
# endif
# define APB1_MAX         42000000
# define APB2_MAX         84000000
#endif

#if defined(MCU_STM32F42) ||  defined(MCU_STM32F446) || defined(MCU_STM32F469)
# if   (STM32_POWERSCALE == 1)
#  if (STM32_OVERDRIVE == ENABLE)
#   define SYSCLK_MAX     180000000
#  elif (STM32_OVERDRIVE == DISABLE)
#   define SYSCLK_MAX     168000000
#  endif
# elif (STM32_POWERSCALE == 2)
#  if (STM32_OVERDRIVE == ENABLE)
#   define SYSCLK_MAX     168000000
#  elif (STM32_OVERDRIVE == DISABLE)
#   define SYSCLK_MAX     144000000
#  endif
# elif (STM32_POWERSCALE == 3)
#  define SYSCLK_MAX      120000000
# else
#  warning Wrong POWERSCALE for F42x/F446
# endif
# if (STM32_OVERDRIVE == ENABLE)
#  define APB1_MAX  45000000
#  define APB2_MAX  90000000
#else
#  define APB1_MAX  42000000
#  define APB2_MAX  84000000
# endif
#endif

#if defined(MCU_STM32F401)
# if   (STM32_POWERSCALE == 1)
#   define SYSCLK_MAX      84000000
# elif (STM32_POWERSCALE == 2)
#   define SYSCLK_MAX      60000000
# else
#  warning Wrong POWERSCALE for F401
# endif
# define APB1_MAX  42000000
# define APB2_MAX  84000000
#endif

#if defined(MCU_STM32F41)
# if   (STM32_POWERSCALE == 1)
#   define SYSCLK_MAX     100000000
# elif (STM32_POWERSCALE == 2)
#   define SYSCLK_MAX      84000000
# elif (STM32_POWERSCALE == 3)
#   define SYSCLK_MAX      64000000
# else
#  warning Wrong POWERSCALE for F41[0..3]
# endif
# define APB1_MAX  50000000
# define APB2_MAX 100000000
#endif

#if defined(MCU_STM32F7)
# if   (STM32_POWERSCALE == 1)
#  if (STM32_OVERDRIVE == ENABLE)
#   define SYSCLK_MAX     216000000
#  elif (STM32_OVERDRIVE == DISABLE)
#   define SYSCLK_MAX     180000000
#  endif
# elif (STM32_POWERSCALE == 2)
#  if (STM32_OVERDRIVE == ENABLE)
#   define SYSCLK_MAX     180000000
#  elif (STM32_OVERDRIVE == DISABLE)
#   define SYSCLK_MAX     168000000
#  endif
# elif (STM32_POWERSCALE == 3)
#  define SYSCLK_MAX      144000000
#  endif

# if (STM32_OVERDRIVE == ENABLE)
#  define APB1_MAX  54000000
#  define APB2_MAX 108000000
#else
#  define APB1_MAX  45000000
#  define APB2_MAX  90000000
# endif
#endif

/* No device dependant values below */

#if !defined(PLLCLK_PREDIV) && !defined(PLLCLK_MULT) && !defined(PLLCLK_DIV)
# if !defined(SYSCLK_FREQ)
/* Choose highest possible frequency with no SYSCLK_FREQ value given */
#  define SYSCLK_FREQ SYSCLK_MAX
# endif
/* Check partial missing values */
#elif defined(PLLCLK_PREDIV) || defined(PLLCLK_MULT) || defined(PLLCLK_DIV)
# warning Please provide all 3 PLL factors
#endif

/* Try to auto-setup PLL values if not given */
#if !defined(PLLCLK_PREDIV) && !defined(PLLCLK_MULT) && !defined(PLLCLK_DIV)
# if   (SYSCLK_FREQ < (PLLVCO_MIN / 8))
#  warning SYSCLK_FREQ to low
# elif (SYSCLK_FREQ < (PLLVCO_MIN / 6))
#  if (SYSCLK_SOURCE == SYSCLK_PLL) &&  (SYSCLK_FREQ % 125000)
#   warning SYSCLK_FREQ must be multiple of 125 kHz
#  endif
#  define PLLCLK_DIV 8
# elif (SYSCLK_FREQ < (PLLVCO_MIN / 4))
#  if (SYSCLK_SOURCE == SYSCLK_PLL)
#   if (((SYSCLK_FREQ * 6) % 1000000) > 100) && (((SYSCLK_FREQ * 6) % 1000000) < 999900)
#    warning SYSCLK_FREQ must be multiple of 1 MHz/6
#   endif
#  endif
#  define PLLCLK_DIV 6
# elif (SYSCLK_FREQ < (PLLVCO_MIN / 2))
#  if (SYSCLK_SOURCE == SYSCLK_PLL) &&  (SYSCLK_FREQ % 125000)
#   warning SYSCLK_FREQ must be multiple of 250 kHz
#  endif
#  define PLLCLK_DIV 4
# else
#  if (SYSCLK_SOURCE == SYSCLK_PLL) &&  (SYSCLK_FREQ % 125000)
#   warning SYSCLK_FREQ must be multiple of 500 kHz
#  endif
#  define PLLCLK_DIV 2
# endif
# if   ((PLLCLK_IN % 1000000) == 0)
#  define PLLCLK_PREDIV (PLLCLK_IN / 1000000)
# elif ((PLLCLK_IN % 1500000) == 0)
#  define PLLCLK_PREDIV (PLLCLK_IN / 1500000)
# elif ((PLLCLK_IN % 1250000) == 0)
#  define PLLCLK_PREDIV (PLLCLK_IN / 1250000)
# elif (SYSCLK_SOURCE == SYSCLK_PLL)
#  warning Please provide PLLCLK PREDIV, MULT and DIV for your PLL input
# endif
# define PLLCLK_MULT ((SYSCLK_FREQ * PLLCLK_DIV) / (PLLCLK_IN / PLLCLK_PREDIV))
#endif

/* Check for data sheet limits*/
#if   (PLLCLK_IN / PLLCLK_PREDIV) <  950000
# warning PLL Input frequency too low
#elif (PLLCLK_IN / PLLCLK_PREDIV) > 2100000
# warning PLL Input frequency too high
#elif ((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) < PLLVCO_MIN
# warning PLL VCO frequency too low
#elif ((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) > PLLVCO_MAX
# warning PLL VCO frequency too high
#elif (SYSCLK_RES  > SYSCLK_MAX)
# warning "SYSCLK_FREQ overclocked"
#endif

/* Provide fallback values for PLLCLK PREDIV, MULT, DIV */
#if !defined(PLLCLK_PREDIV)
# define PLLCLK_PREDIV 2
#endif
#if !defined(PLLCLK_MULT)
# define PLLCLK_MULT   2
#endif
#if !defined(PLLCLK_DIV)
# define PLLCLK_DIV    2
#endif

/* Check PLL factors */
/* 2 <= PLL_MULT   <= 433  (PLLN)
 * 2 <= PLL_PREDIV <=  63  (PLLM)
 * 0 < PLLCLK_DIV /2 < 3 (PLLP)
 */
#if  (PLLCLK_PREDIV < 2) || (PLLCLK_PREDIV > 63)
# warning Wrong PLLCLK_PREDIV
#elif (PLLCLK_MULT < 2) || (PLLCLK_MULT > 433)
# warning Wrong PLLCLK_MULT
#elif (PLLCLK_DIV < 2) || ((PLLCLK_DIV % 2) || (PLLCLK_DIV / 2) > 3)
# warning Wrong PLLCLK_DIV
#endif

/* Check HSE settings if HSE is used */
#if (SYSCLK_SOURCE == SYSCLK_HSE) ||  (PLLCLK_SOURCE == PLLCLK_HSE)
# if (HSE_VALUE > HSE_MAX)
#  warning HSE_VALUE to high
# elif (HSE_VALUE > 26000000) && (HSE_BYPASS == DISABLE)
#  warning HSE_VALUE to high for resonator
# elif (HSE_VALUE <  4000000) && (HSE_BYPASS == DISABLE)
#  warning HSE_VALUE to low for resonator
# elif (HSE_VALUE <  1000000)
#  warning HSE_VALUE to low
# endif
#endif

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 *                        /Q------------------------------ USB
 *                        |               ,--------------- CPU
 *                        |               +--------------- SDIO
 * 4-26   MHz HSE -+--/M*N/P--+-AHBPRES---+-- APB1PRESC--- APB1
 *                 |          |           +-- ABP2PRESC--- ABP2
 * 16MHz HSI ------+----------'           '-- ADCPRESC---- ADC
 *
 *
 *        ***** Setup of system clock configuration *****
 *
 * 1) Select system clock sources
 *
 * To setup system to use HSI call: SetSysClockSource( SYSCLK_HSI);
 * To setup system to use HSE call: SetSysClockSource( SYSCLK_HSE);
 *
 * To setup system to use the PLL output, first setup the PLL source:
 * SetPllClockSource( PLLCLK_HSI);
 * or
 * SetPllClockSource( PLLCLK_HSE);
 * Then call SetSysClockSource( SYSCLK_PLL);
 *
 * 2) Configure prescalers
 * After selecting the right clock sources, the prescalers need to
 * be configured:
 * Call SetSysClock(); to do this automatically.
 *
 */

/* Include common routines*/
#include "stm32_clk.c"

/*!
 * \brief  Update SystemCoreClock according to Clock Register Values
 *
 * This function reads out the CPUs clock and PLL registers and assembles
 * the actual clock speed values into the sys_clock local variable.
 */
static void SystemCoreClockUpdate(void)
{
    RCC_TypeDef *rcc =  (RCC_TypeDef *)RCC_BASE;
    uint32_t tmp = 0;
    uint32_t cfgr;

    /* Get SYSCLK source ---------------------------------------------------*/
    cfgr = RCC->CFGR & RCC_CFGR_SWS;
    switch (cfgr) {
    case RCC_CFGR_SWS_HSE:
        tmp = HSE_VALUE;
        break;
    case RCC_CFGR_SWS_PLL: {
        uint32_t pllcfgr = rcc->PLLCFGR;
        uint32_t n = (pllcfgr & RCC_PLLCFGR_PLLN) >> _BI32(RCC_PLLCFGR_PLLN_0);
        uint32_t m = (pllcfgr & RCC_PLLCFGR_PLLM) >> _BI32(RCC_PLLCFGR_PLLM_0);
        uint32_t p = (pllcfgr & RCC_PLLCFGR_PLLP) >> _BI32(RCC_PLLCFGR_PLLP_0);

        /* Pll Divisor is (p + 1) * 2. Move * 2  into the base clock sonstant*/
        if ((pllcfgr & RCC_PLLCFGR_PLLSRC_HSE) == RCC_PLLCFGR_PLLSRC_HSE) {
            tmp = HSE_VALUE / 2;
        } else {
            tmp = HSI_VALUE / 2;
        }
        tmp = ((tmp / m) * n)/(p + 1);
        break;
    }
    default:
        tmp = HSI_VALUE;
    }
    sys_clock = tmp;
    SetClockShift();
}

/*!
 * \brief  Configures the System clock source: HSE or HSI.
 *
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running else -1.
 */
static int SetPllClockSource( int src)
{
    int rc = -1;
    uint32_t pllcfgr;

    /* Enable PLLCLK source and switch to that source */
    switch (src) {
    case (PLLCLK_HSE):
        rc = CtlHseClock(ENABLE);
        if (rc) {
            return rc;
        }
        rc = rcc_set_and_wait_rdy_value(
            &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE,
            RCC_CFGR_SWS, RCC_CFGR_SWS_HSE, HSE_STARTUP_TIMEOUT);
        if (rc) {
            return rc;
        }
        break;
    case (PLLCLK_HSI) :
        CtlHsiClock(ENABLE);
        rc = rcc_set_and_wait_rdy_value(
            &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI,
            RCC_CFGR_SWS, RCC_CFGR_SWS_HSI, HSE_STARTUP_TIMEOUT);
        if (rc) {
            return rc;
        }
        break;
    }
    pllcfgr =  RCC->PLLCFGR;
    pllcfgr &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP);
    pllcfgr |=    PLLCLK_PREDIV        * RCC_PLLCFGR_PLLM_0;
    pllcfgr |=    PLLCLK_MULT          * RCC_PLLCFGR_PLLN_0;
    pllcfgr |=  ((PLLCLK_DIV / 2) - 1) * RCC_PLLCFGR_PLLP_0;

    switch (src) {
    case (PLLCLK_HSE):
        pllcfgr |= RCC_PLLCFGR_PLLSRC_HSE;
        break;
    case (PLLCLK_HSI):
        pllcfgr |= RCC_PLLCFGR_PLLSRC_HSI;
        break;
    }
    RCC->PLLCFGR = pllcfgr;
    RCC->CR |= RCC_CR_PLLON;
#if defined(PWR_CR1_ODEN)
    /* Procedure "Entering Over-drive mode"*/
    if (STM32_OVERDRIVE == ENABLE) {
        while (!PWR_CSR1_ODRDY) {
            PWR->CR1 |= PWR_CR1_ODEN;
        }
        while (!PWR_CSR1_ODSWRDY) {
            PWR->CR1 |= PWR_CR1_ODSWEN;
        }
    }
#endif
    return rc;
}


/*!
 * \brief  Configures the System clock source: HSI, HSE or PLL.
 * \note   This function should be used only with SYSCLK == HSI or
 * \note   HSE and peripheralks clocks disabled
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
static int SetSysClockSource(int src)
{
    int rc = -1;
    uint32_t old_latency, new_latency, new_sysclk;

    /* Set up RTC clock source and eventually LSE and LSI */
    SetRtcClockSource(RTCCLK_SOURCE);

#if defined(PWR_CR_VOS)
# if !defined(PWR_CR_VOS_0)
# define  PWR_CR_VOS_0 PWR_CR_VOS
# endif
     /* Powerscale is only activated with PLL as system source */
    uint32_t pwr_cr;
    pwr_cr = PWR_CR;
    pwr_cr &= PWR_CR_VOS;
    pwr_cr |= ((~STM32_POWERSCALE & PWR_CR_VOS) * PWR_CR_VOS_0);
    PWR_CR = pwr_cr;
#endif

    old_latency = FLASH->ACR & FLASH_ACR_LATENCY;
    if (src == SYSCLK_PLL) {
        new_sysclk = (((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) / PLLCLK_DIV);
    } else if (src == SYSCLK_HSE) {
        new_sysclk = HSE_VALUE;
    } else {
        new_sysclk = HSI_VALUE;
    }
    new_latency = (new_sysclk - 1)/ FLASH_BASE_FREQ;

    if (new_latency > old_latency) {
        /* New flash latency must be in effect when switching frequency!*/
        while ((FLASH->ACR & FLASH_ACR_LATENCY) != new_latency) {
            uint32_t flash_acr;
            flash_acr = FLASH->ACR;
            flash_acr &= ~FLASH_ACR_LATENCY;
            flash_acr |= new_latency;
            FLASH->ACR = flash_acr;
        }
    }
#if defined(FLASH_ACR_ARTEN)
     if (FLASH_PREFETCH == ENABLE) {
        FLASH->ACR |= FLASH_ACR_PRFTEN;
    } else {
        FLASH->ACR &= ~FLASH_ACR_PRFTEN;
    }
    if (FLASH_ART_ACCELERATION == ENABLE) {
        FLASH->ACR |= FLASH_ACR_ARTEN;
    } else {
        FLASH->ACR &= ~FLASH_ACR_ARTEN;
    }
#else
    if (FLASH_ICACHE == ENABLE) {
        FLASH->ACR |= FLASH_ACR_ICEN;
    } else {
        FLASH->ACR &= ~FLASH_ACR_ICEN;
    }
    if (FLASH_DCACHE == ENABLE) {
        FLASH->ACR |= FLASH_ACR_DCEN;
    } else {
        FLASH->ACR &= ~FLASH_ACR_DCEN;
    }
#endif
    SetBusDividers(AHB_DIV, APB1_DIV, APB2_DIV);
    rc = SwitchSystemClock(src);
    if (new_latency < old_latency) {
        uint32_t flash_acr;
        flash_acr = FLASH->ACR;
        flash_acr &= ~FLASH_ACR_LATENCY;
        flash_acr |= new_latency;
    }
    /* Update core clock information */
    SystemCoreClockUpdate();

    return rc;
}
