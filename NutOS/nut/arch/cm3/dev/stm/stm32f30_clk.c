/*
 * Copyright (C) 2013-2017 by Uwe Bonnes
 *                                (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/dev/stm/stm32f30_clk.c
 * \brief Clock handling for F3, F0 and F100 (not F101/2/3/5/7!)
 *
 * Type       | HSE Range | HSEPREDIV | HSIDIV2 | PLLDIV | PLLMULT
 * F100       | 4..24     | 1..16     |    X    |   -    | 2..16
 * F03/5      | 4..32     | 1..16     |    x    |   -    | 2..16
 * F04/7/9    | 4..32     |    -      |    -    | 1..16  | 2..16
 * F37        | 4..32     | 1..16     |    x    |   -    | 2..16
 * F3x6/8/B/C | 4..32     | (1)2..16  |    x    |   -    | 2..16
 * F3xD/E     | 4..32     |    -      |    -    | 1..16  | 2..16
 *
 * \verbatim
 * $Id: stm32f30_clk.c 6645 2017-05-16 13:01:38Z u_bonnes $
 * \endverbatim
 */
#include <cfg/arch.h>
#include <arch/cm3.h>

#include <arch/cm3/timer.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>
#include <cfg/rtc.h>

#include <arch/cm3/stm/stm32xxxx.h>

/* Prepare some defaults if configuration is incomplete */
#if !defined(SYSCLK_SOURCE)
#define SYSCLK_SOURCE SYSCLK_HSI
#endif

#undef HSI_VALUE
#define HSI_VALUE 8000000
#define HSI48_VALUE 48000000

/* Equalize missing STM32F0 register bits */
#if !defined(RCC_CFGR_PPRE2)
#define RCC_CFGR_PPRE2   0
#define RCC_CFGR_PPRE2_0 1
#endif
#if defined( RCC_CFGR_PPRE) && !defined( RCC_CFGR_PPRE1)
#define RCC_CFGR_PPRE1   RCC_CFGR_PPRE
#define RCC_CFGR_PPRE1_0 RCC_CFGR_PPRE_0
#endif
#if defined(FLASH_ACR_LATENCY) && !defined(FLASH_ACR_LATENCY_0)
#define FLASH_ACR_LATENCY_0 FLASH_ACR_LATENCY
#endif

/* Equalize missing STM32F100 register bits (F100 handled here!)*/
#if defined(RCC_CFGR2_PREDIV1) && !defined(RCC_CFGR2_PREDIV)
# define RCC_CFGR2_PREDIV   RCC_CFGR2_PREDIV1
# define RCC_CFGR2_PREDIV_0 RCC_CFGR2_PREDIV1_0
#endif
#if defined(RCC_CFGR_PLLMULL) && !defined(RCC_CFGR_PLLMUL)
# define RCC_CFGR_PLLMUL   RCC_CFGR_PLLMULL
# define RCC_CFGR_PLLMUL_0 RCC_CFGR_PLLMULL_0
#endif

/* Prepare system limits*/
#define FLASH_BASE_FREQ 24000000
#if   defined(MCU_STM32F0)
# define SYSCLK_MAX     48000000
# define APB1_MAX       48000000
#elif defined(MCU_STM32F100)
# define SYSCLK_MAX     24000000
# define APB1_MAX       24000000
#elif defined(MCU_STM32F3)
# if !defined(RCC_CFGR_PLLSRC_HSI_PREDIV) && (PLLCLK_SOURCE == PLLCLK_HSI)
/* Only 64 Mhz can be reached With PLL input 4 MHZ (HSI / 2) and PLL_MULT <= 16 */
#  define SYSCLK_MAX     64000000
# else
#  define SYSCLK_MAX     72000000
# endif
# define APB1_MAX       36000000
#endif

/* define PLL input clock */
#if (PLLCLK_SOURCE == PLLCLK_HSE)
# define PLLCLK_IN HSE_VALUE
#elif (PLLCLK_SOURCE == PLLCLK_HSI48)
# define PLLCLK_IN HSI48_VALUE
#else
# define PLLCLK_IN HSI_VALUE
#endif

#if (SYSCLK_SOURCE == SYSCLK_PLL)
# if !defined(SYSCLK_FREQ)
#  define SYSCLK_FREQ SYSCLK_MAX
# endif
/* try to set sensible PLLCLK_PREDIV and PLLCLK_MULT if not given */
# if !defined(PLLCLK_PREDIV) && !defined(PLLCLK_MULT)
#  if !defined(RCC_CFGR_PLLSRC_HSI_PREDIV) && (PLLCLK_SOURCE == PLLCLK_HSI)
#   define PLLCLK_PREDIV 2
#   define PLLCLK_MULT   (SYSCLK_FREQ / 4000000)
/*  9 MHz PLL Input */
#  elif ((PLLCLK_IN % 9000000) == 0) && (SYSCLK_FREQ > 9000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 9000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 9000000)
/*  8 MHz PLL Input */
#  elif ((PLLCLK_IN % 8000000) == 0) && (SYSCLK_FREQ > 8000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 8000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 8000000)
/*  6 MHz PLL Input */
#  elif ((PLLCLK_IN % 6000000) == 0) && (SYSCLK_FREQ > 6000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 6000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 6000000)
/*  5 MHz PLL Input */
#  elif ((PLLCLK_IN % 5000000) == 0) && (SYSCLK_FREQ > 5000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 5000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 5000000)
/*  4.5 MHz PLL Input */
#  elif ((PLLCLK_IN % 4500000) == 0) && (SYSCLK_FREQ > 4500000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 4500000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 4500000)
/*  4 MHz PLL Input */
#  elif ((PLLCLK_IN % 4000000) == 0) && (SYSCLK_FREQ > 4000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 4000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 4000000)
/*  3 MHz PLL Input */
#  elif ((PLLCLK_IN % 3000000) == 0) && (SYSCLK_FREQ > 3000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 3000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 3000000)
/*  2.5 MHz PLL Input */
#  elif ((PLLCLK_IN % 2500000) == 0) && (SYSCLK_FREQ > 2500000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 2500000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 2500000)
/*  2 MHz PLL Input */
#  elif ((PLLCLK_IN % 2000000) == 0) && (SYSCLK_FREQ > 2000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 2000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 2000000)
/*  1.5 MHz PLL Input */
#  elif ((PLLCLK_IN % 1500000) == 0) && (SYSCLK_FREQ > 1500000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 1500000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 1500000)
/*  1 MHz PLL Input */
#  elif ((PLLCLK_IN % 1000000) == 0) && (SYSCLK_FREQ > 1000000)
#   define PLLCLK_PREDIV (PLLCLK_IN   / 1000000)
#   define PLLCLK_MULT   (SYSCLK_FREQ / 1000000)
#  else
#   warning Please specify PLLCLK_MULT and PLLCLK_PREDIV for your HSE_VALUE and SYSCLOCK
#  endif
# endif
/* Check PLL input and output clock */
# if ((PLLCLK_IN / PLLCLK_PREDIV) < 1000000)
#  warning PLL input frequency to low
# elif ((PLLCLK_IN / PLLCLK_PREDIV) > 24000000)
#  warning PLL input frequency to high
# elif (((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) < 16000000)
#  warning PLL system frequency to low
# elif (((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) > SYSCLK_MAX)
#  warning PLL system frequency to high
# endif
#endif

/* Fallback values for PLLCLK_PREDIV and PLLCLK_MULT */
# if !defined(PLLCLK_PREDIV)
#  define PLLCLK_PREDIV 1
# endif
# if !defined(PLLCLK_MULT)
#  define PLLCLK_MULT 2
# endif

/* Check PLL factors */
#if  ((PLLCLK_PREDIV < 1) || (PLLCLK_PREDIV > 2)) && !defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
/* Some devices only have /1 and /2 as HSI division*/
# warning Wrong PLLCLK_PREDIV
#elif(PLLCLK_PREDIV < 1) || (PLLCLK_PREDIV > 16)
# warning Wrong PLLCLK_PREDIV
#elif (PLLCLK_MULT < 2) || (PLLCLK_MULT > 16)
# warning Wrong PLLCLK_MULT
#endif

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 *                                         ,--------------- CPU
 *                                         +--------------- SDIO
 *  (1)4-32MHz HSE--+--/M--*N--+--AHBPRES---+-- APB1PRESC--- APB1
 *                  |          |           +-- ABP2PRESC--- ABP2
 *  8MHz HSI -------+----------+
 *                  |          |
 * 48 MHz HSI ------+----------+   ( On some F0 devices)
 *
 * M = 1..16
 * N = 2..16
 * Q = 1.1.5 (On F04/07/09 only 1)
 *
 * On F04/07/09, HSI and HSI48 enter PLL undivded and PLLDIV M is
 * active on HSE/HSI/HSI48
 * Same behaviour on F3XXxD\E with no HSI48;
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
 * the actual clock speed values into the SystemCoreClock local variable.
 */
static void SystemCoreClockUpdate(void)
{
    uint32_t cfgr;
    uint32_t tmp = 0;

    /* Get SYSCLK source ---------------------------------------------------*/
    cfgr = RCC->CFGR;
    switch(cfgr & RCC_CFGR_SWS) {
    case RCC_CFGR_SWS_HSI:
        tmp = HSI_VALUE;
        break;
#if defined(RCC_CFGR_SWS_HSI48)
    case RCC_CFGR_SWS_HSI48:
        tmp = HSI48_VALUE;
        break;
#endif
    case RCC_CFGR_SWS_HSE:
        tmp = HSE_VALUE;
        break;
    case RCC_CFGR_SWS_PLL: {
        uint32_t cfgr, cfgr2;
        uint32_t prediv;
        uint32_t pllmull;

        cfgr2 = RCC->CFGR2;
        prediv = (cfgr2 & RCC_CFGR2_PREDIV) / RCC_CFGR2_PREDIV_0;
        prediv += 1;
        cfgr  = RCC->CFGR;
        pllmull = (cfgr & RCC_CFGR_PLLMUL) / RCC_CFGR_PLLMUL_0;
        pllmull += 2;
        if (pllmull > 16) {
            pllmull = 16;
        }
        switch (cfgr & RCC_CFGR_PLLSRC) {
#if defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
        case RCC_CFGR_PLLSRC_HSI_PREDIV:
            tmp = HSI_VALUE;
            break;
#endif
#if defined(RCC_CFGR_PLLSRC_HSI_DIV2)
        case RCC_CFGR_PLLSRC_HSI_DIV2:
            tmp = HSI_VALUE;
            prediv = 2;
            break;
#endif
#if defined(RCC_CFGR_PLLSRC_HSE_PREDIV)
        case RCC_CFGR_PLLSRC_HSE_PREDIV:
            tmp = HSE_VALUE;
            break;
#endif
        }
        tmp = (tmp / prediv) * pllmull;
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
    int rc;
    uint32_t cfgr, cfgr2;

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
    /* Set PLL Mult/Div values*/
    cfgr2  = RCC->CFGR2;
    cfgr2 &= ~RCC_CFGR2_PREDIV;
    cfgr2 |= (PLLCLK_PREDIV - 1) * RCC_CFGR2_PREDIV_0;
    RCC->CFGR2 = cfgr2;
    cfgr  = RCC->CFGR;
    cfgr &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    cfgr |= (PLLCLK_MULT - 2) * RCC_CFGR_PLLMUL_0;
#if defined(RCC_CFGR_PLLSRC_HSE_PREDIV)
    if (src == PLLCLK_HSE) {
        cfgr |=  RCC_CFGR_PLLSRC_HSE_PREDIV;
    }
#endif
    RCC->CFGR = cfgr;
    return 0;
}

/*!
 * \brief  Configures the System clock coming from HSE or HSI oscillator.
 *
 * Enable HSI/HSE clock and setup HCLK, PCLK2 and PCLK1 prescalers.
 *
 * \param  None.
 * \return 0 on success, -1 on fault of HSE.
 */
/*!
 * \brief  Configures the System clock source: LSI, HSI, (HSI48), HSE or PLL.
 * \note   This function should be used only after reset.
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI, SYSCLK_HSI48 or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
static int SetSysClockSource(int src)
{
    int rc = -1;

    /* Set up RTC clock source and eventually LSE and LSI */
    SetRtcClockSource(RTCCLK_SOURCE);

    /* No voltage range on F0/F3*/

#if defined(FLASH_ACR_LATENCY)
    uint32_t old_latency, new_latency, new_sysclk;
    /* Calculate Latency*/
    old_latency = FLASH->ACR & FLASH_ACR_LATENCY;
    if (src == SYSCLK_HSE) {
        new_sysclk = HSE_VALUE;
#if defined(RCC_CFGR_SW_HSI48)
    } else if (src == SYSCLK_HSI48) {
        new_sysclk = HSI48_VALUE;
#endif
    } else if (src == SYSCLK_PLL) {
        new_sysclk = (PLLCLK_IN * PLLCLK_MULT) / PLLCLK_PREDIV;
    } else {
        new_sysclk = HSI_VALUE;
    }
    new_latency = (new_sysclk - 1) / FLASH_BASE_FREQ;

    if ((FLASH_PREFETCH) && new_latency)  {
        FLASH->ACR |= FLASH_ACR_PRFTBE;
    } else {
        FLASH->ACR &= ~FLASH_ACR_PRFTBE;
    }
    if (new_latency > old_latency) {
        uint32_t flash_acr;
        flash_acr = FLASH->ACR;
        flash_acr &= ~FLASH_ACR_LATENCY;
        flash_acr |= new_latency;
        FLASH->ACR = flash_acr;
    }
#endif
    SetBusDividers(AHB_DIV, APB1_DIV, APB2_DIV);
    rc = SwitchSystemClock(src);

#if defined(FLASH_ACR_LATENCY)
    /* Set lower latency after setting clock*/
    if (new_latency < old_latency) {
        uint32_t flash_acr;
        flash_acr = FLASH->ACR;
        flash_acr &= ~FLASH_ACR_LATENCY;
        flash_acr |= new_latency;
        FLASH->ACR = flash_acr;
    }
#endif
    /* Update core clock information */
    SystemCoreClockUpdate();

    return rc;
}
