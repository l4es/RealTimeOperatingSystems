/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2015-2017 by Uwe Bonnes
 *                               (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \verbatim
 * $Id: stm32f1_clk.c 6595 2017-02-15 15:35:47Z u_bonnes $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>

#include <sys/nutdebug.h>

#include <arch/cm3/timer.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>

#include <arch/cm3/stm/stm32xxxx.h>

#define HSI_VALUE 8000000

#if defined(MCU_STM32F1_CL)
# define HSE_MAX  25000000
#else
# define HSE_MAX  16000000
#endif

/* Prepare system limits*/
#define FLASH_BASE_FREQ 24000000
#if   defined(MCU_STM32F101)
# define SYSCLK_MAX     36000000
# define APB1_MAX       36000000
#elif defined(MCU_STM32F102)
# define SYSCLK_MAX     48000000
# define APB1_MAX       24000000
#else
# if(SYSCLK_SOURCE == SYSCLK_PLL) && (PLLCLK_SOURCE == PLLCLK_HSI)
#  if defined(MCU_STM32F1_CL)
#   define SYSCLK_MAX   36000000
#  else
#   define SYSCLK_MAX   64000000
#  endif
# else
#  define SYSCLK_MAX    72000000
# endif
# define APB1_MAX       36000000
#endif

#if HSE_VALUE > 0
# if HSE_VALUE < 4000000
#  warning HSE_VALUE too low
# elif HSE_VALUE > HSE_MAX
#  warning HSE_VALUE too high
# endif
#endif

/* define PLL input clock */
#if (PLLCLK_SOURCE == PLLCLK_HSE)
# define PLLCLK_IN HSE_VALUE
#else
# define PLLCLK_IN (HSI_VALUE/2)
#endif

#if (SYSCLK_SOURCE == SYSCLK_PLL)
# if !defined(SYSCLK_FREQ)
#  define SYSCLK_FREQ SYSCLK_MAX
# endif
#endif

#if defined(MCU_STM32F1_CL)
# if !defined(PLL2CLK_PREDIV) && !defined(PLL2CLK_MULT) &&      \
    !defined(PLLCLK_PREDIV) && !defined(PLLCLK_MULT)
#  undef PLLCLK_SOURCE
#  if ((PLLCLK_IN % 5000000) == 0) && ((SYSCLK_FREQ % 8000000) == 0)
/* Values From STM32F105xx datasheet*/
#   define PLLCLK_SOURCE     PLLCLK_HSE_PLL2
#   define PLL2CLK_PREDIV    (PLLCLK_IN / 5000000)
#   define PLL2CLK_MULT      8
#   define PLLCLK_DIV        1
#   define PLLCLK_PREDIV     5
#   define PLLCLK_MULT       (SYSCLK_FREQ / 8000000)
#  elif (HSE_VALUE % (14745600 / 4))
#   define PLLCLK_SOURCE     PLLCLK_HSE_PLL2
#   define PLL2CLK_PREDIV    (HSE_VALUE / (14745600 / 4))
#   define PLL2CLK_MULT      12
#   define PLLCLK_DIV        4
#   define PLLCLK_MULT       13
#  elif ((HSE_VALUE % 8000000) == 0) && ((SYSCLK_FREQ % 8000000) == 0)
#   define PLLCLK_SOURCE     PLLCLK_HSE
#   define PLLCLK_PREDIV     (HSE_VALUE / 8000000)
#   define PLLCLK_MULT       (SYSCLK_FREQ / 4000000)
#  else
#   warning Please supply fitting values
#  endif
# elif defined(PLL2CLK_PREDIV) || defined(PLL2CLK_MULT) ||     \
    defined(PLLCLK_PREDIV) || defined(PLLCLK_MULT) ||          \
    defined(PLLCLK_SOURCE)
#  warning Please either none or all values
# endif
#else
/* Now handle Values for all devices but STM32F1_CL */
# if !defined(PLLCLK_DIV) && !defined(PLLCLK_MULT)
#  if  (((PLLCLK_IN ==  8000000) && ((SYSCLK_FREQ %  8000000) == 0)) || \
        ((PLLCLK_IN ==  6000000) && ((SYSCLK_FREQ %  6000000) == 0)) || \
        ((PLLCLK_IN ==  4000000) && ((SYSCLK_FREQ %  4000000) == 0)) || \
        ((PLLCLK_IN ==  9000000) && ((SYSCLK_FREQ %  9000000) == 0)) || \
        ((PLLCLK_IN == 12000000) && ((SYSCLK_FREQ % 12000000) == 0)) || \
        ((PLLCLK_IN == 25000000) && ((SYSCLK_FREQ % 15000000) == 0)))
#   if HSE_VALUE > 0
#    define PLLCLK_SOURCE  PLLCLK_HSE
#   else
#    define PLLCLK_SOURCE  PLLCLK_HSI
#   endif
#   define PLLCLK_DIV     1
#   define PLLCLK_MULT    (SYSCLK_FREQ / PLLCLK_IN)
#  else
#   warning Please supply fitting values
#  endif
# endif
#endif

/* Provide Fallback values*/
#if !defined(SYSCLK_FREQ)
# define SYSCLK_FREQ SYSCLK_MAX
#endif
#if !defined(PLL2CLK_PREDIV)
#define PLL2CLK_PREDIV 1
#endif
#if !defined(PLLCLK_PREDIV)
#define PLLCLK_PREDIV 1
#endif
#if !defined(PLL2CLK_MULT)
#define PLL2CLK_MULT 8
#endif
#if !defined(PLLCLK_MULT)
# if defined(MCU_STM32F1_CL)
# define PLLCLK_MULT   4
#else
# define PLLCLK_MULT   2
#endif
#endif

#if (SYSCLK_SOURCE == SYSCLK_PLL)
/* Check PLL factors */
# if defined(MCU_STM32F1_CL)
#  if (PLL2CLK_PREDIV < 1) || (PLL2CLK_PREDIV > 16)
#   warning PLL2CLK_PREDIV wrong
#  elif (PLL2CLK_MULT < 8) || ((PLL2CLK_MULT > 14) && (PLL2CLK_MULT != 16) && (PLL2CLK_MULT != 20))
#   warning PLL2CLK_MULT wrong
#  elif (PLLCLK_PREDIV < 1) || (PLLCLK_PREDIV > 16)
#   warning PLLCLK_PREDIV wrong
#  elif ((PLLCLK_MULT  != 13) && (PLLCLK_MULT/2 < 4) && (PLLCLK_MULT > 9))
#   warning PLLCLK_MULT wrong
#  endif
# else
#  if  ((PLLCLK_PREDIV < 1) || (PLLCLK_PREDIV > 2))
#  warning Wrong PLLCLK_PREDIV
#  elif (PLLCLK_MULT < 2) || (PLLCLK_MULT > 16)
#   warning Wrong PLLCLK_MULT
#  endif
# endif
# endif

/* Some checks*/
#if SYSCLK_FREQ > SYSCLK_MAX
# warning Requested SYSCLK_FREQ too high
#endif

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 * PLLCLK2--/1..16- (CL)
 *                 |
 * HSE -- /1..16 (CL  VL)     ,--------------------------- USB
 *                 |          |           ,--------------- CPU
 * HSE -- /1..2 ---+          |           +--------------- SDIO
 *                 +---PLLMUL-+-AHBPRES---+-- APB1PRESC--- APB1
 *                /2          |           +-- ABP2PRESC--- ABP2
 * 8MHz HSI -------+----------'           '-- ADCPRESC---- ADC
 *
 *
 *        ***** Setup of system clock configuration *****
 *
 * HSE 4 .. 25 MHz (CL)
 * HSE 4 .. 24 MHz (VL)
 * HSE 4 .. 16 MHz (all other)
 *
 * Maximum Clock
 * STM32F101     : 36 MHz
 * STM32F102     : 48 MHz
 * STM32F103/5/7 : 72 MHz
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
    uint32_t tmp = 0, pllmult = 0, pllinput = 0;
#if defined(RCC_CFGR2_PREDIV1)
    uint32_t pllmuxinput;
    uint32_t prediv1;
    uint32_t prediv2, pll2multval;
    /* Use value * 2 to get integer value for multiplication factor 6.5 */
    const uint8_t pll1mult[16] = { 0,  0,  8, 10, 12, 14, 16, 18,  0,  0,  0,  0,  0, 13, 0 ,  0};
    const uint8_t pll2mult[16] = { 0,  0,  0,  0,  0,  0,  8,  9, 10, 11, 12, 13, 14,  0, 16, 20};
    uint32_t prediv1factor = 0;
#endif

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case RCC_CFGR_SWS_HSI:  /* HSI used as system clock */
            sys_clock = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
            sys_clock = HSE_VALUE;
            break;
        case RCC_CFGR_SWS_PLL:  /* PLL used as system clock */
            /* Get PLL clock source and multiplication factor ----------------------*/
            pllmult = RCC->CFGR & RCC_CFGR_PLLMULL;
            pllmult = pllmult / RCC_CFGR_PLLMULL_0;
#if defined(RCC_CFGR2_PREDIV1)
            pllmult = pll1mult[pllmult];
#else
            pllmult = pllmult + 2;
            if (pllmult > 16)
                pllmult = 16;
#endif
            switch (RCC->CFGR & RCC_CFGR_PLLSRC) {
            case 0 :
                pllinput = HSI_VALUE / 2;
                break;
#if defined(RCC_CFGR2_PREDIV1)
            case RCC_CFGR_PLLSRC :
                pllmuxinput = HSE_VALUE;
                prediv1 =  RCC->CFGR2 &  RCC_CFGR2_PREDIV1;
                prediv1 =  prediv1 / RCC_CFGR2_PREDIV1_0;
                switch (RCC->CFGR2 & RCC_CFGR2_PREDIV1SRC) {
                case RCC_CFGR2_PREDIV1SRC_PLL2:
                    prediv2 = (RCC->CFGR2 & RCC_CFGR2_PREDIV2) / RCC_CFGR2_PREDIV2_0;
                    pll2multval = (RCC->CFGR2 & RCC_CFGR2_PLL2MUL) / RCC_CFGR2_PLL2MUL_0;
                    pllmuxinput = (HSE_VALUE * pll2mult[pll2multval]) / (prediv2 + 1);
                    pllinput = pllmuxinput / (prediv1 + 1);
                    break;
                case RCC_CFGR2_PREDIV1SRC_HSE:
                    /* HSE oscillator clock selected as PREDIV1 clock entry */
                    pllinput = HSE_VALUE / (prediv1factor + 1);
                }
                pllinput = pllmuxinput / (2 *(prediv1 + 1));
                break;
#else
            case RCC_CFGR_PLLSRC :
                if (RCC_CFGR_PLLXTPRE_HSE_DIV2 == (RCC->CFGR & RCC_CFGR_PLLXTPRE)) {
                    pllinput = HSE_VALUE / 2;
                } else {
                    pllinput = HSE_VALUE;
                }
                break;
#endif
            }
            sys_clock = pllinput * pllmult ;
            break;
    }
    SetClockShift();
}

#if defined(RCC_CFGR2_PREDIV1)
/*!
 * \brief  Set RCC_CFGR2 register and eventually start PLL2
 *
 * \param  src is one of PLLCLK_HSE and PLLCLK_HSE_PLL2.
 * \return 0 if clock is running ale -1.
 */
static int SetRccCfgr2(int src)
{
    uint32_t cfgr2;
    uint32_t pll2_mult;
    /* Set PLL 2 parameters */
    cfgr2 = RCC->CFGR2;
    cfgr2 &= ~(RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1SRC);
    cfgr2 |= (PLLCLK_PREDIV  - 1) * RCC_CFGR2_PREDIV1_0;
    cfgr2 |= (PLL2CLK_PREDIV - 1) * RCC_CFGR2_PREDIV2_0;
    switch (PLL2CLK_MULT) {
    case 20:
        pll2_mult = RCC_CFGR2_PLL2MUL20;
        break;
    case 16:
        pll2_mult = RCC_CFGR2_PLL2MUL16;
        break;
    case 8 ... 14:
        pll2_mult = (PLL2CLK_MULT - 2) * RCC_CFGR2_PLL2MUL_0;
        break;
    default:
        NUTASSERT();
    }
    cfgr2 |= pll2_mult;
    if (src == PLLCLK_HSE_PLL2) {
        cfgr2 |= RCC_CFGR2_PREDIV1SRC_PLL2;
        RCC->CFGR2 = cfgr2;
        return rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_PLL2ON, RCC_CR_PLL2RDY,
                                    1, HSE_STARTUP_TIMEOUT);
    }
    RCC->CFGR2 = cfgr2;
    return 0;
}
#endif

/*!
 * \brief  Configures the System clock source: HSE or HSI.
 * \note   This function should be used only after reset.
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running ale -1.
 */
static int SetPllClockSource(int src)
{
    int rc = -1;
    uint32_t cfgr;

    /* Enable PLLCLK source and switch system clock to that source to allow PLL switch*/
    switch (src) {
    case (PLLCLK_HSE):
    case (PLLCLK_HSE_PLL2):
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
    cfgr = RCC->CFGR;
    cfgr &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
#if defined(RCC_CFGR2_PREDIV1)
    cfgr |= RCC_CFGR_PLLSRC;
    switch (PLLCLK_MULT) {
    case 13:
        cfgr |= RCC_CFGR_PLLMULL6_5;
        break;
    default:
        cfgr |= (PLLCLK_MULT - 2) * RCC_CFGR_PLLMULL_0;
        break;
    }
    if (src |= PLLCLK_HSI) {
        SetRccCfgr2(src);
    }
#else
    if (src == PLLCLK_HSE) {
        cfgr |=  RCC_CFGR_PLLSRC;
    }
    if (PLLCLK_PREDIV == 2) {
        cfgr |= RCC_CFGR_PLLXTPRE_HSE_DIV2;
    } else {
        cfgr &= ~RCC_CFGR_PLLXTPRE_HSE_DIV2;
    }
    cfgr |= (PLLCLK_MULT - 2) * RCC_CFGR_PLLMULL_0;
#endif
    RCC->CFGR = cfgr;
    return rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_PLLON, RCC_CR_PLLRDY,
                                  0, HSE_STARTUP_TIMEOUT);
}

/*!
 * \brief  Configures the System clock source: HSI, HS or PLL.
 * \note   This function should be used only after reset.
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
static int SetSysClockSource(int src)
{
    int rc = -1;
    uint32_t old_latency, new_latency, new_sysclk;

    /* Set up RTC clock source and eventually LSE and LSI */
    SetRtcClockSource(RTCCLK_SOURCE);

    /* Calculate Latency*/
    old_latency = FLASH->ACR & FLASH_ACR_LATENCY;
    if (src == SYSCLK_HSE) {
        new_sysclk = HSE_VALUE;
    } else if (src == SYSCLK_PLL) {
        if (PLLCLK_SOURCE == PLLCLK_HSE_PLL2) {
            new_sysclk = (((PLLCLK_IN / PLL2CLK_PREDIV) * PLL2CLK_MULT) / PLLCLK_PREDIV) * PLLCLK_MULT;
        } else {
            new_sysclk = (PLLCLK_IN * PLLCLK_MULT) / PLLCLK_PREDIV;
        }
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
    SetBusDividers(AHB_DIV, APB1_DIV, APB2_DIV);
    rc = SwitchSystemClock(src);
    if (new_latency < old_latency) {
        uint32_t flash_acr;
        flash_acr = FLASH->ACR;
        flash_acr &= ~FLASH_ACR_LATENCY;
        flash_acr |= new_latency;
        FLASH->ACR = flash_acr;
    }
    /* Update core clock information */
    SystemCoreClockUpdate();

    return rc;
}
