/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * (C) 2011 - 2015 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de
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

#include <sys/nutdebug.h>

#include <cfg/arch.h>
#include <arch/cm3.h>

#include <arch/cm3/timer.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>

#include <arch/cm3/stm/stm32xxxx.h>

/* Prepare some defaults if configuration is incomplete */

#define HSI_VALUE  16000000

#define NUM_MSI_FREQ 7
static const uint32_t MSIFreqTable[7] =
{
    65536,
    131072,
    262144,
    524288,
    1048000,
    2097000,
    4194000
};

#define MSI_DEFAULT MSI_4200k

/* Prepare system limits*/
/* Power range may be switched dynamic */
#if defined(MCU_STM32L1_CAT1)
static const uint32_t flash_base_freq[4] = {0, 16000000, 8000000, 2100000};
#else
static const uint32_t flash_base_freq[4] = {0, 16000000, 8000000, 4000000};
#endif

#if   (STM32_POWERSCALE == 1)
# define FLASH_BASE_FREQ 16000000
# define SYSCLK_MAX      32000000
# define PLLVCO_MAX      96000000
#elif (STM32_POWERSCALE == 2)
# define FLASH_BASE_FREQ  8000000
# define SYSCLK_MAX      16000000
# define PLLVCO_MAX      48000000
#elif (STM32_POWERSCALE == 3)
# define PLLVCO_MAX      24000000
# if defined(MCU_STM32L1_CAT1)
#  define FLASH_BASE_FREQ 2100000
#  define SYSCLK_MAX      4200000
# else
#  define FLASH_BASE_FREQ 4000000
#  define SYSCLK_MAX      8000000
# endif
#else
# warning Invalid STM32_POWERSCALE
#endif

/* Define PLL Input Clock */
#if (PLLCLK_SOURCE == PLLCLK_HSE)
# if (HSE_VALUE < 2000000)
#  warning HSE_VALUE not given or too low!
# endif
# define PLLCLK_IN HSE_VALUE
#else
# if defined(MCU_STM32L0)
#  define PLLCLK_IN (HSI_VALUE / ((HSI_DIVIDE_BY_FOUR == ENABLE) ? 4 : 1))
# else
#  define PLLCLK_IN HSI_VALUE
# endif
#endif

/* Calculate PLL values for some well know frequencies */
#if (SYSCLK_SOURCE == SYSCLK_PLL)
# if (STM32_POWERSCALE == 1)
#  if defined(SYSCLK_FREQ)
#   if (SYSCLK_FREQ != 32000000)
#    warning Only SYSCLK_FREQ 32 MHz accepted without PLL factors
#   endif
#   else
#    define SYSCLK_FREQ 32000000
#   endif
/* 96 MHz PLL VCO, 32 MHz PLL output */
#  define PLLCLK_DIV  3
#  if   (PLLCLK_IN == 32000000)
#   define PLLCLK_MULT 3
#  elif (PLLCLK_IN == 24000000)
#   define PLLCLK_MULT 4
#  elif (PLLCLK_IN == 16000000)
#   define PLLCLK_MULT 6
#  elif (PLLCLK_IN == 12000000)
#   define PLLCLK_MULT 8
#  elif (PLLCLK_IN ==  8000000)
#   define PLLCLK_MULT 12
#  elif (PLLCLK_IN ==  6000000)
#   define PLLCLK_MULT 16
#  elif (PLLCLK_IN ==  4000000)
#   define PLLCLK_MULT 24
#  elif (PLLCLK_IN ==  2000000)
#   define PLLCLK_MULT 48
#  else
#   warning Please specify SYSCLK_FREQ,PLLCLK_MULT and PLLCLK_DIV
#  endif
# elif STM32_POWERSCALE == 2
#  if defined(SYSCLK_FREQ)
#   if (SYSCLK_FREQ != 16000000)
#    warning Only SYSCLK_FREQ 16 MHz accepted without PLL factors
#   endif
#   else
#    define SYSCLK_FREQ 16000000
#   endif
#  define PLLCLK_DIV  3
/* 48 MHz PLL VCO, 16 MHz PLL output */
#  if   (PLLCLK_IN == 16000000)
#   define PLLCLK_MULT 3
#  elif (PLLCLK_IN == 12000000)
#   define PLLCLK_MULT 4
#  elif (PLLCLK_IN ==  8000000)
#   define PLLCLK_MULT 6
#  elif (PLLCLK_IN ==  6000000)
#   define PLLCLK_MULT 8
#  elif (PLLCLK_IN ==  4000000)
#   define PLLCLK_MULT 12
#  elif (PLLCLK_IN ==  2000000)
#   define PLLCLK_MULT 24
#  else
#   warning Please specify SYSCLK_FREQ, PLLCLK_MULT and PLLCLK_DIV
#  endif
# elif STM32_POWERSCALE == 3
#  warning Please specify SYSCLK_FREQ, PLLCLK_MULT and PLLCLK_DIV
# endif
/* Check PLL values */
# if (PLLCLK_IN < 2000000)
#  warning  PLL Input to low!
# elif (PLLCLK_IN >  24000000)
#  warning  PLL Input to high!
# elif ((PLLCLK_IN * PLLCLK_MULT) > PLLVCO_MAX)
#  warning PLL VCO frequency to high!
# endif
# if (((PLLCLK_IN  * PLLCLK_MULT) / PLLCLK_DIV) > SYSCLK_MAX)
#  warning SYSCLK_SOURCE PLL frequency too high
# endif
#endif

/* Provide fallback values for PLLCLK PREDIV, MULT, DIV */
#if !defined(PLLCLK_MULT)
# define PLLCLK_MULT   3
#endif
#if !defined(PLLCLK_DIV)
# define PLLCLK_DIV    2
#endif

/*Check PLL values */
#if ((PLLCLK_MULT !=   3) && (PLLCLK_MULT !=  4) && (PLLCLK_MULT !=  6) &&    \
     ( PLLCLK_MULT !=  8) && (PLLCLK_MULT != 12) && (PLLCLK_MULT != 16) &&    \
     ( PLLCLK_MULT != 24) && (PLLCLK_MULT != 32) && (PLLCLK_MULT != 48))
# warning PLLCLK_MULT out of range
#endif
#if (PLLCLK_DIV < 2) && (PLLCLK_DIV > 4)
# warning PLLCLK_DIV out of range
#endif

/* Check flash access parameters */
#if (SYSCLK_SOURCE == SYSCLK_PLL)
# if ((((PLLCLK_IN * PLLCLK_MULT) / PLLCLK_DIV)) > FLASH_BASE_FREQ)
#  if (FLASH_32BIT_ACCESS == ENABLE)
#   warning Disable FLASH_32BIT_ACCESS as wait stated are needed!
#  endif
# endif
#elif (SYSCLK_SOURCE == SYSCLK_MSI)
# if (STM32_POWERSCALE == 3) && (MSI_RANGE >  (NUM_MSI_FREQ - 2))
#  warning MSI_RANGE to high
# endif
#else
# if (SYSCLK_SOURCE == SYSCLK_HSI) &&  (HSI_VALUE > SYSCLK_MAX)
#  warning HSI not allowed in this Voltage range!
# elif ((SYSCLK_SOURCE == SYSCLK_HSE) && (HSE_VALUE > SYSCLK_MAX))
#  warning HSE to high in this Voltage range!
# endif
#endif

#if ((FLASH_32BIT_ACCESS == ENABLE) && (FLASH_PREFETCH == ENABLE))
# warning Disable FLASH_32BIT_ACCESS as prefetch is requested!
#endif

#if ((FLASH_DISABLE_BUFFER == ENABLE) &&                                \
     ((FLASH_PREFETCH == ENABLE) || (FLASH_PRE_READ == ENABLE)))
# warning Disable FLASH_DISABLE_BUFFER as (data) prefetch is requested!
#endif

static const uint8_t PllMulReg2PllMul[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};

static uint32_t msi_clock;

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system is arranged like this:
 *
 *                                       ,--------------------------- USB
 *                                       |            ,--------------- CPU
 *                                       |            +--------------- SDIO
 * 1-32MHz HSE ----+--*PLL_MULT/PLL_DIV--+-/AHB_DIV---+--/APB1_DIV--- APB1
 *                 |                     |            +-- /ABP2_DIV--- ABP2
 * 16MHz HSI ------+---------------------'-------------- ADCPRESC---- ADC
 *                                       |
 *       MSI ----------------------------'
 *
 *        ***** Setup of system clock configuration *****
 *
 * Select system clock sources: SetSysClockSource(src);
 *
 * Common CM3 code calls SetSysClock() which calls
 *   SetSysClockSource(SYSCLK_SOURCE)
 *
 */

/* Forward declarations */
static msi_range_t CtlMsiClock(msi_range_t range);

/* Include common routines*/
#include "stm32_clk.c"

/*!
  * \brief  Get Msi Frequency
  *
  * \retval MsiFrequency
  */
static uint32_t MsiFrequencyUpdate(void)
{
    if (RCC->CR & RCC_CR_MSIRDY) {
        uint32_t msirange;
        msirange = RCC->ICSCR & RCC_ICSCR_MSIRANGE ;
        msirange = msirange / RCC_ICSCR_MSIRANGE_1;
        return MSIFreqTable[msirange];
    }
    return 0;
}

/*!
 * \brief  Update SystemCoreClock according to Clock Register Values
 *
 * This function reads out the CPUs clock and PLL registers and assembles
 * the actual clock speed values into the SystemCoreClock local variable.
 */
static void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0;
    uint32_t hsi_value = HSI_VALUE;
    uint32_t cfgr;

#if defined(RCC_CFGR_HSI16DIVF)
    if (cfgr & RCC_CFGR_HSI16DIVF) {
        hsi_value = hsi_value / 4;
    }
#endif
    msi_clock = MsiFrequencyUpdate();
    /* Get SYSCLK source ---------------------------------------------------*/
    cfgr = RCC->CFGR;
    switch (cfgr & RCC_CFGR_SWS) {
    case RCC_CFGR_SWS_MSI:
        tmp = msi_clock;
        break;
    case RCC_CFGR_SWS_HSE:
        tmp = HSE_VALUE;
        break;
    case RCC_CFGR_SWS_PLL: {
        uint32_t pll_mul, pll_div;
        int index;

        cfgr = RCC->CFGR;
        index = (cfgr & RCC_CFGR_PLLMUL) / RCC_CFGR_PLLMUL_0;
        pll_mul = PllMulReg2PllMul[index];

        index =  (cfgr & RCC_CFGR_PLLDIV) / RCC_CFGR_PLLDIV_0;
        pll_div = index +1;
        if (cfgr & RCC_CFGR_PLLSRC_HSE) {
            tmp = HSE_VALUE;
        } else{
            tmp = hsi_value;
        }
        tmp =  tmp * pll_mul / pll_div;
        break;
    }
    default:
        tmp = hsi_value;
    }
    sys_clock = tmp;
    SetClockShift();
}

/*!
 * \brief Control MSI clock.
 *
 * \param  range Range to set
 * \return 0 on success, -1 on MSI start failed.
 */
static msi_range_t CtlMsiClock(msi_range_t range)
{
    msi_range_t res;
    uint32_t icscr;

    /* Get old MSI_VALUE */
    icscr =   RCC->ICSCR;
    res = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) / RCC_ICSCR_MSIRANGE_1;
    if (!(RCC->CR & RCC_CR_MSION)) {
        res = MSI_OFF;
    }
    if (range != MSI_OFF) {
        int rc = 0;
        /* Switch MSI to selected value.*/
        icscr =   RCC->ICSCR;
        icscr &= ~RCC_ICSCR_MSIRANGE;
        icscr |= (range * RCC_ICSCR_MSIRANGE_1);
        RCC->ICSCR = icscr;
        /* Enable MSI and MSI range selection in CR */
        rc = rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_MSION, RCC_CR_MSIRDY,
                                  1, HSE_STARTUP_TIMEOUT);
        if (rc) {
            res = MSI_FAILURE;
        }
    } else {
        /* Disable HSE clock */
        RCC->CR &= ~RCC_CR_MSION;
    }

    return res;
}

/*!
 * \brief  Configures the System clock source: HSE or HSI.
 *
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running else -1.
 */
static int SetPllClockSource(int src)
{
    int rc;
    uint32_t cfgr;
    int i;

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
    cfgr = RCC->CFGR;
    cfgr &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV | RCC_CFGR_PLLSRC);
    cfgr |= ((PLLCLK_DIV - 1) * RCC_CFGR_PLLDIV_0);
    i = 0;
    while (PllMulReg2PllMul[i] < PLLCLK_MULT) {
        i++;
    }
    cfgr |= (i * RCC_CFGR_PLLMUL_0);
    RCC->CFGR = cfgr;
    if (src == PLLCLK_HSE) {
        cfgr |=  RCC_CFGR_PLLSRC;
    }
    RCC->CFGR = cfgr;
    RCC->CR |= RCC_CR_PLLON;

    return 0;
}

/*!
 * \brief  Configures the System clock source: HSI, HSE, MSI or PLL.
 * \note   This function should be used only after reset.
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
static int SetSysClockSource(int src)
{
    int rc = -1;
    uint32_t cr;
    uint32_t new_latency, new_sysclk, old_scale;

    /* Set up RTC clock source and eventually LSE and LSI */
    SetRtcClockSource(RTCCLK_SOURCE);

    /* Range may change to higher voltage with old frequency*/
    cr = PWR_CR;
    old_scale = cr & PWR_CR_VOS;
    if ((STM32_POWERSCALE * PWR_CR_VOS_0) < old_scale) {
        cr &= ~(PWR_CR_VOS);
        cr |= STM32_POWERSCALE * PWR_CR_VOS_0;
        PWR_CR = cr;
    }

    /* Calculate Latency*/
    if (src == SYSCLK_HSE) {
        new_sysclk = HSE_VALUE;
    } else if (src == SYSCLK_HSI) {
        new_sysclk = HSI_VALUE;
#if defined(RCC_CFGR_HSI16DIVF)
        if (cfgr & RCC_CFGR_HSI16DIVF) {
            new_sysclk = new_sysclk / 4;
        }
#endif
    } else if (src == SYSCLK_PLL) {
        new_sysclk = (PLLCLK_IN * PLLCLK_MULT) / PLLCLK_DIV;
    } else {
        new_sysclk = MSIFreqTable[MSI_RANGE];
    }
    /* Only 1 Flash waist state is possible*/
    if (new_sysclk >  flash_base_freq[STM32_POWERSCALE]) {
        new_latency = 1;
    } else {
        new_latency = 0;
    }
    /* Accordint to RM0035, 3.9.1 FLASH_ACR PRFTEN and
     * LATENCY can only be changed with ACC64 set/
     */
    if (new_latency) {
#if defined(FLASH_ACR_ACC64)
        while (!(FLASH->ACR & FLASH_ACR_ACC64)) {
            FLASH->ACR |= FLASH_ACR_ACC64;
        }
#else
        while (FLASH->ACR & FLASH_ACR_DISAB_BUF) {
            FLASH->ACR &= ~FLASH_ACR_DISAB_BUF;
        }
#endif
        while (!(FLASH->ACR & FLASH_ACR_LATENCY)) {
            FLASH->ACR |= FLASH_ACR_LATENCY;
        }
# if (FLASH_PREFETCH == ENABLE)
       FLASH->ACR |= FLASH_ACR_PRFTEN;
# endif
# if defined(FLASH_ACR_PRE_READ)
       FLASH->ACR &= ~FLASH_ACR_PRE_READ;
       FLASH->ACR |= FLASH_PRE_READ *FLASH_ACR_PRE_READ;
#endif
    }
    SetBusDividers(AHB_DIV, APB1_DIV, APB2_DIV);
    rc = SwitchSystemClock(src);

     /* Range may change to lower voltage only with new frequency set*/
    cr = PWR_CR;
    old_scale = cr & PWR_CR_VOS;
    if ((STM32_POWERSCALE * PWR_CR_VOS_0) > old_scale) {
        cr &= ~(PWR_CR_VOS);
        cr |= STM32_POWERSCALE * PWR_CR_VOS_0;
        PWR_CR = cr;
    }

    /* Latency may only reset when new frequency is already set*/
    if (!new_latency ) {
        if ((FLASH_PREFETCH == ENABLE) && new_latency)  {
            FLASH->ACR |= FLASH_ACR_PRFTEN;
        } else {
            FLASH->ACR &= ~FLASH_ACR_PRFTEN;
        }
        while ((FLASH->ACR & FLASH_ACR_LATENCY)) {
            FLASH->ACR &= ~FLASH_ACR_LATENCY;
        }
#if defined(FLASH_ACR_ACC64)
/* Disable 64-bit access only with Prefetch disabled */
# if (FLASH__32BIT_ACCESS == ENABLE) && (FLASH_PREFETCH == DISABLE)
        FLASH->ACR &= ~FLASH_ACR_ACC64;
# endif
#else
# if ((FLASH_PREFETCH == DISABLE) && (FLASH_PRE_READ == DISABLE) &&     \
      (FLASH_DISABLE_BUFFER == ENABLE))
        FLASH->ACR |= FLASH_ACR_DISAB_BUF;
# endif
#endif
    }
    /* Update core clock information */
    SystemCoreClockUpdate();

    return rc;
}
