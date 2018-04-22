/*
 * Copyright (C) 2015-2016 by Uwe Bonnes
 *                              (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#include <arch/cm3.h>

#include <arch/cm3/timer.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>
#include <dev/rtc.h>
#include <dev/irqreg.h>

#include <arch/cm3/stm/stm32xxxx.h>

/* Prepare some defaults if configuration is incomplete */

#define MSI_DEFAULT MSI_4M

#define HSI_VALUE 16000000

#define NUM_MSI_FREQ 12
static const uint32_t MSI_FREQUENCY[NUM_MSI_FREQ] = {
    /* Values from STM32L476 Rev. 2 datasheet, table 47
     * Exact values only for MSI PLL on.
     */
       98304, /* (   3 * (1 << 15))*/
      196608, /* (   6 * (1 << 15))*/
      393216, /* (  12 * (1 << 15))*/
      786432, /* (  24 * (1 << 15))*/
     1016808, /* (  31 * (1 << 15))*/
     1999848, /* (  61 * (1 << 15))*/
     3997696, /* ( 122 * (1 << 15))*/
     7995392, /* ( 244 * (1 << 15))*/
    15990784, /* ( 488 * (1 << 15))*/
    23986176, /* ( 732 * (1 << 15))*/
    32014336, /* ( 977 * (1 << 15))*/
    48005120, /* (1465 * (1 << 15))*/
};

/* Prepare system limits*/
#if   (STM32_POWERSCALE == 1)
/* Give some tolerance to that MSI_32M use 1 WS and MSI_48M 2 WS*/
# define FLASH_BASE_FREQ 16007000
# define SYSCLK_MAX      80000000
# define PLLVCO_MAX     344000000
#elif (STM32_POWERSCALE == 2)
# define FLASH_BASE_FREQ  6000000
# define SYSCLK_MAX      26000000
# define PLLVCO_MAX     128000000
#else
# warning Invalid STM32_POWERSCALE
#endif

#define PCLK1_MAX   SYSCLK_MAX
#define PCLK2_MAX   SYSCLK_MAX

/* Define PLL Input Clock */
#if (PLLCLK_SOURCE == PLLCLK_HSE)
# if (HSE_VALUE < 1000000)
#  warning HSE_VALUE not given or too low!
# endif
# define PLLCLK_IN HSE_VALUE
#elif (PLLCLK_SOURCE == PLLCLK_HSI)
# define PLLCLK_IN HSI_VALUE
#elif (PLLCLK_SOURCE == PLLCLK_MSI)
/* Use rounded MSI value here to keep calculation below easy */
# define PLLCLK_IN 4000000
# undef MSI_RANGE
# define MSI_RANGE MSI_4M
#else
# define PLLCLK_IN 0
#endif

#if (SYSCLK_SOURCE == SYSCLK_PLL)
/* 7 < PLL_MULT < 87
 * 0 < PLL_PREDIV < 9
 * 0 < PLLCLK_DIV /2 < 3
 */
# if !defined(PLLCLK_PREDIV) && !defined(PLLCLK_MULT) && !defined(PLLCLK_DIV)
#  if (STM32_POWERSCALE == 1)
#   if defined(SYSCLK_FREQ)
#    if (SYSCLK_FREQ != 80000000)
#     warning Only 80 MHz accepted without PLL factors given
#    endif
#   else
#    define SYSCLK_FREQ 80000000
#   endif
/* 160 MHz PLL output */
#   if   ((PLLCLK_IN % 8000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 8000000)
#    define PLLCLK_MULT   20
#    define PLLCLK_DIV     2
#   elif ((PLLCLK_IN % 6000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 6000000)
#    define PLLCLK_MULT   25
#    define PLLCLK_DIV     2
#   elif ((PLLCLK_IN % 5000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 5000000)
#    define PLLCLK_MULT   32
#    define PLLCLK_DIV     2
#   elif ((PLLCLK_IN % 4000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 4000000)
#    define PLLCLK_MULT   40
#    define PLLCLK_DIV     2
#   else
#    warning Provide PLLCLK_PREDIV, PLLCLK_MULT and PLLCLK_DIV to reach 80 MHz
#   endif
#  elif (STM32_VRANGE == 2)
#   if defined(SYSCLK_FREQ)
#    if (SYSCLK_FREQ != 26000000))
#     warning Only 26 MHz accepted without PLL factors given
#    endif
#   else
#    define SYSCLK_FREQ 26000000
#   endif
/*  8 MHz PLL Input, 104 MHz PLL VCO, 26 MHz PLL R output */
#   if   ((PLLCLK_IN % 8000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 8000000)
#    define PLLCLK_MULT   13
#    define PLLCLK_DIV     4
/*  7 MHz PLL Input, 98 MHz PLL VCO, 24,5 MHz PLL R output */
#   elif ((PLLCLK_IN % 7000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 7000000)
#    define PLLCLK_MULT   14
#    define PLLCLK_DIV     4
/*  6 MHz PLL Input, 96 MHz PLL VCO, 24 MHz PLL R output */
#   elif ((PLLCLK_IN % 6000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 6000000)
#    define PLLCLK_MULT   16
#    define PLLCLK_DIV     4
/*  5 MHz PLL Input, 100 MHz PLL VCO, 25 MHz PLL R output */
#   elif ((PLLCLK_IN % 5000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 5000000)
#    define PLLCLK_MULT   20
#    define PLLCLK_DIV     4
/*  4 MHz PLL Input, 104 MHz PLL VCO, 26 MHz PLL R output */
#   elif ((PLLCLK_IN % 4000000) == 0)
#    define PLLCLK_PREDIV (PLLCLK_IN / 4000000)
#    define PLLCLK_MULT   26
#    define PLLCLK_DIV     4
#   else
#    warning Please Provide PLLCLK_PREDIV, PLLCLK_MULT and PLLCLK_DIV
#   endif
#  endif
# endif
# if   (PLLCLK_IN / PLLCLK_PREDIV) < 4000000
#  warning PLL Input frequency too low
# elif (PLLCLK_IN / PLLCLK_PREDIV) > 16000000
#  warning PLL Input frequency too high
# elif ((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) < 64000000
#  warning PLL VCO frequency too low
# elif ((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) > PLLVCO_MAX
#  warning PLL VCO frequency too high
# elif (((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) / PLLCLK_DIV) > SYSCLK_MAX
#  warning "SYSCLK_FREQ overclocked"
# endif
#endif

/* Provide fallback values for PLLCLK PREDIV, MULT, DIV */
#if !defined(PLLCLK_PREDIV)
# define PLLCLK_PREDIV 1
#endif
#if !defined(PLLCLK_MULT)
# define PLLCLK_MULT   8
#endif
#if !defined(PLLCLK_DIV)
# define PLLCLK_DIV    2
#endif

/* Check PLL factors */
#if  (PLLCLK_PREDIV < 1) || (PLLCLK_PREDIV > 8)
# warning Wrong PLLCLK_PREDIV
#elif (PLLCLK_MULT < 8) || (PLLCLK_MULT > 86)
# warning Wrong PLLCLK_MULT
#elif (PLLCLK_DIV < 2) || ((PLLCLK_DIV % 2) || (PLLCLK_DIV / 2) > 3)
# warning Wrong PLLCLK_DIV
#endif

static uint32_t msi_clock;

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 *                        /Q------------------------------ USB
 *                        |               ,--------------- CPU
 *                        |               +--------------- SDIO
 * 4-26   MHz HSE -+--/M*N/R--+-AHBPRES---+-- APB1PRESC--- APB1
 *                 |          |           +-- ABP2PRESC--- ABP2
 * 16MHz HSI ------+----------'           '-- ADCPRESC---- ADC
 *                 |
 * MSI ------------
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

/* Forward declarations */
static msi_range_t CtlMsiClock(msi_range_t range);

/* Include common routines*/
#include "stm32_clk.c"

/**
  * @brief  Get Msi Frequency
  *
  * @retval MsiFrequency
  */
static void MsiFrequencyUpdate(void)
{
    if (RCC->CR & RCC_CR_MSIRDY) {
        uint8_t range;
        if (RCC->CR & RCC_CR_MSIRGSEL) {
            range = (RCC->CR & RCC_CR_MSIRANGE) / RCC_CR_MSIRANGE_1;
        } else {
            range = (RCC->CSR & RCC_CSR_MSISRANGE) / RCC_CSR_MSISRANGE_1 + 5;
        }
        NUTASSERT(range <= NUM_MSI_FREQ);
        msi_clock = MSI_FREQUENCY[range];
    } else {
        msi_clock = 0;
    }
}

/*!
 * \brief  Update SystemCoreClock according to Clock Register Values
 *
 * This function reads out the CPUs clock and PLL registers and assembles
 * the actual clock speed values into the SystemCoreClock local variable.
 */
static void SystemCoreClockUpdate(void)
{
    RCC_TypeDef *rcc =  (RCC_TypeDef *)RCC_BASE;
    uint32_t tmp = 0;
    uint32_t cfgr;

    MsiFrequencyUpdate();
    /* Get SYSCLK source ---------------------------------------------------*/
    cfgr = RCC->CFGR & RCC_CFGR_SWS;
    switch (cfgr) {
    case RCC_CFGR_SWS_MSI:
        tmp = msi_clock;
        break;
    case RCC_CFGR_SWS_HSE:
        tmp = HSE_VALUE;
        break;
    case RCC_CFGR_SWS_PLL: {
        uint32_t pllcfgr = rcc->PLLCFGR;
        uint32_t n = (pllcfgr & RCC_PLLCFGR_PLLN) >> _BI32(RCC_PLLCFGR_PLLN_0);
        uint32_t m = (pllcfgr & RCC_PLLCFGR_PLLM) >> _BI32(RCC_PLLCFGR_PLLM_0);
        uint32_t r = (pllcfgr & RCC_PLLCFGR_PLLR) >> _BI32(RCC_PLLCFGR_PLLR_0);

        m += 1;
        r = 2 * (r + 1);
        if ((pllcfgr & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE) {
            tmp = HSE_VALUE;
        } else if ((pllcfgr & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI) {
            tmp = HSI_VALUE;
        } else {
            tmp = msi_clock;
        }
        tmp = ((tmp * n) / m) / r;
        break;
    }
    default:
        tmp = HSI_VALUE;
    }
    sys_clock = tmp;
    SetClockShift();
}

/*!
 * \brief Control MSI clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on MSI start failed.
 */
static msi_range_t CtlMsiClock(msi_range_t range)
{
    msi_range_t res;
    uint32_t cr;

    cr = RCC->CR;
    if (cr & RCC_CR_MSIRGSEL) {
        res = (cr & RCC_CR_MSIRANGE) / RCC_CR_MSIRANGE_1;
    } else {
        uint32_t csr;

        csr = RCC->CSR;
        res = (csr & RCC_CSR_MSISRANGE) >> 8;
    }

    if (range != MSI_OFF) {
        /* MSIRANGE must NOT be modified when MSI is ON and NOT
         *  ready (MSION=1 and MSIRDY=0)
         */
        while ((RCC_CR & RCC_CR_MSION) && !(RCC_CR & RCC_CR_MSIRDY)) {};
        cr = RCC->CR;
        cr &= ~RCC_CR_MSIRANGE;
        cr |= range * RCC_CR_MSIRANGE_1;
        RCC->CR = cr;
        cr |= RCC_CR_MSIRGSEL;
        RCC->CR = cr;
        cr |= RCC_CR_MSION;
        RCC->CR = cr;
    } else {
        /* Disable MSI clock */
        RCC->CR &= ~RCC_CR_MSION;
    }

    return res;
}

/*!
 * \brief  Configures the System clock source: HSE or HSI.
 * \note   This function should be used with PLL disables
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running else -1.
 */
static int SetPllClockSource(int src)
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
    case (PLLCLK_MSI):
        CtlMsiClock(MSI_4M);
        rc = rcc_set_and_wait_rdy_value(
            &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_MSI,
            RCC_CFGR_SWS, RCC_CFGR_SWS_MSI, HSE_STARTUP_TIMEOUT);
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
    pllcfgr |=  ( PLLCLK_PREDIV - 1)   * RCC_PLLCFGR_PLLM_0;
    pllcfgr |=    PLLCLK_MULT          * RCC_PLLCFGR_PLLN_0;
    pllcfgr |=  ((PLLCLK_DIV / 2) - 1) * RCC_PLLCFGR_PLLR_0;
    pllcfgr |=  RCC_PLLCFGR_PLLREN;

    switch (src) {
    case (PLLCLK_HSE):
        pllcfgr |= RCC_PLLCFGR_PLLSRC_HSE;
        break;
    case (PLLCLK_MSI):
        pllcfgr |= RCC_PLLCFGR_PLLSRC_MSI;
        break;
    case (PLLCLK_HSI):
        pllcfgr |= RCC_PLLCFGR_PLLSRC_HSI;
        break;
    }
    RCC->PLLCFGR = pllcfgr;
    return rc;
}

/*!
 * \brief  Configures the System clock source: MSI, HSI, HS or PLL.
 * \note   This function should be used only after reset.
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
static int SetSysClockSource(int src)
{
    int rc = -1;
    uint32_t cr;
    uint32_t old_latency, new_latency, new_sysclk;

    /* Set up RTC clock source and eventually LSE and LSI */
    SetRtcClockSource(RTCCLK_SOURCE);

    /* Set voltage scaling*/
    cr = PWR->CR1;
    cr &= ~(PWR_CR1_VOS);
    cr |= STM32_POWERSCALE * PWR_CR1_VOS_0;
    PWR->CR1 = cr;

    /* Set Latency*/
    old_latency = FLASH->ACR & FLASH_ACR_LATENCY;
    switch (src) {
    case (SYSCLK_MSI):
        new_sysclk = MSI_FREQUENCY[MSI_RANGE];
        break;
    case (SYSCLK_HSE):
        new_sysclk = HSE_VALUE;
        break;
    case (SYSCLK_PLL):
        new_sysclk = ((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) / PLLCLK_DIV;
        break;
    default:
        new_sysclk = HSI_VALUE;
        break;
    }
    new_latency = (new_sysclk - 1)/ FLASH_BASE_FREQ;

    if (new_latency > old_latency) {
        uint32_t flash_acr;
        flash_acr = FLASH->ACR;
        flash_acr &= ~FLASH_ACR_LATENCY;
        flash_acr |= new_latency;
        FLASH->ACR = flash_acr;
    }
    CM3BBSETVAL(FLASH_R_BASE, FLASH_TypeDef, ACR, _BI32(FLASH_ACR_PRFTEN), FLASH_PREFETCH);
    CM3BBSETVAL(FLASH_R_BASE, FLASH_TypeDef, ACR, _BI32(FLASH_ACR_ICEN),   FLASH_ICACHE);
    CM3BBSETVAL(FLASH_R_BASE, FLASH_TypeDef, ACR, _BI32(FLASH_ACR_DCEN),   FLASH_DCACHE);
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
    if (LSE_VALUE) {
        /* Enable MSI/LSE PLL.
         * This may take up to 2 seconds on a cold start!
         * But neither interrupts nor timers are set up yet,
         * so busy waiting is a resonable choice!*/
        while (!(RCC_BDCR & RCC_BDCR_LSERDY));
        RCC->CR |= RCC_CR_MSIPLLEN;
    }
    return rc;
}
