/*
 * Copyright (C) 2015-2017 by Uwe Bonnes
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

/*
 * \file arch/cm3/stm/stm32_clk.c
 * \brief Common routines for clock setup
 *
 * Include this file in the device specific setup to allow better optimization!
 */
#include <sys/timer.h>
#include <sys/event.h>
#include <dev/irqreg.h>

static int SetPllClockSource(int src);
static void SetRtcClockSource(int source);
static int SetSysClockSource(int src);

#if !defined  (HSE_STARTUP_TIMEOUT)
/*!< Time out for HSE start up, in loops*/
#define HSE_STARTUP_TIMEOUT   0x5000
#endif

static uint32_t sys_clock;
static uint8_t clk_shift[HWCLK_SYS];

static const uint8_t AHBPrescTable[16] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

#if (HSE_BYPASS != ENABLE) &&  (HSE_BYPASS != DISABLE)
# warning HSE_BYPASS needs explicit value
#endif

#if !defined(LSE_VALUE)
# define LSE_VALUE 0
#endif

#if (SYSCLK_SOURCE == SYSCLK_PLL)
/* Check match between requested and resulting SYSCLK_FREQ,
   either auto-setup or user provided */
/* No PLLCLK_DIV on L0 and L1*/
# if !defined(PLLCLK_PREDIV)
#  define PLLCLK_PREDIV 1
# endif
/* No PLLCLK_DIV on F0 and F3*/
# if !defined(PLLCLK_DIV)
#  define PLLCLK_DIV 1
# endif
# if defined(MCU_STM32F1_CL)
#  define SYSCLK_RES ((((PLLCLK_IN / PLL2CLK_PREDIV) * PLL2CLK_MULT) / PLLCLK_PREDIV) * PLLCLK_MULT)
# else
#  define SYSCLK_RES (((PLLCLK_IN / PLLCLK_PREDIV) * PLLCLK_MULT) / PLLCLK_DIV)
# endif
/* Allow 1/1000 toleranz */
# if(((SYSCLK_RES / 1000 * 1001 ) < SYSCLK_FREQ) ||     \
     ((SYSCLK_RES / 1000 *  999 ) > SYSCLK_FREQ))
#  warning Requested and resulting frequency differ!
# endif
#elif (SYSCLK_SOURCE == SYSCLK_HSI)
# define SYSCLK_RES HSI_VALUE
#elif (SYSCLK_SOURCE == SYSCLK_HSE)
# define SYSCLK_RES HSI_VALUE
#elif (SYSCLK_SOURCE == SYSCLK_MSI)
# define SYSCLK_RES MSI_2100k
#else
# warning SYSCLK_SOURCE undefined or unknown!
#endif

#if !defined( APB1_MAX)
# define APB1_MAX SYSCLK_MAX
#endif
#if !defined(APB1_DIV)
# if (SYSCLK_SOURCE == SYSCLK_PLL)
#  if   ((SYSCLK_RES / AHB_DIV) / 1 <= APB1_MAX)
#   define APB1_DIV 1
#  elif ((SYSCLK_RES / AHB_DIV) / 2 <= APB1_MAX)
#   define APB1_DIV 2
#  else
#   define APB1_DIV 4
#  endif
# else
#  define APB1_DIV 1
# endif
#endif

#if !defined( APB2_MAX)
# define APB2_MAX SYSCLK_MAX
#endif
#if !defined(APB2_DIV)
# if (SYSCLK_SOURCE == SYSCLK_PLL)
#  if   ((SYSCLK_RES / AHB_DIV) / 1 <= APB2_MAX)
#   define APB2_DIV 1
#  else
#   define APB2_DIV 2
#  endif
# else
#  define APB2_DIV 1
# endif
#endif

#if defined(CLOCK_DEBUG)
#define XSTR(x) STR(x)
#define STR(x) #x

#pragma message "SYSCLK_SOURCE = " XSTR(SYSCLK_SOURCE)
#pragma message "SYSCLK_FREQ   = " XSTR(SYSCLK_FREQ)
#pragma message "PLLCLK_IN     = " XSTR(PLLCLK_IN)
# if defined(MCU_STM32F1_CL)
#pragma message "PLL2CLK_PREDIV= " XSTR(PLL2CLK_PREDIV)
#pragma message "PLL2CLK_MULT  = " XSTR(PLL2CLK_MULT)
# endif
#pragma message "PLLCLK_PREDIV = " XSTR(PLLCLK_PREDIV)
#pragma message "PLLCLK_MULT   = " XSTR(PLLCLK_MULT)
#pragma message "PLLCLK_DIV    = " XSTR(PLLCLK_DIV)
#pragma message "SYSCLK_MAX    = " XSTR(SYSCLK_MAX)
#pragma message "SYSCLK_RES    = " XSTR(SYSCLK_RES)
#pragma message "AHB_DIV       = " XSTR(AHB_DIV)
#pragma message "APB1_DIV      = " XSTR(APB1_DIV)
#pragma message "APB1_MAX      = " XSTR(APB1_MAX)
# if !defined(MCU_STM32F0)
#pragma message "APB2_DIV      = " XSTR(APB2_DIV)
#pragma message "APB2_MAX      = " XSTR(APB2_MAX)
# endif
#endif

#if defined(MCU_STM32F2) || defined(MCU_STM32F4) || defined(MCU_STM32F7)
/*!
  * \brief  Get divisor for APB clock
  *
  * Return needed divisor so that resulting frequency
  * is smaller or equal the selected frequency or maximum
  * PCLK divisor is reached.
  *
  * @param  target_frequency Selected Frequency
  * @retval RCC_CFGR_PPRE base value
  */
static uint32_t GetPclkDiv(uint32_t target_frequency) __attribute__((unused));
static uint32_t GetPclkDiv(uint32_t target_frequency)
{
    uint32_t div;
    uint32_t res_freq;

    div = 3;
    res_freq = SYSCLK_FREQ;
    if (res_freq > target_frequency) {
        div = div + 1;
        res_freq = res_freq /2;
    }
    if (res_freq > target_frequency) {
        div = div + 1;
        res_freq = res_freq /2;
    }
    if (res_freq > target_frequency) {
        div = div + 1;
        res_freq = res_freq /2;
    }
    if (res_freq > target_frequency) {
        div = div + 1;
        res_freq = res_freq /2;
    }
    return div;
}
#endif

/**
  * \brief  Get timer clock shift
  *
  * \param  div  Connected PCLK APB prescaler
  * \retval Corrected prescaler
  */
static uint8_t GetTimerShift(uint8_t shift)
{
    uint8_t res;
#if defined(RCC_DCKCFGR_TIMPRE)
    if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) ==  RCC_DCKCFGR_TIMPRE) {
        if (shift < 2) {
            res = 0;
        } else {
            res = shift - 2;
        }
        return res;
    }
#endif
    if (shift < 1) {
        res =  0;
    } else {
        res = shift - 1;
    }
    return res;
}

 /*!
 * \brief Set bus dividers
 *
 * \param  ahb_div    Division factor for AHB BUS
 * \param  apb1_div   Division factor for APB1 BUS
 * \param  apb2_div   Division factor for APB2 BUS
 */
static void SetBusDividers(uint32_t ahb_div, uint32_t apb1_div,
                           uint32_t apb2_div) __attribute__((unused));
static void SetBusDividers(uint32_t ahb_div, uint32_t apb1_div,
                           uint32_t apb2_div)
{
    uint32_t cfgr;
    int i;

    cfgr = RCC->CFGR;
    cfgr  &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE2);
    /* Vendor code expects 0 <= HPRE <= 3 for AHB_DIV == 1, while
     * register description allows 0 <= HPRE <= 7.
     * So waste some cycle and start searching at 3. */
    i = 3;
    while ((1 << AHBPrescTable[i]) < AHB_DIV) {
        i++;
    }
    cfgr |= i * RCC_CFGR_HPRE_0;
#if defined(RCC_CFGR_PPRE1)
    cfgr  &= ~RCC_CFGR_PPRE1;
    i = 3;
    while ((1 << APBPrescTable[i]) < APB1_DIV) {
        i++;
    }
    cfgr |= i * RCC_CFGR_PPRE1_0;
#endif
    i = 3;
    while ((1 << APBPrescTable[i]) < APB2_DIV) {
        i++;
    }
    cfgr |= i * RCC_CFGR_PPRE2_0;
    RCC->CFGR = cfgr;
}

/*!
 * \brief Re/Set RCC register bit and wait for same state of connected RDY bit or timeout
 *
 * \param  reg       Register to check
 * \param  setmask   Bit to (re)set
 * \param  checkmask Bit to check
 * \param  value     Value to set and check
 * \param  tout      timeout in delay units.
 * \return 0 on success, -1 on HSE start failed.
 */
static int rcc_set_and_wait_rdy(__IO uint32_t *reg, uint32_t setmask,
                                uint32_t checkmask, int value, uint32_t tout)
{
    if (value) {
        *reg |= setmask;
    } else {
        *reg &= ~setmask;
    }
    for (; tout; tout--) {
        if (((value) && (*reg & checkmask)) ||
            (!value && (!(*reg & checkmask)))) {
            return 0;
        }
    }
    return -1;
}

/*!
 * \brief Set RCC register bits and wait for status bits
 *
 * \param  reg        Register to check
 * \param  setmask    Bit to set/reset
 * \param  setvalue   Value to set
 * \param  checkmask  Bit to check
 * \param  checkvalue Value to set and check
 * \param  tout       timeout in delay units.
 * \return 0 on success, -1 on HSE start failed.
 */
static int rcc_set_and_wait_rdy_value(
    __IO uint32_t *reg, uint32_t setmask, uint32_t setvalue,
    uint32_t checkmask, int checkvalue, uint32_t tout) __attribute__((unused));
static int rcc_set_and_wait_rdy_value(
    __IO uint32_t *reg, uint32_t setmask, uint32_t setvalue,
    uint32_t checkmask, int checkvalue, uint32_t tout)
{
    uint32_t reg_value;

    reg_value = *reg;
    reg_value &= ~setmask;
    reg_value |= setvalue;
    *reg = reg_value;
    for (; tout; tout--) {
        if ((*reg & checkmask) ==  checkvalue) {
            return 0;
        }
    }
    return -1;
}

/*!
 * \brief Control HSE clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on HSE start failed.
 */
static int CtlHseClock(int ena)
{
    int hse_is_on, byp;
    int rc = -1;
    uint32_t cr;

    /* Is HSE set already? */
    cr = RCC->CR;
    hse_is_on = (cr & RCC_CR_HSEON);
    byp = (cr & RCC_CR_HSEBYP);
    /* On L0/L1, RTCPRE can only be changed with HSE off.
     * On F2/F4/F7, RTCPRE can be changed with HSE on.
     * We switch off HSE only if BYPASS or RCC_CR_RTCPRE missmatch.
     * RCC_CR_RTCPRE is non-null only for L0/L1.
     */
    if (hse_is_on && (
            (ena != ENABLE) ||
            (byp != HSE_BYPASS) ||
            (RCC->CR & RCC_CR_RTCPRE) != RTC_PRE_VAL * RCC_CR_RTCPRE_0)) {
        /* switch HSE off as requested or to allow to set HSE_BYPASS
           or to change RTC_CFGR_PRE on L0/L1*/
        rc = rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_HSEON, RCC_CR_HSERDY,
                                  0, HSE_STARTUP_TIMEOUT);
    }
    /* L0/1 use encoded RTC prescaler value.
     * RCC_CR_RTCPRE(_O) is non-null only for L0/L1.
     */
    RCC->CR &= ~RCC_CR_RTCPRE;
    RCC->CR |= RTC_PRE_VAL * RCC_CR_RTCPRE_0;
    /* F2/F4/F7 use unencoded RTC prescaler value.
     * RCC_CFGR_RTCPRE(_O) is non-null only for F2/F4/F7.
     */
    RCC->CFGR &= ~RCC_CFGR_RTCPRE;
    RCC->CFGR |= RTC_PRE * RCC_CFGR_RTCPRE_0;
    if (ena) {
        if (HSE_BYPASS) {
            RCC->CR |= RCC_CR_HSEBYP;
        } else {
            RCC->CR &= ~RCC_CR_HSEBYP;
        }

        /* Enable HSE */
        rc = rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_HSEON, RCC_CR_HSERDY,
                                  1, HSE_STARTUP_TIMEOUT);
    }
    return rc;
}

/*!
 * \brief Control HSI clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on HSI start failed.
 */
static int CtlHsiClock(int ena)
{
    int rc = 0;

#if defined(RCC_CR_HSI16DIVF)
    while ((RCC->CR & RCC_CR_HSI16DIVF) != HSI_DIVIDE_BY_FOUR) {
        if (HSI_DIVIDE_BY_FOUR) {
            RCC->CR |= RCC_CR_HSI16DIVEN;
        } else {
            RCC->CR &= ~RCC_CR_HSI16DIVEN;
        }
    }
#endif
    rc = rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_HSION, RCC_CR_HSIRDY,
            ena, HSE_STARTUP_TIMEOUT);

    return rc;
}

/**
  * \brief  Set RTC clock to selected source.
  *
  * Call early during startup, as LSE Startup may take two seconds!
  *
  * Check, if LSEON, LSEBYP, RTCSEL or LSE_DRIVE_LEVEL need change.
  * Reset of LSEON or LSEBYP or change in  RTCSEL or LSE_DRIVE_LEVEL
  * needs reset of backup domain.
  *
  * Check success later when using RTC clock.
  *
  * \param  source Clock source NONE/LSI/LSE/HSE
  * \retval NONE
  */
static void SetRtcClockSource(int source)
{
    int bd_change = 0;
    if (LSI_ON) {
        RCC->CSR |= RCC_CSR_LSION;
    } else {
        RCC->CSR &= ~RCC_CSR_LSION;
    }
    if (source == RTCCLK_KEEP) {
        return;
    }
    if ((source == RTCCLK_NONE) && (RCC_BDCR & RCC_BDCR_RTCEN) ){
        bd_change = 1;
    } else if (((RCC_BDCR & RCC_BDCR_LSEON) && (LSE_VALUE == 0)) ||
               ((RCC_BDCR & RCC_BDCR_LSEBYP) && LSE_BYPASS)      ||
               ((source * RCC_BDCR_RTCSEL_0) != (RCC_BDCR & RCC_BDCR_RTCSEL)) ||
               ((LSE_DRIVE_LEVEL * RCC_BDCR_LSEDRV_0) !=
                (RCC_BDCR & RCC_BDCR_LSEDRV))) {
        bd_change = 1;
    }
    /* Enable backup domain access*/
    PWR_CR |= PWR_CR_DBP;
    if (bd_change) {
       /* Backup Domain protected bits need to be reset by BDRST. */
        RCC_BDCR = RCC_BDCR_BDRST;
    }
    RCC_BDCR =
        source * RCC_BDCR_RTCSEL_0 |
        ((LSE_VALUE)? RCC_BDCR_LSEON : 0) |
        LSE_BYPASS * RCC_BDCR_LSEBYP |
        LSE_DRIVE_LEVEL * RCC_BDCR_LSEDRV_0;
    PWR_CR &= ~PWR_CR_DBP;
}

static void SystemCoreClockUpdate(void);
/**
  * \brief  requests frequency for a given clock type
  *
  * \param  idx index of clock in clock_index_t
  * \retval clock or 0 if idx points to an invalid clock
  */
uint32_t Stm32ClockGet(clock_index_t idx)
{
    if (!sys_clock) {
        SystemCoreClockUpdate();
    }
    switch (idx) {
#if defined(RCC_CFGR3_USART1SW) || defined(RCC_DCKCFGR2_USART1SEL) ||\
    defined(RCC_CCIPR_USART2SEL)
    case HWCLK_LSE:
        return LSE_VALUE;
    case HWCLK_HSI:
        return HSI_VALUE;
#endif
#if defined(MCU_STM32L4)
    case HWCLK_MSI:
        return msi_clock;
#endif
#if 0
/* FIXME: Handle case when PLL is running as USB clock,
   but SYSCLK is not PLLCLK!
*/
# if defined(MCU_STM32F3)
    case HWCLK_PLL:
        return 0;
# endif
#endif
    default:
        return sys_clock >> clk_shift[idx];
    }
    return 0;
}

/*!
 * \brief Switch to PLL with selected source
 *
 * \param  ahb_div    Division factor for AHB BUS
 * \param  apb1_div   Division factor for APB1 BUS
 * \param  apb2_div   Division factor for APB2 BUS
 */
static int SwitchSystemClock(int source)  __attribute__((unused));
static int SwitchSystemClock(int source)
{
    int rc = -1;

    switch (source) {
    case SYSCLK_HSI:
        rc = CtlHsiClock(1);
        if (!rc) {
            rc = rcc_set_and_wait_rdy_value(
                &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI,
                RCC_CFGR_SWS, RCC_CFGR_SWS_HSI, HSE_STARTUP_TIMEOUT);
        }
        break;
#if defined(RCC_CFGR_SWS_HSI48)
    case SYSCLK_HSI48:
        rc = rcc_set_and_wait_rdy(&RCC->CR2, RCC_CR2_HSI48ON, RCC_CR2_HSI48RDY,
                                  1, HSE_STARTUP_TIMEOUT);
        if (!rc) {
            rc = rcc_set_and_wait_rdy_value(
                &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI48,
                RCC_CFGR_SWS, RCC_CFGR_SWS_HSI48, HSE_STARTUP_TIMEOUT);
        }
        break;
#endif
#if defined(RCC_CFGR_SWS_MSI)
    case SYSCLK_MSI:
        rc = CtlMsiClock(MSI_DEFAULT);
        if (!rc) {
            rc = rcc_set_and_wait_rdy_value(
                &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_MSI,
                RCC_CFGR_SWS, RCC_CFGR_SWS_MSI, HSE_STARTUP_TIMEOUT);
        }
        break;
#endif
    case SYSCLK_HSE:
        rc = CtlHseClock(1);
        if (!rc) {
            rc = rcc_set_and_wait_rdy_value(
                &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE,
                RCC_CFGR_SWS, RCC_CFGR_SWS_HSE, HSE_STARTUP_TIMEOUT);
        }
        break;
    case SYSCLK_PLL:
        rc = SetPllClockSource(PLLCLK_SOURCE);
        if (!rc) {
            /* Switch On PLL */
            rc =  rcc_set_and_wait_rdy(&RCC->CR, RCC_CR_PLLON, RCC_CR_PLLRDY,
                                       1, HSE_STARTUP_TIMEOUT);
            if (!rc) {
                rc = rcc_set_and_wait_rdy_value(
                    &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL,
                    RCC_CFGR_SWS, RCC_CFGR_SWS_PLL, HSE_STARTUP_TIMEOUT);
            }
        }
        break;
    }
    if (rc) {
#if defined(RCC_CFGR_SWS_MSI)
        /* something went wrong, switch to MSI */
        CtlMsiClock(MSI_DEFAULT);
        rcc_set_and_wait_rdy_value(
            &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_MSI,
            RCC_CFGR_SWS, RCC_CFGR_SWS_MSI, HSE_STARTUP_TIMEOUT);
#else
        CtlHsiClock(1);
        rcc_set_and_wait_rdy_value(
            &RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI,
            RCC_CFGR_SWS, RCC_CFGR_SWS_HSI, HSE_STARTUP_TIMEOUT);
#endif
    }
    return rc;
}

/*!
 * \brief Restart Clock after Stop Mode
 *
 */
void Stm32RestartClock(void)
{
    SwitchSystemClock(SYSCLK_SOURCE);
}

/*!
 * \brief Set Clock Shift array
 *
 */
static void SetClockShift(void)  __attribute__((unused));
static void SetClockShift(void)
{
    uint32_t hpre;
    uint32_t cfgr;
    uint32_t tmp;

    cfgr = RCC->CFGR;
    hpre = (cfgr & RCC_CFGR_HPRE) / RCC_CFGR_HPRE_0;
    clk_shift[HWCLK_CPU] = AHBPrescTable[hpre];
    tmp = (RCC->CFGR & RCC_CFGR_PPRE1) / RCC_CFGR_PPRE1_0;
    clk_shift[HWCLK_APB1] = APBPrescTable[tmp];
    clk_shift[HWCLK_APB1_TIMER] = GetTimerShift(clk_shift[HWCLK_APB1]);
#if defined(APB2PERIPH_BASE) || defined(MCU_STM32L0)
    tmp = (RCC->CFGR & RCC_CFGR_PPRE2) / RCC_CFGR_PPRE2_0;
    clk_shift[HWCLK_APB2] = APBPrescTable[tmp];
    clk_shift[HWCLK_APB2_TIMER] = GetTimerShift(clk_shift[HWCLK_APB2]);
#endif
}

int SetSysClock(void)
{
    return SetSysClockSource(SYSCLK_SOURCE);
}

/*!
 * \brief Enable RTC clock from source selected  in configuration
 *
 * Called by RtcInit, LcdInit and BackupInit.
 * LSE startup can take up to 2 seconds. User initialization is done
 * late and so eventually less waiting for LSE is needed.
 *
 * \return 0 on success, -1.
 */
int Stm32EnableRtcClock(void)
{
    int source;
    int res = 0;

    if (RTCCLK_SOURCE == RTCCLK_NONE) {
        return -1;
    }
    source = (RCC_BDCR & RCC_BDCR_RTCSEL) / RCC_BDCR_RTCSEL_0;
    if ((RTCCLK_SOURCE != RTCCLK_KEEP) && (source != RTCCLK_SOURCE)) {
        /* Source mismatch.*/
        res = -1;
        goto done;
    }
    if (RCC_BDCR & RCC_BDCR_RTCEN) {
    /* Clock is already running.*/
        goto done;
    }
    PWR_CR |= PWR_CR_DBP;
    switch (source) {
    case RTCCLK_LSE:
        if (!(RCC_BDCR & RCC_BDCR_LSERDY)) {
            int i;
            RCC_BDCR |= RCC_BDCR_LSEON;
            for (i = 0; i < 4000; i++) {
                NutSleep(1);
                if (RCC_BDCR & RCC_BDCR_LSERDY) {
                    break;
                }
            }
            if (!(RCC_BDCR & RCC_BDCR_LSERDY)) {
                res = -1;
                goto done;
            }
        }
        break;
    case RTCCLK_LSI:
        if (!(RCC->CSR & RCC_CSR_LSIRDY)) {
            RCC->CSR |= RCC_CSR_LSION;
            NutSleep(1);
            }
        if (!(RCC->CSR & RCC_CSR_LSIRDY)) {
            res = -1;
            goto done;
        }
        break;
    case RTCCLK_HSE:
        if (!(RCC->CR & RCC_CR_HSERDY)) {
            /* RCC_CR_RTCPRE_0 only != 0 on L0/1.
             * HSE was not set up yet.
             * Set divisor and bypass before enabling HSE.
             */
            RCC->CR |= (RTC_PRE_VAL * RCC_CR_RTCPRE_0) |
                (HSE_BYPASS * RCC_CR_HSEBYP);
            RCC->CR |= RCC_CR_HSEON;
            NutSleep(4);
        }
        if (!(RCC->CR & RCC_CR_HSERDY)) {
            res = -1;
            goto done;
        }
        break;
    }
    while( (RCC_BDCR & RCC_BDCR_RTCEN) == 0 ){
        /* NUCLEO-F7 seems to hang here with RTC == LSE
         * when commands where only issued once.
         */
        RCC_BDCR |= RCC_BDCR_RTCEN;
    }
done:
    PWR_CR &= ~PWR_CR_DBP;
    return res;
}
