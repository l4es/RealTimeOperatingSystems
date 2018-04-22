#ifndef _STM32_CLK_H_
#define _STM32_CLK_H_

/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * $Id: stm32_gpio.c 3182 2010-10-17 21:46:04Z Astralix $
 * \endverbatim
 */
#include <stdint.h>

#if !defined(HSE_VALUE)
# define HSE_VALUE 0
#endif

# if !defined(RTC_PRE)
/* Calculate RTC_PRE divisor as it is not given*/
#  if defined(RCC_CR_RTCPRE)
/*  L0/1 have HSE prescaler 2/4/8/16 */
#   if HSE_VALUE >   8000000
#    define RTC_PRE     16
#   elif HSE_VALUE > 4000000
#    define RTC_PRE      8
#   elif HSE_VALUE > 2000000
#    define RTC_PRE      4
#   else
#    define RTC_PRE      2
#   endif
#  elif defined(RCC_CFGR_RTCPRE)
/* F2/F4 has HSE prescaler 2.. 32*/
#   if HSE_VALUE <= 2000000
#    define RTC_PRE 2
#   elif HSE_VALUE > 32000000
#    warning HSE_VALUE to high to reach 1 MHz RTC clock
#   else
#    define RTC_PRE (((HSE_VALUE -1 )/ 1000000) + 1)
#   endif
#  elif defined(MCU_STM32F1)
#   define RTC_PRE 128
/* F3/F0/L4 has fixed HSE prescaler of 32*/
#  else
#   define RTC_PRE 32
#  endif
# endif

/* Check value of calculated or given RTC_PRE and find RTC_PRE_VAL*/
# if defined(RCC_CR_RTCPRE)
#  if  RTC_PRE == 16
#   define  RTC_PRE_VAL  3
#  elif  RTC_PRE == 8
#   define  RTC_PRE_VAL  2
#  elif  RTC_PRE == 4
#   define  RTC_PRE_VAL  1
#  elif  RTC_PRE == 2
#   define  RTC_PRE_VAL  0
#  else
#   warning Illegal RTC_PRE for L1/L0 given
#  endif
# elif defined(RCC_CFGR_RTCPRE_2)
#  if RTC_PRE  < 2 || RTC_PRE > 32
#   warning Illegal RTC_PRE for F2/F4 given
#  else
#   define RTC_PRE_VAL (RTC_PRE -1 )
#  endif
# elif defined(MCU_STM32F1)
#  if RTC_PRE != 128
#    warning Illegal RTC_PRE for F1 given, only 128 allowed
#  else
#   define RTC_PRE_VAL       0
#   define RCC_CR_RTCPRE     0
#   define RCC_CR_RTCPRE_0   0
#   define RCC_CFGR_RTCPRE_0 0
#  endif
# else
#  if RTC_PRE != 32
#    warning Illegal RTC_PRE for F0/F3 given, only 32 allowed
#  else
#   define RTC_PRE_VAL 0
#   define RCC_CR_RTCPRE     0
#   define RCC_CR_RTCPRE_0   0
#   define RCC_CFGR_RTCPRE_0 0
#  endif
# endif

/* L0/L1 use RCC_CR_RTCPRE, define to 0 else */
# if !defined(RCC_CR_RTCPRE)
#  define RCC_CR_RTCPRE   0
#  define RCC_CR_RTCPRE_0 0
# endif
/* F2/4/7 use RCC_CFGR_RTCPRE, define to 0 else */
# if !defined(RCC_CFGR_RTCPRE)
#  define RCC_CFGR_RTCPRE   0
#  define RCC_CFGR_RTCPRE_0 0
#endif

/* STM32 Clock source selectors */
#define SYSCLK_HSI    1
#define SYSCLK_PLL    2
#define SYSCLK_HSE    3
#define SYSCLK_HSI48  4
#define SYSCLK_MSI    5
#define SYSCLK_HSIDIV 6
#define SYSCLK_PLLR   7

#define PLLCLK_AUTO         0
#define PLLCLK_HSI_DIV2     1
#define PLLCLK_HSI_PREDIV   2
#define PLLCLK_HSE_PREDIV   3
#define PLLCLK_HSI48_PREDIV 4
#define PLLCLK_HSI          5
#define PLLCLK_HSE          6
#define PLLCLK_HSE_PLL2     7
#define PLLCLK_MSI          8

/* RTC clock sources */
#define RTCCLK_KEEP  0
#define RTCCLK_LSE   1
#define RTCCLK_LSI   2
#define RTCCLK_HSE   3
#define RTCCLK_NONE  4

#if !defined(HSE_VALUE)
# define HSE_VALUE 0
#endif

#if !defined(LSE_VALUE)
# define LSE_VALUE 0
#endif

/* MSI clock ranges*/
typedef enum
{
    MSI_OFF = -2,
    MSI_FAILURE,
#if   defined(MCU_STM32L0) || defined(MCU_STM32L1)
    MSI_65_5k,
    MSI_131k,
    MSI_262k,
    MSI_524k,
    MSI_1050k,
    MSI_2100k,
    MSI_4200k,
#elif defined(MCU_STM32L4)
    MSI_100k,
    MSI_200k,
    MSI_400k,
    MSI_800k,
    MSI_1M,
    MSI_2M,
    MSI_4M,
    MSI_8M,
    MSI_16M,
    MSI_24M,
    MSI_32M,
    MSI_48M
#endif
}msi_range_t;

typedef enum
{
    HWCLK_CPU = 0,
    HWCLK_APB1,
    HWCLK_APB1_TIMER,
#if defined(APB2PERIPH_BASE) || defined(MCU_STM32L0)
    HWCLK_APB2,
    HWCLK_APB2_TIMER,
#endif
    HWCLK_SYS,
#if defined(RCC_CFGR3_USART1SW) || defined(RCC_DCKCFGR2_USART1SEL) ||\
    defined(RCC_CCIPR_USART2SEL)
    HWCLK_HSI,
    HWCLK_LSE,
#endif
#if defined(MCU_STM32L4)
    HWCLK_MSI,
#endif
    HWCLK_MAX,
}clock_index_t;

#if defined(APB2PERIPH_BASE)
# define BASE2CLKSRC(base) ((base < APB2PERIPH_BASE) ? HWCLK_APB1 : HWCLK_APB2)
# define BASE2TCLKSRC(base) ((base < APB2PERIPH_BASE) ? \
                             HWCLK_APB1_TIMER : HWCLK_APB2_TIMER)
#else
# define BASE2CLKSRC(base) HWCLK_APB1
# define BASE2TCLKSRC(base) HWCLK_APB1_TIMER
#endif

extern uint32_t Stm32ClockGet(clock_index_t idx);

#define SysCtlClockGet() Stm32ClockGet(HWCLK_CPU)

extern int SetSysClock(void);
int Stm32EnableRtcClock(void);
extern void Stm32RestartClock(void);

#endif /* _STM32_CLK_H_ */
