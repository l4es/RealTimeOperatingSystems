#ifndef _LPC176X_WDT_H_
#define _LPC176X_WDT_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 *
 **************************************************************************
 *
 * Parts taken from lpc17xx_wdt.h
 *
 * file     : lpc17xx_wdt.h
 * brief    : Contains all macro definitions and function prototypes
 *              support for WDT firmware library on LPC17xx
 * version  : 1.0
 * date     : 9. April. 2009
 * author   : HieuNguyen
 **************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **************************************************************************/

/*!
 * \verbatim
 * $Id: $
 * \endverbatim
 */


/*============================================================================*
  LPC177x_8x Windowed watchdog timer
 *============================================================================*/

/*----------------------------------------------------------------------------*
  Some macros
 *----------------------------------------------------------------------------*/

#define PPCLK_WDT   4000000

/* Calculation macros, time is calculated by usec */
#define WDT_GET_FROM_USEC(time)     (time/((WDT_US_INDEX * 4)/PPCLK_WDT))
#define WDT_GET_USEC(counter)       (counter * ((WDT_US_INDEX * 4)/PPCLK_WDT))

/*----------------------------------------------------------------------------*
  WDT Control register
 *----------------------------------------------------------------------------*/

/* WDT interrupt enable bit */
#define WDT_WDMOD_WDEN              _BV(0)
/* WDT interrupt enable bit */
#define WDT_WDMOD_WDRESET           _BV(1)
/* WDT time out flag bit */
#define WDT_WDMOD_WDTOF             _BV(2)
/* WDT Time Out flag bit */
#define WDT_WDMOD_WDINT             _BV(3)
/* WDT Mode */
#define WDT_WDMOD(n)                _BV(1)


/*----------------------------------------------------------------------------*
  Some defines
 *----------------------------------------------------------------------------*/

/* Define divider index for microsecond ( us ) */
#define WDT_US_INDEX                1000000
/* WDT Time out minimum value */
#define WDT_TIMEOUT_MIN             0xFF
/* WDT Time out maximum value */
#define WDT_TIMEOUT_MAX             0xFFFFFFFF


/* Watchdog mode register mask */
#define WDT_WDMOD_MASK              0x02
/* Watchdog timer constant register mask */
#define WDT_WDTC_MASK               0xFFFFFFFF
/* Watchdog feed sequence register mask */
#define WDT_WDFEED_MASK             0x000000FF
/* Watchdog timer value register mask */
#define WDT_WDCLKSEL_MASK           0x03
/* Clock selected from internal RC */
#define WDT_WDCLKSEL_RC             0x00
/* Clock selected from PCLK */
#define WDT_WDCLKSEL_PCLK           0x01
/* Clock selected from external RTC */
#define WDT_WDCLKSEL_RTC            0x02


/*----------------------------------------------------------------------------*
  WDT enums
 *----------------------------------------------------------------------------*/


/* Clock source option for WDT */
typedef enum {
    WDT_CLKSRC_IRC  = 0,   /*!< Clock source from Internal RC oscillator */
    WDT_CLKSRC_PCLK = 1,   /*!< Selects the APB peripheral clock (PCLK) */
    WDT_CLKSRC_RTC  = 2    /*!< Selects the RTC oscillator */
} WDT_CLK_OPT;

#define PARAM_WDT_CLK_OPT(OPTION) ((OPTION ==WDT_CLKSRC_IRC)||\
                                  (OPTION ==WDT_CLKSRC_IRC)||\
                                  (OPTION ==WDT_CLKSRC_IRC))

/* WDT operation mode */
typedef enum {
    WDT_MODE_INT_ONLY = 0, /*!< Use WDT to generate interrupt only */
    WDT_MODE_RESET    = 1  /*!< Use WDT to generate interrupt and reset MCU */
} WDT_MODE_OPT;

#define PARAM_WDT_MODE_OPT(OPTION)  ((OPTION ==WDT_MODE_INT_ONLY)||\
                                  (OPTION ==WDT_MODE_RESET))

#endif /* _LPC176X_WDT_H_ */
