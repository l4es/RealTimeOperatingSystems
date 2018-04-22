#ifndef _LPC177X_8X_WWDT_H_
#define _LPC177X_8X_WWDT_H_

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
 * Parts taken from lpc177x_8x_rtc.h       2011-06-02
 * file     lpc177x_8x_wwdt.h
 * brief    Contains all macro definitions and function prototypes
 *          support forWindow Watchdog Timer firmware library on LPC177x_8x
 * version  1.0
 * date     02. June. 2011
 * author   NXP MCU SW Application Team
 *
 * Copyright(C) 2011, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
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
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors'
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 **********************************************************************/

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

/* Calculation macros, time is calculated by usec */
#define WDT_GET_FROM_USEC(time)     (time/((WDT_US_INDEX * 4)/WDT_OSC))
#define WDT_GET_USEC(counter)       (counter * ((WDT_US_INDEX * 4)/WDT_OSC))


/*----------------------------------------------------------------------------*
  Bit definitions
 *----------------------------------------------------------------------------*/

/* WDT interrupt enable bit */
#define WDT_WDMOD_WDEN             _BV(0)
/* WDT interrupt enable bit */
#define WDT_WDMOD_WDRESET          _BV(1)
/* WDT time out flag bit */
#define WDT_WDMOD_WDTOF            _BV(2)
/* WDT Time Out flag bit */
#define WDT_WDMOD_WDINT            _BV(3)
/* WDT Protect flag bit */
#define WDT_WDMOD_WDPROTECT        _BV(4)

/* Define divider index for microsecond ( us ) */
#define WDT_US_INDEX               1000000

/* WDT Time out minimum value */
#define WDT_TIMEOUT_MIN            0xFF
/* WDT Time out maximum value */
#define WDT_TIMEOUT_MAX            0x00FFFFFF

/* WDT Warning minimum value */
#define WDT_WARNINT_MIN            0xFF
/* WDT Warning maximum value */
#define WDT_WARNINT_MAX            0x000003FF

/* WDT Windowed minimum value */
#define WDT_WINDOW_MIN             0xFF
/* WDT Windowed minimum value */
#define WDT_WINDOW_MAX             0x00FFFFFF

/* WDT timer constant register mask */
#define WDT_WDTC_MASK              0x00FFFFFF
/* WDT warning value register mask */
#define WDT_WDWARNINT_MASK         0x000003FF
/* WDT feed sequence register mask */
#define WDT_WDFEED_MASK            0x000000FF

/* WDT flag */
#define WDT_WARNINT_FLAG           0
#define WDT_TIMEOUT_FLAG           1

/* WDT mode definitions */
#define WDT_PROTECT_MODE           0
#define WDT_RESET_MODE             1


/* WDT Timer value definition (us) */
#define WDT_TIMEOUT_USEC_MIN       ((uint32_t)(WDT_GET_USEC(WDT_TIMEOUT_MIN)))
#define WDT_TIMEOUT_USEC_MAX       ((uint32_t)(WDT_GET_USEC(WDT_TIMEOUT_MAX)))

#define WDT_TIMEWARN_USEC_MIN      ((uint32_t)(WDT_GET_USEC(WDT_WARNINT_MIN)))
#define WDT_TIMEWARN_USEC_MAX      ((uint32_t)(WDT_GET_USEC(WDT_WARNINT_MAX)))

#define WDT_TIMEWINDOWED_USEC_MIN  ((uint32_t)(WDT_GET_USEC(WDT_WINDOW_MIN)))
#define WDT_TIMEWINDOWED_USEC_MAX  ((uint32_t)(WDT_GET_USEC(WDT_WINDOW_MAX)))

#endif /* _LPC177X_8X_WWDT_H_ */
