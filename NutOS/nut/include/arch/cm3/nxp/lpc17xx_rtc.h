#ifndef _LPC17XX_RTC_H_
#define _LPC17XX_RTC_H_

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
 * Parts taken from lpc177x_8x_rtc.h       2011-06-02
 * file     lpc177x_8x_rtc.h
 * brief    Contains all macro definitions and function prototypes
 *          support for Ethernet MAC firmware library on LPC177x_8x
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
 **********************************************************************/

/*!
 * \verbatim
 * $Id: $
 * \endverbatim
 */


/*============================================================================*
  LPC17xx RTC misc register defines
 *============================================================================*/

/*----------------------------------------------------------------------------*
  ILR register definitions
 *----------------------------------------------------------------------------*/

/* ILR register mask */
#define RTC_ILR_BITMASK         0x00000003

/* Bit inform the source interrupt is counter increment*/
#define RTC_IRL_RTCCIF          _BV(0)

/* Bit inform the source interrupt is alarm match*/
#define RTC_IRL_RTCALF          _BV(1)


/*----------------------------------------------------------------------------*
  CCR register definitions
 *----------------------------------------------------------------------------*/

/* CCR register mask */
#define RTC_CCR_BITMASK         0x00000013

/* Clock enable */
#define RTC_CCR_CLKEN           _BV(0)

/* Clock reset */
#define RTC_CCR_CTCRST          _BV(1)

/* Calibration counter enable */
#define RTC_CCR_CCALEN          _BV(4)


/*----------------------------------------------------------------------------*
  CIIR register definitions
 *----------------------------------------------------------------------------*/

/* Counter Increment Interrupt bit for second */
#define RTC_CIIR_IMSEC          _BV(0)

/* Counter Increment Interrupt bit for minute */
#define RTC_CIIR_IMMIN          _BV(1)

/* Counter Increment Interrupt bit for hour */
#define RTC_CIIR_IMHOUR         _BV(2)

/* Counter Increment Interrupt bit for day of month */
#define RTC_CIIR_IMDOM          _BV(3)

/* Counter Increment Interrupt bit for day of week */
#define RTC_CIIR_IMDOW          _BV(4)

/* Counter Increment Interrupt bit for day of year */
#define RTC_CIIR_IMDOY          _BV(5)

/* Counter Increment Interrupt bit for month */
#define RTC_CIIR_IMMON          _BV(6)

/* Counter Increment Interrupt bit for year */
#define RTC_CIIR_IMYEAR         _BV(7)

/* CIIR bit mask */
#define RTC_CIIR_BITMASK        0xFF


/*----------------------------------------------------------------------------*
  AMR register definitions
 *----------------------------------------------------------------------------*/

/* Counter Increment Select Mask bit for second */
#define RTC_AMR_AMRSEC          _BV(0)

/* Counter Increment Select Mask bit for minute */
#define RTC_AMR_AMRMIN          _BV(1)

/* Counter Increment Select Mask bit for hour */
#define RTC_AMR_AMRHOUR         _BV(2)

/* Counter Increment Select Mask bit for day of month */
#define RTC_AMR_AMRDOM          _BV(3)

/* Counter Increment Select Mask bit for day of week */
#define RTC_AMR_AMRDOW          _BV(4)

/* Counter Increment Select Mask bit for day of year */
#define RTC_AMR_AMRDOY          _BV(5)

/* Counter Increment Select Mask bit for month */
#define RTC_AMR_AMRMON          _BV(6)

/* Counter Increment Select Mask bit for year */
#define RTC_AMR_AMRYEAR         _BV(7)

/* AMR bit mask */
#define RTC_AMR_BITMASK         0xFF


/*----------------------------------------------------------------------------*
  RTC_AUX register definitions
 *----------------------------------------------------------------------------*/

/* RTC Oscillator Fail detect flag */
#define RTC_AUX_RTC_OSCF         _BV(4)


/*----------------------------------------------------------------------------*
  RTC_AUXEN register definitions
 *----------------------------------------------------------------------------*/

/* Oscillator Fail Detect interrupt enable*/
#define RTC_AUXEN_RTC_OSCFEN     _BV(4)


/*============================================================================*
  Consolidated time registers
 *============================================================================*/

/*----------------------------------------------------------------------------*
  Consolidated Time Register 0 definitions
 *----------------------------------------------------------------------------*/

#define RTC_CTIME0_SECONDS_MASK     0x3F
#define RTC_CTIME0_MINUTES_MASK     0x3F00
#define RTC_CTIME0_HOURS_MASK       0x1F0000
#define RTC_CTIME0_DOW_MASK         0x7000000

/*----------------------------------------------------------------------------*
  Consolidated Time Register 1 definitions
 *----------------------------------------------------------------------------*/

#define RTC_CTIME1_DOM_MASK         0x1F
#define RTC_CTIME1_MONTH_MASK       0xF00
#define RTC_CTIME1_YEAR_MASK        0xFFF0000


/*----------------------------------------------------------------------------*
  Consolidated Time Register 2 definitions
 *----------------------------------------------------------------------------*/

#define RTC_CTIME2_DOY_MASK         0x0FFF


/*----------------------------------------------------------------------------*
  Time Counter Group and Alarm register
 *----------------------------------------------------------------------------*/

/* SEC register mask */
#define RTC_SEC_MASK            0x0000003F

/* MIN register mask */
#define RTC_MIN_MASK            0x0000003F

/* HOUR register mask */
#define RTC_HOUR_MASK           0x0000001F

/* DOM register mask */
#define RTC_DOM_MASK            0x0000001F

/* DOW register mask */
#define RTC_DOW_MASK            0x00000007

/* DOY register mask */
#define RTC_DOY_MASK            0x000001FF

/* MONTH register mask */
#define RTC_MONTH_MASK          0x0000000F

/* YEAR register mask */
#define RTC_YEAR_MASK           0x00000FFF


/* Maximum value of second */
#define RTC_SECOND_MAX          59

/* Maximum value of minute*/
#define RTC_MINUTE_MAX          59

/* Maximum value of hour*/
#define RTC_HOUR_MAX            23

/* Minimum value of month*/
#define RTC_MONTH_MIN           1

/* Maximum value of month*/
#define RTC_MONTH_MAX           12

/* Minimum value of day of month*/
#define RTC_DAYOFMONTH_MIN      1

/* Maximum value of day of month*/
#define RTC_DAYOFMONTH_MAX      31

/* Maximum value of day of week*/
#define RTC_DAYOFWEEK_MAX       6

/* Minimum value of day of year*/
#define RTC_DAYOFYEAR_MIN       1

/* Maximum value of day of year*/
#define RTC_DAYOFYEAR_MAX       366

/* Maximum value of year*/
#define RTC_YEAR_MAX            4095


/*----------------------------------------------------------------------------*
  Calibration register
 *----------------------------------------------------------------------------*/

/* Calibration value */
#define RTC_CALIBRATION_CALVAL_MASK     0x1FFFF

/* Calibration direction */
#define RTC_CALIBRATION_LIBDIR          _BV(17)

/* Calibration max value */
#define RTC_CALIBRATION_MAX             0x20000

/* Calibration definitions */
#define RTC_CALIB_DIR_FORWARD           0
#define RTC_CALIB_DIR_BACKWARD          1


#endif /* __LPC17XX_RTC_H_ */
