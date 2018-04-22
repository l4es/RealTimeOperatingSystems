#ifndef _ARCH_CM3_NXP_MACH_LPC_ADC_H_
#define _ARCH_CM3_NXP_MACH_LPC_ADC_H_

/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/cm3/nxp/mach/lpc_adc.h
 * \brief LPC ADC definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcAdc
 */
/*@{*/

/*! \name ADC Control Register */
/*@{*/
#define ADC_CR_OFF          0x00000000

#define ADC_CR_CH_SEL(x)    _BV(x)
#define ADC_CR_CLKDIV(d)    ((n) << 8)
#define ADC_CR_BURST        _BV(16)
#define ADC_CR_PDN          _BV(21)
#define ADC_CR_START_MODE_SEL(x) ((x) << 24)
#define ADC_CR_START_MASK   ADC_CR_START_MODE_SEL(7)
#define ADC_CR_START_NOW    (1UL<<24))
#define ADC_CR_START_EINT0  (2 << 24)
#define ADC_CR_START_CAP01  (3 << 24)
#define ADC_CR_START_MAT01  (4 << 24)
#define ADC_CR_START_MAT03  (5 << 24)
#define ADC_CR_START_MAT10  (6 << 24)
#define ADC_CR_START_MAT11  (7 << 24)
#define ADC_CR_EDGE         _BV(27)
/*@}*/

/*! \name ADC Global Data Register */
/*@{*/
#define ADC_GDR_OFF         0x00000004

#define ADC_GDR_RESULT(x)   (((x) >> 4) & 0xFFF)
#define ADC_GDR_CH(x)       (((x) >> 24) & 7)
#define ADC_GDR_CH_MASK     (7 << 24)
#define ADC_GDR_OVERRUN_FLAG _BV(30)
#define ADC_GDR_DONE_FLAG   _BV(31)
/*@}*/

/*! \name ADC Interrupt Register */
/*@{*/
#define ADC_INTEN_OFF       0x0000000C

#define ADC_INTEN_CH(x)     _BV(x)
#define ADC_INTEN_GLOBAL    _BV(8)
/*@}*/

/*! \name ADC Data Registers */
/*@{*/
#define ADC_DR_OFF(x)       ((x) * 4 + 0x00000010)

#define ADC_DR_OVERRUN_FLAG _BV(30)
#define ADC_DR_DONE_FLAG    _BV(31)
/*@}*/

/*! \name ADC Status Register */
/*@{*/
#define ADC_STAT_OFF        0x00000030

#define ADC_STAT_CH_DONE_FLAG(x)    _BV(x)
#define ADC_STAT_CH_OVERRUN_FLAG(x) _BV((x) + 8)
#define ADC_STAT_INT_FLAG           _BV(16)
/*@}*/

/*! \name ADC Trim Register */
/*@{*/
#define ADC_TRIM_OFF        0x00000034
/*@}*/


#if defined(LPC_ADC_BASE)
#define AD0CR       (LPC_ADC_BASE + ADC_CR_OFF)
#define AD0GDR      (LPC_ADC_BASE + ADC_GDR_OFF)
#define AD0INTEN    (LPC_ADC_BASE + ADC_INTEN_OFF)
#define AD0DR(x)    (LPC_ADC_BASE + ADC_DR_OFF(x))
#define AD0STAT     (LPC_ADC_BASE + ADC_STAT_OFF)
#define AD0TRIM     (LPC_ADC_BASE + ADC_TRIM_OFF)
#endif

/*@}*/
#endif
