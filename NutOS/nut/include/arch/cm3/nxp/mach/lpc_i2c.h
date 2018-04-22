#ifndef _ARCH_CM3_NXP_MACH_LPC_I2C_H_
#define _ARCH_CM3_NXP_MACH_LPC_I2C_H_

/*
 * Copyright 2012 by egnite GmbH
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
 * \file arch/cm3/nxp/mach/lpc_i2c.h
 * \brief LPC I2C definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchCm3LpcI2c
 */
/*@{*/

/*! \name I2C Control Set Register */
/*@{*/
#define I2C_CONSET_OFF  0x0000
#define I2C0CONSET      (LPC_I2C0_BASE + I2C_CONSET_OFF)
#define I2C1CONSET      (LPC_I2C1_BASE + I2C_CONSET_OFF)

#define I2C_CONSET_AA   _BV(2)
#define I2C_CONSET_SI   _BV(3)
#define I2C_CONSET_STO  _BV(4)
#define I2C_CONSET_STA  _BV(5)
#define I2C_CONSET_I2EN _BV(6)
/*@}*/

/*! \name I2C Control Clear Register */
/*@{*/
#define I2C_CONCLR_OFF  0x0018
#define I2C0CONCLR      (LPC_I2C0_BASE + I2C_CONCLR_OFF)
#define I2C1CONCLR      (LPC_I2C1_BASE + I2C_CONCLR_OFF)

#define I2C_CONCLR_AAC   _BV(2)
#define I2C_CONCLR_SIC   _BV(3)
#define I2C_CONCLR_STAC  _BV(5)
#define I2C_CONCLR_I2ENC _BV(6)
/*@}*/

/*! \name I2C Status Register */
/*@{*/
#define I2C_STAT_OFF    0x0004
#define I2C0STAT        (LPC_I2C0_BASE + I2C_STAT_OFF)
#define I2C1STAT        (LPC_I2C1_BASE + I2C_STAT_OFF)
/*@}*/

/*! \name I2C Data Register */
/*@{*/
#define I2C_DAT_OFF     0x0008
#define I2C0DAT         (LPC_I2C0_BASE + I2C_DAT_OFF)
#define I2C1DAT         (LPC_I2C1_BASE + I2C_DAT_OFF)
/*@}*/

/*! \name I2C Slave Address Registers */
/*@{*/
#define I2C_ADR0_OFF    0x000C
#define I2C_ADR1_OFF    0x0020
#define I2C_ADR2_OFF    0x0024
#define I2C_ADR3_OFF    0x0028
#define I2C0ADR0        (LPC_I2C0_BASE + I2C_ADR0_OFF)
#define I2C0ADR1        (LPC_I2C0_BASE + I2C_ADR1_OFF)
#define I2C0ADR2        (LPC_I2C0_BASE + I2C_ADR2_OFF)
#define I2C0ADR3        (LPC_I2C0_BASE + I2C_ADR3_OFF)
#define I2C1ADR0        (LPC_I2C1_BASE + I2C_ADR0_OFF)
#define I2C1ADR1        (LPC_I2C1_BASE + I2C_ADR1_OFF)
#define I2C1ADR2        (LPC_I2C1_BASE + I2C_ADR2_OFF)
#define I2C1ADR3        (LPC_I2C1_BASE + I2C_ADR3_OFF)

#define I2C_ADR_GC      _BV(0)
/*@}*/

/*! \name I2C Slave Address Mask Registers */
/*@{*/
#define I2C_MASK0_OFF   0x0030
#define I2C_MASK1_OFF   0x0034
#define I2C_MASK2_OFF   0x0038
#define I2C_MASK3_OFF   0x003C
#define I2C0MASK0       (LPC_I2C0_BASE + I2C_MASK0_OFF)
#define I2C0MASK1       (LPC_I2C0_BASE + I2C_MASK1_OFF)
#define I2C0MASK2       (LPC_I2C0_BASE + I2C_MASK2_OFF)
#define I2C0MASK3       (LPC_I2C0_BASE + I2C_MASK3_OFF)
#define I2C1MASK0       (LPC_I2C1_BASE + I2C_MASK0_OFF)
#define I2C1MASK1       (LPC_I2C1_BASE + I2C_MASK1_OFF)
#define I2C1MASK2       (LPC_I2C1_BASE + I2C_MASK2_OFF)
#define I2C1MASK3       (LPC_I2C1_BASE + I2C_MASK3_OFF)
/*@}*/

/*! \name Duty Cycle Registers */
/*@{*/
#define I2C_SCLL_OFF    0x0014
#define I2C0SCLL        (LPC_I2C0_BASE + I2C_SCLL_OFF)
#define I2C1SCLL        (LPC_I2C1_BASE + I2C_SCLL_OFF)
#define I2C_SCLH_OFF    0x0010
#define I2C0SCLH        (LPC_I2C0_BASE + I2C_SCLH_OFF)
#define I2C1SCLH        (LPC_I2C1_BASE + I2C_SCLH_OFF)
/*@}*/

/*! \name I2C Monitor Mode Control Register */
/*@{*/
#define I2C_MMCTRL_OFF  0x001C
#define I2C0MMCTRL      (LPC_I2C0_BASE + I2C_MMCTRL_OFF)
#define I2C1MMCTRL      (LPC_I2C1_BASE + I2C_MMCTRL_OFF)

#define I2C_MMCTRL_MM_ENA       _BV(0)
#define I2C_MMCTRL_ENA_SCL      _BV(1)
#define I2C_MMCTRL_MATCH_ALL    _BV(2)
/*@}*/

/*! \name I2C Data Buffer Register */
/*@{*/
#define I2C_DATA_BUFFER_OFF 0x002C
#define I2C0DATA_BUFFER (LPC_I2C0_BASE + I2C_DATA_BUFFER_OFF)
#define I2C1DATA_BUFFER (LPC_I2C1_BASE + I2C_DATA_BUFFER_OFF)
/*@}*/

/*@}*/
#endif
