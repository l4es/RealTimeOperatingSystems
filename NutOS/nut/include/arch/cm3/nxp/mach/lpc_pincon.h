#ifndef _ARCH_CM3_NXP_MACH_LPC_PINCON_H_
#define _ARCH_CM3_NXP_MACH_LPC_PINCON_H_

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
 * \file arch/cm3/nxp/mach/lpc_pincon.h
 * \brief LPC pin connect block definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmLpcPinCon
 */
/*@{*/


/*! \name Pin Function Select Registers */
/*@{*/
#define PINSEL_OFF(x)       (x * 4)
#define PINSEL(x)           (LPC_PINCON_BASE + PINSEL_OFF(x))
/*@}*/

/*! \name Pin Mode Select Registers */
/*@{*/
#define PINMODE_OFF(x)      (0x00000040 + x * 4)
#define PINMODE(x)          (LPC_PINCON_BASE + PINMODE_OFF(x))
/*@}*/

/*! \name Open Drain Mode Control Registers */
/*@{*/
#define PINMODE_OD_OFF(x)   (0x00000068 + x * 4)
#define PINMODE_OD(x)       (LPC_PINCON_BASE + PINMODE_OD_OFF(x))
/*@}*/

/*! \name I2C Pin Configuration Register */
/*@{*/
#define I2CPADCFG_OFF       0x0000007C
#define I2CPADCFG           (LPC_PINCON_BASE + I2CPADCFG_OFF)
/*@}*/


/*@}*/
#endif
