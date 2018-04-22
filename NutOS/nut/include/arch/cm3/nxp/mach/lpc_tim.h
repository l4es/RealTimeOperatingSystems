#ifndef _ARCH_CM3_NXP_MACH_LPC_TIM_H_
#define _ARCH_CM3_NXP_MACH_LPC_TIM_H_

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
 * \file arch/cm3/nxp/mach/lpc_tim.h
 * \brief LPC timer/counter definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmLpcTim
 */
/*@{*/


/*! \name Timer Interrupt Register */
/*@{*/
#define TIM_IR_OFF          0x00000000

#define TIM_IR_MR0          0x00000001
#define TIM_IR_MR1          0x00000002
#define TIM_IR_MR2          0x00000004
#define TIM_IR_MR3          0x00000008
#define TIM_IR_MR(x)        _BV(x)
#define TIM_IR_CR0          0x00000010
#define TIM_IR_CR1          0x00000020
#define TIM_IR_CR(x)        _BV((x) + 4)
/*@}*/

/*! \name Timer Control Register */
/*@{*/
#define TIM_TCR_OFF         0x00000004

#define TIM_TCR_ENA         0x00000001
#define TIM_ENABLE          TIM_TCR_ENA
#define TIM_TCR_RST         0x00000002
#define TIM_RESET           TIM_TCR_RST
/*@}*/

/*! \name Timer Counter Register */
/*@{*/
#define TIM_TC_OFF          0x00000008
/*@}*/

/*! \name Timer Prescale Register */
/*@{*/
#define TIM_PR_OFF          0x0000000C
/*@}*/

/*! \name Timer Prescale Counter Register */
/*@{*/
#define TIM_PC_OFF          0x00000010
/*@}*/

/*! \name Timer Match Control Register */
/*@{*/
#define TIM_MCR_OFF         0x00000014
#define TIM_MCR_MR0I        0x00000001
#define TIM_MCR_MR0R        0x00000002
#define TIM_MCR_MR0S        0x00000004
#define TIM_MCR_MR1I        0x00000008
#define TIM_MCR_MR1R        0x00000010
#define TIM_MCR_MR1S        0x00000020
#define TIM_MCR_MR2I        0x00000040
#define TIM_MCR_MR2R        0x00000080
#define TIM_MCR_MR2S        0x00000100
#define TIM_MCR_MR3I        0x00000200
#define TIM_MCR_MR3R        0x00000400
#define TIM_MCR_MR3S        0x00000800

#define TIM_INT_ON_MATCH(x)     _BV((x) * 3)
#define TIM_RESET_ON_MATCH(x)   _BV((x) * 3 + 1)
#define TIM_STOP_ON_MATCH(x)    _BV((x) * 3 + 2)
#define TIM_MCR_CHANNEL_MASK(x) (7 << ((x) * 3))
/*@}*/

/*! \name Timer Match Registers */
/*@{*/
#define TIM_MR_OFF(x)       (0x00000018 + ((x) * 4))
/*@}*/

/*! \name Timer Capture Control Register */
/*@{*/
#define TIM_CCR_OFF         0x00000028
#define TIM_CAP_RISING(x)   _BV((x) * 3)
#define TIM_CAP_FALLING(x)  _BV((x) * 3 + 1)
#define TIM_INT_ON_CAP(x)   _BV((x) * 3 + 2)
#define TIM_EDGE_MASK(x)    (3 << ((x) * 3))
#define TIM_CCR_CHANNEL_MASK(x) (7 << ((x) * 3))
/*@}*/

/*! \name Timer Capture Registers */
/*@{*/
#define TIM_CR_OFF(x)       (0x0000002C + ((x) * 4)
/*@}*/

/*! \name Timer External Match Register */
/*@{*/
#define TIM_EMR_OFF         0x0000003C

#define TIM_EM(x)           _BV(x)
#define TIM_EM_NOTHING      0x0
#define TIM_EM_LOW          0x1
#define TIM_EM_HIGH         0x2
#define TIM_EM_TOGGLE       0x3
#define TIM_EM_SET(x,f)     ((f) << ((x) + 4))
#define TIM_EM_MASK(x)      (3 << ((x) + 4))
/*@}*/

/*! \name Timer Count Control Register */
/*@{*/
#define TIM_CTCR_OFF        0x00000070

#define TIM_TIMER_MODE           0
#define TIM_COUNTER_RISING_MODE  1
#define TIM_COUNTER_FALLING_MODE 2
#define TIM_COUNTER_ANY_MODE     3
#define TIM_CTCR_MODE_LSB   0
#define TIM_CTCR_MODE_MSK   0x3
#define TIM_CTCR_INPUT_LSB  2
#define TIM_CTCR_INPUT_MSK  0xC
/*@}*/


#if defined(LPC_TIM0_BASE)
#define TIM0IR    (LPC_TIM0_BASE + TIM_IR_OFF)
#define TIM0TCR   (LPC_TIM0_BASE + TIM_TCR_OFF)
#define TIM0TC    (LPC_TIM0_BASE + TIM_TC_OFF)
#define TIM0PR    (LPC_TIM0_BASE + TIM_PR_OFF)
#define TIM0PC    (LPC_TIM0_BASE + TIM_PC_OFF)
#define TIM0MCR   (LPC_TIM0_BASE + TIM_MCR_OFF)
#define TIM0MR(x) (LPC_TIM0_BASE + TIM_MR_OFF(x))
#define TIM0CCR   (LPC_TIM0_BASE + TIM_CCR_OFF)
#define TIM0CR(x) (LPC_TIM0_BASE + TIM_CR_OFF(x))
#define TIM0EMR   (LPC_TIM0_BASE + TIM_EMR_OFF)
#define TIM0CTCR  (LPC_TIM0_BASE + TIM_CTCR_OFF)
#endif

#if defined(LPC_TIM1_BASE)
#define TIM1IR    (LPC_TIM1_BASE + TIM_IR_OFF)
#define TIM1TCR   (LPC_TIM1_BASE + TIM_TCR_OFF)
#define TIM1TC    (LPC_TIM1_BASE + TIM_TC_OFF)
#define TIM1PR    (LPC_TIM1_BASE + TIM_PR_OFF)
#define TIM1PC    (LPC_TIM1_BASE + TIM_PC_OFF)
#define TIM1MCR   (LPC_TIM1_BASE + TIM_MCR_OFF)
#define TIM1MR(x) (LPC_TIM1_BASE + TIM_MR_OFF(x))
#define TIM1CCR   (LPC_TIM1_BASE + TIM_CCR_OFF)
#define TIM1CR(x) (LPC_TIM1_BASE + TIM_CR_OFF(x))
#define TIM1EMR   (LPC_TIM1_BASE + TIM_EMR_OFF)
#define TIM1CTCR  (LPC_TIM1_BASE + TIM_CTCR_OFF)
#endif

#if defined(LPC_TIM2_BASE)
#define TIM2IR    (LPC_TIM2_BASE + TIM_IR_OFF)
#define TIM2TCR   (LPC_TIM2_BASE + TIM_TCR_OFF)
#define TIM2TC    (LPC_TIM2_BASE + TIM_TC_OFF)
#define TIM2PR    (LPC_TIM2_BASE + TIM_PR_OFF)
#define TIM2PC    (LPC_TIM2_BASE + TIM_PC_OFF)
#define TIM2MCR   (LPC_TIM2_BASE + TIM_MCR_OFF)
#define TIM2MR(x) (LPC_TIM2_BASE + TIM_MR_OFF(x))
#define TIM2CCR   (LPC_TIM2_BASE + TIM_CCR_OFF)
#define TIM2CR(x) (LPC_TIM2_BASE + TIM_CR_OFF(x))
#define TIM2EMR   (LPC_TIM2_BASE + TIM_EMR_OFF)
#define TIM2CTCR  (LPC_TIM2_BASE + TIM_CTCR_OFF)
#endif

#if defined(LPC_TIM3_BASE)
#define TIM3IR    (LPC_TIM3_BASE + TIM_IR_OFF)
#define TIM3TCR   (LPC_TIM3_BASE + TIM_TCR_OFF)
#define TIM3TC    (LPC_TIM3_BASE + TIM_TC_OFF)
#define TIM3PR    (LPC_TIM3_BASE + TIM_PR_OFF)
#define TIM3PC    (LPC_TIM3_BASE + TIM_PC_OFF)
#define TIM3MCR   (LPC_TIM3_BASE + TIM_MCR_OFF)
#define TIM3MR(x) (LPC_TIM3_BASE + TIM_MR_OFF(x))
#define TIM3CCR   (LPC_TIM3_BASE + TIM_CCR_OFF)
#define TIM3CR(x) (LPC_TIM3_BASE + TIM_CR_OFF(x))
#define TIM3EMR   (LPC_TIM3_BASE + TIM_EMR_OFF)
#define TIM3CTCR  (LPC_TIM3_BASE + TIM_CTCR_OFF)
#endif

/*@}*/
#endif
