#ifndef _DEV_ARCH_CM3_STM_HWTIMER_STM32_H_
#define _DEV_ARCH_CM3_STM_HWTIMER_STM32_H_
/*
 * Copyright (C) 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: hwtimer_stm32.c 3731 2012-01-12 18:23:31Z olereinhardt $
 * \endverbatim
 */

#include <stddef.h>
#include <string.h>
#include <cfg/arch.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <dev/irqreg.h>

#if defined(TIM1_BASE)
#define NUTTIMER1   TIM1_BASE
#endif
#if defined(TIM2_BASE)
#define NUTTIMER2   TIM2_BASE
#endif
#if defined(TIM3_BASE)
#define NUTTIMER3   TIM3_BASE
#endif
#if defined(TIM4_BASE)
#define NUTTIMER4   TIM4_BASE
#endif
#if defined(TIM5_BASE)
#define NUTTIMER5   TIM5_BASE
#endif
#if defined(TIM6_BASE)
#define NUTTIMER6   TIM6_BASE
#endif
#if defined(TIM7_BASE)
#define NUTTIMER7   TIM7_BASE
#endif
#if defined(TIM8_BASE)
#define NUTTIMER8   TIM8_BASE
#endif
#if defined(TIM9_BASE)
#define NUTTIMER9   TIM9_BASE
#endif
#if defined(TIM10_BASE)
#define NUTTIMER10  TIM10_BASE
#endif
#if defined(TIM11_BASE)
#define NUTTIMER11  TIM11_BASE
#endif
#if defined(TIM12_BASE)
#define NUTTIMER12  TIM12_BASE
#endif
#if defined(TIM13_BASE)
#define NUTTIMER13  TIM13_BASE
#endif
#if defined(TIM14_BASE)
#define NUTTIMER14  TIM14_BASE
#endif
#if defined(TIM15_BASE)
#define NUTTIMER15  TIM15_BASE
#endif
#if defined(TIM16_BASE)
#define NUTTIMER16  TIM16_BASE
#endif
#if defined(TIM17_BASE)
#define NUTTIMER17  TIM17_BASE
#endif

#define TIM_Control1( timer )              CM3REG(timer, TIM_TypeDef, CR1 )
#define TIM_Control2( timer )              CM3REG(timer, TIM_TypeDef, CR2 )
#define TIM_AutoReloadValue( timer )       CM3REG(timer, TIM_TypeDef, ARR )
#define TIM_SlaveModeControl( timer )      CM3REG(timer, TIM_TypeDef, SMCR )
#define TIM_DMA_IRQ( timer )               CM3REG(timer, TIM_TypeDef, DIER )
#define TIM_EventGeneration( timer )       CM3REG(timer, TIM_TypeDef, EGR )
#define TIM_CCMode1( timer )               CM3REG(timer, TIM_TypeDef, CCMR1 )
#define TIM_CCMode2( timer )               CM3REG(timer, TIM_TypeDef, CCMR2 )
#define TIM_Prescaler( timer)              CM3REG(timer, TIM_TypeDef, PSC )
#define TIM_Compare1( timer)               CM3REG(timer, TIM_TypeDef, CCR1)
#define TIM_Compare2( timer)               CM3REG(timer, TIM_TypeDef, CCR2)
#define TIM_Compare3( timer)               CM3REG(timer, TIM_TypeDef, CCR3)
#define TIM_Compare4( timer)               CM3REG(timer, TIM_TypeDef, CCR4)
#define TIM_Counter( timer)                CM3REG(timer, TIM_TypeDef, CNT )
#define TIM_Status( timer)                 CM3REG(timer, TIM_TypeDef, SR )
#define TIM_CCEnable( timer)               CM3REG(timer, TIM_TypeDef, CCER )
#define TIM_DMAControl( timer)             CM3REG(timer, TIM_TypeDef, DCR )
#define TIM_DMA_Address( timer)            CM3REG(timer, TIM_TypeDef, DMAR )
#define TIM_Option( timer)                 CM3REG(timer, TIM_TypeDef, OR )
#define TIM_Break_Deadtime( timer)         CM3REG(timer, TIM_TypeDef, BDTR )
#define TIM_IRQEnable( timer )             CM3BBSET(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_UIE  ))
#define TIM_IRQEnable( timer )             CM3BBSET(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_UIE  ))
#define TIM_IRQDisable( timer )            CM3BBCLR(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_UIE  ))
#define TIM_C1IRQEnable( timer )           CM3BBSET(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC1IE))
#define TIM_C1IRQDisable( timer )          CM3BBCLR(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC1IE))
#define TIM_C2IRQEnable( timer )           CM3BBSET(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC2IE))
#define TIM_C2IRQDisable( timer )          CM3BBCLR(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC2IE))
#define TIM_C3IRQEnable( timer )           CM3BBSET(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC3IE))
#define TIM_C3IRQDisable( timer )          CM3BBCLR(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC3IE))
#define TIM_C4IRQEnable( timer )           CM3BBSET(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC4IE))
#define TIM_C4IRQDisable( timer )          CM3BBCLR(timer, TIM_TypeDef, DIER, _BI16(TIM_DIER_CC4IE))
#define TIM_StartTimer( timer)             CM3BBSET(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_CEN   ))
#define TIM_StopTimer( timer )             CM3BBCLR(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_CEN   ))
#define TIM_OnePulse( timer)               CM3BBSET(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_OPM   ))
#define TIM_ContPulse( timer)              CM3BBCLR(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_OPM   ))
#define TIM_AutoReload( timer )            CM3BBSET(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_ARPE  ))
#define TIM_NoReload( timer )              CM3BBCLR(timer, TIM_TypeDef, CR1 , _BI16(TIM_CR1_ARPE  ))
#define TIM_Update( timer )                CM3BBSET(timer, TIM_TypeDef, EGR , _BI16(TIM_EGR_UG    ))
#define TIM_ClearInterruptFlag( timer)     CM3BBCLR(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_UIF    ))
#define TIM_C1ClearInterruptFlag( timer )  CM3BBCLR(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC1IF  ))
#define TIM_C2ClearInterruptFlag( timer )  CM3BBCLR(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC2IF  ))
#define TIM_C3ClearInterruptFlag( timer )  CM3BBCLR(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC3IF  ))
#define TIM_C4ClearInterruptFlag( timer )  CM3BBCLR(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC4IF  ))
#define TIM_C1InterruptFlag( timer )       CM3BBGET(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC1IF  ))
#define TIM_C2InterruptFlag( timer )       CM3BBGET(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC2IF  ))
#define TIM_C3InterruptFlag( timer )       CM3BBGET(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC3IF  ))
#define TIM_C4InterruptFlag( timer )       CM3BBGET(timer, TIM_TypeDef, SR  , _BI16(TIM_SR_CC4IF  ))

#endif
