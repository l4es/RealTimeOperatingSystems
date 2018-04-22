/*!
 * Copyright (C) 2014 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * Use a timer to count (prescaled) pulses from different sources.
 *
 * Rollover must be handled by user!
 * $Id: stm32_timer.c 6568 2016-12-12 10:16:18Z u_bonnes $
 */
#include <stdint.h>
#include <cfg/arch.h>
#include <arch/cm3/stm/stm32_timer.h>

/*!
 * \brief  Configure a Timer
 * Set Prescaler and Autoreload Value before starting the timer
 * Handle Rollover in Userspace!
 *
 * \param  tim           Timer to set
 *
 * \param  clk_mode      Clock source
 * \param  trg_sel       Trigger source
 * \param  slave_mode    Behaviour on Trigger
 * \param  master_mode1  Trigger output to other timers
 *
 */

void Stm32TimerConfig(
    TIM_TypeDef       *tim,
    TIM_CLK_MODE      clk_mode,
    TIM_TRG_SELECTION trg_sel,
    TIM_SLAVE_MODE    slave_mode,
    TIM_MASTER_MODE1  master_mode1)
{
    uint32_t smcr;
    int i;

    if (master_mode1 > TIM_MASTER_MODE1_NONE) {
        uint32_t cr2;
        cr2 = tim->CR2;
        cr2 &= ~TIM_CR2_MMS;
        cr2 |= (master_mode1 * TIM_CR2_MMS_0) & TIM_CR2_MMS;
        tim->CR2 = cr2;
    }

    i = 0;
    if (trg_sel > TIM_TRG_SELECTION_ETR8) {
        trg_sel -= 4;
        smcr = TIM_SMCR_ETP;
    }
    else
        smcr = 0;
    switch(trg_sel) {
    case TIM_TRG_SELECTION_NONE: break;
    case TIM_TRG_SELECTION_ETR8: i++;
    case TIM_TRG_SELECTION_ETR4: i++;
    case TIM_TRG_SELECTION_ETR2: i++;
    default:
        smcr |= i * TIM_SMCR_ETPS_0;
        smcr |= (trg_sel * TIM_SMCR_TS_0) & TIM_SMCR_TS;
    }

    switch(clk_mode) {
    case TIM_CLK_MODE_ETR:
        smcr = smcr | TIM_SMCR_ECE;
        /* Fall through to treat like internal clock mode */
    case TIM_CLK_MODE_CKINT:
        switch(slave_mode){
#if defined(TIM_SMCR_SMS_3)
        case TIM_SLAVE_MODE_TRG_RESET_CNT:
            smcr = smcr | TIM_SMCR_SMS_3;
            break;
#endif
        default: smcr = smcr | ((slave_mode * TIM_SMCR_SMS_0) & TIM_SMCR_SMS);
        }
        break;
    case TIM_CLK_MODE_TI1:
        smcr = smcr | 7 * TIM_SMCR_SMS_0 | 5 * TIM_SMCR_TS_0;
        break;
    case TIM_CLK_MODE_TI2:
        smcr = smcr | 7 * TIM_SMCR_SMS_0 | 6 * TIM_SMCR_TS_0;
        break;
    }
    tim->SMCR = smcr;
#if defined(TIM_BDTR_MOE)
    tim->BDTR = TIM_BDTR_MOE;
#endif
    if(tim->ARR == 0)
        tim->ARR = -1;
}

/*!
 * \brief  Configure a Timer Channel
 *
 * \param  tim           Timer to set
 *
 * \param  channel       Channel to set values, negative values for CHxN
 * \param  filter        Input capture filter
 * \param  fin           Input capture function
 * \param  fout          Output compare function
 * \param  polarity      Function polarity
 *
 * \return -1 if illegal/ inconsistent parameters or 0 on success
 */
int Stm32TimerChannelConfig(
    TIM_TypeDef    *tim,
    int8_t          channel,
    uint8_t         filter,
    TIM_CC_FIN      fin,
    TIM_CC_FOUT     fout,
    TIM_CC_POLARITY polarity)
{
    uint32_t tmp, ccer;
    int ch, channel_index;

    if (!channel) {
        return -1;
    } else if (channel < 0 ) {
        ch = -channel - 1;
    } else {
        ch = channel - 1;
    }
    if ((ch > 3) && (fin != TIM_CC_OUTPUT)) {
        /* Only 4 input channels even on 6 channel timers */
        return - 1;
    }
    if ((channel < -3) ||          /* No CH(4|5|6)N */
        (ch > 5 ) ||               /* Only 6 channels max. */
        (filter > 0xf) ||
        ((fin != TIM_CC_OUTPUT) && (fout != TIM_CC_FROZEN))) {
        return -1;
    }
    tmp = 0;
    if (fout != TIM_CC_FROZEN) {
        if (fout < TIM_CC_FROZEN_DIRECT) {
            /* Enable preload PWM value.
             * New value gets active on timer update.*/
            tmp |= TIM_CCMR1_OC1PE;
        } else {
            fout = fout - TIM_CC_FROZEN_DIRECT;
        }
        /* Set output function. */
        tmp |= (fout  * TIM_CCMR1_OC1M_0);
    } else {
        /* Set Input Capture Filter and function*/
        tmp = (filter * TIM_CCMR1_IC1F_0) | (fin);
    }
    switch (ch) {
    case 0:
        tim->CCMR1 &= 0x100ff00;
        tim->CCMR1 |= tmp;
        break;
    case 1:
        tim->CCMR1 &= 0x0100ff;
        tim->CCMR1 |= tmp << 8;
        break;
    case 2:
        tim->CCMR2 &= 0x100ff00;
        tim->CCMR2 |= tmp;
        break;
    case 3:
        tim->CCMR2 &= 0x0100ff;
        tim->CCMR2 |= tmp << 8;
        break;
#if defined(TIM_CCMR3_OC5M)
    case 4:
        tim->CCMR3 &= 0x100ff00;
        tim->CCMR3 |= tmp;
        break;
    case 5:
        tim->CCMR3 &= 0x0100ff;
        tim->CCMR3 |= tmp << 8;
        break;
#endif
    }
    /* Select polarity of input capture/ output compare and enable channel*/
    /* Clear CC1NP/CC1P */
    channel_index =  ch * 4;
    ccer = tim->CCER;
    ccer &= ~( 0xf << channel_index);
    tmp = TIM_CCER_CC1E + polarity * TIM_CCER_CC1P;
    if (channel < 0) {
        tmp = tmp << 2;
    }
    tmp <<= channel_index;
    ccer |= tmp;
    tim->CCER = ccer;
    return 0;
}

/*!
 * \brief  Get Timer clock
 *
 * \param  Timer to query
 *
 * \return Timer Clock in Hz
 */
uint32_t Stm32TimerGetClock(TIM_TypeDef *tim)
{
    int ret;
#if defined(APB2PERIPH_BASE)
    if ((uint32_t) tim > APB2PERIPH_BASE)
        ret = NutClockGet(HWCLK_APB2_TIMER);
#else
    if (0) {
    }
#endif
    else
        ret = NutClockGet(HWCLK_APB1_TIMER);
    return ret;
}
