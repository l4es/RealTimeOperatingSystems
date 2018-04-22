/*!
 * Copyright (C) 2013-15 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * brief Implement PWM with timer with at least of compare channel.
 */
#include <stdint.h>

#include <cfg/arch.h>
#include <cfg/pwm.h>

#include <dev/pwm.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_timer.h>

/*!
 * \brief Constant local data PWM with timer
 *
 * Used for values only needed during initialization!
 * Flash access will need flash access wait states.
 *
 * ToDo: Order in a way that few padding happens!
 *
 */
typedef struct _STM32_PWM_HW STM32_PWM_HW;

struct _STM32_PWM_HW {
    /*! \brief Used timer. */
    TIM_TypeDef *const pwm_timer;
#if defined(MCU_STM32F1)
    /*! \brief F1 pin remapping. Set by user if needed.*/
    volatile uint32_t *const remap_reg;
    /*! \brief Remap mask on F1.  Set by user if needed.*/
    const uint32_t remap_mask;
    /*! \brief Remap value on F1.  Set by user if needed.*/
    const uint32_t remap_value;
#endif
#if defined(MCU_STM32F3)
    /*! \brief RCC_CFGR3 bit indicating if PLLCLKx2 is selected */
    const uint32_t pll_sw;
#endif
    /*! \brief Device Clock enable register */
    volatile uint32_t *const enable_reg;
    /*! \brief Device Clock enable mask */
    const uint32_t enable_mask;
    /*! \brief Device Reset register */
    volatile uint32_t *const reset_reg;
    /*! \brief PWM Compare register */
    volatile uint32_t *const ccr;
    /*! \brief PWM Pin. */
    nutgpio_t pwm_pin;
    /*! \brief OWI Pinmux. */
    uint8_t pwm_pin_af;
    /*! \brief PWM Channel, negative values for CHxN. */
    const int8_t pwm_channel;
};

/* -1 on error, 0 else*/
int Stm32PwmInit(
    NUTPWM *pwm_dev, unsigned int reload, unsigned int prescaler)
{
    STM32_PWM_HW *hw;
    TIM_TypeDef *timer;
    int res;

    hw = (STM32_PWM_HW *) pwm_dev->hw;
    *hw->enable_reg |= hw->enable_mask;
    *hw->reset_reg  |= hw->enable_mask;
    *hw->reset_reg  &= ~hw->enable_mask;
    timer = hw->pwm_timer;
    Stm32TimerConfig
        (timer, TIM_CLK_MODE_CKINT, TIM_TRG_SELECTION_NONE,
                       TIM_SLAVE_MODE_NONE, TIM_MASTER_MODE1_NONE);
    res = Stm32TimerChannelConfig(timer, hw->pwm_channel, 0,
        TIM_CC_OUTPUT, TIM_CC_ACTIVE_IF_LESS, TIM_CC_POL_TRUE);
    if (res) {
        return -1;
    }
    prescaler--;
    timer->PSC = prescaler;
    if (timer->PSC != prescaler) {
        return -1;
    }
    reload --;
    timer->ARR = reload;
    if (timer->ARR != reload) {
        return -1;
    }
#if defined(STM32_PWM_BTDR)
    timer->BDTR |= TIM_BDTR_MOE;
#endif
#if defined(MCU_STM32F1)
    uint32_t mapr;
    mapr =  *hw->remap_reg;
    mapr &= ~hw->remap_mask;
    mapr |= (hw->remap_value & hw->remap_mask);
    *hw->remap_reg = mapr;
#endif
    Stm32GpioConfigSet(hw->pwm_pin, GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT,
                       hw->pwm_pin_af);
    timer->CR1 |= TIM_CR1_CEN;
    return 0;
}

void Stm32PwmSet(NUTPWM *pwm_dev, unsigned int value)
{
    STM32_PWM_HW *hw;
    TIM_TypeDef *timer;

    hw = (STM32_PWM_HW *) pwm_dev->hw;
    timer = hw->pwm_timer;
    if (value > timer->ARR) {
        value = timer->ARR;
    }
    *hw->ccr = value;
}

unsigned int Stm32PwmGet(NUTPWM *pwm_dev)
{
    STM32_PWM_HW *hw;

    hw = (STM32_PWM_HW *) pwm_dev->hw;
    return *hw->ccr;
}

/* Return Timer Clock*/
uint32_t Stm32PwmGetClock(NUTPWM *pwm_dev)
{
    uint32_t clk;
#if defined(MCU_STM32F3)
    uint32_t cfgr3;
    STM32_PWM_HW *hw;

    hw = (STM32_PWM_HW *) pwm_dev->hw;
    cfgr3 = RCC->CFGR3;
    if (cfgr3 & hw->pll_sw) {
        clk = Stm32ClockGet(HWCLK_CPU);
        clk *= 2;
    } else {
        clk = BASE2TCLKSRC((uint32_t)hw);
    }
#else
    clk = BASE2TCLKSRC((uint32_t)pwm_dev->hw);
#endif
    return clk;
}

#if defined(STM32_PWM0) && defined(STM32_PWM0_TIMER_CHANNEL)   \
    && defined(STM32_PWM0_TIMER_ID)

# undef  STM32TIMER_ID
# define STM32TIMER_ID STM32_PWM0_TIMER_ID
# include <arch/cm3/stm/stm32timertran.h>
static const STM32_PWM_HW Stm32Pwm0Hw = {
    .pwm_timer       = (TIM_TypeDef *) STM32TIMER_BASE,
#if defined(MCU_STM32F1)
    .remap_reg       = &AFIO->STM32_REMAP_REG,
    .remap_mask      = STM32TIMER_REMAP_SHIFT,
    .remap_value     = TIM_ID2REMAP(STM32_PWM0_TIMER_ID),
#endif
#if defined(STM32TIMER_SW)
    .pll_sw          = STM32TIMER_SW,
#endif
    .enable_reg      = BASE2TIM_ENR(STM32TIMER_BASE),
    .enable_mask     = STM32TIMER_MASK,
    .reset_reg       = BASE2TIM_RSTR(STM32TIMER_BASE),
    .ccr             = CCR_REG(STM32TIMER_BASE, STM32_PWM0_TIMER_CHANNEL),
    .pwm_channel     = STM32_PWM0_TIMER_CHANNEL,
    .pwm_pin         = STM32_PWM0,
    .pwm_pin_af      = STM32TIMER_AF(STM32_PWM0),
};

NUTPWM Stm32Pwm0Tim = {
    .hw         = (uintptr_t)&Stm32Pwm0Hw,
    .PwmInit    = Stm32PwmInit,
    .PwmSet     = Stm32PwmSet,
    .PwmGet     = Stm32PwmGet,
    .PwmGetClock = Stm32PwmGetClock,
};
#endif

#if defined(STM32_PWM1) && defined(STM32_PWM1_TIMER_CHANNEL)   \
    && defined(STM32_PWM1_TIMER_ID)
# undef  STM32TIMER_ID
# define STM32TIMER_ID STM32_PWM1_TIMER_ID
# include <arch/cm3/stm/stm32timertran.h>
static const STM32_PWM_HW Stm32Pwm1Hw = {
    .pwm_timer       = (TIM_TypeDef *) STM32TIMER_BASE,
#if defined(MCU_STM32F1)
    .remap_reg       = &AFIO->STM32_REMAP_REG,
    .remap_mask      = STM32TIMER_REMAP_SHIFT,
    .remap_value     = TIM_ID2REMAP(STM32_PWM1_TIMER_ID),
#endif
#if defined(STM32TIMER_SW)
    .pll_sw          = STM32TIMER_SW,
#endif
    .enable_reg      = BASE2TIM_ENR(STM32TIMER_BASE),
    .enable_mask     = STM32TIMER_MASK,
    .reset_reg       = BASE2TIM_RSTR(STM32TIMER_BASE),
    .pwm_pin_af      = STM32TIMER_AF(STM32_PWM1),
    .ccr             = (volatile uint32_t*) (
        STM32TIMER_BASE + offsetof(TIM_TypeDef, CCR1) +
        ((sizeof(uint32_t)) * (STM32_PWM1_TIMER_CHANNEL - 1))),
    .pwm_channel     = STM32_PWM1_TIMER_CHANNEL,
    .pwm_pin         = STM32_PWM1,
};

NUTPWM Stm32Pwm1Tim = {
    .hw          = (uintptr_t)&Stm32Pwm1Hw,
    .PwmInit     = Stm32PwmInit,
    .PwmSet      = Stm32PwmSet,
    .PwmGet      = Stm32PwmGet,
    .PwmGetClock = Stm32PwmGetClock,
};
#endif
