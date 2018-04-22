/*!
 * Copyright (C) 2013, 2015 Uwe Bonnes
 *                                   (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/dev/stm/stm32_qenc32.c
 * \brief STM 32 bit quadrature encoder using 32-bit timer.
 *
 * Implement quadrature encoders with an STM32 timera. The
 * encoder runs on it's own and uses no interrupts.
 * Attention : Configured values for the used pins are not checked here!
 *
 * Typical usage:
 * \code
 *  #include <dev/qenc.h>
 *  QencInit(&Stm32Qenc0);
 *  ...
 *  value = QencGet(&Stm32Qenc0);
 *  ...
 * \endcode
 *
 * If hardware is a 16-bit timer, call QencGet() before overflow happens.
 */

#include <cfg/arch.h>

#include <sys/atom.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <cfg/qenc.h>
#include <dev/gpio.h>
#include <dev/qenc.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_timer.h>

/*!
 * \brief Constant local data for 32-bit quadrature encoder
 *
 * Used for values only needed during initialization!
 * Flash access will need flash access wait states.
 *
 * ToDo: Order in a way that few padding happens!
 *
 */
typedef struct _STM32_QENC_HW STM32_QENC_HW;
struct _STM32_QENC_HW {
    /*! \brief Used timer. */
    TIM_TypeDef *const qenc_timer;
#if defined(MCU_STM32F1)
    /*! \brief F1 pin remapping. Set by user if needed.*/
    volatile uint32_t *const remap_reg;
    /*! \brief Remap mask on F1.  Set by user if needed.*/
    const uint32_t remap_mask;
    /*! \brief Remap value on F1. Set by user if needed.*/
    const uint32_t remap_value;
#endif
    /*! \brief Device Clock enable register */
    volatile uint32_t *const enable_reg;
    /*! \brief Device Clock enable mask */
    const uint32_t enable_mask;
    /*! \brief Device Reset register */
    volatile uint32_t *const reset_reg;
    /*! \brief QENC I Pin. */
    const nutgpio_t qenc_i_pin;
    /*! \brief QENC I Pinmux. */
    const uint8_t qenc_i_pin_af;
    /*! \brief QENC Q Pin. */
    const nutgpio_t qenc_q_pin;
    /*! \brief QENC Q Pinmux. */
    const uint8_t qenc_q_pin_af;
    /*! \brief QENC Index Pin. */
    const nutgpio_t qenc_index_pin;
    /*! \brief QENC IndexPinmux. */
    const uint8_t qenc_index_pin_af;
    /*! \brief QENC Direction inversion. */
    TIM_CC_POLARITY qenc_invert;
    /*! \brief QENC timer width. */
    const uint8_t qenc_width;
};

/* User one structure for both 16/32 bit, loosing about 8 byte RAM */
typedef struct _STM32_QENC_INFO STM32_QENC_INFO;
struct _STM32_QENC_INFO {
    const STM32_QENC_HW *hw;
    volatile uint32_t   *counter;
    volatile int        value;
    volatile int16_t    last_value;
};

/*!
 * \brief Initialize the quadrature encoder
 *
 * No checking is done here!
 *
 * \param qenc_dev Quadra to inialize
 *
 * \return -1 on failure
 */

int Stm32QencInit(NUTQENC *qenc_dev)
{
    STM32_QENC_INFO     *info;
    const STM32_QENC_HW *hw;
    TIM_TypeDef         *timer;

    info = (STM32_QENC_INFO *)qenc_dev->qenc_info;
    hw   = info->hw;
    *hw->enable_reg |= hw->enable_mask;
    *hw->reset_reg  |= hw->enable_mask;
    *hw->reset_reg  &= ~hw->enable_mask;
    timer = hw->qenc_timer;
    Stm32TimerConfig(timer, TIM_CLK_MODE_CKINT, TIM_TRG_SELECTION_NONE,
                       TIM_SLAVE_MODE_ENCODER_BOTH, TIM_MASTER_MODE1_NONE);
    if (hw->qenc_invert) {
        timer->CCER = TIM_CCER_CC1P;
    }
    timer->ARR = 0xffffffff;
    if (hw->qenc_width == 32) {
        if (timer->ARR != 0xffffffff) {
            /* Not a 32-bit timer */
            return -1;
        }
    }
    timer->EGR = TIM_EGR_UG;

#if defined(MCU_STM32F1)
    uint32_t mapr;
    mapr =  *hw->remap_reg;
    mapr &= ~hw->remap_mask;
    mapr |= (hw->remap_value & hw->remap_mask);
    *hw->remap_reg = mapr;
#endif
    Stm32GpioConfigSet(hw->qenc_i_pin, GPIO_CFG_PERIPHAL, hw->qenc_i_pin_af);
    Stm32GpioConfigSet(hw->qenc_q_pin, GPIO_CFG_PERIPHAL, hw->qenc_q_pin_af);
    Stm32GpioConfigSet(hw->qenc_index_pin, GPIO_CFG_PERIPHAL, hw->qenc_index_pin_af);
    timer->CR1 |= TIM_CR1_CEN;
    return 0;
}

/*!
 * \brief Return quadrature encoder value
 *
 * \param  None.
 *
 * \return .
 */

int Stm32Qenc32Get(NUTQENC *qenc_dev)
{
    STM32_QENC_INFO *info;

    info = (STM32_QENC_INFO *)qenc_dev->qenc_info;
    return *(int *)info->counter;
}

int Stm32Qenc16Get(NUTQENC *qenc_dev)
{
    STM32_QENC_INFO *info;
    uint16_t actual_value;
    int16_t diff;

    info = (STM32_QENC_INFO *)qenc_dev->qenc_info;
    NutEnterCritical();
    actual_value =  *info->counter;
    diff = actual_value - info->last_value;
    info->last_value = actual_value;
    info->value += diff;
    NutExitCritical();
    return info->value;
}

/*!
 * \brief Set quadrature encoder value
 *
 * \param  Value to set.
 *
 * \return None.
 */
void Stm32QencSet(NUTQENC *qenc_dev, int value)
{
    STM32_QENC_INFO     *info;
    const STM32_QENC_HW *hw;
    TIM_TypeDef         *timer;

    info = (STM32_QENC_INFO *)qenc_dev->qenc_info;
    hw   = info->hw;
    timer = hw->qenc_timer;
    timer->CR1 &= ~TIM_CR1_CEN;
    if (hw->qenc_width == 32) {
        *info->counter = 0;
        timer->EGR |= TIM_EGR_UG;
    } else {
        info->value = 0;
        info->last_value = *info->counter;
    }
    timer->CR1 |= TIM_CR1_CEN;
}

/* Only compile code if needed defines are given*/
#if defined(STM32_QENC0_I_GPIO) && defined(STM32_QENC0_Q_GPIO) && \
    defined(STM32_QENC0_TIMER_ID)
#undef STM32TIMER_ID
#define STM32TIMER_ID STM32_QENC0_TIMER_ID
#include <arch/cm3/stm/stm32timertran.h>
static const STM32_QENC_HW Stm32Qenc0Hw = {
    .qenc_timer = (TIM_TypeDef *) STM32TIMER_BASE,
#if defined(MCU_STM32F1)
    .remap_reg       = &AFIO->STM32_REMAP_REG,
    .remap_mask      = STM32TIMER_REMAP_SHIFT,
    .remap_value     = TIM_ID2REMAP(STM32_PWM1_TIMER_ID),
#endif
    .enable_reg      = BASE2TIM_ENR(STM32TIMER_BASE),
    .enable_mask     = STM32TIMER_MASK,
    .reset_reg       = BASE2TIM_RSTR(STM32TIMER_BASE),
    .qenc_i_pin      = STM32_QENC0_I_GPIO,
    .qenc_i_pin_af   = STM32TIMER_AF(STM32_QENC0_I_GPIO),
    .qenc_q_pin      = STM32_QENC0_Q_GPIO,
    .qenc_q_pin_af   = STM32TIMER_AF(STM32_QENC0_Q_GPIO),
#if defined(STM32_QENC0_INDEX_GPIO)
    .qenc_index_pin  = STM32_QENC0_INDEX_GPIO,
#else
    .qenc_index_pin  = PIN_NONE,
#endif
    .qenc_invert     = STM32_QENC0_INVERT,
    .qenc_width      = STM32TIMER_WIDTH,
};
static STM32_QENC_INFO Stm32Qenc0Info = {
    .hw              = &Stm32Qenc0Hw,
    .counter         = &((TIM_TypeDef *) STM32TIMER_BASE)->CNT,
};

NUTQENC  Stm32Qenc0  = {
    .qenc_info       = (uintptr_t)&Stm32Qenc0Info,
    .QencInit        = Stm32QencInit,
#if (STM32TIMER_WIDTH == 32)
    .QencGet         = Stm32Qenc32Get,
#else
    .QencGet         = Stm32Qenc16Get,
#endif
    .QencSet         = Stm32QencSet,
};
#endif
