/*
 * Copyright (C) 2013-17 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/owibus-stm32tim.c
 * \brief Common implementation of One-Wire primitives with dual channel
 *        STM32 timer.
 *
 * Connection alternatives:
 * - Single pin, with open-drain output and (external) pull-up (default).
 *   Neighbouring channels 1/2 or 3/4 will be used.
 * - Dual pin, slew rate limited. See Appendix C in AN148 from Maxim.
 *   Default if TX pin is given. Channels related to the pins will be used.
 *   TX is push/pull.
 * - Dual pin with non-inverting TX pin if TX pin and
 *   STM32TIM_OWIx_TX_INVERT = DISABLE are given.
 *   TX is push/pull.
 *
 * \verbatim
 * $Id: stm32_owitim.c 6596 2017-02-15 15:35:52Z u_bonnes $
 * \endverbatim
 */
#include <cfg/arch.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/owibus.h>
#include <dev/gpio.h>
#include <arch/cm3.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_timer.h>
#include <cfg/owi.h>
#include <arch/cm3/stm/stm32_owitimer.h>

/*!
 * \brief Load reloaded values and start timer
 *
 * Before timer start, TX ist forced get active. Timer match
 * sets output inactive again. Interruption will prolong active period.
 * So guard by global interrupt disable.
 *
 * \param info STM32_OWIBUS_TIMER_INFO
 *
 * \return None
 */
static inline void OwiTimerStart(STM32_OWIBUS_TIMER_INFO *info)
{
    uint16_t ccmr_value, ccmr_force;

#if defined (MCU_CM_NO_BITBAND)
    *info->timer_egr |= TIM_EGR_UG;
#else
    *info->timer_egr = 1;
#endif
    NutEnterCritical();
    ccmr_value = *info->ccmr;
    ccmr_force = ccmr_value & info->ccmr_mask;
    ccmr_force = ccmr_force | info->ccmr_force;
    *info->ccmr = ccmr_force;
    *info->ccmr = ccmr_value;
#if defined (MCU_CM_NO_BITBAND)
    *info->timer_cr1 |= TIM_CR1_CEN;
#else
    *info->timer_cr1 = 1;
#endif
    NutExitCritical();
}

/*!
 * \brief Owi Timer interrupt
 *
 */
static void Stm32TimOwiInterrupt(void *arg)
{
    STM32_OWIBUS_TIMER_INFO *info;
    uint16_t flags;

    info = (STM32_OWIBUS_TIMER_INFO *)arg;
    flags = info->timer->SR;
    /* Reset all flags we have catched */
    info->timer->SR = ~flags;
    if (flags & TIM_SR_UIF) {
        if (info->owi_rxp) {
            unsigned int value;

            /* read value before starting next cycle*/
            value = *info->ccr_capture;
            if (info->owi_rx_len > 1) {
                *info->ccr_pulse = info->sample_value;
                OwiTimerStart(info);
            }
            if (info->owi_rx_len) {
                int res;
                res = 1 << info->owi_index;
                if (value < info->owi_compare) {
                    *info->owi_rxp |= res;
                } else {
                    *info->owi_rxp &= ~res;
                }
                info->owi_rx_len--;
                if ( info->owi_index >= 7) {
                    info->owi_index = 0;
                    info->owi_rxp++;
                } else {
                    info->owi_index++;
                }
            }
            if (!info->owi_rx_len) {
               NutEventPostFromIrq(&info->mutex);
            }
        }
        if (info->owi_txp) {
            if (info->owi_tx_len) {
                int data;

                data = *info->owi_txp & (1 << info->owi_index);
                if (data) {
                    *info->ccr_pulse     = info->owi_tim_values[WRITE_1];
                } else {
                    *info->ccr_pulse     = info->owi_tim_values[WRITE_0];
                }
                OwiTimerStart(info);
                  if (info->owi_index >= 7) {
                    info->owi_index = 0;
                    info->owi_txp++;
                 } else {
                    info->owi_index++;
                 }
                info->owi_tx_len --;
            } else {
                NutEventPostFromIrq(&info->mutex);
            }
        }
    }
}

/*!
 * \brief Perform One-Wire transaction.
 *
 * \param bus     Specifies the One-Wire bus.
 * \param command Either OWI_CMD_RESET or OWI_CMD_RWBIT.
 * \param value   The value to send.
 *
 * \return The value read on success, a negative value otherwise.
 */
static int Stm32TimOwiTransaction(NUTOWIBUS *bus)
{
    STM32_OWIBUS_TIMER_INFO *info;
    int res;

    info = (STM32_OWIBUS_TIMER_INFO *)bus->owibus_info;
    info->timer->SR  = 0;
    *info->ccr_pulse = info->sample_value;
    /* Transfer Values to shadow register*/
    OwiTimerStart(info);
     if (NutEventWait(&info->mutex, 10)) {
        res =  OWI_DATA_ERROR;
     } else {
        res = OWI_SUCCESS;
     }
    info->owi_txp = 0;
    info->owi_rxp = 0;
    return res;
}

/*!
 * \brief Reset the One-Wire bus and check if device(s) present.
 *
 *  With a connected device, we seldom see a single rising edge after
 *  the sample point. Mostly there is a first rising edge short after
 *  OWI line has been released by the master. The capture register
 *  only contains the later rising edge. So evaluating CCR after the
 *  update event will always give us the right edge. No need to evaluate
 *  the overcapture flag.
 *
 * \param bus Specifies the One-Wire bus.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int Stm32TimOwiTouchReset(NUTOWIBUS *bus)
{
    STM32_OWIBUS_TIMER_INFO *info;
    uint8_t data;
    int res;

    info = (STM32_OWIBUS_TIMER_INFO *)bus->owibus_info;
    info->owi_index      = 0;
    info->owi_rx_len     = 1;
    info->owi_rxp        = &data;
    info->timer->ARR     = info->owi_tim_values[RESET_CYCLE];
    info->sample_value   = info->owi_tim_values[RESET_ACTIVE];
    info->owi_compare    = info->owi_tim_values[RESET_SAMPLE];
    res = Stm32TimOwiTransaction(bus);
    if ((res) || ((data & 1) == 1)) {
        return OWI_PRESENCE_ERR;
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Write a block of data bits to the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits to send.
 * \param len  Number of bits to send.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int Stm32TimOwiWriteBlock(
    NUTOWIBUS *bus, const uint8_t *data, uint_fast8_t len)
{
    STM32_OWIBUS_TIMER_INFO *info;
    int res;

    if (len == 0) {
        return OWI_SUCCESS;
    }
    info = (STM32_OWIBUS_TIMER_INFO *)bus->owibus_info;
    info->owi_tx_len     = len -1;
    info->owi_txp        = data;
    info->timer->ARR = info->owi_tim_values[CYCLE];
    if (*data & 1) {
        info->sample_value = info->owi_tim_values[WRITE_1];
    } else {
        info->sample_value = info->owi_tim_values[WRITE_0];
    }
    info->owi_index      = 1;
    res = Stm32TimOwiTransaction(bus);
    if (res) {
        return OWI_HW_ERROR;
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Read a block of data bits from the One-Wire bus.
 *
 * \param bus  Specifies the One-Wire bus.
 * \param data Data bits received.
 * \param len  Number of bits to read.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int Stm32TimOwiReadBlock(
    NUTOWIBUS *bus, uint8_t *data, uint_fast8_t len)
{
    STM32_OWIBUS_TIMER_INFO *info;
    int res;

    info = (STM32_OWIBUS_TIMER_INFO *)bus->owibus_info;
    info->owi_rx_len     = len;
    info->owi_rxp        = data;
    *info->owi_rxp       = 0;               /* Reset whole byte*/
    info->timer->ARR = info->owi_tim_values[CYCLE];
    info->sample_value   = info->owi_tim_values[WRITE_1];
    info->owi_index      = 0;
    info->owi_compare    = info->owi_tim_values[SAMPLE];
    res = Stm32TimOwiTransaction(bus);
    if (res) {
        return OWI_HW_ERROR;
    }
    return OWI_SUCCESS;
}

/*!
 * \brief Initialize the driver.
 *
 * \param bus  Specifies the One-Wire bus.
 *
 * \return OWI_SUCCESS on success, a negative value otherwise.
 */
int Stm32TimOwiSetup(NUTOWIBUS *bus)
{
    STM32_OWIBUS_TIMER_INFO *info;
    const STM32_OWIBUS_TIMER_HW *hw;
    uint32_t freq;
    int res;
    int prescaler;
    uint16_t *p;
    const uint16_t *q;

    info = (STM32_OWIBUS_TIMER_INFO *)bus->owibus_info;
    hw = info->owi_hw;
    *hw->enable_reg |= hw->enable_mask;
    *hw->reset_reg  |= hw->enable_mask;
    *hw->reset_reg  &= ~hw->enable_mask;

    Stm32TimerConfig(info->timer, TIM_CLK_MODE_CKINT, TIM_TRG_SELECTION_NONE,
                       TIM_SLAVE_MODE_NONE, TIM_MASTER_MODE1_NONE);
    /* Capture rising edg on receive channel*/
    res = Stm32TimerChannelConfig(
        info->timer, hw->owi_rx_channel, 0,
        (hw->owi_tx_pin != PIN_NONE)? TIM_CC_DIRECT: TIM_CC_EXCHANGE,
        TIM_CC_FROZEN, TIM_CC_POL_TRUE);
    /* On match set inactive on transmit channel */
    res |= Stm32TimerChannelConfig(
        info->timer, hw->owi_tx_channel, 0,
        TIM_CC_OUTPUT, TIM_CC_INACTIVE_ON_MATCH, hw->owi_tx_invert);
     if (res) {
        return -1;
    }
    info->owi_txp = 0;
    info->owi_rxp = 0;
    info->timer->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_OPM;
    /* Store registers for capture/compare access
     * And set Output compare 1 preload enable
     */
    freq =  Stm32ClockGet(hw->clk_idx);
    /* We need as longest a 1 ms periode.
       We have 16 bit timers available.
       So at 2^16 * 1kHz ~= 64 MHz can be handled without prescaler */
    prescaler = 1;
    while (freq / prescaler >= ((1 << 16) * 1000)) {
        prescaler ++;
    }
    freq = freq / prescaler;
    if (prescaler > 1) {
        prescaler --;
        info->timer->PSC = prescaler;
    }
    /* Calculate values.  Care not to overflow*/
    freq = freq / 32;
    p = info->owi_tim_values;
    q = owi_tim_times_us;
    do {
        *p++ = ((*q++  * freq)/(1000000 /32)) -1;
    }
    while (q < &owi_tim_times_us[OWI_TIM_SIZE]);
    /* Set the counter to one before overflow*/
    info->timer->DIER = TIM_DIER_UIE;
    NutRegisterIrqHandler( hw->owi_irq, &Stm32TimOwiInterrupt, info);
    NutIrqEnable(hw->owi_irq);
#if defined(MCU_STM32F1)
    uint32_t mapr;
    mapr =  *hw->remap_reg;
    mapr &= ~hw->remap_mask;
    mapr |= (hw->remap_value & hw->remap_mask);
    *hw->remap_reg = mapr;
#endif
    if (hw->owi_tx_pin != PIN_NONE) {
        uint32_t reg32u;
        reg32u  = GPIO_CFG_PERIPHAL | GPIO_CFG_PULLUP ;
        Stm32GpioConfigSet(hw->owi_pin, reg32u, hw->owi_pin_af);
        reg32u  = GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT ;
        if (hw->owi_tx_invert == DISABLE) {
            reg32u |=  GPIO_CFG_INIT_LOW;
        } else {
            reg32u |=  GPIO_CFG_INIT_HIGH;
        }
        Stm32GpioConfigSet(hw->owi_tx_pin, reg32u, hw->owi_tx_pin_af);
    } else {
        uint32_t reg32u;
        reg32u  = GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT | GPIO_CFG_PULLUP ;
        reg32u |= GPIO_CFG_MULTIDRIVE  | GPIO_CFG_INIT_HIGH;
        Stm32GpioConfigSet(hw->owi_pin, reg32u, hw->owi_pin_af);
    }
    return OWI_SUCCESS;
}

/* Do not test STM32TIM_OWI0_TX_GPIO against e.g. PIN_NONE in the
 * preprocessor, as enums are involved that always resolve to 0!
 */

#if defined(STM32TIM_OWI0_GPIO) && defined(STM32TIM_OWI0_CHANNEL) \
    && defined(STM32TIM_OWI0_TIMER_ID)
# undef  STM32TIMER_ID
# define STM32TIMER_ID STM32TIM_OWI0_TIMER_ID
# include <arch/cm3/stm/stm32timertran.h>
# define CC_EXCHANGE_CHANNEL (((STM32TIM_OWI0_CHANNEL - 1) ^ 1) + 1)
static const STM32_OWIBUS_TIMER_HW Stm32Owi0TimHw = {
    .owi_base        = STM32TIMER_BASE,
    .owi_irq         = &STM32TIMER_SIG,
    .clk_idx         = BASE2TCLKSRC(STM32TIMER_BASE),
#if defined(MCU_STM32F1)
    .remap_reg       = &STM32TIMER_REMAP_REG,
    .remap_mask      = STM32TIMER_REMAP_MASK,
    .remap_value     = 0,
#endif
    .enable_reg      = BASE2TIM_ENR(STM32TIMER_BASE),
    .enable_mask     = STM32TIMER_MASK,
    .reset_reg       = BASE2TIM_RSTR(STM32TIMER_BASE),
    .owi_pin         = STM32TIM_OWI0_GPIO,
    .owi_pin_af      = STM32TIMER_AF(STM32TIM_OWI0_GPIO),
#if defined(STM32TIM_OWI0_TX_GPIO)
    .owi_rx_channel  = STM32TIM_OWI0_CHANNEL,
    .owi_tx_pin      = STM32TIM_OWI0_TX_GPIO,
    .owi_tx_pin_af   = STM32TIMER_AF(STM32TIM_OWI0_TX_GPIO),
    .owi_tx_channel  = STM32TIM_OWI0_TX_CHANNEL,
    .owi_tx_invert   = STM32TIM_OWI0_TX_TRUE,
#else
    .owi_rx_channel  = CC_EXCHANGE_CHANNEL,
    .owi_tx_channel  = STM32TIM_OWI0_CHANNEL,
    .owi_tx_invert   = ENABLE,
    .owi_tx_pin      = PIN_NONE,
#endif
};

static STM32_OWIBUS_TIMER_INFO Stm32Owi0TimInfo = {
    .owi_hw = &Stm32Owi0TimHw,
    .timer  = (TIM_TypeDef *) STM32TIMER_BASE,
#if defined (MCU_CM_NO_BITBAND)
    .timer_egr = &(((TIM_TypeDef *) STM32TIMER_BASE)->EGR),
    .timer_cr1 = &(((TIM_TypeDef *) STM32TIMER_BASE)->CR1),
#else
    .timer_egr = CM3BBADDR(STM32TIMER_BASE, TIM_TypeDef, EGR,
                           _BI32(TIM_EGR_UG)),
    .timer_cr1 = CM3BBADDR(STM32TIMER_BASE, TIM_TypeDef, CR1,
                           _BI32(TIM_CR1_CEN)),
#endif
#if defined(STM32TIM_OWI0_TX_GPIO)
    .ccr_capture = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI0_CHANNEL),
    .ccr_pulse   = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI0_TX_CHANNEL),
    .ccmr        = CCMR_REG (STM32TIMER_BASE, STM32TIM_OWI0_TX_CHANNEL),
    .ccmr_mask   = (STM32TIM_OWI0_TX_CHANNEL & 1) ? ~0x070 : ~0x7000,
    .ccmr_force  = (STM32TIM_OWI0_TX_CHANNEL & 1) ? 0x0050 : 0x5000,
#else
    .ccr_capture = CCR_REG  (STM32TIMER_BASE, CC_EXCHANGE_CHANNEL),
    .ccr_pulse   = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI0_CHANNEL),
    .ccmr        = CCMR_REG (STM32TIMER_BASE, STM32TIM_OWI0_CHANNEL),
    .ccmr_mask   = (STM32TIM_OWI0_CHANNEL & 1) ? ~0x0070 : ~0x7000,
    .ccmr_force  = (STM32TIM_OWI0_CHANNEL & 1) ? 0x0050 : 0x5000,
#endif
};

NUTOWIBUS owiBus0Stm32Tim = {
    .owibus_info = (uintptr_t)&Stm32Owi0TimInfo,
    .mode = OWI_MODE_NORMAL,
    .OwiSetup = Stm32TimOwiSetup,
    .OwiTouchReset = Stm32TimOwiTouchReset,
    .OwiReadBlock = Stm32TimOwiReadBlock,
    .OwiWriteBlock = Stm32TimOwiWriteBlock,
};
#undef CC_EXCHANGE_CHANNEL
#endif

#if defined(STM32TIM_OWI1_GPIO) && defined(STM32TIM_OWI1_CHANNEL) \
    && defined(STM32TIM_OWI1_TIMER_ID)
# undef  STM32TIMER_ID
# define STM32TIMER_ID STM32TIM_OWI1_TIMER_ID
# include <arch/cm3/stm/stm32timertran.h>
# define CC_EXCHANGE_CHANNEL (((STM32TIM_OWI1_CHANNEL - 1) ^ 1) + 1)
static const STM32_OWIBUS_TIMER_HW Stm32Owi1TimHw = {
    .owi_base        = STM32TIMER_BASE,
    .owi_irq         = &STM32TIMER_SIG,
    .clk_idx         = BASE2TCLKSRC(STM32TIMER_BASE),
#if defined(MCU_STM32F1)
    .remap_reg       = &STM32TIMER_REMAP_REG,
    .remap_mask      = STM32TIMER_REMAP_MASK,
    .remap_value     = 0,
#endif
    .enable_reg      = BASE2TIM_ENR(STM32TIMER_BASE),
    .enable_mask     = STM32TIMER_MASK,
    .reset_reg       = BASE2TIM_RSTR(STM32TIMER_BASE),
    .owi_pin         = STM32TIM_OWI1_GPIO,
    .owi_pin_af      = STM32TIMER_AF(STM32TIM_OWI1_GPIO),
#if defined(STM32TIM_OWI1_TX_GPIO)
    .owi_rx_channel  = STM32TIM_OWI1_CHANNEL,
    .owi_tx_pin      = STM32TIM_OWI1_TX_GPIO,
    .owi_tx_pin_af   = STM32TIMER_AF(STM32TIM_OWI1_TX_GPIO),
    .owi_tx_channel  = STM32TIM_OWI1_TX_CHANNEL,
    .owi_tx_invert   = STM32TIM_OWI1_TX_TRUE,
#else
    .owi_rx_channel  = CC_EXCHANGE_CHANNEL,
    .owi_tx_channel  = STM32TIM_OWI1_CHANNEL,
    .owi_tx_invert   = ENABLE,
    .owi_tx_pin      = PIN_NONE,
#endif
};

static STM32_OWIBUS_TIMER_INFO Stm32Owi1TimInfo = {
    .owi_hw = &Stm32Owi1TimHw,
    .timer  = (TIM_TypeDef *) STM32TIMER_BASE,
#if defined (MCU_CM_NO_BITBAND)
    .timer_egr = &(((TIM_TypeDef *) STM32TIMER_BASE)->EGR),
    .timer_cr1 = &(((TIM_TypeDef *) STM32TIMER_BASE)->CR1),
#else
    .timer_egr = CM3BBADDR(STM32TIMER_BASE, TIM_TypeDef, EGR,
                           _BI32(TIM_EGR_UG)),
    .timer_cr1 = CM3BBADDR(STM32TIMER_BASE, TIM_TypeDef, CR1,
                           _BI32(TIM_CR1_CEN)),
#endif
#if defined(STM32TIM_OWI1_TX_GPIO)
    .ccr_capture = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI1_CHANNEL),
    .ccr_pulse   = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI1_TX_CHANNEL),
    .ccmr        = CCMR_REG (STM32TIMER_BASE, STM32TIM_OWI1_TX_CHANNEL),
    .ccmr_mask   = (STM32TIM_OWI1_TX_CHANNEL & 1) ? ~0x0070 : ~0x7000,
    .ccmr_force  = (STM32TIM_OWI1_TX_CHANNEL & 1) ? 0x0050 : 0x5000,
#else
    .ccr_capture = CCR_REG  (STM32TIMER_BASE, CC_EXCHANGE_CHANNEL),
    .ccr_pulse   = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI1_CHANNEL),
    .ccmr        = CCMR_REG (STM32TIMER_BASE, STM32TIM_OWI1_CHANNEL),
    .ccmr_mask   = (STM32TIM_OWI1_CHANNEL & 1) ? ~0x0070 : ~0x7000,
    .ccmr_force  = (STM32TIM_OWI1_CHANNEL & 1) ? 0x0050 : 0x5000,
#endif
};

NUTOWIBUS owiBus1Stm32Tim = {
    .owibus_info = (uintptr_t)&Stm32Owi1TimInfo,
    .mode = OWI_MODE_NORMAL,
    .OwiSetup = Stm32TimOwiSetup,
    .OwiTouchReset = Stm32TimOwiTouchReset,
    .OwiReadBlock = Stm32TimOwiReadBlock,
    .OwiWriteBlock = Stm32TimOwiWriteBlock,
};
#undef CC_EXCHANGE_CHANNEL
#endif

#if defined(STM32TIM_OWI2_GPIO) && defined(STM32TIM_OWI2_CHANNEL) \
    && defined(STM32TIM_OWI2_TIMER_ID)
# undef  STM32TIMER_ID
# define STM32TIMER_ID STM32TIM_OWI2_TIMER_ID
# include <arch/cm3/stm/stm32timertran.h>
# define CC_EXCHANGE_CHANNEL (((STM32TIM_OWI2_CHANNEL - 1) ^ 1) + 1)
static const STM32_OWIBUS_TIMER_HW Stm32Owi2TimHw = {
    .owi_base        = STM32TIMER_BASE,
    .owi_irq         = &STM32TIMER_SIG,
    .clk_idx         = BASE2TCLKSRC(STM32TIMER_BASE),
#if defined(MCU_STM32F1)
    .remap_reg       = &STM32TIMER_REMAP_REG,
    .remap_mask      = STM32TIMER_REMAP_MASK,
    .remap_value     = 0,
#endif
    .enable_reg      = BASE2TIM_ENR(STM32TIMER_BASE),
    .enable_mask     = STM32TIMER_MASK,
    .reset_reg       = BASE2TIM_RSTR(STM32TIMER_BASE),
    .owi_pin         = STM32TIM_OWI2_GPIO,
    .owi_pin_af      = STM32TIMER_AF(STM32TIM_OWI2_GPIO),
#if defined(STM32TIM_OWI2_TX_GPIO)
    .owi_rx_channel  = STM32TIM_OWI2_CHANNEL,
    .owi_tx_pin      = STM32TIM_OWI2_TX_GPIO,
    .owi_tx_pin_af   = STM32TIMER_AF(STM32TIM_OWI2_TX_GPIO),
    .owi_tx_channel  = STM32TIM_OWI2_TX_CHANNEL,
    .owi_tx_invert   = STM32TIM_OWI2_TX_TRUE,
#else
    .owi_rx_channel  = CC_EXCHANGE_CHANNEL,
    .owi_tx_channel  = STM32TIM_OWI2_CHANNEL,
    .owi_tx_invert   = ENABLE,
    .owi_tx_pin      = PIN_NONE,
#endif
};

static STM32_OWIBUS_TIMER_INFO Stm32Owi2TimInfo = {
    .owi_hw = &Stm32Owi2TimHw,
    .timer  = (TIM_TypeDef *) STM32TIMER_BASE,
#if defined (MCU_CM_NO_BITBAND)
    .timer_egr = &(((TIM_TypeDef *) STM32TIMER_BASE)->EGR),
    .timer_cr1 = &(((TIM_TypeDef *) STM32TIMER_BASE)->CR1),
#else
    .timer_egr = CM3BBADDR(STM32TIMER_BASE, TIM_TypeDef, EGR,
                           _BI32(TIM_EGR_UG)),
    .timer_cr1 = CM3BBADDR(STM32TIMER_BASE, TIM_TypeDef, CR1,
                           _BI32(TIM_CR1_CEN)),
#endif
#if defined(STM32TIM_OWI2_TX_GPIO)
    .ccr_capture = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI2_CHANNEL),
    .ccr_pulse   = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI2_TX_CHANNEL),
    .ccmr        = CCMR_REG (STM32TIMER_BASE, STM32TIM_OWI2_TX_CHANNEL),
    .ccmr_mask   = (STM32TIM_OWI2_TX_CHANNEL & 1) ? ~0x0070 : ~0x7000,
    .ccmr_force  = (STM32TIM_OWI2_TX_CHANNEL & 1) ? 0x0050 : 0x5000,
#else
    .ccr_capture = CCR_REG  (STM32TIMER_BASE, CC_EXCHANGE_CHANNEL),
    .ccr_pulse   = CCR_REG  (STM32TIMER_BASE, STM32TIM_OWI2_CHANNEL),
    .ccmr        = CCMR_REG (STM32TIMER_BASE, STM32TIM_OWI2_CHANNEL),
    .ccmr_mask   = (STM32TIM_OWI2_CHANNEL & 1) ? ~0x0070 : ~0x7000,
    .ccmr_force  = (STM32TIM_OWI2_CHANNEL & 1) ? 0x0050 : 0x5000,
#endif
};

NUTOWIBUS owiBus2Stm32Tim = {
    .owibus_info = (uintptr_t)&Stm32Owi2TimInfo,
    .mode = OWI_MODE_NORMAL,
    .OwiSetup = Stm32TimOwiSetup,
    .OwiTouchReset = Stm32TimOwiTouchReset,
    .OwiReadBlock = Stm32TimOwiReadBlock,
    .OwiWriteBlock = Stm32TimOwiWriteBlock,
};
#undef CC_EXCHANGE_CHANNEL
#endif
