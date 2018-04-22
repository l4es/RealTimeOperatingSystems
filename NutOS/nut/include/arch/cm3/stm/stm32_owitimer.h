#ifndef _OWI_STM32TIM_H_
#define _OWI_STM32TIM_H_
/*
 * Copyright (C) 2013, 2016 by Uwe Bonnes
 *                              (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/stm32_owitimer.c
 * \brief Header for the One-Wire API Implementation for STM32 Timers
 *
 * \verbatim
 * $Id: stm32_owitimer.h 6532 2016-09-07 10:21:31Z u_bonnes $
 * \endverbatim
 */
#include <stdint.h>
#include <sys/types.h>

typedef enum {
    RESET_CYCLE = 0,
    RESET_ACTIVE,
    RESET_SAMPLE,
    CYCLE,
    WRITE_1,
    WRITE_0,
    SAMPLE,
    OWI_TIM_SIZE
}OWI_TIM_TIMES;

/* Total duration, period low, sample point  straight from AN126*/
static const uint16_t owi_tim_times_us[OWI_TIM_SIZE]  = {
    960, /* H+I+J     */
    480, /* H         */
    550, /* H+I       */
    70,  /* A+E+F/C+D */
    6,   /* A         */
    60,  /* C         */
    15   /* A+E       */
};

/*!
 * \brief Constant local data of the Owibus with dual channel timer
 *
 * Used for values only needed during initialization!
 *
 * ToDo: Order in a way that few padding happens!
 * Flash access will need flash access wait states.
 *
 */
typedef struct _STM32_OWIBUS_TIMER_HW STM32_OWIBUS_TIMER_HW;

struct _STM32_OWIBUS_TIMER_HW {
    /*! \brief Timer to use. Set by configuration*/
    const uint32_t owi_base;
    /*! \brief Timer Interrupt*/
    IRQ_HANDLER *const owi_irq;
    /*! \brief Clock index of timer*/
    const clock_index_t clk_idx;
#if defined(MCU_STM32F1)
    /*! \brief F1 pin remapping. Set by user if needed.*/
    volatile uint32_t *const remap_reg;
    /*! \brief Remap mask on F1.  Set by user if needed.*/
    const uint32_t remap_mask;
    /*! \brief Remap value on F1.  Set by user if needed.*/
    const uint32_t remap_value;
#endif
    /*| \brief Device Clock enable register */
    volatile uint32_t *const enable_reg;
    /*| \brief Device Clock enable mask */
    const uint32_t enable_mask;
    /*| \brief Device Reset register */
    volatile uint32_t *const reset_reg;
    /*! \brief OWI Pin. */
    const nutgpio_t owi_pin;
    /*! \brief OWI Pinmux. */
    const uint8_t owi_pin_af;
    /*! \brief Owi timer channel. */
    const uint8_t owi_rx_channel;
    /*! \brief OWI TX Pin. PIN_NONE is unsed. */
    const nutgpio_t owi_tx_pin;
    /*! \brief OWI TX Pinmux. */
    const uint8_t owi_tx_pin_af;
     /*! \brief Owi TX timer channel. */
    const int8_t owi_tx_channel;
     /*! \brief Do not invert Owi TX timer. */
    const TIM_CC_POLARITY owi_tx_invert;
};

/*!
 * \brief Runtime data of the Owibus with dual channel timer
 *
 * ToDo: Order in a way that few padding happens!
 */
typedef struct _STM32_OWIBUS_TIMER_INFO STM32_OWIBUS_TIMER_INFO;

struct _STM32_OWIBUS_TIMER_INFO {
    /*! \brief Wait queue. */
    HANDLE mutex;
    /*! \brief Hardware information. */
    const STM32_OWIBUS_TIMER_HW *owi_hw;
    /*! \brief Used timer. */
    TIM_TypeDef *timer;
    /*! \brief (Bitband) Register for Update generation. */
    volatile uint32_t *const timer_egr;
     /*! \brief (Bitband) Register for Counter start. */
    volatile uint32_t *const timer_cr1;
     /*! \brief Timer Register to set output active length on next update. */
    volatile uint32_t *ccr_pulse;
    /*! \brief Timer Register that captured rising edge. */
    volatile uint32_t *ccr_capture;
    /*! \brief CCMR register. */
    volatile uint32_t *ccmr;
    /*! \brief CCMR register mask to clear CCS selection. */
    uint16_t ccmr_mask;
    /*! \brief CCMR register CCS value for "Force active value". */
    uint16_t ccmr_force;
    /*! \brief Active length of next cycle. */
    uint16_t sample_value;
    /*! \brief Reference value for rising edge. */
    uint16_t owi_compare;
    /*! \brief Array of pulse length in counter tick. */
    uint16_t owi_tim_values[OWI_TIM_SIZE];
    /*! \brief Current receive bit. */
    uint8_t owi_rx_len;
    /*! \brief Current transmit bit. */
    uint8_t owi_tx_len;
    /*! \brief index in current byte. */
    uint8_t owi_index;
    /*! \brief Pointer to current transmit byte. */
    const uint8_t  *owi_txp;
    /*! \brief Pointer to current receive byte.. */
    uint8_t  *owi_rxp;
};

extern int Stm32TimOwiTouchReset(NUTOWIBUS *bus);
extern int Stm32TimOwiWriteBlock(NUTOWIBUS *, const uint8_t *, uint_fast8_t);
extern int Stm32TimOwiReadBlock (NUTOWIBUS *, uint8_t *, uint_fast8_t);
extern int Stm32TimOwiSetup(NUTOWIBUS *bus);
#endif
