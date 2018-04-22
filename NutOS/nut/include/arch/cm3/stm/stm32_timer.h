#if !defined(_STM32_TIMER_H)
#define _STM32_TIMER_H

#include <cfg/arch.h>
#include <sys/timer.h>
#include <arch/cm3/stm/stm32_clk.h>

struct TIMERINFO{
    TIM_TypeDef *tim;
    uint8_t      rst_bb;
    uint8_t      en_bb;
    uint8_t      base;
    uint8_t      width;
    uint8_t      n_channels;
};

typedef enum {
    TIM_TRG_SELECTION_NONE = -1,
    TIM_TRG_SELECTION_ITR0,
    TIM_TRG_SELECTION_ITR1,
    TIM_TRG_SELECTION_ITR2,
    TIM_TRG_SELECTION_ITR3,
    TIM_TRG_SELECTION_TI1F_ED,
    TIM_TRG_SELECTION_TI1FP1,
    TIM_TRG_SELECTION_TI2FP2,
    TIM_TRG_SELECTION_ETR,
    TIM_TRG_SELECTION_ETR2,
    TIM_TRG_SELECTION_ETR4,
    TIM_TRG_SELECTION_ETR8,
    TIM_TRG_SELECTION_ETRN,
    TIM_TRG_SELECTION_ETRN2,
    TIM_TRG_SELECTION_ETRN4,
    TIM_TRG_SELECTION_ETRN8
}TIM_TRG_SELECTION;

typedef enum{
    TIM_MASTER_MODE1_NONE = -1,
    TIM_MASTER_MODE1_RESET,
    TIM_MASTER_MODE1_ENABLE,
    TIM_MASTER_MODE1_UPDATE,
    TIM_MASTER_MODE1_COMPARE,
    TIM_MASTER_MODE1_OC1REF,
    TIM_MASTER_MODE1_OC2REF,
    TIM_MASTER_MODE1_OC3REF,
#if defined(MCU_STM32F3)
    TIM_MASTER_MODE1_OC4REF,
    TIM_MASTER_MODE1_OC5REF,
    TIM_MASTER_MODE1_OC6REF
#else
    TIM_MASTER_MODE1_OC4REF
#endif
}TIM_MASTER_MODE1;

typedef enum{
    TIM_SLAVE_MODE_NONE           = 0,
    TIM_SLAVE_MODE_ENCODER_TI1FP1 = 1,
    TIM_SLAVE_MODE_ENCODER_TI2FP2 = 2,
    TIM_SLAVE_MODE_ENCODER_BOTH   = 3,
    CNT_SLAVE_MODE_TRG_RESET      = 4,
    TIM_SLAVE_MODE_TRG_GATE       = 5,
    TIM_SLAVE_MODE_TRG_START      = 6,
#if defined(TIM_SMCR_SMS_3)
    TIM_SLAVE_MODE_TRG_CNT        = 7,
    TIM_SLAVE_MODE_TRG_RESET_CNT  = 8
#else
    TIM_SLAVE_MODE_TRG_CNT        = 7
#endif
}TIM_SLAVE_MODE;

typedef enum{
    TIM_CLK_MODE_CKINT = 0,
    TIM_CLK_MODE_TI1   = 1,
    TIM_CLK_MODE_TI2   = 2,
    TIM_CLK_MODE_ETR   = 3
}TIM_CLK_MODE;

typedef enum{
    TIM_CC_POL_TRUE    = 0,
    TIM_CC_POL_INVERT  = 1,
    TIM_CC_POL_INVALID = 2,
    TIM_CC_POL_BOTH    = 3
}TIM_CC_POLARITY;

typedef enum{
    TIM_CC_OUTPUT    = 0,
    TIM_CC_DIRECT    = 1,
    TIM_CC_EXCHANGE  = 2,
    TIM_CC_TRIGGER   = 3
}TIM_CC_FIN;

typedef enum{
    TIM_CC_FROZEN    = 0,
    TIM_CC_ACTIV_ON_MATCH    = 1,
    TIM_CC_INACTIVE_ON_MATCH = 2,
    TIM_CC_TOGGLE_ON_MATCH   = 3,
    TIM_CC_FORCE_INACTIVE    = 4,
    TIM_CC_FORCE_ACTIVE      = 5,
    TIM_CC_ACTIVE_IF_LESS    = 6,
    TIM_CC_ACTIVE_IF_GREATER = 7,
    TIM_CC_FROZEN_DIRECT,
    TIM_CC_ACTIV_ON_MATCH_DIRECT,
    TIM_CC_INACTIVE_ON_MATCH_DIRECT,
    TIM_CC_TOGGLE_ON_MATCH_DIRECT,
    TIM_CC_FORCE_INACTIVE_DIRECT,
    TIM_CC_FORCE_ACTIVE_DIRECT,
    TIM_CC_ACTIVE_IF_LESS_DIRECT,
    TIM_CC_ACTIVE_IF_GREATER_DIRECT,
}TIM_CC_FOUT;

/* APB1 devices are always below 0x40010000! */
# define BASE2TIM_ENR(base)  ((base < (PERIPH_BASE + 0x10000)) ?        \
                              &RCC->APB1ENR  : &RCC->APB2ENR)
# define BASE2TIM_RSTR(base) ((base < (PERIPH_BASE + 0x10000)) ?        \
                              &RCC->APB1RSTR : &RCC->APB2RSTR)

/*!
 * \brief Return address of CCRx register for given Timerbase and channel
 */
#if defined(TIM_CCR6_CCR6)
#define CCR_REG(BASE, CH) (                                           \
        (((CH) == 1) || ((CH) == -1))? &((TIM_TypeDef *)BASE)->CCR1:  \
        (((CH) == 2) || ((CH) == -2))? &((TIM_TypeDef *)BASE)->CCR2:  \
        (((CH) == 3) || ((CH) == -3))? &((TIM_TypeDef *)BASE)->CCR3:  \
        (((CH) == 4) || ((CH) == -4))? &((TIM_TypeDef *)BASE)->CCR4:  \
        (((CH) == 5) || ((CH) == -5))? &((TIM_TypeDef *)BASE)->CCR5   \
        :                              &((TIM_TypeDef *)BASE)->CCR6)

#define CCMR_REG(BASE, CH) (                                          \
        (((CH) == 1) || ((CH) == -1))? &((TIM_TypeDef *)BASE)->CCMR1: \
        (((CH) == 2) || ((CH) == -2))? &((TIM_TypeDef *)BASE)->CCMR1: \
        (((CH) == 3) || ((CH) == -3))? &((TIM_TypeDef *)BASE)->CCMR2: \
        (((CH) == 4) || ((CH) == -4))? &((TIM_TypeDef *)BASE)->CCMR2: \
        (((CH) == 5) || ((CH) == -5))? &((TIM_TypeDef *)BASE)->CCMR3  \
        :                              &((TIM_TypeDef *)BASE)->CCMR3)
#else
#define CCR_REG(BASE, CH) (                                           \
        (((CH) == 1) || ((CH) == -1))? &((TIM_TypeDef *)BASE)->CCR1:  \
        (((CH) == 2) || ((CH) == -2))? &((TIM_TypeDef *)BASE)->CCR2:  \
        (((CH) == 3) || ((CH) == -3))? &((TIM_TypeDef *)BASE)->CCR3   \
        :                              &((TIM_TypeDef *)BASE)->CCR4)

#define CCMR_REG(BASE, CH) (                                          \
        (((CH) == 1) || ((CH) == -1))? &((TIM_TypeDef *)BASE)->CCMR1: \
        (((CH) == 2) || ((CH) == -2))? &((TIM_TypeDef *)BASE)->CCMR1: \
        (((CH) == 3) || ((CH) == -3))? &((TIM_TypeDef *)BASE)->CCMR2  \
        :                              &((TIM_TypeDef *)BASE)->CCMR2)
#endif

int Stm32TimerChannelConfig(
    TIM_TypeDef    *tim,
    int8_t          channel,
    uint8_t         filter,
    TIM_CC_FIN      fin,
    TIM_CC_FOUT     fout,
    TIM_CC_POLARITY polarity);

void Stm32TimerConfig(
    TIM_TypeDef      *tim,
    TIM_CLK_MODE      clk_mode,
    TIM_TRG_SELECTION trg_sel,
    TIM_SLAVE_MODE    slave_mode,
    TIM_MASTER_MODE1  master_mode1);
#define Stm32TimerSetReload(x, y) x->ARR = (y)
#define Stm32TimerGetReload(x)    x->ARR;
#define Stm32TimerSetPrescaler(x, y)  x->PSC = (y)
#define Stm32TimerGetPrescaler(x   )  x->PSC
#define Stm32TimerStart(x)        x->CR1 |= TIM_CR1_CEN
#define Stm32TimerStop(x)         x->CR1 &= ~TIM_CR1_CEN
uint32_t Stm32TimerGetClock(TIM_TypeDef *tim);
#endif
