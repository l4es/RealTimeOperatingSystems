#ifndef _STM32F1_DMA_H_
#define _STM32F1_DMA_H_
/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 *
 */

/*!
 * \verbatim
 * $Id: ih_stm32_usart1.c 3131 2010-09-22 21:47:23Z Astralix $
 * \endverbatim
 */

/*!
 * \brief DMA Channel Naming
 *
 * These defines keep Nut/OS internal channel numbering
 * aligned with the STM32 documentation RM0008.
 */
#include <cfg/devices.h>

#define DMA1_CH1   0
#define DMA1_CH2   1
#define DMA1_CH3   2
#define DMA1_CH4   3
#define DMA1_CH5   4
#define DMA1_CH6   5
#define DMA1_CH7   6

#define DMA2_CH1   7
#define DMA2_CH2   8
#define DMA2_CH3   9
#define DMA2_CH4  10
#define DMA2_CH5  11
#define DMA2_CH6  12
#define DMA2_CH7  13
#if defined(HW_DMA2_STM32F1)
# if defined(HW_DMA2_STM32L4)
#  define STM_HAS_DMA2 7
# else
#  define STM_HAS_DMA2 5
# endif
#endif

/* Argh! F0 and L0/4 define DMA CSELR in different structures! */
#if defined(MCU_STM32L0)
#define SPI1_DMA_RX_SEL(ch) ((ch == DMA1_CH2)? 1 : 4)
#define SPI1_DMA_TX_SEL(ch) ((ch == DMA1_CH3)? 1 : 4)
#define SPI2_DMA_RX_SEL(ch) 2
#define SPI2_DMA_TX_SEL(ch) 2
#define DMA1_CSELR_REG &DMA1_CSELR->CSELR
#elif defined(MCU_STM32L4)
#define SPI1_DMA_RX_SEL(ch) ((ch == DMA1_CH2)? 1 : 4)
#define SPI1_DMA_TX_SEL(ch) ((ch == DMA1_CH3)? 1 : 4)
#define SPI2_DMA_TX_SEL(ch) 1
#define SPI2_DMA_RX_SEL(ch) 1
#define SPI3_DMA_TX_SEL(ch) 3
#define SPI3_DMA_RX_SEL(ch) 3
#define DMA1_CSELR_REG &DMA1_CSELR->CSELR
#define DMA2_CSELR_REG &DMA2_CSELR->CSELR
#elif defined(MCU_STM32F09)
#define SPI1_DMA_RX_SEL(ch) 3
#define SPI1_DMA_TX_SEL(ch) 3
#define SPI2_DMA_RX_SEL(ch) 3
#define SPI2_DMA_TX_SEL(ch) 3
#define DMA1_CSELR_REG &DMA1->CSELR
#define DMA2_CSELR_REG &DMA2->CSELR
#endif

#define ADC1_DMA                       DMA1_CH1
#define ADC1_DMA_IRQ                   sig_DMA1_CH1
#define TIM2_CH3_DMA                   DMA1_CH1
#define TIM2_CH3_DMA_IRQ               sig_DMA1_CH1
#define TIM4_CH1_DMA                   DMA1_CH1
#define TIM4_CH1_DMA_IRQ               sig_DMA1_CH1
#define TIM17_CH1_UP_DMA               DMA1_CH1
#define TIM17_CH1_UP_DMA_IRQ           sig_DMA1_CH1
#define TIM19_CH3_CH4_DMA              DMA1_CH1
#define TIM19_CH3_CH4_DMA_IRQ          sig_DMA1_CH1

#define SPI1_RX_DMA                    DMA1_CH2
#define SPI1_RX_DMA_IRQ                sig_DMA1_CH2
#define USART3_TX_DMA                  DMA1_CH2
#define USART3_TX_DMA_IRQ              sig_DMA1_CH2
#define TIM1_CH1_DMA                   DMA1_CH2
#define TIM1_CH1_DMA_IRQ               sig_DMA1_CH2
#define TIM2_UP_DMA                    DMA1_CH2
#define TIM2_UP_DMA_IRQ                sig_DMA1_CH2
#define TIM3_CH3_DMA                   DMA1_CH2
#define TIM3_CH3_DMA_IRQ               sig_DMA1_CH2
#define TIM19_CH1_DMA                  DMA1_CH2
#define TIM19_CH1_DMA_IRQ              sig_DMA1_CH2

#define SPI1_TX_DMA                    DMA1_CH3
#define SPI1_TX_DMA_IRQ                sig_DMA1_CH3
#define USART3_RX_DMA                  DMA1_CH3
#define USART3_RX_DMA_IRQ              sig_DMA1_CH3
#define TIM3_CH4_UP_DMA                DMA1_CH3
#define TIM3_CH4_UP_DMA_IRQ            sig_DMA1_CH3
#define TIM6_UP_DAC1_CH1_DMA           DMA1_CH3
#define TIM6_UP_DAC1_CH1_DMA_IRQ       sig_DMA1_CH3
#define TIM15_CH1_UP_DMA               DMA1_CH3
#define TIM15_CH1_UP_DMA_IRQ           sig_DMA1_CH3
#define TIM19_CH2_DMA                  DMA1_CH3
#define TIM19_CH2_DMA_IRQ              sig_DMA1_CH3

#define SPI2_RX_DMA                    DMA1_CH4
#define SPI2_RX_DMA_IRQ                sig_DMA1_CH4
#define USART1_TX_DMA                  DMA1_CH4
#define USART1_TX_DMA_IRQ              sig_DMA1_CH4
#define I2C2_TX_DMA                    DMA1_CH4
#define I2C2_TX_DMA_IRQ                sig_DMA1_CH4
#define TIM4_CH2_DMA                   DMA1_CH4
#define TIM4_CH2_DMA_IRQ               sig_DMA1_CH4
#define TIM7_UP_DAC1_CH2_DMA           DMA1_CH4
#define TIM7_UP_DAC1_CH2_DMA_IRQ       sig_DMA1_CH4
#define TIM19_UP_DMA                   DMA1_CH4
#define TIM19_UP_DMA_IRQ               sig_DMA1_CH4

#define SPI2_TX_DMA                    DMA1_CH5
#define SPI2_TX_DMA_IRQ                sig_DMA1_CH5
#define USART1_RX_DMA                  DMA1_CH5
#define USART1_RX_DMA_IRQ              sig_DMA1_CH5
#define TIM1_UP_DMA                    DMA1_CH5
#define TIM1_UP_DMA_IRQ                sig_DMA1_CH5
#define TIM4_CH3_DMA                   DMA1_CH5
#define TIM4_CH3_DMA_IRQ               sig_DMA1_CH5
#define TIM18_UP_DAC2_CH1_DMA          DMA1_CH5
#define TIM18_UP_DAC2_CH1_DMA_IRQ      sig_DMA1_CH5
#define TIM15_CH1_UP_TRIG_COM_DMA      DMA1_CH5
#define TIM15_CH1_UP_TRIG_COM_DMA_IRQ  sig_DMA1_CH5

#define USART2_RX_DMA                  DMA1_CH6
#define USART2_RX_DMA_IRQ              sig_DMA1_CH6
#define I2C1_TX_DMA                    DMA1_CH6
#define I2C1_TX_DMA_IRQ                sig_DMA1_CH6
#define TIM3_CH1_TRIG_DMA              DMA1_CH6
#define TIM3_CH1_TRIG_DMA_IRQ          sig_DMA1_CH6
#define TIM16_CH1_UP_DMA               DMA1_CH6
#define TIM16_CH1_UP_DMA_IRQ           sig_DMA1_CH6

#define USART2_TX_DMA                  DMA1_CH7
#define USART2_TX_DMA_IRQ              sig_DMA1_CH7
#define I2C1_RX_DMA                    DMA1_CH7
#define I2C1_RX_DMA_IRQ                sig_DMA1_CH7
#define TIM2_CH2_CH4_DMA               DMA1_CH7
#define TIM2_CH2_CH4_DMA_IRQ           sig_DMA1_CH7
#define TIM4_UP_DMA                    DMA1_CH7
#define TIM4_UP_DMA_IRQ                sig_DMA1_CH7
#define TIM17_CH1_UP_ALT_DMA           DMA1_CH7
#define TIM17_CH1_UP_ALT_DMA_IRQ       sig_DMA1_CH7

#define SPI3_RX_DMA                    DMA2_CH1
#define SPI3_RX_DMA_IRQ                sig_DMA2_CH1
#define TIM5_CH4_TRIG_DMA              DMA2_CH1
#define TIM5_CH4_TRIG_DMA_IRQ          sig_DMA2_CH1
#define TIM8_CH3_UP_DMA                DMA2_CH1
#define TIM8_CH3_UP_DMA_IRQ            sig_DMA2_CH1

#define ADC4_DMA                       DMA2_CH2
#define ADC4_DMA_IRQ                   sig_DMA2_CH2
#define SPI3_TX_DMA                    DMA2_CH2
#define SPI3_TX_DMA_IRQ                sig_DMA2_CH2
#define TIM5_CH3_UP_DMA                DMA2_CH2
#define TIM5_CH3_UP_DMA_IRQ            sig_DMA2_CH2
#define TIM8_CH4_TRIG_COM_DMA          DMA2_CH2
#define TIM8_CH4_TRIG_COM_DMA_IRQ      sig_DMA2_CH2

#define USART4_RX_DMA                  DMA2_CH3
#define USART4_RX_DMA_IRQ              sig_DMA2_CH3
#define SDADC1_DMA                     DMA2_CH3
#define SDADC1_DMA_IRQ                 sig_DMA2_CH3
#define TIM6_UP_DAC1_CH1_ALT_DMA       DMA2_CH3
#define TIM6_UP_DAC1_CH1_ALT_DMA_IRQ   sig_DMA2_CH3
#define TIM8_CH1_DMA                   DMA2_CH3
#define TIM8_CH1_DMA_IRQ               sig_DMA2_CH3

#define ADC4_ALT_DMA                   DMA2_CH4
#define ADC4_ALT_DMA_IRQ               sig_DMA2_CH4
#define SDADC2_DMA                     DMA2_CH4
#define SDADC2_DMA_IRQ                 sig_DMA2_CH4
#define TIM5_CH2_DMA                   DMA2_CH4
#define TIM5_CH2_DMA_IRQ               sig_DMA2_CH4
#define TIM7_UP_DAC1_CH2_ALT_DMA       DMA2_CH4
#define TIM7_UP_DAC1_CH2_ALT_DMA_IRQ   sig_DMA2_CH4

#define ADC3_DMA                       DMA2_CH5
#define ADC3_DMA_IRQ                   sig_DMA2_CH5
#define UART4_TX_DMA                   DMA2_CH5
#define UART4_TX_DMA_IRQ               sig_DMA2_CH5
#define SDADC3_DMA                     DMA2_CH5
#define SDADC3_DMA_IRQ                 sig_DMA2_CH5
#define TIM5_CH1                       DMA2_CH5
#define TIM5_CH1_IRQ                   sig_DMA2_CH5
#define TIM18_UP_DAC2_CH1_ALT_DMA      DMA2_CH5
#define TIM18_UP_DAC2_CH1_ALT_DMA_IRQ  sig_DMA2_CH5
#define TIM8_CH2_DMA                   DMA2_CH5
#define TIM8_CH2_DMA_IRQ               sig_DMA2_CH5

/* Care for ADC2 on F3 devices without DMA2 */
#if defined(HW_DMA2_STM32F1)
#define ADC2_DMA                       DMA2_CH1
#define ADC2_DMA_IRQ                   sig_DMA2_CH1
#define ADC2_ALT_DMA                   DMA2_CH3
#define ADC2_ALT_DMA_IRQ               sig_DMA2_CH3
#else
#define ADC2_DMA                       DMA1_CH2
#define ADC2_DMA_IRQ                   sig_DMA1_CH2
#define ADC2_ALT_DMA                   DMA1_CH4
#define ADC2_ALT_DMA_IRQ               sig_DMA1_CH4
#endif
/*!
 * \brief STM32 F0/F1/F3/L0/L3 DMA Control Flags.
 */
#define DMA_EN          DMA_CCR_EN
#define DMA_TEIE        DMA_CCR_TEIE
#define DMA_HTIE        DMA_CCR_HTIE
#define DMA_TCIE        DMA_CCR_TCIE
#define DMA_P2M         0
#define DMA_M2P         DMA_CCR_DIR
#define DMA_M2M         DMA_CCR_MEM2MEM
#define DMA_MINC        DMA_CCR_MINC
#define DMA_PINC        DMA_CCR_PINC
#define DMA_CIRC        DMA_CCR_CIRC
#define DMA_MSIZE_8     0
#define DMA_MSIZE_16    DMA_CCR_MSIZE_0
#define DMA_MSIZE_32    DMA_CCR_MSIZE_1
#define DMA_PSIZE_8     0
#define DMA_PSIZE_16    DMA_CCR_PSIZE_0
#define DMA_PSIZE_32    DMA_CCR_PSIZE_1
#define DMA_PRIO_LOW    0
#define DMA_PRIO_MEDIUM DMA_CCR_PL_0
#define DMA_PRIO_HIGH   DMA_CCR_PL_1
#define DMA_PRIO_HIGEST DMA_CCR_PL
#define DMA_MEM2MEN     DMA_CCR_MEM2MEM

#if defined(HW_DMA_COMBINED_IRQ_STM32)
# if   defined(SYSCFG_ITLINE10_SR_DMA1_CH2)
#  define DMA_CH2IRQ_P(ch) (                 \
        (ch == DMA_NONE) ? NULL            : \
        (ch == DMA2_CH5) ? &sig_DMA_GROUP2 : \
        (ch == DMA2_CH4) ? &sig_DMA_GROUP2 : \
        (ch == DMA2_CH3) ? &sig_DMA_GROUP2 : \
        (ch == DMA2_CH2) ? &sig_DMA_GROUP1 : \
        (ch == DMA2_CH1) ? &sig_DMA_GROUP1 : \
        (ch == DMA1_CH7) ? &sig_DMA_GROUP2 : \
        (ch == DMA1_CH6) ? &sig_DMA_GROUP2 : \
        (ch == DMA1_CH5) ? &sig_DMA_GROUP2 : \
        (ch == DMA1_CH4) ? &sig_DMA_GROUP2 : \
        (ch == DMA1_CH3) ? &sig_DMA_GROUP1 : \
        (ch == DMA1_CH2) ? &sig_DMA_GROUP1 : &sig_DMA1_CH1)
# elif defined(HW_DMA2_STM32F1)
#  define DMA_CH2IRQ_P(ch) (               \
        (ch == DMA_NONE) ? NULL          : \
        (ch == DMA2_CH5) ? &sig_DMA2_CH4 : \
        (ch == DMA2_CH4) ? &sig_DMA2_CH4 : \
        (ch == DMA2_CH3) ? &sig_DMA2_CH3 : \
        (ch == DMA2_CH2) ? &sig_DMA2_CH2 : \
        (ch == DMA2_CH1) ? &sig_DMA2_CH1 : \
        (ch == DMA1_CH7) ? &sig_DMA1_CH7 : \
        (ch == DMA1_CH6) ? &sig_DMA1_CH6 : \
        (ch == DMA1_CH5) ? &sig_DMA1_CH5 : \
        (ch == DMA1_CH4) ? &sig_DMA1_CH4 : \
        (ch == DMA1_CH3) ? &sig_DMA1_CH3 : \
        (ch == DMA1_CH2) ? &sig_DMA1_CH2 : &sig_DMA1_CH1)
# else
#  define DMA_CH2IRQ_P(ch) (                 \
        (ch == DMA_NONE) ? NULL            : \
        (ch >= DMA1_CH4) ? &sig_DMA_GROUP2 : \
        (ch >= DMA1_CH2) ? &sig_DMA_GROUP1 : &sig_DMA1_CH1 )
# endif
#else
# if defined(HW_DMA2_STM32L4)
#  define DMA_CH2IRQ_P(ch) (               \
        (ch == DMA_NONE) ? NULL         :  \
        (ch == DMA2_CH7) ? &sig_DMA2_CH7 : \
        (ch == DMA2_CH6) ? &sig_DMA2_CH6 : \
        (ch == DMA2_CH5) ? &sig_DMA2_CH5 : \
        (ch == DMA2_CH4) ? &sig_DMA2_CH4 : \
        (ch == DMA2_CH3) ? &sig_DMA2_CH3 : \
        (ch == DMA2_CH2) ? &sig_DMA2_CH2 : \
        (ch == DMA2_CH1) ? &sig_DMA2_CH1 : \
        (ch == DMA1_CH7) ? &sig_DMA1_CH7 : \
        (ch == DMA1_CH6) ? &sig_DMA1_CH6 : \
        (ch == DMA1_CH5) ? &sig_DMA1_CH5 : \
        (ch == DMA1_CH4) ? &sig_DMA1_CH4 : \
        (ch == DMA1_CH3) ? &sig_DMA1_CH3 : \
        (ch == DMA1_CH2) ? &sig_DMA1_CH2 : &sig_DMA1_CH1 )
# else
#  define DMA_CH2IRQ_P(ch) (             \
        (ch == DMA_NONE) ? NULL         : \
        (ch == DMA2_CH5) ? &sig_DMA2_CH5 : \
        (ch == DMA2_CH4) ? &sig_DMA2_CH4 : \
        (ch == DMA2_CH3) ? &sig_DMA2_CH3 : \
        (ch == DMA2_CH2) ? &sig_DMA2_CH2 : \
        (ch == DMA2_CH1) ? &sig_DMA2_CH1 : \
        (ch == DMA1_CH7) ? &sig_DMA1_CH7 : \
        (ch == DMA1_CH6) ? &sig_DMA1_CH6 : \
        (ch == DMA1_CH5) ? &sig_DMA1_CH5 : \
        (ch == DMA1_CH4) ? &sig_DMA1_CH4 : \
        (ch == DMA1_CH3) ? &sig_DMA1_CH3 : \
        (ch == DMA1_CH2) ? &sig_DMA1_CH2 : &sig_DMA1_CH1)
# endif
#endif

/*!
 * \brief STM32 F0/F1/F3/L0/L1 DMA Status and Interrupt Flags.
 */
#define DMA_TEIF DMA_ISR_TEIF1  /*< Channel x Transfer Error Flag */
#define DMA_HTIF DMA_ISR_HTIF1  /*< Channel x Half Transfer Complete Flag */
#define DMA_TCIF DMA_ISR_TCIF1  /*< Channel x Transfer Complete Flag */
#define DMA_GIF  DMA_ISR_GIF1   /*< Channel x Global Flag */

#define DMA_FLAGMASK (DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_GIF)
#define DMA_IRQMASK (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE)

/* Internally used struct and table to align
 * DMA channels and interrupts. */
typedef struct {
    DMA_TypeDef *dma;            /*< DMA Controller Register Base Address */
    uint32_t fofs;               /*< DMA Channel Flags Offset */
    DMA_Channel_TypeDef* dma_ch; /*< DMA Channel Register Base Address */
} DMATAB;

#if defined(HW_DMA2_7CH_STM32)
#define DMA_COUNT 14
#elif defined(HW_DMA2_STM32F1)
#define DMA_COUNT 12
#elif !defined(HW_DMA1_5CH_STM32)
#define DMA_COUNT  7
#else
#define DMA_COUNT  5
#endif

# if defined(HW_DMA_COMBINED_IRQ_STM32)

typedef struct _dma_signal DMA_SIGNAL;

struct _dma_signal{
    uint8_t ch;
    void (*dma_channel_handler) (void *);
    void *dma_channel_arg;
# if defined(SYSCFG_ITLINE10_SR_DMA1_CH2)
    __IO uint32_t *it_line_sr;
    uint8_t   it_line_mask;
#endif
    DMA_SIGNAL *next;
};
#  define DmaIrqEnable(x, y) 0
#  define DmaIrqDisable(x, y) 0

# else
#  define DMA_SIGNAL  IRQ_HANDLER
#  define DmaEnableHandler(signal, ch)  NutIrqEnable(signal)
#  define DmaDisableHandler(signal, ch) NutIrqDisable(signal)
# endif
#endif /* _STM32F1_DMA_H_ */
