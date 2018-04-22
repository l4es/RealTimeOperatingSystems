/*
 * Copyright (C) 2012, 2014-2016 by Uwe Bonnes
 *                           (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#if !defined(_STM32F2_DMA_H_)
#define _STM32F2_DMA_H_

#define DMA_CONTROL1 0x00
#define DMA_CONTROL2 0x80
#define DMA_STREAM0  0x00
#define DMA_STREAM1  0x10
#define DMA_STREAM2  0x20
#define DMA_STREAM3  0x30
#define DMA_STREAM4  0x40
#define DMA_STREAM5  0x50
#define DMA_STREAM6  0x60
#define DMA_STREAM7  0x70
#define DMA_CHANNEL0 0x00
#define DMA_CHANNEL1 0x01
#define DMA_CHANNEL2 0x02
#define DMA_CHANNEL3 0x03
#define DMA_CHANNEL4 0x04
#define DMA_CHANNEL5 0x05
#define DMA_CHANNEL6 0x06
#define DMA_CHANNEL7 0x07

#define DMA_CH2IRQ_P(ch) (                                             \
        (ch == DMA_NONE)                                                              ? NULL              : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM7))? &sig_DMA2_STREAM7 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM6))? &sig_DMA2_STREAM6 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM5))? &sig_DMA2_STREAM5 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM4))? &sig_DMA2_STREAM4 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM3))? &sig_DMA2_STREAM3 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM2))? &sig_DMA2_STREAM2 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM1))? &sig_DMA2_STREAM1 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL2) && ((ch & DMA_STREAM7) == DMA_STREAM0))? &sig_DMA2_STREAM0 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM7))? &sig_DMA1_STREAM7 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM6))? &sig_DMA1_STREAM6 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM5))? &sig_DMA1_STREAM5 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM4))? &sig_DMA1_STREAM4 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM3))? &sig_DMA1_STREAM3 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM2))? &sig_DMA1_STREAM2 : \
        (((ch & DMA_CONTROL2) == DMA_CONTROL1) && ((ch & DMA_STREAM7) == DMA_STREAM1))? &sig_DMA1_STREAM1 : \
                                                                                        &sig_DMA1_STREAM0)

/* Also tedious, lets define all channels and interrupts here so
 * we have everything in one place.
 */
#define SPI3_RX_DMA                (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL0)
#define SPI3_RX_DMA_IRQ            (sig_DMA1_STREAM0)
#define SPI3_RX_ALT_DMA            (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL0)
#define SPI3_RX_ALT_DMA_IRQ        (sig_DMA1_STREAM2)
#define SPI2_RX_DMA                (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL0)
#define SPI2_RX_DMA_IRQ            (sig_DMA1_STREAM3)
#define SPI2_TX_DMA                (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL0)
#define SPI2_TX_DMA_IRQ            (sig_DMA1_STREAM4)
#define SPI3_TX_DMA                (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL0)
#define SPI3_TX_DMA_IRQ            (sig_DMA1_STREAM5)
#define SPI3_TX_ALT_DMA            (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL0)
#define SPI3_TX_ALT_DMA_IRQ        (sig_DMA1_STREAM7)

#define I2C1_RX_DMA                (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL1)
#define I2C1_RX_DMA_IRQ            (sig_DMA1_STREAM0)
#define TIM7_UP_DMA                (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL1)
#define TIM7_UP_DMA_IRQ            (sig_DMA1_STREAM2)
#define TIM7_UP_ALT_DMA            (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL1)
#define TIM7_UP_ALT_DMA_IRQ        (sig_DMA1_STREAM4)
#define I2C1_RX_ALT_DMA            (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL1)
#define I2C1_RX_DMA_ALT_IRQ        (sig_DMA1_STREAM5)
#define I2C1_TX_DMA                (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL1)
#define I2C1_TX_DMA_IRQ            (sig_DMA1_STREAM6)
#define I2C1_TX_ALT_DMA            (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL1)
#define I2C1_TX_DMA_ALT_IRQ        (sig_DMA1_STREAM7)

#define TIM4_CH1_DMA               (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL2)
#define TIM4_CH1_DMA_IRQ           (sig_DMA1_STREAM0)
#define I2S3_EXT_RX_DMA            (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL2)
#define I2S3_EXT_RX_DMA_IRQ        (sig_DMA1_STREAM2)
#define TIM4_CH2_DMA               (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL2)
#define TIM4_CH2_DMA_IRQ           (sig_DMA1_STREAM3)
#define I2S2_EXT_TX_DMA            (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL2)
#define I2S2_EXT_TX_DMA_IRQ        (sig_DMA1_STREAM4)
#define I2S3_EXT_TX_DMA            (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL2)
#define I2S3_EXT_TX_DMA_IRQ        (sig_DMA1_STREAM5)
#define TIM4_UP_DMA                (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL2)
#define TIM4_UP_DMA_IRQ            (sig_DMA1_STREAM6)
#define TIM4_CH3_DMA               (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL2)
#define TIM4_CH3_DMA_IRQ           (sig_DMA1_STREAM7)

#define I2S3_EXT_RX_ALT_DMA        (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL3)
#define I2S3_EXT_RX_ALT_DMA_IRQ    (sig_DMA1_STREAM0)
#define TIM2_UP_CH3_DMA            (DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL3)
#define TIM2_UP_CH3_DMA_IRQ        (sig_DMA1_STREAM1)
#define I2C3_RX_DMA                (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL3)
#define I2C3_RX_DMA_IRQ            (sig_DMA1_STREAM2)
#define I2S2_EXT_RX_ALT_DMA        (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL3)
#define I2S2_EXT_RX_ALT_DMA_IRQ    (sig_DMA1_STREAM3)
#define I2C3_TX_DMA                (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL3)
#define I2C3_TX_DMA_IRQ            (sig_DMA1_STREAM4)
#define TIM2_CH1_DMA               (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL3)
#define TIM2_CH1_DMA_IRQ           (sig_DMA1_STREAM5)
#define TIM2_CH2_CH4_CH3_DMA       (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL3)
#define TIM2_CH2_CH4_CH3_DMA_IRQ   (sig_DMA1_STREAM6)
#define TIM2_UP_CH4_CH3_DMA        (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL3)
#define TIM2_UP_CH4_CH3_DMA_IRQ    (sig_DMA1_STREAM7)

#define UART5_RX_DMA               (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL4)
#define UART5_RX_DMA_IRQ           (sig_DMA1_STREAM0)
#define USART3_RX_DMA              (DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL4)
#define USART3_RX_DMA_IRQ          (sig_DMA1_STREAM1)
#define UART4_RX_DMA               (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL4)
#define UART4_RX_DMA_IRQ           (sig_DMA1_STREAM2)
#define USART3_TX_DMA              (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL4)
#define USART3_TX_DMA_IRQ          (sig_DMA1_STREAM3)
#define UART4_TX_DMA               (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL4)
#define UART4_TX_DMA_IRQ           (sig_DMA1_STREAM4)
#define USART2_RX_DMA              (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL4)
#define USART2_RX_DMA_IRQ          (sig_DMA1_STREAM5)
#define USART2_TX_DMA              (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL4)
#define USART2_TX_DMA_IRQ          (sig_DMA1_STREAM6)
#define UART5_TX_DMA               (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL4)
#define UART5_TX_DMA_IRQ           (sig_DMA1_STREAM7)

#define UART8_TX_DMA               (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL5)
#define UART8_TX_DMA_IRQ           (sig_DMA1_STREAM0)
#define UART7_TX_DMA               (DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL5)
#define UART7_TX_DMA_IRQ           (sig_DMA1_STREAM1)
#define TIM3_CH4_UP_DMA            (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL5)
#define TIM3_CH4_UP_DMA_IRQ        (sig_DMA1_STREAM2)
#define UART7_RX_DMA               (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL5)
#define UART7_RX_DMA_IRQ           (sig_DMA1_STREAM3)
#define TIM3_CH1_TRIG_DMA          (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL5)
#define TIM3_CH1_TRIG_DMA_IRQ      (sig_DMA1_STREAM4)
#define TIM3_CH2_DMA               (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL5)
#define TIM3_CH2_DMA_IRQ           (sig_DMA1_STREAM5)
#define UART8_RX_DMA               (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL5)
#define UART8_RX_DMA_IRQ           (sig_DMA1_STREAM6)
#define TIM3_CH3_DMA               (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL5)
#define TIM3_CH3_DMA_IRQ           (sig_DMA1_STREAM7)

#define TIM5_CH3_UP_DMA            (DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL6)
#define TIM5_CH3_UP_DMA_IRQ        (sig_DMA1_STREAM0)
#define TIM5_CH4_TRIG_DMA          (DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL6)
#define TIM5_CH4_TRIG_DMA_IRQ      (sig_DMA1_STREAM1)
#define TIM5_CH1_DMA               (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL6)
#define TIM5_CH1_DMA_IRQ           (sig_DMA1_STREAM2)
#define TIM5_CH4_TRIG_ALT_DMA      (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL6)
#define TIM5_CH4_TRIG_ALR_DMA_IRQ  (sig_DMA1_STREAM3)
#define TIM5_CH2_DMA               (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL6)
#define TIM5_CH2_DMA_IRQ           (sig_DMA1_STREAM4)
#define TIM5_UP_DMA                (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL6)
#define TIM5_UP_DMA_IRQ            (sig_DMA1_STREAM6)

#define TIM6_UP_DMA                (DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL7)
#define TIM6_UP_DMA_IRQ            (sig_DMA1_STREAM1)
#define I2C2_RX_DMA                (DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL7)
#define I2C2_RX_DMA_IRQ            (sig_DMA1_STREAM2)
#define I2C2_RX_ALT_DMA            (DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL7)
#define I2C2_RX_ALT_DMA_IRQ        (sig_DMA1_STREAM3)
#define USART3_TX_ALT_DMA          (DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL7)
#define USART3_TX_ALT_DMA_IRQ      (sig_DMA1_STREAM4)
#define DAC1_DMA                   (DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL7)
#define DAC1_DMA_IRQ               (sig_DMA1_STREAM5)
#define DAC2_DMA                   (DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL7)
#define DAC2_DMA_IRQ               (sig_DMA1_STREAM6)
#define I2C2_TX_DMA                (DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL7)
#define I2C2_TX_DMA_IRQ            (sig_DMA1_STREAM7)

#define ADC1_DMA                   (DMA_CONTROL2 | DMA_STREAM0 | DMA_CHANNEL0)
#define ADC1_DMA_IRQ               (sig_DMA2_STREAM0)
#define SAI1_A_DMA                 (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL0)
#define SAI1_A_DMA_IRQ             (sig_DMA2_STREAM1)
#define TIM8_CH1_CH2_CH3_DMA       (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL0)
#define TIM8_CH1_CH2_CH3_DMA_IRQ   (sig_DMA2_STREAM2)
#define SAI1_A_ALT_DMA             (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL0)
#define SAI1_A_ALT_DMA_IRQ         (sig_DMA2_STREAM3)
#define ADC1_ALT_DMA               (DMA_CONTROL2 | DMA_STREAM4 | DMA_CHANNEL0)
#define ADC1_ALT_DMA_IRQ           (sig_DMA2_STREAM4)
#define SAI1_B_DMA                 (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL0)
#define SAI1_B_DMA_IRQ             (sig_DMA2_STREAM5)
#define TIM1_CH1_CH2_CH3_DMA       (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL0)
#define TIM1_CH1_CH2_CH3_DMA_IRQ   (sig_DMA2_STREAM6)

#define DCMI_DMA                   (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL1)
#define DCMI_DMA_IRQ               (sig_DMA2_STREAM1)
#define ADC2_DMA                   (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL1)
#define ADC2_DMA_IRQ               (sig_DMA2_STREAM2)
#define ADC2_ALT_DMA               (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL1)
#define ADC2_ALT_DMA_IRQ           (sig_DMA2_STREAM3)
#define SAI1_B_ALT_DMA             (DMA_CONTROL2 | DMA_STREAM4 | DMA_CHANNEL1)
#define SAI1_B_ALT_DMA_IRQ         (sig_DMA2_STREAM4)
#define SPI6_TX_DMA                (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL1)
#define SPI6_TX_DMA_IRQ            (sig_DMA2_STREAM5)
#define SPI6_RX_DMA                (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL1)
#define SPI6_RX_DMA_IRQ            (sig_DMA2_STREAM6)
#define DCMI_ALT_DMA               (DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL1)
#define DCMI_ALT_DMA_IRQ           (sig_DMA2_STREAM7)

#define ADC3_DMA                   (DMA_CONTROL2 | DMA_STREAM0 | DMA_CHANNEL2)
#define ADC3_DMA_IRQ               (sig_DMA2_STREAM0)
#define ADC3_ALT_DMA               (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL2)
#define ADC3_ALT_DMA_IRQ           (sig_DMA2_STREAM1)
#define SPI5_RX_DMA                (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL2)
#define SPI5_RX_DMA_IRQ            (sig_DMA2_STREAM3)
#define SPI5_TX_DMA                (DMA_CONTROL2 | DMA_STREAM4 | DMA_CHANNEL2)
#define SPI5_TX_DMA_IRQ            (sig_DMA2_STREAM4)
#define CRYPT_OUT_DMA              (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL2)
#define CRYPT_OUT_DMA_IRQ          (sig_DMA2_STREAM5)
#define CRYPT_IN_DMA               (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL2)
#define CRYPT_IN_DMA_IRQ           (sig_DMA2_STREAM6)
#define HASH_IN_DMA                (DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL2)
#define HASH_IN_DMA_IRQ            (sig_DMA2_STREAM7)

#define SPI1_RX_DMA                (DMA_CONTROL2 | DMA_STREAM0 | DMA_CHANNEL3)
#define SPI1_RX_DMA_IRQ            (sig_DMA2_STREAM0)
#define SPI1_RX_ALT_DMA            (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL3)
#define SPI1_RX_ALT_DMA_IRQ        (sig_DMA2_STREAM2)
#define SPI1_TX_DMA                (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL3)
#define SPI1_TX_DMA_IRQ            (sig_DMA2_STREAM3)
#define SPI1_TX_ALT_DMA            (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL3)
#define SPI1_TX_ALT_DMA_IRQ        (sig_DMA2_STREAM5)

#define SPI4_RX_DMA                (DMA_CONTROL2 | DMA_STREAM0 | DMA_CHANNEL4)
#define SPI4_RX_DMA_IRQ            (sig_DMA2_STREAM0)
#define SPI4_TX_DMA                (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL4)
#define SPI4_TX_DMA_IRQ            (sig_DMA2_STREAM1)
#define USART1_RX_DMA              (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL4)
#define USART1_RX_DMA_IRQ          (sig_DMA2_STREAM2)
#define SDIO_DMA                   (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL4)
#define SDIO_DMA_IRQ               (sig_DMA2_STREAM3)
#define USART1_RX_ALT_DMA          (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL4)
#define USART1_RX_ALT_DMA_IRQ      (sig_DMA2_STREAM5)
#define SDIO_ALT_DMA               (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL4)
#define SDIO_ALT_DMA_IRQ           (sig_DMA2_STREAM6)
#define USART1_TX_DMA              (DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL4)
#define USART1_TX_DMA_IRQ          (sig_DMA2_STREAM7)

#define USART6_RX_DMA              (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL5)
#define USART6_RX_DMA_IRQ          (sig_DMA2_STREAM1)
#define USART6_RX_ALT_DMA          (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL5)
#define USART6_RX_ALT_DMA_IRQ      (sig_DMA2_STREAM2)
#define SPI4_RX_ALT_DMA            (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL5)
#define SPI4_RX_ALT_DMA_IRQ        (sig_DMA2_STREAM3)
#define SPI4_TX_ALT_DMA            (DMA_CONTROL2 | DMA_STREAM4 | DMA_CHANNEL5)
#define SPI4_TX_ALT_DMA_IRQ        (sig_DMA2_STREAM4)
#define USART6_TX_DMA              (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL5)
#define USART6_TX_DMA_IRQ          (sig_DMA2_STREAM6)
#define USART6_TX_ALT_DMA          (DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL5)
#define USART6_TX_ALT_DMA_IRQ      (sig_DMA2_STREAM7)

#define TIM1_TRIG_DMA              (DMA_CONTROL2 | DMA_STREAM0 | DMA_CHANNEL6)
#define TIM1_TRIG_DMA_IRQ          (sig_DMA2_STREAM0)
#define TIM1_CH1_DMA               (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL6)
#define TIM1_CH1_DMA_IRQ           (sig_DMA2_STREAM1)
#define TIM1_CH2_DMA               (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL6)
#define TIM1_CH2_DMA_IRQ           (sig_DMA2_STREAM2)
#define TIM1_CH1_ALT_DMA           (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL6)
#define TIM1_CH1_ALT_DMA_IRQ       (sig_DMA2_STREAM3)
#define TIM1_CH4_TRIG_COM_DMA      (DMA_CONTROL2 | DMA_STREAM4 | DMA_CHANNEL6)
#define TIM1_CH4_TRIG_COM_DMA_IRQ  (sig_DMA2_STREAM4)
#define TIM1_UP_DMA                (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL6)
#define TIM1_UP_DMA_IRQ            (sig_DMA2_STREAM5)
#define TIM1_CH3_DMA               (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL6)
#define TIM1_CH3_DMA_IRQ           (sig_DMA2_STREAM6)

#define TIM8_UP_DMA                (DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL7)
#define TIM8_UP_DMA_IRQ            (sig_DMA2_STREAM1)
#define TIM8_CH1_DMA               (DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL7)
#define TIM8_CH1_DMA_IRQ           (sig_DMA2_STREAM2)
#define TIM8_CH2_DMA               (DMA_CONTROL2 | DMA_STREAM3 | DMA_CHANNEL7)
#define TIM8_CH2_DMA_IRQ           (sig_DMA2_STREAM3)
#define TIM8_CH3_DMA               (DMA_CONTROL2 | DMA_STREAM4 | DMA_CHANNEL7)
#define TIM8_CH3_DMA_IRQ           (sig_DMA2_STREAM4)
#define SPI5_RX_ALT_DMA            (DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL7)
#define SPI5_RX_ALT_DMA_IRQ        (sig_DMA2_STREAM5)
#define SPI5_TX_ALT_DMA            (DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL7)
#define SPI5_TX_ALT_DMA_IRQ        (sig_DMA2_STREAM6)
#define TIM8_CH4_TRIG_COM_DMA      (DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL7)
#define TIM8_CH4_TRIG_COM_DMA_IRQ  (sig_DMA2_STREAM7)

/*!
 * \brief STM32 F2/F4 DMA Control Flags.
 */
#define DMA_EN           DMA_SxCR_EN
#define DMA_TEIE         DMA_SxCR_TEIE
#define DMA_HTIE         DMA_SxCR_HTIE
#define DMA_TCIE         DMA_SxCR_TCIE
#define DMA_P2M          0
#define DMA_M2P          DMA_SxCR_DIR_0
#define DMA_M2M          DMA_SxCR_DIR_2
#define DMA_MINC         DMA_SxCR_MINC
#define DMA_PINC         DMA_SxCR_PINC
#define DMA_CIRC         DMA_SxCR_CIRC
#define DMA_MSIZE_8      0
#define DMA_MSIZE_16     DMA_SxCR_MSIZE_0
#define DMA_MSIZE_32     DMA_SxCR_MSIZE_1
#define DMA_PSIZE_8      0
#define DMA_PSIZE_16     DMA_SxCR_PSIZE_0
#define DMA_PSIZE_32     DMA_SxCR_PSIZE_1
#define DMA_PRIO_LOW     0
#define DMA_PRIO_MEDIUM  DMA_SxCR_PL_0
#define DMA_PRIO_HIGH    DMA_SxCR_PL_1
#define DMA_PRIO_HIGHEST DMA_SxCR_PL
#define DMA_MEM2MEN      DMA_SxCR_MEM2MEM

/*!
 * \brief STM32 F2/F4 DMA Status and Interrupt Flags.
 */
#define DMA_TEIF DMA_LISR_TEIF0  /*< Channel x Transfer Error Flag */
#define DMA_HTIF DMA_LISR_HTIF0  /*< Channel x Half Transfer Complete Flag */
#define DMA_TCIF DMA_LISR_TCIF0  /*< Channel x Transfer Complete Flag */

#define DMA_GIF  (DMA_TEIF | DMA_HTIF | DMA_TCIF | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0 )

#define DMA_FLAGMASK DMA_GIF
#define DMA_IRQMASK  DMA_GIF

#define DMA_SIGNAL  IRQ_HANDLER
#define DmaEnableHandler(signal, ch)  NutIrqEnable(signal)
#define DmaDisableHandler(signal, ch) NutIrqDisable(signal)
#define DmaChannelSelection(ch, csel)
#endif
