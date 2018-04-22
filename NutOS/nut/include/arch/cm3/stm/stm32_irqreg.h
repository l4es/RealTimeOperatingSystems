#ifndef _DEV_IRQREG_ARCH_CM3_STM32_H_
#define _DEV_IRQREG_ARCH_CM3_STM32_H_

/*
 * Copyright (C) 2001-2007 by egnite Software GmbH. All rights reserved.
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
 * \verbatim
 * $Id: $
 * \endverbatim
 */

extern IRQ_HANDLER sig_INTERRUPT0;      // EXTI 0
extern IRQ_HANDLER sig_INTERRUPT1;      // EXTI 1
extern IRQ_HANDLER sig_INTERRUPT2;      // EXTI 2
extern IRQ_HANDLER sig_INTERRUPT3;      // EXTI 3
extern IRQ_HANDLER sig_INTERRUPT4;      // EXTI 4
extern IRQ_HANDLER sig_INTERRUPT9_5;    // EXTI 9_5
extern IRQ_HANDLER sig_INTERRUPT15_10;  // EXTI 10 to 15
extern IRQ_HANDLER sig_INTERRUPT0_1;    // EXTI 0 to 1 on F0/L0
extern IRQ_HANDLER sig_INTERRUPT2_3;    // EXTI 2 to 3 on F0/L0
extern IRQ_HANDLER sig_INTERRUPT4_15;   // EXTI 4 to 15 on F0/L0
extern IRQ_HANDLER sig_LCD;         // LCD Display
extern IRQ_HANDLER sig_RTC;         // Real Time Clock
extern IRQ_HANDLER sig_RCC;         // Reset and Clock setup
extern IRQ_HANDLER sig_RTC_WAKEUP;  // Real Time Clock periodic Wake
extern IRQ_HANDLER sig_SDIO;        // SDIO Controller
extern IRQ_HANDLER sig_SPI1;        // SPI 1 Controller
extern IRQ_HANDLER sig_SPI2;        // SPI 2 Controller
extern IRQ_HANDLER sig_SPI3;        // SPI 2 Controller
extern IRQ_HANDLER sig_SPI4;        // SPI 2 Controller
extern IRQ_HANDLER sig_SPI5;        // SPI 2 Controller
extern IRQ_HANDLER sig_SPI6;        // SPI 2 Controller
extern IRQ_HANDLER sig_TWI1_EV;     // I2C 1 Data/Event
extern IRQ_HANDLER sig_TWI2_EV;     // I2C 2 Data/Event
extern IRQ_HANDLER sig_TWI3_EV;     // I2C 2 Data/Event
extern IRQ_HANDLER sig_TWI1_ER;     // I2C 1 Error
extern IRQ_HANDLER sig_TWI2_ER;     // I2C 2 Error
extern IRQ_HANDLER sig_TWI3_ER;     // I2C 2 Error
extern IRQ_HANDLER sig_CAN1_TX;     // CAN 1 TX
extern IRQ_HANDLER sig_CAN1_RX0;    // CAN 1 RX0
extern IRQ_HANDLER sig_CAN1_RX1;    // CAN 1 RX1
extern IRQ_HANDLER sig_CAN1_SCE;    // CAN 1 SCE
extern IRQ_HANDLER sig_CAN2_TX;     // CAN 2 TX
extern IRQ_HANDLER sig_CAN2_RX0;    // CAN 2 RX0
extern IRQ_HANDLER sig_CAN2_RX1;    // CAN 2 RX1
extern IRQ_HANDLER sig_CAN2_SCE;    // CAN 2 SCE
extern IRQ_HANDLER sig_USART1;      // USART 1
extern IRQ_HANDLER sig_USART2;      // USART 2
extern IRQ_HANDLER sig_USART3;      // USART 3
extern IRQ_HANDLER sig_USART4;      // USART 4
extern IRQ_HANDLER sig_USART5;      // USART 5
extern IRQ_HANDLER sig_USART4_5;    // Combined USART4-5 on L0 Cat.5
extern IRQ_HANDLER sig_USART6;      // USART 6
extern IRQ_HANDLER sig_USART7;      // USART 7
extern IRQ_HANDLER sig_USART8;      // USART 8
extern IRQ_HANDLER sig_LPUART1;     // LPUART 1
extern IRQ_HANDLER sig_USART_GROUP; // Combined LPUART1/RNG/AES on L0
extern IRQ_HANDLER sig_LPUART1_GROUP; // Combined USART3-[4/6/8] on F0
extern IRQ_HANDLER sig_OTG_FS;      // USB OTG on F2/F4
/* On F30, USB interrupt is remapped from CAN on startup. On F1, beside
   communication line devices, USB and CAN can't be used together*/
extern IRQ_HANDLER sig_USB_HP;      // USB FS High Priority
extern IRQ_HANDLER sig_USB_LP;      // USB FS Low Priority
extern IRQ_HANDLER sig_USB_Wake;    // USB FS Wakeup
extern IRQ_HANDLER sig_DMA_GROUP1;  // F0 DMA Combined Group 1
extern IRQ_HANDLER sig_DMA_GROUP2;  // F0 DMA Combined Group 2
extern IRQ_HANDLER sig_DMA1_CH1;    // DMA Controller 1 Channel 2
extern IRQ_HANDLER sig_DMA1_CH2;    // DMA Controller 1 Channel 2
extern IRQ_HANDLER sig_DMA1_CH2;    // DMA Controller 1 Channel 2
extern IRQ_HANDLER sig_DMA1_CH2;    // DMA Controller 1 Channel 2
extern IRQ_HANDLER sig_DMA1_CH3;    // DMA Controller 1 Channel 3
extern IRQ_HANDLER sig_DMA1_CH4;    // DMA Controller 1 Channel 4
extern IRQ_HANDLER sig_DMA1_CH5;    // DMA Controller 1 Channel 5
extern IRQ_HANDLER sig_DMA1_CH6;    // DMA Controller 1 Channel 6
extern IRQ_HANDLER sig_DMA1_CH7;    // DMA Controller 1 Channel 7
extern IRQ_HANDLER sig_DMA2_CH1;    // DMA Controller 2 Channel 1
extern IRQ_HANDLER sig_DMA2_CH2;    // DMA Controller 2 Channel 2
extern IRQ_HANDLER sig_DMA2_CH3;    // DMA Controller 2 Channel 3
extern IRQ_HANDLER sig_DMA2_CH4;    // DMA Controller 2 Channel 4
extern IRQ_HANDLER sig_DMA2_CH5;    // DMA Controller 2 Channel 5
extern IRQ_HANDLER sig_DMA2_CH6;    // DMA Controller 2 Channel 6
extern IRQ_HANDLER sig_DMA2_CH7;    // DMA Controller 2 Channel 7

extern IRQ_HANDLER sig_DMA1_STREAM0;    // DMA Controller 1 Stream 0
extern IRQ_HANDLER sig_DMA1_STREAM1;    // DMA Controller 1 Stream 1
extern IRQ_HANDLER sig_DMA1_STREAM2;    // DMA Controller 1 Stream 2
extern IRQ_HANDLER sig_DMA1_STREAM3;    // DMA Controller 1 Stream 3
extern IRQ_HANDLER sig_DMA1_STREAM4;    // DMA Controller 1 Stream 4
extern IRQ_HANDLER sig_DMA1_STREAM5;    // DMA Controller 1 Stream 5
extern IRQ_HANDLER sig_DMA1_STREAM6;    // DMA Controller 1 Stream 6
extern IRQ_HANDLER sig_DMA1_STREAM7;    // DMA Controller 1 Stream 7
extern IRQ_HANDLER sig_DMA2_STREAM0;    // DMA Controller 2 Stream 0
extern IRQ_HANDLER sig_DMA2_STREAM1;    // DMA Controller 2 Stream 1
extern IRQ_HANDLER sig_DMA2_STREAM2;    // DMA Controller 2 Stream 2
extern IRQ_HANDLER sig_DMA2_STREAM3;    // DMA Controller 2 Stream 3
extern IRQ_HANDLER sig_DMA2_STREAM4;    // DMA Controller 2 Stream 4
extern IRQ_HANDLER sig_DMA2_STREAM5;    // DMA Controller 2 Stream 5
extern IRQ_HANDLER sig_DMA2_STREAM6;    // DMA Controller 2 Stream 6
extern IRQ_HANDLER sig_DMA2_STREAM7;    // DMA Controller 2 Stream 7

extern IRQ_HANDLER sig_EMAC;        // Ethernet global interrupt

extern IRQ_HANDLER sig_TIM1_BRK;
extern IRQ_HANDLER sig_TIM1_BRK_UP_TRG_COM;
extern IRQ_HANDLER sig_TIM1_BRK_TIM9;
extern IRQ_HANDLER sig_TIM1_BRK_TIM15;
extern IRQ_HANDLER sig_TIM1_CC;
extern IRQ_HANDLER sig_TIM1_TRG_COM;
extern IRQ_HANDLER sig_TIM1_TRG_COM_TIM11;
extern IRQ_HANDLER sig_TIM1_TRG_COM_TIM17;
extern IRQ_HANDLER sig_TIM1_UP_TIM16;
extern IRQ_HANDLER sig_TIM1_UP;
extern IRQ_HANDLER sig_TIM1_UP_TIM10;
extern IRQ_HANDLER sig_TIM1_UP_TIM16;
extern IRQ_HANDLER sig_TIM2;
extern IRQ_HANDLER sig_TIM3;
extern IRQ_HANDLER sig_TIM4;
extern IRQ_HANDLER sig_TIM5;
extern IRQ_HANDLER sig_TIM6;
extern IRQ_HANDLER sig_TIM6_DAC;
extern IRQ_HANDLER sig_TIM6_DAC1;
extern IRQ_HANDLER sig_TIM7;
extern IRQ_HANDLER sig_TIM7_DAC2;
extern IRQ_HANDLER sig_TIM8_BRK;
extern IRQ_HANDLER sig_TIM8_BRK_TIM12;
extern IRQ_HANDLER sig_TIM8_CC;
extern IRQ_HANDLER sig_TIM8_TRG_COM;
extern IRQ_HANDLER sig_TIM8_TRG_COM_TIM14;
extern IRQ_HANDLER sig_TIM8_UP;
extern IRQ_HANDLER sig_TIM8_UP_TIM13;
extern IRQ_HANDLER sig_TIM9;
extern IRQ_HANDLER sig_TIM10;
extern IRQ_HANDLER sig_TIM11;
extern IRQ_HANDLER sig_TIM12;
extern IRQ_HANDLER sig_TIM13;
extern IRQ_HANDLER sig_TIM14;
extern IRQ_HANDLER sig_TIM15;
extern IRQ_HANDLER sig_TIM16;
extern IRQ_HANDLER sig_TIM17;
extern IRQ_HANDLER sig_TIM18;
extern IRQ_HANDLER sig_TIM18_DAC2;
extern IRQ_HANDLER sig_TIM19;

extern IRQ_HANDLER sig_ADC1;
extern IRQ_HANDLER sig_ADC2;
extern IRQ_HANDLER sig_ADC3;
extern IRQ_HANDLER sig_ADC4;

extern IRQ_HANDLER sig_RNG;
extern IRQ_HANDLER sig_AES;
extern IRQ_HANDLER sig_HASH;

extern IRQ_HANDLER *Stm32RngInstallHandler(IRQ_HANDLER *sig);

#endif
