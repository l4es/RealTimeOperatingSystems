/*
 * Copyright (C) 2013-17 Uwe Bonnes
 *                             (bon@elektron.ikp.physik.tu-darmstadty.de).
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

/*
 * $Log$
 * Revision 1.0  2011/12/09 17:30:10  ulrichprinz
 * Initial version.
 *
 */

#include <cfg/arch.h>
#include <cfg/devices.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>

#ifndef NUT_IRQPRI_DEF
#define NUT_IRQPRI_DEF  4
#endif
/* sig_INTERRUPTX needs special handling for level sensivity and multiple
   occupancy*/

/* We can't use XXX_IRQn to check for availability, as XXX_IRQn is an enum
   not usable for the preprocessor.
   - If the Device is available on all devices in a family but not for all
     families, for the device specific structure (e.g. USART6).
   - If the device is only available on some F1 devices, create a configurator
     item with the HW_XXX_STM32 value.
*/
#if defined(MCU_STM32F0) || defined(MCU_STM32L0)
CREATE_HANDLER(INTERRUPT0_1,  EXTI0_1,     NUT_IRQPRI_DEF); /* EXTI Line 0 and 1 */
CREATE_HANDLER(INTERRUPT2_3,  EXTI2_3,     NUT_IRQPRI_DEF); /* EXTI Line 2 and 3 */
CREATE_HANDLER(INTERRUPT4_15, EXTI4_15,    NUT_IRQPRI_DEF); /* EXTI Line 4 to 15 */
#else
CREATE_HANDLER(INTERRUPT0,    EXTI0,       NUT_IRQPRI_DEF); /* EXTI Line 0 */
CREATE_HANDLER(INTERRUPT1,    EXTI1,       NUT_IRQPRI_DEF); /* EXTI Line 1 */
#if defined(MCU_STM32F3)
CREATE_HANDLER(INTERRUPT2,    EXTI2_TSC,   NUT_IRQPRI_DEF); /* EXTI Line 2 and Touch controller*/
#else
CREATE_HANDLER(INTERRUPT2,    EXTI2,       NUT_IRQPRI_DEF); /* EXTI Line 2 */
#endif
CREATE_HANDLER(INTERRUPT3,    EXTI3,       NUT_IRQPRI_DEF); /* EXTI Line 3 */
CREATE_HANDLER(INTERRUPT4,    EXTI4,       NUT_IRQPRI_DEF); /* EXTI Line 4 */
CREATE_HANDLER(INTERRUPT9_5,  EXTI9_5,     NUT_IRQPRI_DEF); /* EXTI Line 5 to 9 */
CREATE_HANDLER(INTERRUPT15_10,EXTI15_10,   NUT_IRQPRI_DEF); /* EXTI Line 10 to 15 */
#endif
#if defined (HW_PVD_STM32)
CREATE_HANDLER(PVD,           PVD,      NUT_IRQPRI_DEF);    /* PVD through EXTI */
#endif
#if defined (HW_PVD_VDDIO2_STM32)
CREATE_HANDLER(PVD,           PVD_VDDIO2, NUT_IRQPRI_DEF);    /* PVD through EXTI */
#endif
CREATE_HANDLER(FLASH,         FLASH,    NUT_IRQPRI_DEF);    /* Flash global */
#if defined (STM32F072)
CREATE_HANDLER(RCC,           RCC_CRS,  NUT_IRQPRI_DEF);    /* RCC global */
#else
CREATE_HANDLER(RCC,           RCC,      NUT_IRQPRI_DEF);    /* RCC global */
#endif
#if defined(MCU_STM32F1) || defined(MCU_STM32F0) || defined(MCU_STM32L0)
CREATE_HANDLER(RTC,           RTC,      NUT_IRQPRI_DEF);    // Real Time Clock
#elif defined(HW_RTC_STM32_V2)
CREATE_HANDLER(RTC,           RTC_Alarm,NUT_IRQPRI_DEF);    // Real Time Clock
CREATE_HANDLER(RTC_WAKEUP,    RTC_WKUP, NUT_IRQPRI_DEF);    // Real Time Clock
#endif

#if defined(HW_LCD_STM32)
CREATE_HANDLER(LCD,           LCD,      NUT_IRQPRI_DEF);
#endif

CREATE_HANDLER(SPI1,          SPI1,     NUT_IRQPRI_DEF);     // SPI 1 Controller
#if defined(HW_SPI2_STM32)
CREATE_HANDLER(SPI2,          SPI2,     NUT_IRQPRI_DEF);     // SPI 2 Controller
#endif
#if defined(HW_SPI3_STM32)
CREATE_HANDLER(SPI3,          SPI3,     NUT_IRQPRI_DEF);     // SPI 3 Controller
#endif
#if defined(HW_SPI4_STM32)
CREATE_HANDLER(SPI4,          SPI4,     NUT_IRQPRI_DEF);     // SPI 4 Controller
#endif
#if defined(HW_SPI5_STM32)
CREATE_HANDLER(SPI5,          SPI5,     NUT_IRQPRI_DEF);     // SPI 5 Controller
#endif
#if defined(HW_SPI6_STM32)
CREATE_HANDLER(SPI6,          SPI6,     NUT_IRQPRI_DEF);     // SPI 6 Controller
#endif

#if defined(HW_SDIO_STM32)
CREATE_HANDLER(SDIO,          SDIO,     NUT_IRQPRI_DEF);     // SDIO Controller
#endif

#if defined(HW_SAI1_STM32)
CREATE_HANDLER(SAI1,          SAI1,     NUT_IRQPRI_DEF);     // SPI 2 Controller
#endif

#if defined (MCU_STM32L0) || defined (MCU_STM32F0)
CREATE_HANDLER(TWI1_EV,       I2C1,     NUT_IRQPRI_DEF);     // I2C 1 Data/Event
#else
CREATE_HANDLER(TWI1_EV,       I2C1_EV,  NUT_IRQPRI_DEF);     // I2C 1 Data/Event
CREATE_HANDLER(TWI1_ER,       I2C1_ER,  NUT_IRQPRI_DEF);     // I2C 2 Data/Event
#endif

#if defined(HW_I2C2_STM32)
#if defined (MCU_STM32L0) || defined (MCU_STM32F0)
CREATE_HANDLER(TWI2_EV,       I2C2,     NUT_IRQPRI_DEF);     // I2C 1 Data/Event
#else
CREATE_HANDLER(TWI2_EV,       I2C2_EV,  NUT_IRQPRI_DEF);     // I2C 2 Data/Event
CREATE_HANDLER(TWI2_ER,       I2C2_ER,  NUT_IRQPRI_DEF);     // I2C 1 Error
#endif
#endif
#if defined(HW_I2C3_STM32)
# if defined (MCU_STM32L0) || defined (MCU_STM32F0)
CREATE_HANDLER(TWI3_EV,      I2C3,     NUT_IRQPRI_DEF);     // I2C 1 Data/Event
# else
CREATE_HANDLER(TWI3_EV,       I2C3_EV,  NUT_IRQPRI_DEF);     // I2C 2 Error
CREATE_HANDLER(TWI3_ER,       I2C3_ER,  NUT_IRQPRI_DEF);     // I2C 2 Error
# endif
#endif
#if defined(HW_USB_CAN1_STM32)
CREATE_HANDLER(CAN1_TX,       USB_HP_CAN1_TX,  NUT_IRQPRI_DEF);  // CAN 1 TX
CREATE_HANDLER(CAN1_RX0,      USB_LP_CAN1_RX0, NUT_IRQPRI_DEF);  // CAN 1 RX0
CREATE_HANDLER(CAN1_RX1,      CAN1_RX1,        NUT_IRQPRI_DEF);  // CAN 1 RX1
CREATE_HANDLER(CAN1_SCE,      CAN1_SCE,        NUT_IRQPRI_DEF);  // CAN 1 SCE
#elif defined(HW_USB_CAN_STM32)
CREATE_HANDLER(CAN1_TX,       USB_HP_CAN_TX,   NUT_IRQPRI_DEF);  // CAN 1 TX
CREATE_HANDLER(CAN1_RX0,      USB_LP_CAN_RX0,  NUT_IRQPRI_DEF);  // CAN 1 RX0
CREATE_HANDLER(CAN1_RX1,      CAN_RX1,         NUT_IRQPRI_DEF);  // CAN 1 RX1
CREATE_HANDLER(CAN1_SCE,      CAN_SCE,         NUT_IRQPRI_DEF);  // CAN 1 SCE
#elif defined(HW_CAN1_STM32)
CREATE_HANDLER(CAN1_TX,       CAN1_TX,         NUT_IRQPRI_DEF);  // CAN 1 TX
CREATE_HANDLER(CAN1_RX0,      CAN1_RX0,        NUT_IRQPRI_DEF);  // CAN 1 RX0
CREATE_HANDLER(CAN1_RX1,      CAN1_RX1,        NUT_IRQPRI_DEF);  // CAN 1 RX1
CREATE_HANDLER(CAN1_SCE,      CAN1_SCE,        NUT_IRQPRI_DEF);  // CAN 1 SCE
#elif defined(HW_CAN_STM32)
CREATE_HANDLER(CAN1_TX,       CAN_TX,          NUT_IRQPRI_DEF);  // CAN 1 TX
CREATE_HANDLER(CAN1_RX0,      CAN_RX0,         NUT_IRQPRI_DEF);  // CAN 1 RX0
CREATE_HANDLER(CAN1_RX1,      CAN_RX1,         NUT_IRQPRI_DEF);  // CAN 1 RX1
CREATE_HANDLER(CAN1_SCE,      CAN_SCE,         NUT_IRQPRI_DEF);  // CAN 1 SCE
#endif
#if defined(HW_CAN2_STM32)
CREATE_HANDLER(CAN2_TX,       CAN2_TX,         NUT_IRQPRI_DEF);  // CAN 2 TX
CREATE_HANDLER(CAN2_RX0,      CAN2_RX0,        NUT_IRQPRI_DEF);  // CAN 2 RX0
CREATE_HANDLER(CAN2_RX1,      CAN2_RX1,        NUT_IRQPRI_DEF);  // CAN 2 RX1
CREATE_HANDLER(CAN2_SCE,      CAN2_SCE,        NUT_IRQPRI_DEF);  // CAN 2 SCE
#endif
#if defined(HW_USB_STM32)
# if defined(MCU_STM32L0) || defined(MCU_STM32F0) || defined(MCU_STM32L4)
CREATE_HANDLER(USB,           USB,             NUT_IRQPRI_DEF);  // USB High Priority, separted
#else
CREATE_HANDLER(USB_HP,        USB_HP,          NUT_IRQPRI_DEF);  // USB High Priority, separted
CREATE_HANDLER(USB_LP,        USB_LP,          NUT_IRQPRI_DEF);  // USB Low Priority, separted
#endif
# if defined(MCU_STM32L0) || defined(MCU_STM32F0) || defined(MCU_STM32L4)
#elif defined(MCU_STM32F30)
CREATE_HANDLER(USB_WAKE,      USBWakeUp_RMP,   NUT_IRQPRI_DEF);  // USB Wake Priority, separted
#elif defined(MCU_STM32L1)
CREATE_HANDLER(USB_WAKE,      USB_FS_WKUP,   NUT_IRQPRI_DEF);  // USB Wake Priority, separted
#else
CREATE_HANDLER(USB_WAKE,      USBWakeUp,   NUT_IRQPRI_DEF);  // USB Wake Priority, separted
#endif
#endif

#if defined(HW_USART1_STM32)
CREATE_HANDLER(USART1,        USART1,          NUT_IRQPRI_DEF);  // USART 1
#endif
#if defined(HW_USART2_STM32)
CREATE_HANDLER(USART2,        USART2,          NUT_IRQPRI_DEF);  // USART 2
#endif
#if defined(HW_USART3_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART3,        USART3,          NUT_IRQPRI_DEF);  // USART 3
#endif
#if defined(HW_UART4_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART4,        UART4,           NUT_IRQPRI_DEF);  // UART 4
#endif
#if defined(HW_USART4_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART4,        USART4,          NUT_IRQPRI_DEF);  // USART 4
#endif
#if defined(HW_UART5_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART5,        UART5,           NUT_IRQPRI_DEF);  // UART 5
#endif
#if defined(HW_USART5_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART5,        USART5,          NUT_IRQPRI_DEF);  // USART 5
#endif
#if defined(HW_USART6_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART6,        USART6,          NUT_IRQPRI_DEF);  // USART 6
#endif
#if defined(HW_UART7_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART7,        UART7,          NUT_IRQPRI_DEF);  // UART 7
#endif
#if defined(HW_UART8_STM32) && !defined(HW_USART_COMBINED_IRQ_STM32)
CREATE_HANDLER(USART8,        UART8,          NUT_IRQPRI_DEF);  // UART 8
#endif

#if   defined(HW_USART3_8_STM32)
CREATE_HANDLER(USART_GROUP,   USART3_8,        NUT_IRQPRI_DEF);  // USART3_8
# elif defined(HW_USART3_6_STM32)
CREATE_HANDLER(USART_GROUP,   USART3_6,        NUT_IRQPRI_DEF);  // USART3_6
# elif defined(HW_USART3_4_STM32)
CREATE_HANDLER(USART_GROUP,   USART3_4,        NUT_IRQPRI_DEF);  // USART3_4
# endif
#if defined(HW_USART4_5_STM32)
CREATE_HANDLER(USART4_5,      USART4_5,        NUT_IRQPRI_DEF);  // UART 4/5 combined
#endif
#if defined(HW_LPUART1_STM32) && !defined(HW_LPUART1_COMBINED_IRQ_STM32)
CREATE_HANDLER(LPUART1,       LPUART1,        NUT_IRQPRI_DEF);  // LPUART1
#endif
#if defined(HW_RNG_LPUART1_STM32)
CREATE_HANDLER(LPUART1_GROUP, RNG_LPUART1,    NUT_IRQPRI_DEF);  // LPUART1/RNG
#elif defined(HW_AES_LPUART1_STM32)
CREATE_HANDLER(LPUART1_GROUP, AES_LPUART1,    NUT_IRQPRI_DEF);  // LPUART1/RNG
#elif defined(HW_AES_RNG_LPUART1_STM32)
CREATE_HANDLER(LPUART1_GROUP, AES_RNG_LPUART1, NUT_IRQPRI_DEF);  // LPUART1/RNG
#endif

#if defined (MCU_STM32L0)
CREATE_HANDLER(DMA1_CH1,   DMA1_Channel1      ,   NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA_GROUP1, DMA1_Channel2_3    ,   NUT_IRQPRI_DEF); // DMA Controller 1 Group 1
CREATE_HANDLER(DMA_GROUP2, DMA1_Channel4_5_6_7,   NUT_IRQPRI_DEF); // DMA Controller 1 Group 2
#elif defined (MCU_STM32F0)
# if defined(HW_DMA2_STM32F1)
CREATE_HANDLER(DMA1_CH1,   DMA1_Channel1      ,   NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA_GROUP1, DMA1_Ch2_3_DMA2_Ch1_2, NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA_GROUP2, DMA1_Ch4_7_DMA2_Ch3_5, NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
# elif defined(HW_DMA1_5CH_STM32)
CREATE_HANDLER(DMA1_CH1,   DMA1_Channel1      ,   NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA_GROUP1, DMA1_Channel2_3    ,   NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA_GROUP2, DMA1_Channel4_5    ,   NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
# else
CREATE_HANDLER(DMA1_CH1,   DMA1_Channel1      ,   NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA_GROUP1, DMA1_Channel2_3,       NUT_IRQPRI_DEF); // DMA Controller 1 Group 1
CREATE_HANDLER(DMA_GROUP2, DMA1_Channel4_5_6_7,   NUT_IRQPRI_DEF); // DMA Controller 1 Group 2
# endif
#elif defined (MCU_STM32F1) || defined(MCU_STM32L1) || defined(MCU_STM32F3) || defined(MCU_STM32L4)
CREATE_HANDLER(DMA1_CH1,   DMA1_Channel1,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA1_CH2,   DMA1_Channel2,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA1_CH3,   DMA1_Channel3,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA1_CH4,   DMA1_Channel4,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA1_CH5,   DMA1_Channel5,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA1_CH6,   DMA1_Channel6,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA1_CH7,   DMA1_Channel7,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
# if defined(HW_DMA2_STM32F1)
CREATE_HANDLER(DMA2_CH1,   DMA2_Channel1,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA2_CH2,   DMA2_Channel2,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA2_CH3,   DMA2_Channel3,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
#  if defined(HW_DMA_COMBINED_IRQ_STM32)
CREATE_HANDLER(DMA2_CH4,   DMA2_Channel4_5,       NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
#  else
CREATE_HANDLER(DMA2_CH4,   DMA2_Channel4,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
CREATE_HANDLER(DMA2_CH5,   DMA2_Channel5,         NUT_IRQPRI_DEF); // DMA Controller 1 Channel 1
#  endif
#  if defined(HW_DMA2_7CH_STM32)
CREATE_HANDLER(DMA2_CH6,   DMA2_Channel6,         NUT_IRQPRI_DEF); // DMA Controller 2 Channel 6
CREATE_HANDLER(DMA2_CH7,   DMA2_Channel7,         NUT_IRQPRI_DEF); // DMA Controller 2 Channel 7
#  endif
# endif
#else
/* STM32F2, DTM32F4 */
CREATE_HANDLER(DMA1_STREAM0, DMA1_Stream0,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 1
CREATE_HANDLER(DMA1_STREAM1, DMA1_Stream1,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 1
CREATE_HANDLER(DMA1_STREAM2, DMA1_Stream2,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 2
CREATE_HANDLER(DMA1_STREAM3, DMA1_Stream3,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 3
CREATE_HANDLER(DMA1_STREAM4, DMA1_Stream4,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 4
CREATE_HANDLER(DMA1_STREAM5, DMA1_Stream5,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 5
CREATE_HANDLER(DMA1_STREAM6, DMA1_Stream6,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 6
CREATE_HANDLER(DMA1_STREAM7, DMA1_Stream7,    NUT_IRQPRI_DEF); // DMA Controller 0 Stream 7
CREATE_HANDLER(DMA2_STREAM0, DMA2_Stream0,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 1
CREATE_HANDLER(DMA2_STREAM1, DMA2_Stream1,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 2
CREATE_HANDLER(DMA2_STREAM2, DMA2_Stream2,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 3
CREATE_HANDLER(DMA2_STREAM3, DMA2_Stream3,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 4
CREATE_HANDLER(DMA2_STREAM4, DMA2_Stream4,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 5
CREATE_HANDLER(DMA2_STREAM5, DMA2_Stream5,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 6
CREATE_HANDLER(DMA2_STREAM6, DMA2_Stream6,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 7
CREATE_HANDLER(DMA2_STREAM7, DMA2_Stream7,    NUT_IRQPRI_DEF); // DMA Controller 1 Stream 7
#endif

#if defined(HW_EMAC_STM32)
CREATE_HANDLER(EMAC,          ETH,          NUT_IRQPRI_DEF);     // Ethernet global interrupt
CREATE_HANDLER(EMAC_WAKE,     ETH_WKUP,     NUT_IRQPRI_DEF);     // Ethernet global interrupt
#endif

#if defined(MCU_STM32F0)
CREATE_HANDLER(TIM1_BRK_UP_TRG_COM,      TIM1_BRK_UP_TRG_COM,     NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_CC,       TIM1_CC,      NUT_IRQPRI_DEF);
#elif defined(MCU_STM32F30) || defined(STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined(STM32F10X_HD_VL) || defined(MCU_STM32L4)
CREATE_HANDLER(TIM1_BRK_TIM15,      TIM1_BRK_TIM15,     NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_UP_TIM16,       TIM1_UP_TIM16,      NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_TRG_COM_TIM17,  TIM1_TRG_COM_TIM17, NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_CC,             TIM1_CC,            NUT_IRQPRI_DEF);
#elif defined(MCU_STM32F4) || defined (MCU_STM32F2)
CREATE_HANDLER(TIM1_BRK_TIM9,       TIM1_BRK_TIM9,      NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_UP_TIM10,       TIM1_UP_TIM10,      NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_TRG_COM_TIM11,  TIM1_TRG_COM_TIM11, NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_CC,             TIM1_CC,            NUT_IRQPRI_DEF);
#elif defined(HW_TIM1_STM32)
CREATE_HANDLER(TIM1_BRK,      TIM1_BRK,     NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_UP,       TIM1_UP,      NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_TRG_COM,  TIM1_TRG_COM, NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM1_CC,       TIM1_CC,      NUT_IRQPRI_DEF);
#endif

/* Only Tim2 up to TIM5 have always a single TIMx_IRQn*/
#if defined(HW_TIM2_STM32)
CREATE_HANDLER(TIM2,          TIM2,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM3_STM32)
CREATE_HANDLER(TIM3,          TIM3,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM4_STM32)
CREATE_HANDLER(TIM4,          TIM4,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM5_STM32)
CREATE_HANDLER(TIM5,          TIM5,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM6_STM32)
CREATE_HANDLER(TIM6,          TIM6,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM6_DAC_STM32)
CREATE_HANDLER(TIM6_DAC,      TIM6_DAC,     NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM6_DAC1_STM32)
CREATE_HANDLER(TIM6_DAC1,     TIM6_DAC1,    NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM7_STM32)
CREATE_HANDLER(TIM7,          TIM7,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM7_DAC2_STM32)
CREATE_HANDLER(TIM7_DAC2,     TIM7_DAC2,    NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM8_STM32)
CREATE_HANDLER(TIM8_BRK,      TIM8_BRK,     NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM8_UP,       TIM8_UP,      NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM8_TRG_COM,  TIM8_TRG_COM, NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM8_CC,       TIM8_CC,     NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM8_TIM12_TIM13_TIM14_STM32)
CREATE_HANDLER(TIM8_BRK_TIM12,    TIM8_BRK_TIM12,    NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM8_UP_TIM13,     TIM8_UP_TIM13,     NUT_IRQPRI_DEF);
CREATE_HANDLER(TIM8_TRG_COM_TIM1, TIM8_TRG_COM_TIM1, NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM10_STM32)
CREATE_HANDLER(TIM10,          TIM10,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM11_STM32)
CREATE_HANDLER(TIM11,          TIM11,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM12_STM32)
CREATE_HANDLER(TIM12,          TIM12,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM13_STM32)
CREATE_HANDLER(TIM13,          TIM13,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM14_STM32)
CREATE_HANDLER(TIM14,          TIM14,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM15_STM32)
CREATE_HANDLER(TIM15,          TIM15,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM16_STM32)
CREATE_HANDLER(TIM16,          TIM16,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM17_STM32)
CREATE_HANDLER(TIM17,          TIM17,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_TIM19_STM32)
CREATE_HANDLER(TIM19,          TIM19,         NUT_IRQPRI_DEF);
#endif

#if defined(HW_ADC_STM32)
CREATE_HANDLER(ADC,            ADC,           NUT_IRQPRI_DEF);
#endif
#if defined(HW_ADC1_STM32) || defined(HW_ADC1_STM32F1)
CREATE_HANDLER(ADC1,           ADC1,           NUT_IRQPRI_DEF);
#endif
#if defined(HW_ADC1_2_STM32) || defined(HW_ADC1_2_STM32F1)
CREATE_HANDLER(ADC1_2,         ADC1_2,           NUT_IRQPRI_DEF);
#endif
#if defined(HW_ADC3_STM32) || defined(HW_ADC3_STM32F1)
CREATE_HANDLER(ADC3,           ADC3,           NUT_IRQPRI_DEF);
#endif
#if defined(HW_ADC4_STM32)
CREATE_HANDLER(ADC4,           ADC4,           NUT_IRQPRI_DEF);
#endif

#if defined(HW_SDADC1_STM32)
CREATE_HANDLER(SDADC1,         SDADC1,       NUT_IRQPRI_DEF);
#endif
#if defined(HW_SDADC2_STM32)
CREATE_HANDLER(SDADC2,         SDADC2,       NUT_IRQPRI_DEF);
#endif
#if defined(HW_SDADC3_STM32)
CREATE_HANDLER(SDADC3,         SDADC3,       NUT_IRQPRI_DEF);
#endif

#if defined(HW_HASH_STM32) && !defined(HW_HASH_COMBINED_IRQ_STM32)
CREATE_HANDLER(HASH,            HASH,         NUT_IRQPRI_DEF);
#endif

#if defined(HW_HASH_RNG_STM32)
CREATE_HANDLER(HASH_RNG,        HASH_RNG,     NUT_IRQPRI_DEF);
#endif
#if defined(HW_CRYP_STM32)
CREATE_HANDLER(CRYP,            CRYP,         NUT_IRQPRI_DEF);
#endif

#if defined(HW_DCMI_STM32)
CREATE_HANDLER(DCMI,            DCMI,         NUT_IRQPRI_DEF);
#endif

#if defined(HW_DMA2D_STM32)
CREATE_HANDLER(DMA2D,           DMA2D,        NUT_IRQPRI_DEF);
#endif

#if defined(HW_LTDC_STM32)
CREATE_HANDLER(LTDC,            LTDC,         NUT_IRQPRI_DEF);
#endif

#if defined(HW_FSMC_STM32)
CREATE_HANDLER(FSMC,            FSMC,         NUT_IRQPRI_DEF);
#endif

#if defined(HW_FMC_STM32)
CREATE_HANDLER(FMC,             FMC,          NUT_IRQPRI_DEF);
#endif

#if defined(HW_COMP1_2_3_STM32)
CREATE_HANDLER(COMP1_2_3,       COMP1_2_3,    NUT_IRQPRI_DEF);
#endif
#if defined(HW_COMP4_5_6_STM32)
CREATE_HANDLER(COMP4_5_6,       COMP4_5_6,    NUT_IRQPRI_DEF);
#endif
#if defined(HW_COMP_STM32)
CREATE_HANDLER(COMP,            COMP,         NUT_IRQPRI_DEF);
#endif
#if defined(HW_COMP_ACQ_STM32)
CREATE_HANDLER(COMP_ACQ,        COMP_ACQ,     NUT_IRQPRI_DEF);
#endif

#if defined(HW_CEC_STM32)
CREATE_HANDLER(CEC,             CEC,          NUT_IRQPRI_DEF);
#endif

#if defined( __FPU_PRESENT)
CREATE_HANDLER(FPU,             FPU,          NUT_IRQPRI_DEF);
#endif

#if !defined(HW_USART_COMBINED_IRQ_STM32)
IRQ_HANDLER *Stm32UsartInstallHandler(int nr, IRQ_HANDLER *sig)
{
    NutIrqEnable(sig);
    return sig;
}
#endif

#if defined(HW_RNG_STM32) && !defined(HW_RNG_COMBINED_IRQ_STM32)
CREATE_HANDLER(RNG,             RNG,          NUT_IRQPRI_DEF);
IRQ_HANDLER *Stm32RngInstallHandler(IRQ_HANDLER *sig)
{
    NutIrqEnable(sig);
    return sig;
}
#endif
#if defined(HW_AES_STM32) && !defined(HW_AES_COMBINED_IRQ_STM32)
CREATE_HANDLER(AES,       AES,        NUT_IRQPRI_DEF);  // AES
IRQ_HANDLER *Stm32AesInstallHandler(IRQ_HANDLER *sig)
{
    NutIrqEnable(sig);
    return sig;
}
#endif
