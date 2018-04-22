--
--  * Copyright (C) 2015 - 2017
--               Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
-- 3. Neither the name of the copyright holders nor the names of
--    contributors may be used to endorse or promote products derived
--    from this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-- COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
-- OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
-- AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
-- THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- For additional information see http://www.ethernut.de/
--

-- ARM CortexM4 Architecture
-- STMicroelectronics STM32L4 Family Devices
--
--

-- *****************************************************************************
-- STM32L4 Family
-- *****************************************************************************
--
-- rm0351: L476
-- rm0392: L471
-- rm0392: L432/L442/L452/L462
-- rm0394: L431/L422/L443
-- rm0395: L4x5

nutarch_cm3_stm32l4 =
{
    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32L4_family",
        brief = "MCU L4 Family",
        requires = { "HW_MCU_STM32L4" },
        options =
        {
            {
                macro = "MCU_STM32L4",
                brief = "STM32L4 common devices",
                description = "Provides devices common in all STM32L4 devices.",
                default = 1,
                provides =
                {
-- Todo: FLASH_WRPROT
                    "HW_VOS_STM32",
                    "HW_ICACHE_STM32",
                    "HW_DCACHE_STM32",
--                   "HW_CAN1_STM32",  TODO: Can Timing
                    "HW_RTC_STM32_V2",
                    "HW_GPIO_STM32V2",
                    "HW_DMA1_STM32F1",
                    "HW_DMA2_STM32L4",
                    "HW_EXTI04_STM32",
                    "HW_EXTI95_STM32",
                    "HW_I2C_STM32V2",
                    "HW_I2C1_STM32",
                    "HW_I2C3_STM32",
                    "HW_LPUART1_STM32",
                    "HW_RNG_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI3_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_USART_STM32V2",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L43",
                brief = "STM32L43x/44x.",
                description = "Provides common devices in L43x/L44x",
                default = 1,
                requires = {"HW_MCU_STM32L43"},
                provides =
                {
                   "HW_TIM1_TIM15_TIM16_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L45",
                brief = "STM32L45x/46x.",
                description = "Provides common devices in L45x/L46x",
                default = 1,
                requires = {"HW_MCU_STM32L45"},
                provides =
                {
                    "HW_I2C2_STM32",
                    "HW_TIM1_TIM15_TIM16_STM32",
                    "HW_TIM3_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L47",
                brief = "STM32L47x/48x.",
                description = "Provides common devices in L47x/L48x",
                default = 1,
                requires = {"HW_MCU_STM32L47"},
                provides =
                {
                    "HW_I2C2_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM5_32BIT_STM32",
                    "HW_TIM8_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L431xx",
                brief = "STM32L431 128/256 kB.",
                description = "Provides vendor header for STM32L431.",
                default = 1,
                requires = {"HW_MCU_STM32L431"},
                provides =
                {
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART3_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L432xx",
                brief = "STM32L432 128/256 kB.",
                description = "Provides vendor header for STM32L432.",
                default = 1,
                requires = {"HW_MCU_STM32L432"},
                provides =
                {
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L433xx",
                brief = "STM32L433 128/256 kB.",
                description = "Provides vendor header for STM32L433.",
                default = 1,
                requires = {"HW_MCU_STM32L433"},
                provides =
                {
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART3_STM32",
                    "HW_USB_STM32",
                    "HW_LCD_STM32",
                 },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L442xx",
                brief = "STM32L442 128/256 kB.",
                description = "Provides vendor header for STM32L442.",
                default = 1,
                requires = {"HW_MCU_STM32L442"},
                provides =
                {
                    "HW_USB_STM32",
                    "HW_AES_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L443xx",
                brief = "STM32L443 128/256 kB.",
                description = "Provides vendor header for STM32L443.",
                default = 1,
                requires = {"HW_MCU_STM32L443"},
                provides =
                {
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART3_STM32",
                    "HW_USB_STM32",
                    "HW_LCD_STM32",
                    "HW_AES_STM32",
                 },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L471xx",
                brief = "STM32L471 512/1024 kB.",
                description = "Provides vendor header for STM32L471.",
                default = 1,
                requires = {"HW_MCU_STM32L471"},
                provides =
                {
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L475xx",
                brief = "STM32L475 512/1024 kB.",
                description = "Provides vendor header for STM32L475.",
                default = 1,
                requires = {"HW_MCU_STM32L475"},
                provides =
                {
                     "HW_OTG_STM32",
               },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L475xx",
                brief = "STM32L475 256/512/1024 kB.",
                description = "Provides vendor header for STM32L475 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32L475"},
                provides =
                {
                    "HW_OTG_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L476xx",
                brief = "STM32L476xx 256/512/1024 kB.",
                description = "Provides vendor header for STM32L476 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32L476"},
                provides = {
                    "HW_OTG_STM32",
                    "HW_LCD_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L485xx",
                brief = "STM32L485 256/512/1024 kB.",
                description = "Provides vendor header for STM32L485 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32L485"},
                provides =
                {
                    "HW_AES_STM32",
                    "HW_OTG_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L486xx",
                brief = "STM32L486xx 256/512/1024 kB.",
                description = "Provides vendor header for STM32L486 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32L486"},
                provides = {
                    "HW_AES_STM32",
                    "HW_OTG_STM32",
                    "HW_LCD_STM32",
                 },
                file = "include/cfg/arch.h"
            },
        },
    },
}
