--
--  * Copyright (C) 2015
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

-- ARM CortexM3 Architecture
-- STMicroelectronics STM32L0 Family Devices
--
--

-- *****************************************************************************
-- STM32L0 Family
-- *****************************************************************************
--

nutarch_cm3_stm32l0 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32L0_family",
        brief = "MCU L0 Family",
        requires = { "HW_MCU_STM32L0" },
        options =
        {
            {
                macro = "MCU_STM32L0",
                brief = "STM32L0 common devices",
                description = "Provides devices common in all STM32L0 devices.",
                default = 1,
                provides =
                {
                    "HW_RCC_STM32L",
-- Todo: FLASH_WRPROT
                    "HW_VOS_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_GPIO_STM32L_0_1",
                    "HW_GPIO_STM32V2",
                    "HW_USART_STM32V2",
                    "HW_DMA1_STM32L0",
                    "HW_EXTI01_STM32",
                    "HW_EXTI23_STM32",
                    "HW_EXTI415_STM32",
                    "HW_ADC1_COMP_STM32",
                    "HW_I2C_STM32V2",
                    "HW_I2C1_STM32",
                    "HW_PVD_STM32",
                    "HW_SPI1_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM21_STM32",
                    "HW_USART2_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L0X1",
                brief = "STM32L0X1 Subfamily",
                description = "STM32L0 Access Subfamily",
                requires = {"HW_MCU_STM32L0X1"},
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32L0X2",
                brief = "STM32L0X2 Subfamily",
                description = "STM32L0 USB Subfamily",
                requires = {"HW_MCU_STM32L0X2"},
                default = 1,
                provides =
                {
                    "HW_CRS_STM32",
                    "HW_TSC_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM22_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32L0X3",
                brief = "STM32L0X3 Subfamily",
                description = "STM32L0 USB & LCD Subfamily",
                requires = {"HW_MCU_STM32L0X3"},
                default = 1,
                provides =
                {
                    "HW_CRS_STM32",
                    "HW_TSC_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM22_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_LCD_STM32",
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h"
            },
-- Use macro names so that ST vendor defines are produced
            {
                macro = "STM32L011xx",
                brief = "STM32L011 8/16 kB.",
                description = "Provides vendor header for STM32L011 8/16 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L011" },
                provides =
                {
                   "HW_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L021xx",
                brief = "STM32L021 8/16 kB.",
                description = "Provides vendor header for STM32L021 8/16 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L021" },
                provides =
                {
                    "HW_AES_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L031xx",
                brief = "STM32L031 16/32 kB.",
                description = "Provides vendor header for STM32L031 16/32 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L031" },
                provides =
                {
                    "HW_TIM22_STM32",
                    "HW_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L041xx",
                brief = "STM32L041 16/32 kB.",
                description = "Provides vendor header for STM32L041 16/32 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L41" },
                provides =
                {
                    "HW_TIM22_STM32",
                    "HW_AES_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L051xx",
                brief = "STM32L051 32/64 kB.",
                description = "Provides vendor header for STM32L051 32/64 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L051" },
                provides =
                {
                    "HW_TIM6_STM32",
                    "HW_TIM22_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L052xx",
                brief = "STM32L052 32/64 kB.",
                description = "Provides vendor header for STM32L052 32/64 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L052" },
                provides =
                {
                   "HW_RNG_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L053xx",
                brief = "STM32L053 32/64 kb",
                description = "Provides vendor header for STM32L053 32/64 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L053" },
                provides =
                {
                    "HW_RNG_LPUART1_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L061xx",
                brief = "STM32L061 32/64 kB.",
                description = "Provides vendor header for STM32L061 32/64 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L061" },
                provides =
                {
                    "HW_TIM6_STM32",
                    "HW_TIM22_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_AES_LPUART1_STM32",
                },
            },
            {
                macro = "STM32L062xx",
                brief = "STM32L062 32/64 kB.",
                description = "Provides vendor header for STM32L062 32/64 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L062" },
                provides =
                {
                    "HW_AES_RNG_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L063xx",
                brief = "STM32L063 32/64 kB.",
                description = "Provides vendor header for STM32L063 32/64 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L063" },
                provides =
                {
                    "HW_AES_RNG_LPUART1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L071xx",
                brief = "STM32L071 64/128/192 kb",
                description = "Provides vendor header for STM32L071 64/128/192 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L071" },
                provides =
                {
                    "HW_USART4_5_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_I2C3_STM32",
                    "HW_TIM22_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                    "HW_LPUART1_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L072xx",
                brief = "STM32L072 64/128/192 kb",
                description = "Provides vendor header for STM32L072 64/128/192 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L072" },
                provides =
                {
                    "HW_RNG_LPUART1_STM32",
                    "HW_USART4_5_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM7_STM32",
                    "HW_I2C3_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L073xx",
                brief = "STM32L073 64/128/192 kb",
                description = "Provides vendor header for STM32L073 64/128/192 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L073" },
                provides =
                {
                    "HW_USART4_5_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM7_STM32",
                    "HW_I2C3_STM32",
                    "HW_RNG_LPUART1_STM32",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L081xx",
                brief = "STM32L081 64/128/192 kb",
                description = "Provides vendor header for STM32L081 64/128/192 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L081" },
                provides =
                {
                    "HW_USART4_5_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_I2C3_STM32",
                    "HW_TIM22_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                    "HW_AES_LPUART1_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L082xx",
                brief = "STM32L082 64/128/192 kb",
                description = "Provides vendor header for STM32L082 64/128/192 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L082" },
                provides =
                {
                    "HW_USART4_5_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM7_STM32",
                    "HW_AES_RNG_LPUART1_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L083xx",
                brief = "STM32L083 64/128/192 kb",
                description = "Provides vendor header for STM32L083 64/128/192 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L083" },
                provides =
                {
                    "HW_USART4_5_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM7_STM32",
                    "HW_I2C3_STM32",
                    "HW_AES_RNG_LPUART1_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_CM_NO_BITBAND",
                brief = "No Bitbanding",
                description = "Selected family has no bitband support",
                default = 1,
                file = "include/cfg/arch.h"
            },
        }
    },
}
