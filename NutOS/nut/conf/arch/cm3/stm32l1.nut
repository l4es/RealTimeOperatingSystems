--
--  * Copyright (C) 2011-2017
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
-- STMicroelectronics STM32L Family Devices
--
--

-- *****************************************************************************
-- STM32L1 Family
-- *****************************************************************************
--

-- MCU_STM32L1XX is used to offer available devices in the configurator

nutarch_cm3_stm32l1 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32L1_family",
        brief = "MCU L1 Family",
        requires = { "HW_MCU_STM32L1" },
        options =
        {
            {
                macro = "MCU_STM32L1",
                brief = "Provides devices available in all STM32L1 devices",
                default = 1,
                provides =
                {
                    "HW_VOS_STM32",
                    "HW_RCC_STM32L",
                    "HW_GPIO_STM32L_0_1",
                    "HW_GPIO_STM32V2",
                     "HW_DMA1_STM32F1",
                    "HW_EXTI04_STM32",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_ADC1_STM32",
                    "HW_COMP_STM32",
                    "HW_DAC_STM32",
                    "HW_I2C_STM32V1",
                    "HW_I2C1_STM32",
                    "HW_I2C2_STM32",
                    "HW_PVD_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_TIM2_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM9_STM32",
                    "HW_TIM10_STM32",
                    "HW_TIM11_STM32",
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L1_CAT1",
                brief = "STM32L1 Cat1.",
                description = "Additional devices in STM32L1 Cat 1 Devices.",
                default = 1,
                requires = { "HW_MCU_STM32L1_CAT1" },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L1_CAT2",
                brief = "STM32L1 Cat2.",
                description = "Additional devices in STM32L1 Cat 1 Devices.",
                default = 1,
                requires = { "HW_MCU_STM32L1_CAT2" },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L1_CAT3",
                brief = "STM32L1 Cat3.",
                description = "Additional devices in STM32L1 Cat 3 Devices.",
                default = 1,
                requires = { "HW_MCU_STM32L1_CAT3" },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L1_CAT4",
                brief = "STM32L1 Cat4.",
                description = "Additional devices in STM32L1 Cat 4 Devices.",
                default = 1,
                requires = { "HW_MCU_STM32L1_CAT4" },
                provides = {
                   "HW_SDIO_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L1_CAT5",
                brief = "STM32L1 Cat5.",
                description = "Additional devices in STM32L1 Cat 5 Devices.",
                default = 1,
                requires = { "HW_MCU_STM32L1_CAT5" },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32L1_CAT6",
                brief = "STM32L1 Cat6.",
                description = "Additional devices in STM32L1 Cat 6 Devices.",
                default = 1,
                requires = { "HW_MCU_STM32L1_CAT6" },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L100xB",
                brief = "STM32L100 32/64/128 kB.",
                description = "Provides vendor header for STM32L100 32/64/128 kB and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L100_MD" },
                provides =
                {
                    "HW_LCD_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L100xBA",
                brief = "STM32L100A 32/64/128 kB.",
                description = "Provides vendor header for STM32L100 32/64/128 kB A-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L100_MDA" },
                provides =
                {
                     "HW_LCD_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L151xB",
                brief = "STM32L151 32/64/128 kb",
                description = "Provides vendor header for STM32L151 32/64/128 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L151_MD" },
                provides = {
                    "HW_MCU_STM32L1_MD";
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L151xBA",
                brief = "STM32L151 32/64/128 kb A",
                description = "Provides vendor header for STM32L151 32/64/128 kB A-Series and additional devices.",
                default = 1,
                provides = {
                    "HW_MCU_STM32L1_MD";
                },
                requires = { "HW_MCU_STM32L151_MDA" },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L151xC",
                brief = "STM32L151 256 kb",
                description = "Provides vendor header for STM32L151 256 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L151_C" },
                provides = {
                    "HW_MCU_STM32L1_MDP";
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L151xCA",
                brief = "STM32L151 256 kb A",
                description = "Provides vendor header for STM32L151 256 kb A-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L151_CA" },
                provides = {
                    "HW_MCU_STM32L1_MDP";
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L151xD",
                brief = "STM32L151 384 kb",
                description = "Provides vendor header for STM32L151 384 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L151_D" },
                provides = {
                    "HW_MCU_STM32L1_HD";
                    "HW_FSMC_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L151xDX",
                brief = "STM32L151 384 kb X",
                description = "Provides vendor header for STM32L151 384 kb X-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L151_DA" },
                provides = {
                    "HW_MCU_STM32L1_HD";
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L151xE",
                brief = "STM32L151 512 kb",
                description = "Provides vendor header for STM32L151 512 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L151_E" },
                provides = {
                    "HW_MCU_STM32L1_XL";
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L152xB",
                brief = "STM32L152 32/64/128 kb",
                description = "Provides vendor header for STM32L152 32/64/128 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_MD" },
                provides = {
                    "HW_MCU_STM32L1_MD";
                    "HW_LCD_STM32",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L152xBA",
                brief = "STM32L152 32/64/128 kb A",
                description = "Provides vendor header for STM32L152 32/64/128 kB A-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_MDA" },
                provides = {
                    "HW_MCU_STM32L1_MD";
                    "HW_LCD_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L152xC",
                brief = "STM32L152 256 kb",
                description = "Provides vendor header for STM32L152 256 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_C" },
                provides = {
                    "HW_MCU_STM32L1_MDP";
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L152xCA",
                brief = "STM32L152 256 kb A",
                description = "Provides vendor header for STM32L152 256 kb A-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_CA" },
                provides = {
                    "HW_MCU_STM32L1_MDP";
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L152xD",
                brief = "STM32L152 384 kb",
                description = "Provides vendor header for STM32L152 384 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_D" },
                provides = {
                    "HW_MCU_STM32L1_HD";
                    "HW_FSMC_STM32",
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L152xDX",
                brief = "STM32L152 384 kb X",
                description = "Provides vendor header for STM32L152 384 kb X-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_DA" },
                provides = {
                    "HW_MCU_STM32L1_HD";
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L152xE",
                brief = "STM32L152 512 kb",
                description = "Provides vendor header for STM32L152 512 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L152_E" },
                provides = {
                    "HW_MCU_STM32L1_XL";
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L162xC",
                brief = "STM32L162 256 kb",
                description = "Provides vendor header for STM32L162 256 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L162_C" },
                provides = {
                    "HW_MCU_STM32L1_MDP";
                    "HW_CRYP_STM32",
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L162xCA",
                brief = "STM32L162 256 kb A",
                description = "Provides vendor header for STM32L162 256 kb A-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L162_CA" },
                provides = {
                    "HW_MCU_STM32L1_MDP";
                    "HW_CRYP_STM32",
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L162xD",
                brief = "STM32L162 384 kb",
                description = "Provides vendor header for STM32L162 384 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L162_D" },
                provides = {
                    "HW_MCU_STM32L1_HD";
                    "HW_CRYP_STM32",
                    "HW_FSMC_STM32",
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L162xDX",
                brief = "STM32L162 384 kb X",
                description = "Provides vendor header for STM32L162 384 kb X-Series and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L162_DA" },
                provides = {
                    "HW_MCU_STM32L1_HD";
                    "HW_CRYP_STM32",
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32L162xE",
                brief = "STM32L162 512 kb",
                description = "Provides vendor header for STM32L162 512 kb and additional devices.",
                default = 1,
                requires = { "HW_MCU_STM32L162_E" },
                provides = {
                    "HW_MCU_STM32L1_XL";
                    "HW_CRYP_STM32",
                    "HW_LCD_STM32",
                    "HW_TIM5_STM32",
                    "HW_SPI3_STM32",
                    "HW_DMA2_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP_ACQ",
                },
                file = "include/cfg/arch.h",
            },
        }
    },
}
