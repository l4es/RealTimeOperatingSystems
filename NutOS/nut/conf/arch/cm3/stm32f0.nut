--
--  * Copyright (C) 2014-2015, 2017
--             by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
-- STMicroelectronics STM32F0 Family Devices
--
-- $Id$
--
--


--
-- ********************************************************************************
-- STM32F1 Family
-- ********************************************************************************
--

nutarch_cm3_stm32f0 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f0_family",
        brief = "MCU F0 Family",
        requires = { "HW_MCU_STM32F0" },
        options =
        {
            {
                macro = "MCU_STM32F0",
                brief = "STM32F0",
                description = "Devices available in all STM32F0 devices",
                default = 1,
                provides = {
                    "HW_HSI8_STM32F",
                    "HW_RTC_STM32_V2",
                    "HW_GPIO_STM32V2",
                    "HW_EXTI01_STM32",
                    "HW_EXTI23_STM32",
                    "HW_EXTI415_STM32",
                    "HW_SPI1_STM32",
                    "HW_USART_STM32V2",
                    "HW_USART1_STM32",
                    "HW_TIM1_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM14_STM32",
                    "HW_TIM16_STM32",
                    "HW_TIM17_STM32",
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
            {
                macro = "MCU_STM32F030",
                brief = "STM32F030 w/o uinique ID",
                description = "Seleted devices have no unique ID.",
                requires = { "HW_MCU_STM32F030"},
                type = "integer",
                default = 1,
                provides = {
                    "HW_HSI8_DIV2_STM32",
                    "HW_DMA1_STM32F0",
                    "HW_ADC1_STM32",
                },
                file = "include/cfg/arch.h"
            },
-- Use macro names so that ST vendor defines are produced
            {
                macro = "STM32F030x6",
                brief = "STM32F030  16/32 kB",
                description = "Provides vendor header for STM32F030 16/32 kb and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F030_LD"},
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F030x8",
                brief = "STM32F030 64 kB",
                description = "Provides vendor header for STM32F030 64 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F030x8"},
                provides = {
                    "HW_USART2_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM15_STM32",
                   },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F030xC",
                brief = "STM32F030 256 kB",
                description = "Provides vendor header for STM32F030 256 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F030xC"},
                provides = {
                    "HW_USART2_STM32",
                    "HW_USART3_6_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM15_STM32",
                   },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F038xx",
                brief = "STM32F038",
                description = "Provides vendor header for STM32F038 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F038xx"},
                provides = {
                    "HW_TIM2_32BIT_STM32",
                     },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32F04",
                brief = "STM32F04x with USB/CAN",
                description = "Devices available in all STM32F04x.",
                requires = { "HW_MCU_STM32F04"},
                type = "integer",
                default = 1,
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F0",
                    "HW_ADC1_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_PVD_VDDIO2_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TSC_STM32",
                    "HW_USART2_STM32",
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F042x6",
                brief = "STM32F042 16/32 kB",
                description = "Provides vendor header for STM32F042 16/32kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F042"},
                provides = {
                    "HW_PVD_VDDIO2_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F048xx",
                brief = "STM32F048.",
                description = "Provides vendor header for STM32F042 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F048xx"},
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F0",
                    "HW_ADC1_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_VDDIO2_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TSC_STM32",
                    "HW_USART2_STM32",
                    "HW_USB_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F051x8",
                brief = "STM32F051 16/32/64 kB",
                description = "Provides vendor header for STM32F051 16/32/64 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F051_LD"},
                provides = {
                    "HW_HSI8_DIV2_STM32",
                    "HW_DMA1_STM32F0",
                    "HW_ADC1_COMP_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_PVD_STM32",
                    "HW_TSC_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM15_STM32",
                    "HW_USART2_STM32",
                     },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F070x6",
                brief = "STM32F070 16/32 kB",
                description = "Provides vendor header for STM32F070 16/32 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F070_LD"},
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F07",
                    "HW_SPI2_DMA_REMAP_STM32F0",
                    "HW_ADC1_STM32",
                    "HW_USART2_STM32",
                    "HW_USB_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F070xB",
                brief = "STM32F070 64/128 kB",
                description = "Provides vendor header for STM32F070 128 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F070_MD"},
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F07",
                    "HW_SPI2_DMA_REMAP_STM32F0",
                    "HW_ADC1_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM15_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_4_STM32",
                    "HW_USB_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F071xB",
                brief = "STM32F071 64/128 kB",
                description = "Provides vendor header for STM32F071 64/128 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F071_MD"},
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F07",
                    "HW_SPI2_DMA_REMAP_STM32F0",
                    "HW_ADC1_COMP_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_I2C2_STM32",
                    "HW_PVD_VDDIO2_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM15_STM32",
                    "HW_TSC_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_4_STM32",
                    "HW_USB_STM32",
                    },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F072xB",
                brief = "STM32F072 64/128 kB",
                description = "Provides vendor header for STM32F072 64/128 kB and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F072_MD"},
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F07",
                    "HW_SPI2_DMA_REMAP_STM32F0",
                    "HW_ADC1_COMP_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_PVD_VDDIO2_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM15_STM32",
                    "HW_TSC_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_4_STM32",
                    },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F078xx",
                brief = "STM32F078",
                description = "Provides vendor header for STM32F078 and additional devices.",
                default = 1,
                requires = {"HW_MCU_STM32F078xx"},
                provides = {
                    "HW_CRS_STM32",
                    "HW_DMA1_STM32F07",
                    "HW_SPI2_DMA_REMAP_STM32F0",
                    "HW_ADC1_COMP_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_VDDIO2_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM15_STM32",
                    "HW_TSC_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_4_STM32",
                    },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32F09",
                brief = "STM32F09x",
                description = "Provide STM32F09 class devices.",
                requires = { "HW_MCU_STM32F09" },
                default = 1,
                provides =
                {
                    "HW_CRS_STM32",
                    "HW_DMA1_2_STM32F09",
                    "HW_ADC1_COMP_STM32",
                    "HW_CEC_CAN_STM32",
                    "HW_CLK48_STM32",
                    "HW_SPI2_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM14_STM32",
                    "HW_TIM15_STM32",
                    "HW_TSC_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_8_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F091xC",
                brief = "STM32F091",
                description = "Provides vendor header for STM32F091 and additional devices.",
                requires = { "HW_MCU_STM32F091xC" },
                default = 1,
                provides =
                {
                    "HW_PVD_VDDIO2_STM32",
                 },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F098xx",
                brief = "STM32F098",
                description = "Provides vendor header for STM32F098 and additional devices.",
                requires = { "HW_MCU_STM32F098xx" },
                default = 1,
                provides =
                {
                    "HW_VDDIO2_STM32",
                 },
                file = "include/cfg/arch.h",
             },
        },
    },
}
