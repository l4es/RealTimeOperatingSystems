--
--  * Copyright (C) 2013,2015-16
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
-- STM32F30 Family
-- *****************************************************************************
--

nutarch_cm3_stm32f3 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f3_family",
        brief = "MCU F3 Family",
        requires = { "HW_MCU_STM32F3" },
        options =
        {
            {
                macro = "MCU_STM32F3",
                brief = "STM32F3 family",
                description = "Common devices in all STM32F3",
                default = 1,
                provides =
                {
                    "HW_GPIO_STM32V2",
                    "HW_DMA1_STM32F1",
                    "HW_EXTI0_STM32",
                    "HW_EXTI1_STM32",
                    "HW_EXTI2_TSC_STM32",
                    "HW_EXTI3_STM32",
                    "HW_EXTI4_STM32",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_I2C_STM32V2",
                    "HW_I2C1_STM32",
                    "HW_PVD_STM32",
                    "HW_USART_STM32V2",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_TIM2_32BIT_STM32",
                    "HW_DAC1_STM32",
                    "HW_MCU_FPU",
                },
                makedefs = {
                    "FLASH_PAGE_SIZE=2048",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32F30",
                brief = "STM32F30 family",
                requires = {"HW_MCU_STM32F30"},
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F301x8",
                brief = "STM32F301",
                description = "Additional devices in STM32F301",
                requires = { "HW_MCU_STM32F301" },
                default = 1,
                provides =
                {
                    "HW_ADC1_STM32",
                    "HW_I2C2_STM32",
                    "HW_I2C3_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_COMP2_STM32",
                    "HW_COMP4_6_STM32",
                    "HW_OP1_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F302x8",
                brief = "STM32F302 32/64 kB",
                description = "Provide vendor header and additional devices in STM32F302 32/64 kB",
                requires = { "HW_MCU_STM32F302_LD" },
                default = 1,
                provides =
                {
                    "HW_ADC1_STM32",
                    "HW_I2C2_STM32",
                    "HW_I2C3_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_USB_CAN_STM32",
                    "HW_ADC1_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_COMP2_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                    "HW_OP_2_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F302xC",
                brief = "STM32F302 128/256 kB",
                description = "Provide vendor header and additional devices in STM32F302 128/256 kB",
                requires = { "HW_MCU_STM32F302_MD" },
                default = 1,
                provides =
                {
                    "HW_DMA2_STM32F1",
                    "HW_ADC1_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_USB_CAN_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_COMP1_2_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                    "HW_OP_2_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F302xE",
                brief = "STM32F302 512 kB",
                description = "Provide vendor header and additional devices in STM32F302 512 kB",
                requires = { "HW_MCU_STM32F302_HD" },
                default = 1,
                provides =
                {
                    "HW_HSI8_STM32",
                    "HW_DMA2_STM32F1",
                    "HW_ADC1_2_STM32",
                    "HW_I2C2_STM32",
                    "HW_I2C3_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_USB_CAN_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_COMP1_2_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                    "HW_OP_2_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F303x8",
                brief = "STM32F303 32/64 kB",
                description = "Provide vendor header and additional devices in STM32F303 32/64 kB",
                requires = { "HW_MCU_STM32F303_LD" },
                default = 1,
                provides =
                {
                    "HW_ADC1_2_STM32",
                    "HW_CCM_STM32",
                    "HW_SPI1_DMA_REMAP_STM32F1",
                    "HW_USB_CAN_STM32",
                    "HW_SPI1_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_DAC2_STM32",
                    "HW_COMP2_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                    "HW_OP_2_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F303xC",
                brief = "STM32F303 128/256 kB",
                description = "Provide vendor header and additional devices in STM32F303 128/256 kB",
                requires = { "HW_MCU_STM32F303_MD" },
                default = 1,
                provides =
                {
                    "HW_DMA2_STM32F1",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_ADC3_STM32",
                    "HW_ADC4_STM32",
                    "HW_CCM_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_USB_CAN_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_STM32",
                    "HW_COMP1_2_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                    "HW_OP_2_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F303xE",
                brief = "STM32F303 512 kB",
                description = "Provide vendor header and additional devices in STM32F303 512 kB",
                requires = { "HW_MCU_STM32F303_HD" },
                default = 1,
                provides =
                {
                    "HW_HSI8_STM32",
                    "HW_DMA2_STM32F1",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_ADC3_STM32",
                    "HW_ADC4_STM32",
                    "HW_CCM_STM32",
                    "HW_I2C2_STM32",
                    "HW_I2C3_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_USB_CAN_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_STM32",
                    "HW_TIM8_STM32",
                    "HW_COMP1_2_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                    "HW_OP_2_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F318xx",
                brief = "STM32F318 64 kB",
                description = "Provide vendor header and additional devices in STM32F318 64 kB",
                requires = { "HW_MCU_STM32F318xx" },
                default = 1,
                provides =
                {
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_I2C2_STM32",
                    "HW_I2C3_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_COMP2_STM32",
                    "HW_COMP4_5_6_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F328xx",
                brief = "STM32F328 64 kB",
                description = "Provide vendor header and additional devices in STM32F328 64 kB",
                requires = { "HW_MCU_STM32F328xx" },
                default = 1,
                provides =
                {
                    "HW_SPI1_DMA_REMAP_STM32F1",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_CAN_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_CCM_STM32",
                    "HW_SPI1_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_DAC2_STM32",
                    "HW_COMP2_STM32",
                    "HW_COMP4_5_6_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F334x8",
                brief = "STM32F334  up to 64 kB",
                description = "Provide vendor header and additional devices in STM32F334 up to  64 kB",
                requires = { "HW_MCU_STM32F334_LD" },
                default = 1,
                provides =
                {
                    "HW_SPI1_DMA_REMAP_STM32F1",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_CAN_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_CCM_STM32",
                    "HW_SPI1_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_DAC2_STM32",
                    "HW_HRTIM1_STM32",
                    "HW_COMP2_STM32",
                    "HW_COMP4_5_6_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F358xx",
                brief = "STM32F358 256 kB",
                description = "Provide vendor header and additional devices in STM32F358 256 kB",
                requires = { "HW_MCU_STM32F358xx" },
                default = 1,
                provides =
                {
                    "HW_DMA2_STM32F1",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_CAN_STM32",
                    "HW_I2C2_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_ADC3_STM32",
                    "HW_ADC4_STM32",
                    "HW_CCM_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_STM32",
                    "HW_COMP1_2_3_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32F37",
                brief = "STM32F37x family",
                description = "Additional devices in STM32F37x subfamily",
                requires = { "HW_MCU_STM32F37" },
                default = 1,
                provides =
                {
                    "HW_HSI8_DIV2_STM32",
                    "HW_DMA2_STM32F1",
                    "HW_ADC1_STM32",
                    "HW_I2C2_STM32",
                    "HW_CAN_STM32",
                    "HW_COMP_STM32",
                    "HW_SDADC1_STM32",
                    "HW_SDADC2_STM32",
                    "HW_SDADC3_STM32",
                    "HW_TIM12_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM5_STM32",
                    "HW_TIM6_DAC1_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM12_STM32",
                    "HW_TIM13_STM32",
                    "HW_TIM14_STM32",
                    "HW_TIM15_STM32",
                    "HW_TIM16_STM32",
                    "HW_TIM17_STM32",
                    "HW_TIM18_DAC2_STM32",
                    "HW_TIM19_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_STM32",
                    "HW_CEC_STM32",
                 },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F373xC",
                brief = "STM32F373 64 up to 256 kB",
                description = "Provide vendor header and additional devices in STM32F373 64 up to 256 kB",
                requires = { "HW_MCU_STM32F373_MD" },
                default = 1,
                provides =
                {
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F378xx",
                brief = "STM32F378 256 kB",
                description = "Provide vendor header and additional devices in STM32F378 256 kB",
                requires = { "HW_MCU_STM32F378xx" },
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32F398xx",
                brief = "STM32F398 512 kB",
                description = "Provide vendor header and additional devices in STM32F398 512 kB",
                brief = "Include needed for STM32F398",
                requires = { "HW_MCU_STM32F398xx" },
                default = 1,
                provides =
                {
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_DMA2_STM32F1",
                    "HW_CAN_STM32",
                    "HW_ADC1_2_STM32",
                    "HW_ADC3_STM32",
                    "HW_ADC4_STM32",
                    "HW_CCM_STM32",
                    "HW_I2C2_STM32",
                    "HW_I2C3_STM32",
                    "HW_TIM12_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_STM32",
                    "HW_TIM20_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_COMP1_2_3_STM32",
                    "HW_COMP4_5_6_STM32",
                    "HW_COMP7_STM32",
                },
                file = "include/cfg/arch.h"
            },
        }
    },
}
