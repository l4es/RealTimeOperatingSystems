--
--  * Copyright (C) 2015, 2016
--    by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

-- ARM CortexM7 Architecture
-- STMicroelectronics STM32F7 Family Devices
--
--
-- *****************************************************************************
-- STM32F7 Family
-- *****************************************************************************
--

nutarch_cm3_stm32f7 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f7_family",
        brief = "MCU F7 Family",
        requires = { "HW_MCU_STM32F7" },
        options =
        {
            {
                macro = "MCU_CM_NO_BITBAND",
                brief = "No Bitbanding",
                description = "Selected family has no bitband support",
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32F7",
                brief = "STM32F7 Devices",
                description = "Provide devices available in all STM32F7xx devices",
                default = 1,
                provides =
                {
                    "HW_VOS_STM32",
                    "HW_OVERDRIVE_STM32",
                    "HW_FLASH_STM32F2_4",
                    "HW_GPIO_STM32V2",
                    "HW_BKPSRAM_STM32",
                    "HW_CAN1_STM32",
                    "HW_DMA_STM32F2",
                    "HW_EXTI04_STM32",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_I2C_STM32V2",
                    "HW_I2C3_STM32",
                    "HW_PVD_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_SPI5_STM32",
                    "HW_USART_STM32V2",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_USART6_STM32",
                    "HW_UART7_STM32",
                    "HW_UART8_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_SDMMC1_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM5_32BIT_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_COM_TIM14_STM32",
                    "HW_ADC_STM32",
                    "HW_ADC1_STM32F2_4",
                    "HW_WWDG_STM32",
                 },
                file = "include/cfg/arch.h",
                makedefs = {"ITCM_LENGTH=16k", "RAM2_LENGTH=16K"}
            },
            {
                macro = "MCU_STM32F72",
                brief = "STM32F72 family, 256/512, 256 kB Ram",
                description = "STM32F72 family with single precision FPU.",
                requires = { "HW_MCU_STM32F72" },
                default = 1,
                provides =
                {
                    "HW_MCU_FPU",
                },
                file = "include/cfg/arch.h",
                makedefs = {"RAM0_LENGTH=64k", "RAM1_LENGTH=176K", "RAM2_LENGHT = 16k"}
            },
            {
                macro = "STM32F723xx",
                brief = "STM32F72 family, 256/512, 256 kB Ram",
                description = "STM32F74 family with single precision FPU.",
                requires = { "HW_MCU_STM32F723" },
                default = 1,
                provides =
                {
                    "HW_RNG_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "MCU_STM32F74",
                brief = "STM32F745 family, 512/1024 kB flash, 320 kB Ram",
                description = "STM32F74 family with single precision FPU.",
                requires = { "HW_MCU_STM32F74" },
                default = 1,
                provides =
                {
                    "HW_MCU_FPU",
                    "HW_CAN2_STM32",
                    "HW_DCMI_STM32",
                    "HW_EMAC_STM32",
                    "HW_I2C4_STM32",
                    "HW_QSPI_STM32",
                    "HW_SPI6_STM32",
                },
                file = "include/cfg/arch.h",
                makedefs = {"RAM0_LENGTH=64k", "RAM1_LENGTH=240K"}
             },
             {
                macro = "STM32F745xx",
                brief = "STM32F745 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F745.",
                requires = { "HW_MCU_STM32F745" },
                default = 1,
                provides =
                {
                    "HW_RNG_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F746xx",
                brief = "STM32F746 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F745.",
                requires = { "HW_MCU_STM32F746" },
                default = 1,
                provides =
                {
                    "HW_RNG_STM32",
                    "HW_LTDC_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F756xx",
                brief = "STM32F746 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F745.",
                requires = { "HW_MCU_STM32F746" },
                default = 1,
                provides =
                {
                    "HW_CRYPT_STM32",
                    "HW_HASH_RNG_STM32",
                    "HW_LTDC_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "MCU_STM32F76",
                brief = "STM32F76 family 1024/2048 KB flash, 512 kB ram",
                description = "STM32F76 family with double precision FPU.",
                requires = { "HW_MCU_STM32F76" },
                default = 1,
                provides =
                {
                    "HW_CAN2_STM32",
                    "HW_CAN3_STM32",
                    "HW_DCMI_STM32",
                    "HW_DFSDM_STM32",
                    "HW_EMAC_STM32",
                    "HW_MCU_DFPU",
                    "HW_I2C4_STM32",
                    "HW_MIOS_STM32",
                    "HW_QSPI_STM32",
                    "HW_SPI6_STM32",
                },
                file = "include/cfg/arch.h",
                makedefs = {"RAM0_LENGTH=128k", "RAM1_LENGTH=368K"}
             },
             {
                macro = "STM32F765xx",
                brief = "STM32F765 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F765.",
                requires = { "HW_MCU_STM32F765" },
                default = 1,
                provides =
                {
                    "HW_RNG_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F767xx",
                brief = "STM32F767 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F767.",
                requires = { "HW_MCU_STM32F767" },
                default = 1,
                provides =
                {
                    "HW_LTDC_STM32",
                    "HW_RNG_STM32",
                    "HW_JPEG_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F769xx",
                brief = "STM32F769 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F769.",
                requires = { "HW_MCU_STM32F769" },
                default = 1,
                provides =
                {
                    "HW_DSI_STM32",
                    "HW_LTDC_STM32",
                    "HW_RNG_STM32",
                    "HW_JPEG_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F777xx",
                brief = "STM32F777 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F777.",
                requires = { "HW_MCU_STM32F777" },
                default = 1,
                provides =
                {
                    "HW_LTDC_STM32",
                    "HW_CRYPT_STM32",
                    "HW_HASH_RNG_STM32",
                    "HW_JPEG_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F779xx",
                brief = "STM32F779 512/1024 kB",
                description = "Provides vendor header and additional devices for STM32F769.",
                requires = { "HW_MCU_STM32F779" },
                default = 1,
                provides =
                {
                    "HW_DSI_STM32",
                    "HW_LTDC_STM32",
                    "HW_CRYPT_STM32",
                    "HW_HASH_RNG_STM32",
                    "HW_JPEG_STM32",
                },
                file = "include/cfg/arch.h",
             },
        }
    },
}
