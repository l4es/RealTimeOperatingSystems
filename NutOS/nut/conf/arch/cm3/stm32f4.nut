--
--  * Copyright (C) 2011-15 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
-- STMicroelectronics STM32F4 Family Devices
--
--

-- *****************************************************************************
-- STM32F4 Family
-- *****************************************************************************
--

nutarch_cm3_stm32f4 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f4_family",
        brief = "MCU F4 Family",
        requires = { "HW_MCU_STM32F4" },
        options =
        {
            {
                macro = "MCU_STM32F4",
                brief = "STM32F4",
                description = "Devices available in all STM32F4xx devices",
                default = 1,
                provides =
                {
                    "HW_VOS_STM32",
                    "HW_ICACHE_STM32",
                    "HW_DCACHE_STM32",
                    "HW_MCU_FPU",
                    "HW_FLASH_STM32F2_4",
                    "HW_GPIO_STM32V2",
                    "HW_DMA_STM32F2",
                    "HW_EXTI04_STM32",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_I2C_STM32V1",
                    "HW_I2C1_STM32",
                    "HW_I2C2_STM32",
                    "HW_PVD_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_USART1_STM32",
                    "HW_USART2_STM32",
                    "HW_USART6_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_TIM5_32BIT_STM32",
                    "HW_ADC_STM32",
                    "HW_ADC1_STM32F2_4",
                    "HW_WWDG_STM32",
                 },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F401",
                brief = "STM32F401 256/512 kB",
                description = "Provides additional devices in STM32F401",
                requires = { "HW_MCU_STM32F401" },
                default = 1,
                provides =
                {
                    "HW_I2C3_STM32",
                    "HW_SDIO_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_OTG_FS_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F401xC",
                brief = "STM32F401 256 kB",
                description = "Provides vender header for STM32F401 256 kB Low Power with USB.",
                requires = { "HW_MCU_STM32F401xC" },
                default = 1,
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F401xE",
                brief = "STM32F401 512 kB Low Power with USB",
                description = "Provides vender header for STM32F401 512 kB Low Power with USB.",
                requires = { "HW_MCU_STM32F401xE" },
                default = 1,
                file = "include/cfg/arch.h",
             },
             {
                macro = "MCU_STM32F40",
                brief = "STM32F405/407/415/417",
                description = "Provides additional devices in STM32F405/407/415/417.",
                requires = { "HW_MCU_STM32F40" },
                default = 1,
                provides =
                {
                    "HW_BKPSRAM_STM32",
                    "HW_CCM_STM32",
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_I2C3_STM32",
                    "HW_FSMC_STM32",
                    "HW_SPI3_STM32",
                    "HW_USART3_STM32",
                    "HW_SDIO_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_TIM14_STM32",
                    "HW_ADC2_STM32F2_4",
                    "HW_ADC3_STM32F2_4",
                    "HW_DAC1_STM32",
                    "HW_DAC2_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_OTG_HS_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F405xx",
                brief = "STM32F405",
                description = "Provides vendor header for STM32F405.",
                requires = { "HW_MCU_STM32F405" },
                default = 1,
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F407xx",
                brief = "STM32F407",
                description = "Provides vendor header for STM32F407 and additional devices.",
                requires = { "HW_MCU_STM32F407" },
                default = 1,
                provides =
                {
                    "HW_DCMI_STM32",
                    "HW_EMAC_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F41",
                brief = "STM32F41[0-3] Access line 100 MHz",
                description = "Devices common to STM32F41[0-3].",
                requires = { "HW_MCU_STM32F41" },
                default = 1,
                provides =
                {
                    "HW_SPI5_STM32",
                },
                file = "include/cfg/arch.h",
             },
              {
                macro = "MCU_STM32F411",
                brief = "STM32F411 Low Power",
                description = "Provides additional devices for STM32F411.",
                requires = { "HW_MCU_STM32F411" },
                default = 1,
                provides =
                {
                    "HW_I2C3_STM32",
                    "HW_SDIO_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_OTG_FS_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F411xE",
                brief = "STM32F411 Low Power",
                description = "Provides vendor header for STM32F411xE.",
                requires = { "HW_MCU_STM32F411xE" },
                default = 1,
                file = "include/cfg/arch.h",
             },
             {
                macro = "MCU_STM32F412",
                brief = "STM32F412 Low Power",
                description = "Provides additional devices for STM32F412.",
                requires = { "HW_MCU_STM32F412" },
                default = 1,
                provides =
                {
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_DFSDM1_FLT0_STM32",
                    "HW_DFSDM1_FLT1_STM32",
                    "HW_FMPI2C1_STM32",
                    "HW_I2C3_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_SDIO_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_SPI5_STM32",
                    "HW_RNG_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_TIM14_STM32",
               },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F412Cx",
                brief = "STM32F412 Low Power 48 Pin",
                description = "Provides vendor header for STM32F412Cx.",
                requires = { "HW_MCU_STM32F412Cx" },
                default = 1,
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F412Rx",
                brief = "STM32F412 Low Power 64 pin",
                description = "Provides vendor header for STM32F412Rx.",
                requires = { "HW_MCU_STM32F412Rx" },
                default = 1,
                provides = {
                   "HW_USART3_STM32",
                   "HW_QUADSPI_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F412Vx",
                brief = "STM32F412 Low Power 100 Pin",
                description = "Provides vendor header for STM32F412Vx.",
                requires = { "HW_MCU_STM32F412Vx" },
                provides = {
                    "HW_USART3_STM32",
                    "HW_QUADSPI_STM32",
                },
                default = 1,
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F412Zx",
                brief = "STM32F412 Low Power 144 Pins",
                description = "Provides vendor header for STM32F412Zx.",
                requires = { "HW_MCU_STM32F412Zx" },
                provides = {
                    "HW_USART3_STM32",
                    "HW_QUADSPI_STM32",
                },
                default = 1,
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F413xx",
                brief = "STM32F413 Low Power",
                description = "Provides additional devices for STM32F413.",
                requires = {"HW_MCU_STM32F413"},
                default = 1,
                provides =
                {
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_CAN3_STM32",
                    "HW_DFSDM1_FLT0_STM32",
                    "HW_DFSDM1_FLT1_STM32",
                    "HW_DFSDM2_FLT0_STM32",
                    "HW_DFSDM2_FLT1_STM32",
                    "HW_FMPI2C1_STM32",
                    "HW_I2C3_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_SDIO_STM32",
                    "HW_SAI1_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_QUADSPI_STM32",
                    "HW_RNG_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_TIM14_STM32",
                    "HW_LPTIM1_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_USART6_STM32",
                    "HW_UART7_STM32",
                    "HW_UART8_STM32",
                    "HW_UART9_STM32",
                    "HW_UART10_STM32",
               },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F415xx",
                brief = "STM32F415",
                description = "Provides vendor header for STM32F415 and additional devices.",
                requires = { "HW_MCU_STM32F415" },
                default = 1,
                provides =
                {
                    "HW_HASH_RNG_STM32",
                    "HW_CRYP_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F417xx",
                brief = "STM32F417",
                description = "Provides vendor header for STM32F417 and additional devices.",
                requires = { "HW_MCU_STM32F417" },
                default = 1,
                provides =
                {
                    "HW_HASH_RNG_STM32",
                    "HW_CRYP_STM32",
                    "HW_DCMI_STM32",
                    "HW_EMAC_STM32",
                 },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F42",
                brief = "STM32F427/29/37/39",
                description = "Additional devices in STM32F427/29/37/39.",
                requires = { "HW_MCU_STM32F42" },
                default = 1,
                provides =
                {
                    "HW_CCM_STM32",
                    "HW_OVERDRIVE_STM32",
                    "HW_BKPSRAM_STM32",
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_DCMI_STM32",
                    "HW_DMA2D_STM32",
                    "HW_FMC_STM32",
                    "HW_I2C3_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_UART7_STM32",
                    "HW_UART8_STM32",
                    "HW_SDIO_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_SPI5_STM32",
                    "HW_SPI6_STM32",
                    "HW_SAI1_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_TIM14_STM32",
                    "HW_ADC2_STM32F2_4",
                    "HW_ADC3_STM32F2_4",
                    "HW_DAC1_STM32",
                    "HW_DAC2_STM32",
                    "HW_EMAC_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_OTG_HS_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F427xx",
                brief = "STM32F427",
                description = "Provides vendor header for STM32F427.",
                requires = { "HW_MCU_STM32F427" },
                default = 1,
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F429xx",
                brief = "STM32F429",
                description = "Provides vendor header for STM32F429 and additional devices.",
                requires = { "HW_MCU_STM32F429" },
                default = 1,
                provides =
                {
                    "HW_DCMI_STM32",
                    "HW_LTDC_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F437xx",
                brief = "STM32F437",
                description = "Provides vendor header for STM32F437 and additional devices.",
                requires = { "HW_MCU_STM32F437" },
                default = 1,
                provides =
                {
                    "HW_CRYP_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F439xx",
                brief = "STM32F439",
                description = "Provides vendor header for STM32F439 and additional devices.",
                requires = { "HW_MCU_STM32F439" },
                default = 1,
                provides =
                {
                    "HW_CRYP_STM32",
                    "HW_DCMI_STM32",
                    "HW_LTDC_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F446",
                brief = "MCU_STM32F446",
                description = "Provides additional devices for STM32F446.",
                requires = { "HW_MCU_STM32F44" },
                default = 1,
                provides =
                {
                    "HW_OVERDRIVE_STM32",
                    "HW_BKPSRAM_STM32",
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_CEC_STM32",
                    "HW_CRYP_STM32",
                    "HW_DCMI_STM32",
                    "HW_I2C3_STM32",
                    "HW_QUADSPI_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_SAI1_STM32",
                    "HW_SAI2_STM32",
                    "HW_SDIO_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_TIM14_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_OTG_HS_STM32",
                 },
                file = "include/cfg/arch.h"
             },
             {
                macro = "STM32F446xx",
                brief = "STM32F446xx",
                description = "Provides vendor header for STM32F446.",
                requires = { "HW_MCU_STM32F446" },
                default = 1,
                file = "include/cfg/arch.h"
             },
             {
                 macro = "MCU_STM32F469",
                 brief = "MCU_STM32F469",
                 description = "Provides additional devices for STM32F46 subfamily.",
                 requires = { "HW_MCU_STM32F46" },
                 default = 1,
                 provides =
                 {
                    "HW_OVERDRIVE_STM32",
                    "HW_BKPSRAM_STM32",
                    "HW_CCM_STM32",
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_DCMI_STM32",
                    "HW_I2C3_STM32",
                    "HW_QUADSPI_STM32",
                    "HW_RTC_STM32_V2",
                    "HW_SAI1_STM32",
                    "HW_SAI2_STM32",
                    "HW_SDIO_STM32",
                    "HW_SPI3_STM32",
                    "HW_SPI4_STM32",
                    "HW_TIM1_TIM9_TIM10_TIM11_STM32",
                    "HW_TIM6_DAC_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM8_TIM12_TIM13_TIM14_STM32",
                    "HW_TIM2_32BIT_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_OTG_FS_STM32",
                    "HW_OTG_HS_STM32",
                 },
                 file = "include/cfg/arch.h"
              },
              {
                 macro = "STM32F469xx",
                 brief = "STM32F469xx",
                 description = "Provides vendor header for STM32F469.",
                 requires = { "HW_MCU_STM32F469" },
                 default = 1,
                 file = "include/cfg/arch.h"
              }
         }
    },
}
