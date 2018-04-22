--
--  * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
-- STMicroelectronics STM32F1 Family Devices
--
-- $Id: stm32f1.nut 6658 2017-05-16 13:02:41Z u_bonnes $
--
--

--
-- ****************************************************************************
-- STM32F1 Family
-- ****************************************************************************
--
-- STM32F100x4/6   Low    density value line        -> stm32f100x6.h
-- STM32F100x8/B   Medium density value line        -> stm32f100xb.h
-- STM32F100xC/D/E High   density value line        -> stm32f100xe.h
-- STM32F101x4/6   Low    density access line       -> stm32f101x6.h
-- STM32F101x8/B   Medium density access line       -> stm32f101xb.h
-- STM32F101xC/D/E High   density access line       -> stm32f101xe.h
-- STM32F101xF/G   XL     density access line       -> stm32f101xg.h
-- STM32F102x4/6   Low    density USB access line   -> stm32f102x6.h
-- STM32F102x8/B   Medium density USB access line   -> stm32f102xb.h
-- STM32F103x4/6   Low    density performance line  -> stm32f103x6.h
-- STM32F103x8/B   Medium density performance line  -> stm32f103xb.h
-- STM32F103xC/D/E High   density performance line  -> stm32f103xe.h
-- STM32F103xF/G   XL     density performance line  -> stm32f103xg.h
-- STM32F105                      Connectivity line -> stm32f105xc.h
-- STM32F107                      Connectivity line -> stm32f107xc.h

function GetF1ClockHandler()
    if c_is_provided("HW_MCU_STM32F100") then
        return {"cm3/dev/stm/stm32f30_clk.c"}
    end
    return {"cm3/dev/stm/stm32f1_clk.c"}
end

nutarch_cm3_stm32f1 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f1_family",
        brief = "MCU F1 Family",
        sources = function() return GetF1ClockHandler() end,
        options =
        {
            {
                macro = "MCU_STM32F1",
                brief = "STM32F1",
                description = "Provides STM32F1 common peripherals.",
                default = 1,
                provides = {
                        "HW_GPIO_STM32V1",
                        "HW_EXTI04_STM32",
                        "HW_EXTI95_STM32",
                        "HW_EXTI1510_STM32",
                        "HW_DMA1_STM32F1",
                        "HW_I2C_STM32V1",
                        "HW_I2C1_STM32",
                        "HW_PVD_STM32",
                        "HW_SPI1_STM32",
                        "HW_USART1_STM32",
                        "HW_USART2_STM32",
                        "HW_TIM2_STM32",
                        "HW_TIM3_STM32",
                },
                file = "include/cfg/arch.h"
            },
-- Base are devices found in STM32F101X6, ADC1/DMA3 excluded
-- Provide ADC1(_2) and DMA2 on a per include file base
-- Value line provides() are additional to xxxx density provides{}
--
            {
                macro = "MCU_STM32F1_LD",
                brief = "STM32F LD Series",
                description = "STM32F1 Medium density additional peripherals.",
                flavor = "booldata",
                requires = { "HW_MCU_STM32F1_LD" },
                default = 1,
            },
            {
                macro = "MCU_STM32F1_MD",
                brief = "STM32F MD Series",
                description = "STM32F Medium Density additional peripherals.",
                requires = { "HW_MCU_STM32F1_MD" },
                default = 1,
                provides = {
                   "HW_I2C2_STM32",
                   "HW_SPI2_STM32",
                   "HW_TIM4_STM32",
                   "HW_USART3_STM32",
                },
                file = "include/cfg/arch.h"
             },
            {
                macro = "MCU_STM32F1_MD_VL",
                brief = "STM32F MD Series",
                description = "STM32F Medium Density Value line additional peripherals to STM32F1 MD.",
                requires = { "HW_MCU_STM32F1_MD_VL" },
                default = 1,
                provides = {
                  "HW_CEC_STM32",
                  "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                  "HW_TIM6_DAC_STM32",
                  "HW_TIM7_STM32",
               },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F1_HD",
                brief = "STM32F HD Series",
                description = "STM32F1 High Density additional peripherals.",
                requires = { "HW_MCU_STM32F1_HD" },
                default = 1,
                provides = {
                   "HW_I2C2_STM32",
                   "HW_SPI2_STM32",
                   "HW_SPI3_STM32",
                   "HW_TIM4_STM32",
                   "HW_TIM5_STM32",
                   "HW_TIM7_STM32",
                   "HW_USART3_STM32",
                   "HW_UART4_STM32",
                   "HW_UART5_STM32",
                   "HW_FSMC_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F1_HD_VL",
                brief = "STM32F HD VL Series",
                description = "STM32F1 High Density Value line.\n"..
                            "Additional peripherals to STM32F1 HD.",
                requires = { "HW_MCU_STM32F1_HD_VL" },
                default = 1,
                provides = {
                    "HW_CEC_STM32",
                    "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                    "HW_TIM7_STM32",
                    "HW_TIM12_STM32",
                    "HW_TIM13_STM32",
                    "HW_TIM14_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F1_XL",
                brief = "STM32F XL Series",
                description = "STM32F XL Density additional peripherals.",
                requires = { "HW_MCU_STM32F1_XL" },
                default = 1,
                provides = {
                   "HW_I2C2_STM32",
                   "HW_SPI2_STM32",
                   "HW_SPI3_STM32",
                   "HW_TIM4_STM32",
                   "HW_TIM5_STM32",
                   "HW_TIM6_STM32",
                   "HW_TIM7_STM32",
                   "HW_TIM9_STM32",
                   "HW_TIM10_STM32",
                   "HW_TIM11_STM32",
                   "HW_TIM12_STM32",
                   "HW_TIM13_STM32",
                   "HW_TIM14_STM32",
                   "HW_USART3_STM32",
                   "HW_UART4_STM32",
                   "HW_UART5_STM32",
                   "HW_FSMC_STM32",
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F1_CL",
                brief = "STM32F CL Series",
                description = "STM32F1 Communication Line additional peripherals.",
                requires = { "HW_MCU_STM32F1_CL" },
                default = 1,
                provides = {
                    "HW_CAN1_STM32",
                    "HW_CAN2_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                    "HW_TIM2_STM32",
                    "HW_TIM3_STM32",
                    "HW_TIM4_STM32",
                    "HW_TIM5_STM32",
                    "HW_TIM6_STM32",
                    "HW_TIM7_STM32",
                    "HW_USART3_STM32",
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_OTG_STM32,"
                },
                file = "include/cfg/arch.h"
             },
             {
                macro = "MCU_STM32F100",
                brief = "STM32F100 Basic devices",
                description = "STM32F100 Family\n",
                requires = { "HW_MCU_STM32F100" },
                type = "integer",
                default = 1,
                provides = {
                  "HW_HSI8_DIV2_STM32",
                  "HW_ADC1_2_STM32",
                  "HW_CEC_STM32",
                  "HW_TIM6_DAC_STM32",
                  "HW_TIM7_DAC_STM32",
                  "HW_TIM1_TIM15_TIM16_TIM17_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F100xB",
                brief = "STM32F100 64/128 kB",
                description = "STM32F100 64/128 kB header\n",
                requires = { "HW_MCU_STM32F100_MD" },
                type = "integer",
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "MCU_STM32F101",
                brief = "STM32F101 additional device",
                description = "STM32F101  additional device",
                requires = { "HW_MCU_STM32F101" },
                type = "integer",
                default = 1,
                provides = {
                    "HW_ADC1_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F101x6",
                brief = "STM32F101 16/32 kB",
                description = "Provides vendor header for STM32F101 16/32 kB and additional device.",
                requires = { "HW_MCU_STM32F101x6" },
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F101xB",
                brief = "STM32F101 64/128 kB",
                description = "Provides vendor header for STM32F101 64/128 kB and additional device.",
                requires = { "HW_MCU_STM32F101xB" },
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F101xE",
                brief = "STM32F101 256/384/512 kB",
                description = "Provides vendor header for STM32F101 256/384/512 kB and additional device.",
                requires = { "HW_MCU_STM32F101xE" },
                default = 1,
                provides = {
                    "HW_DMA2_CH4_5_STM32F1",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F101xG",
                brief = "STM32F101 768/1024 kB",
                description = "STM32F101 768/1024  kB and additional device.",
                requires = { "HW_MCU_STM32F101xG" },
                default = 1,
                provides = {
                    "HW_DMA2_CH4_5_STM32F1",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "MCU_STM32F102",
                brief = "STM32F102 additional devices.",
                description = "STM32F102 additional devices.",
                requires = { "HW_MCU_STM32F102" },
                type = "integer",
                default = 1,
                provides = {
                    "HW_ADC1_2_STM32",
                    "HW_USB_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F102x6",
                brief = "STM32F102 16/32 kB",
                description = "Provides vendor header for STM32F102 16/32 kB and additional device.",
                requires = { "HW_MCU_STM32F102x6" },
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F102xB",
                brief = "STM32F102 64/128 kB",
                description = "Provides vendor header for STM32F102 64/128 kB and additional device.",
                requires = { "HW_MCU_STM32F102xB" },
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "MCU_STM32F103",
                brief = "STM32F103 additional devices.",
                description = "STM32F103 additional devices.",
                requires = { "HW_MCU_STM32F103" },
                type = "integer",
                default = 1,
                provides = {
                   "HW_ADC1_2_STM32",
                   "HW_CAN1_STM32",
                   "HW_TIM1_STM32",
                   "HW_USB_CAN1_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F103x6",
                brief = "STM32F103 16/32 kB",
                description = "Provides vendor header for STM32F103 16/32 kB and additional device.",
                requires = { "HW_MCU_STM32F103_LD" },
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F103xB",
                brief = "STM32F103 64/128 kB",
                description = "Provides vendor header for STM32F103 64/128 kB and additional device.",
                requires = { "HW_MCU_STM32F103_MD" },
                default = 1,
                provides = {
                },
                file = "include/cfg/arch.h",
            },
             {
                macro = "STM32F103xE",
                brief = "STM32F103 256/384/512 kB",
                description = "Provides vendor header for STM32F103 256/384/512 kB and additional device.",
                requires = { "HW_MCU_STM32F103_HD" },
                default = 1,
                provides = {
                    "HW_DMA2_CH4_5_STM32F1",
                    "HW_ADC3_STM32",
                    "HW_TIM8_STM32",
                    "HW_SDIO_STM32",
                },
                file = "include/cfg/arch.h",
             },
             {
                macro = "STM32F103xG",
                brief = "STM32F103 768/1024 kB",
                description = "Provides vendor header for STM32F103 768/1024 kB and additional device.",
                requires = { "HW_MCU_STM32F103_XL" },
                default = 1,
                provides = {
                    "HW_DMA2_CH4_5_STM32F1",
                    "HW_ADC1_2_STM32",
                    "HW_ADC3_STM32",
                    "HW_CAN1_STM32",
                    "HW_TIM1_STM32",
                    "HW_TIM8_STM32",
                    "HW_SDIO_STM32",
-- FIXME: What Name "HW_USB_STM32",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F105xC",
                brief = "STM32F105 256 kB",
                description = "Provides vendor header for STM32F103 1024 kB and additional device.",
                requires = { "HW_MCU_STM32F105xC" },
                default = 1,
                provides = {
                    "HW_RCC_STM32F1_CL",
                    "HW_DMA2_STM32F1",
                },
                file = "include/cfg/arch.h",
            },
            {
                macro = "STM32F107xC",
                brief = "STM32F107 256 kB",
                description = "STM32F106 256 kB.",
                requires = { "HW_MCU_STM32F107" },
                type = "integer",
                default = 1,
                provides = {
                    "HW_RCC_STM32F1_CL",
                    "HW_DMA2_STM32F1",
                    "HW_ETH_STM32",
                },
                file = "include/cfg/arch.h",
            },
        }
    },

    --
    -- STM32F Flexible Static Memory Controller
    --
    {
        name = "nutarch_cm3_stm32f_fsmc",
        brief = "STM32F Static Memory Controller",
        description = "Routines for configuration of the FSMC in STM32F controllers.\n"..
                      "The FSMC access access to SRAM, NAND- and NOR-Flash and PC-Memory Cards.",
        requires = { "HW_FSMC_STM32" },
--        sources = { "cm3/dev/stm/stm32_fsmc.c" },
    },

    --
    -- STM32F Flash Memory Controller
    --
    {
        name = "nutarch_cm3_stm32f_flash",
        brief = "STM32F Flash Memory Controller",
        description = "Routines for setup and programming STM32F series internal FLASH.\n",
        requires = { "HW_FLASHF1_STM32" },
        sources = { "cm3/dev/stm/stm32f1_flash.c" },
    },

}

