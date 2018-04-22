--
-- Copyright (C) 2015 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de.
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
--
-- Retrieve I2C1 pins available on the device.
--
function GetI2c1Sda()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB07", "PB09", "PF00", "PA10", "PB11", "PA12", "PB14", "PF07"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PB07", "PB09", "PA14"  }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB07", "PB09", "PC07", "PD13", "PD15", "PF15"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB07", "PB09", "PG13"}
    end
    return { "PA07", "PA09" }
end
function GetI2c1Scl()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB06", "PB08", "PF01", "PA09", "PB10", "PA11", "PB13", "PF06" }
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PB06", "PB08", "PA15" }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB06", "PB08", "PC06", "PD12", "PD14", "PF14"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB06", "PB08", "PG14"}
    end
    return { "PA06", "PA08" }
end
function GetI2c1Smba()
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PIN_NONE", "PB05", "PD11", "PF13"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB05", "PG15"}
    end
    return { "PIN_NONE", "PB05" }
end

--
-- Retrieve I2C2 pins available on the device.
--
function GetI2c2Sda()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PB11", "PB14"}
    end
    if c_is_provided("HW_MCU_STM32L1") then
        return "PB11"
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return {"PB11", "PB14", "PF00"}
    end
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB11", "PB14", "PA12"}
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return { "PB11", "PF00", "PH05"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PA10", "PF00", "PF07" }
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return { "PB11", "PB03" }
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return { "PB11", "PB03" , "PB09" }
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return { "PB11", "PF00" , "PH05" }
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return { "PB11", "PF00" , "PH05" }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB11", "PB03" , "PC12" , "PF00"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return {"PB11", "PF00", "PH05"}
    end
end
function GetI2c2SdaDefault()
    if c_is_provided("HW_MCU_STM32F3") then
        return "PA10"
    end
    return "PB11"
end
function GetI2c2Scl()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PB10", "PB13"}
    end
    if c_is_provided("HW_MCU_STM32L1") then
        return "PB10"
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return {"PB10", "PB13", "PF01"}
    end
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB10", "PB13", "PA11"}
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return { "PB10", "PF01", "PH04"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PA09", "PF01", "PF06" }
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return "PB10"
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return "PB10"
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return { "PB10", "PF01" , "PH04" }
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return { "PB10", "PF01" , "PH04" }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB10", "PF01"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return {"PB10", "PF01", "PH05"}
    end
end
function GetI2c2SclDefault()
    if c_is_provided("HW_MCU_STM32F3") then
        return "PA09"
    end
    return "PB10"
end
function GetI2c2Smba()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PIN_NONE", "PB12"}
    end
    if c_is_provided("HW_MCU_STM32L1") then
        return { "PIN_NONE", "PB12"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return {"PB12", "PF02"}
    end
    if c_is_provided("HW_MCU_STM32F0") then
        return "PIN_NONE"
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return { "PIN_NONE", "PB12", "PF02", "PH06"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PIN_NONE" , "PA08", "PB12", "PF02" }
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return "PB12"
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return "PB12"
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return { "PB12", "PF02" , "PH06" }
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return { "PB12", "PF02" , "PH06" }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB12", "PF02"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return {"PB12", "PF02", "PH06"}
    end
end
--
-- Retrieve I2C3 pins available on the device.
--
function GetI2c3Sda()
    if c_is_provided("HW_MCU_STM32F2") then
        return { "PC09", "PH08"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PC09", "PB05"}
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return { "PC09", "PB04"}
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return { "PC09", "PB04", "PB08"}
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return { "PC09", "PH08"}
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return { "PC09", "PH08"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PC09", "PB04"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return {"PC09", "PH08"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return { "PC01", "PG08"}
    end
end
function GetI2c3Scl()
    if c_is_provided("HW_MCU_STM32F2") then
        return { "PA08", "PH07"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return "PA08"
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return "PA08"
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return "PA08"
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return { "PA08", "PH07"}
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return { "PA08", "PH07"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return "PA08"
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return { "PA08", "PH07"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PC00", "PG07"}
    end
end
function GetI2c3Smba()
    if c_is_provided("HW_MCU_STM32F2") then
        return {"PIN_NONE", "PA08" }
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return {"PIN_NONE", "PA08" }
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return {"PIN_NONE", "PA08" }
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return {"PIN_NONE", "PA08" , "PH09"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return {"PIN_NONE", "PA08", "PA09"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return {"PIN_NONE", "PB02", "PG06"}
    end
end
nutarch_cm3_stm32_i2c_devices =
{
    -- ******************************************
    --
    -- STM32 I2C Device Configuration and drivers
    --
    -- ******************************************
    --
    -- STM32 I2C Bus 1 Configuration
    --
    {
        name = "nutarch_cm3_stm32f_i2c1",
        brief = "STM32 I2C Bus 1 Controller",
        description = "STM32 hardware I2C1 support.",
        requires = { "HW_I2C1_STM32" },
        provides = { "DEV_TWI" },
        options =
        {
            {
                macro = "I2C1_AS_DEFAULT",
                brief = "Use as default I2C",
                description = "Enabling this options reconfigures TwiXxx() Macros to use this bus.\n"..
                              "This is for compatibility with Nut/OS.",
                exclusivity = { "I2C1_AS_DEFAULT", "I2C2_AS_DEFAULT" },
                flavor = "booldata",
                file = "include/cfg/twi.h"

            },
            {
                macro = "I2C1_USE_DMA",
                brief = "Use DMA with I2C1",
                description = "Enabling this options configures I2C bus to use DMA.\n\n"..
                              "Errata Sheet 14574 Rev.9:\n"..
                              "This should be default for STM32F10xxB/8 devices according this Errata "..
                              "to overcome the mentioned limitations.",
                flavor = "booldata",
                file = "include/cfg/twi.h"
            },
       },
    },

    --
    -- STM32 I2C Bus 2 Configuration
    --
    {
        name = "nutarch_cm3_stm32f_i2c2",
        brief = "STM32 I2C Bus 2 Controller",
        description = "STM32 hardware I2C support.\n\n This bus cannot be remapped on F1/L1. Pins used are:\n"..
                      "SCL PB10\nSDA PB11\n",
        requires = { "HW_I2C2_STM32" },
        provides = { "DEV_TWI" },
        options =
        {
            {
                macro = "I2C2_AS_DEFAULT",
                brief = "Use as default I2C",
                description = "Enabling this options reconfigures TwiXxx() Macros to use this bus.\n"..
                              "This is for compatibility with Nut/OS.",
                exclusivity = { "I2C1_AS_DEFAULT", "I2C2_AS_DEFAULT" },
                flavor = "booldata",
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C2_USE_DMA",
                brief = "Use DMA with I2C2",
                description = "Enabling this options configures I2C bus to use DMA.\n"..
                              "This should be default for STM32F10xxB/8 devices according Errata Sheet 14574 Rev.9.\n"..
                              "to overcome the mentioned limitations.",
                flavor = "booldata",
                file = "include/cfg/twi.h"
            },
        }
    },

     -- I2C Pin Configuration
     {
        name = "nutarch_cm3_stm32_i2c1_pin",
        brief = "STM32 I2C1 Pin mappings",
        description = "STM32 I2C1 Pin mappings",
        options =
        {
            {
                macro = "I2C1_REMAP_I2C",
                brief = "Use Alternate Pins",
                description = "Choose Pin remap. Default = 0 means no remap.\n\n"..
                              "Default 0 Pin Mapping is:\n SCL PB06\n SDA PB07\n"..
                              "Remap   1 Pin Mapping is:\n SCL PB08\n SDA PB09\n",
                requires = { "HW_GPIO_STM32V1" },
                type = "enumerated",
                choices = {"0", "1"},
                default = "0",
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C1_USE_SMBA",
                brief = "Use I2C1 SMBA on F1",
                description = "Use I2C1 SMBA on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "I2C1_SDA",
                brief = "I2C1 SDA GPIO",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C1 SDA according to availability.",
                type = "enumerated",
                default = "PB07",
                choices = function() return GetI2c1Sda() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C1_SCL",
                brief = "I2C1 SCL GPIO",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C1 SCL Pin according to availability.",
                type = "enumerated",
                default = "PB06",
                choices = function() return GetI2c1Scl() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C1_SMBA",
                brief = "I2C1 SCL GPIO",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C1 SMBA PA5 or -1 to not use SMBA.",
                default = "PIN_NONE",
                type = "enumerated",
                choices = function() return GetI2c1Smba() end,
                file = "include/cfg/twi.h"
            }
         }
     },
     {
        name = "nutarch_cm3_stm32_i2c2_pin",
        brief = "STM32 I2C2 Pin mappings",
        description = "STM32 I2C2 Pin mappings",
        options =
        {
            {
                macro = "I2C2_SDA",
                brief = "I2C2 SDA",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C2 SDA according to availability. Default: PA10 on F3, PB11 else.",
                type = "enumerated",
                default = function() return GetI2c2SdaDefault() end,
                choices = function() return GetI2c2Sda() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C2_SCL",
                brief = "I2C2 SCL",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C2 SCL Pin according to availability. Default: PA9 on F3, PB10 else.",
                type = "enumerated",
                choices = function() return GetI2c2Scl() end,
                default = function() return GetI2c2SclDefault() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C2_SMBA",
                brief = "I2C2 SMBA",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C2 SMBA Pin according to availability. Choose -1 to not use SMBA.",
                default = "PIN_NONE",
                choices = function() return GetI2c2Smba() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C2_USE_SMBA",
                brief = "Use I2C2 SMBA on F1",
                description = "Use I2C2 SMBA on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
         }
    },
     {
        name = "nutarch_cm3_stm32_i2c3_pin",
        brief = "STM32 I2C3 Pin mappings",
        description = "STM32 I2C3 Pin mappings",
        requires = { "HW_I2C3_STM32" },
        options =
        {
            {
                macro = "I2C3_SDA",
                brief = "I2C3 SDA",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C3 SDA according to availability. Default: PC09",
                type = "enumerated",
                default = "PC09",
                choices = function() return GetI2c3Sda() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C3_SCL",
                brief = "I2C3 SCL",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C3 SCL Pin according to availability. Default: PA08.",
                type = "enumerated",
                default = "PA08",
                choices = function() return GetI2c3Scl() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C3_SMBA",
                brief = "I2C3 SMBA",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C3 SMBA Pin according to availability. Default: PIN_NONE to not use SMBA.",
                type = "enumerated",
                default = "PIN_NONE",
                choices = function() return GetI2c3Smba() end,
                file = "include/cfg/twi.h"
            }
         }
    },
    {
        name = "nutarch_cm3_stm32_i2c4_pin",
        brief = "STM32 I2C4 Pin mappings",
        description = "STM32 I2C4 Pin mappings",
        requires = { "HW_I2C4_STM32" },
        options =
        {
            {
                macro = "I2C4_SDA",
                brief = "I2C4 SDA",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C4 SDA according to availability. Default: PC09",
                type = "enumerated",
                default = "PC09",
                choices = function() return GetI2c4Sda() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C4_SCL",
                brief = "I2C4 SCL",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C4 SCL Pin according to availability. Default: PA08.",
                type = "enumerated",
                default = "PA08",
                choices = function() return GetI2c4Scl() end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C4_SMBA",
                brief = "I2C4 SMBA",
                requires = { "HW_GPIO_STM32V2" },
                description = "Choose I2C4 SMBA Pin according to availability. Default: PIN_NONE to not use SMBA.",
                type = "enumerated",
                default = "PIN_NONE",
                choices = function() return GetI2c4Smba() end,
                file = "include/cfg/twi.h"
            }
        }
    }
}
