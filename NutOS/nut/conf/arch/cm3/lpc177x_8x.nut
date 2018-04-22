--
-- Copyright (C) 2011 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
-- NXP LPC17xx Family Devices
--
-- $Id: lpc177x_8x.nut $
--
--

lpc177x_8x_device_class = { "LPC1778", "LPC1788", "LPC4078", "LPC4088" }

--
-- ********************************************************************************
-- LCP17xx Family
-- ********************************************************************************
--

nutarch_cm3_lpc177x_8x_lpc407x_8x=
{
    --
    -- LPC177x_8x MCU Classes
    --
    {
        name = "nutarch_cm3_lpc177x_8x_class",
        brief = "LPC177x_8x / LPC407x_8x Device Classes",
        requires = { "HW_MCU_LPC177x_8x_LPC407x_8x" },
        options =
        {
            {
                macro = "MCU_LPC1778",
                brief = "LPC1778",
                description = "NXP LPC1778",
                flavor = "booldata",
                exclusivity = lpc177x_8x_device_class,
                provides =
                {
                    "HW_MCU_LPC1778",
                    "HW_UART0_LPC17xx",
                    "HW_UART1_LPC17xx",
                    "HW_UART2_LPC17xx",
                    "HW_UART3_LPC17xx",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_LPC1788",
                brief = "LPC1788",
                description = "NXP LPC1788",
                flavor = "booldata",
                exclusivity = lpc177x_8x_device_class,
                provides =
                {
                    "HW_MCU_LPC1788",
                    "HW_UART0_LPC17xx",
                    "HW_UART1_LPC17xx",
                    "HW_UART2_LPC17xx",
                    "HW_UART3_LPC17xx",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_LPC4078",
                brief = "LPC4078",
                description = "NXP LPC4078",
                flavor = "booldata",
                exclusivity = lpc177x_8x_device_class,
                provides =
                {
                    "HW_MCU_LPC4088",
                    "HW_UART0_LPC17xx",
                    "HW_UART1_LPC17xx",
                    "HW_UART2_LPC17xx",
                    "HW_UART3_LPC17xx",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_LPC4088",
                brief = "LPC4088",
                description = "NXP LPC4088",
                flavor = "booldata",
                exclusivity = lpc177x_8x_device_class,
                provides =
                {
                    "HW_MCU_LPC4088",
                    "HW_UART0_LPC17xx",
                    "HW_UART1_LPC17xx",
                    "HW_UART2_LPC17xx",
                    "HW_UART3_LPC17xx",
                },
                file = "include/cfg/arch.h"
            }
        }
    },
    --
    -- LPC177x_8x External Memory Controller
    --
    {
        name = "nutarch_cm3_lpc177x_8x_emc",
        brief = "LPC177x_8x External Memory Controller",
        description = "Routines for setup and programming LPC177x_8x series external memory bus and SDRAM support.\n",
        requires = { "HW_EMC_LPC177x_8x" },
        sources  = { "cm3/dev/nxp/lpc177x_8x_emc.c" }
    },
    --
    -- LPC177x_8x EEPROM Memory
    --
    {
        name = "nutarch_cm3_lpc177x_8x_eeprom",
        brief = "LPC177x_8x EEPROM Access",
        requires = { "HW_EEPROM_LPC177x_8x" },
        provides = { "DEV_NVMEM" },
        sources = { "cm3/dev/nxp/lpc177x_8x_eeprom.c" }
    },
    --
    -- LPC177x_8x PLL Configuration
    --
    {
        name = "nutarch_cm3_lpc177x_8x_pll",
        brief = "LPC177x_8x PLL Setup",
        description = "PLL configuration. Currently it is hard coded. May be user configurable later.\n",
        requires = { "HW_MCU_LPC177x_8x", "TOOL_CC_CM3", "TOOL_GCC" },
        sources =
        {
            "cm3/dev/nxp/system_lpc177x_8x.c",
            "cm3/dev/nxp/lpc177x_8x_clk.c"
        }
    },

    --
    -- LPC407x_8x PLL Configuration
    --
    {
        name = "nutarch_cm3_lpc407x_8x_pll",
        brief = "LPC407x_8x PLL Setup",
        description = "PLL configuration. Currently it is hard coded. May be user configurable later.\n",
        requires = { "HW_MCU_LPC407x_8x", "TOOL_CC_CM3", "TOOL_GCC" },
        sources =
        {
            "cm3/dev/nxp/system_lpc407x_8x.c",
            "cm3/dev/nxp/lpc407x_8x_clk.c"
        }
    },


    --
    -- LPC177x_8x Device Drivers
    --

    --
    -- LPC177x_8x GPIO Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_gpio",
        brief = "LPC177x_8x GPIO",
        description = "Generic port I/O API.",
        requires = { "HW_GPIO_LPC177x_8x" },
        provides = { "HW_GPIO" },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_gpio.c",
            "cm3/dev/nxp/lpc17xx_gpioirq.c",
        }
    },

    --
    -- LPC177x_8x DEBUG UART Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_debug",
        brief = "LPC177x_8x Debug UART Driver",
        description = "Polling UART driver, which can be used from interrupt context\n",
        requires = { "HW_UART0_LPC17xx", "HW_UART1_LPC17xx", "HW_UART2_LPC17xx", "HW_UART3_LPC17xx" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_debug0.c",
            "cm3/dev/nxp/lpc177x_8x_debug1.c",
            "cm3/dev/nxp/lpc177x_8x_debug2.c",
            "cm3/dev/nxp/lpc177x_8x_debug3.c"
        },
        options =
        {
            {
                macro = "DEBUG_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the debug UART is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            }
        }
    },

    --
    -- LPC177x_8x USART0 Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_usart0",
        brief = "LPC177x_8x USART0 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART0_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC",
            "DEV_UART0_GPIO_RTS",
            "DEV_UART0_GPIO_CTS"
        },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_usart0.c",
        },
        options =
        {
            {
                macro = "USART0_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART0_SUPPORT_IRQ",
                brief = "Use Interrupt Transfer",
                description = "When selected, the driver will use interrupt for transfers. ",
                flavor = "booldata",
                provides = { "USART0_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART0_SUPPORT_DMA",
                brief = "Support DMA Blocktransfer",
                description = "When selected, the driver can use DMA for block transfers.\n"..
                              "Block transfers can be enabled or disabled by calling _ioctl "..
                              "with the following parameters:\n"..
                              "UART_SETBLOCKREAD\nUART_SETBLOCKWRITE",
                flavor = "booldata",
                requires = { "USART0_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART0_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            }
        }
    },
    --
    -- LPC177x_8x USART1 Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_usart1",
        brief = "LPC177x_8x USART1 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART1_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC",
            "DEV_UART1_GPIO_RTS",
            "DEV_UART1_GPIO_CTS"
        },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_usart1.c",
        },
        options =
        {
            {
                macro = "USART1_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_SUPPORT_IRQ",
                brief = "Use Interrupt Transfer",
                description = "When selected, the driver will use interrupt for transfers. ",
                flavor = "booldata",
                provides = { "USART1_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_SUPPORT_DMA",
                brief = "Support DMA Blocktransfer",
                description = "When selected, the driver can use DMA for block transfers.\n"..
                              "Block transfers can be enabled or disabled by calling _ioctl "..
                              "with the following parameters:\n"..
                              "UART_SETBLOCKREAD\nUART_SETBLOCKWRITE",
                flavor = "booldata",
                requires = { "USART1_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            }
        }
    },
    --
    -- LPC177x_8x USART2 Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_usart2",
        brief = "LPC177x_8x USART2 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART2_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC",
            "DEV_UART2_GPIO_RTS",
            "DEV_UART2_GPIO_CTS"
        },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_usart2.c",
        },
        options =
        {
            {
                macro = "USART2_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_SUPPORT_IRQ",
                brief = "Use Interrupt Transfer",
                description = "When selected, the driver will use interrupt for transfers. ",
                flavor = "booldata",
                provides = { "USART2_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_SUPPORT_DMA",
                brief = "Support DMA Blocktransfer",
                description = "When selected, the driver can use DMA for block transfers.\n"..
                              "Block transfers can be enabled or disabled by calling _ioctl "..
                              "with the following parameters:\n"..
                              "UART_SETBLOCKREAD\nUART_SETBLOCKWRITE",
                flavor = "booldata",
                requires = { "USART2_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            }
        }
    },
    --
    -- LPC177x_8x USART3 Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_usart3",
        brief = "LPC177x_8x USART3 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART3_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC",
            "DEV_UART3_GPIO_RTS",
            "DEV_UART3_GPIO_CTS"
        },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_usart3.c",
        },
        options =
        {
            {
                macro = "USART3_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_SUPPORT_IRQ",
                brief = "Use Interrupt Transfer",
                description = "When selected, the driver will use interrupt for transfers. ",
                flavor = "booldata",
                provides = { "USART3_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_SUPPORT_DMA",
                brief = "Support DMA Blocktransfer",
                description = "When selected, the driver can use DMA for block transfers.\n"..
                              "Block transfers can be enabled or disabled by calling _ioctl "..
                              "with the following parameters:\n"..
                              "UART_SETBLOCKREAD\nUART_SETBLOCKWRITE",
                flavor = "booldata",
                requires = { "USART3_SUPPORT_IRQ" },
                file = "include/cfg/uart.h"
            }
        }
    },

    --
    -- LPC177x_8x MCI SDIO Interface
    --
    {
        name = "nutarch_cm3_lpc177x_8x_mci",
        brief = "LPC177x_8x MCI (SDIO) MMC driver",
        description = "Low level MMC interface for LPC177x_8x",
        requires = { "HW_MCI_LPC177x_8x" },
        provides = { "DEV_MMCLL" },
        sources =
        {
            "cm3/dev/nxp/lpc177x_8x_mmcard_sdio.c",
            "cm3/dev/nxp/lpc177x_8x_mci.c",
        },
        options =
        {
            {
                macro = "BRD_MCI_POWERED_ACTIVE_LEVEL",
                brief = "MCI power active level",
                description = "MCI power active level -> set to (0) or (1) depending your board HW.",
                type = "enumerated",
                choices = { "1", "0" },
                file = "include/cfg/mmci.h"
            },
        },
    },

    --
    -- LPC17xx I2C Controller 0, 1, 2
    --
    {
        name = "nutarch_cm3_lpc17xx_i2c0",
        brief = "LPC17xx I2C bus 0 API",
        description = "Routines for setup and programming LPC17xx series I2C controller 0.\n",
        requires = { "HW_I2C_LPC17xx", },
--        sources = { "cm3/dev/nxp/lpc17xx_i2cbus0.c"}
    },
    {
        name = "nutarch_cm3_lpc17xx_i2c1",
        brief = "LPC17xx I2C bus 1 API",
        description = "Routines for setup and programming LPC17xx series I2C controller 1.\n",
        requires = { "HW_I2C_LPC17xx", },
--        sources = { "cm3/dev/nxp/lpc17xx_i2cbus1.c"}
    },
    {
        name = "nutarch_cm3_lpc17xx_i2c2",
        brief = "LPC17xx I2C bus 2 API",
        description = "Routines for setup and programming LPC17xx series I2C controller 2.\n",
        requires = { "HW_I2C_LPC17xx", },
--        sources = { "cm3/dev/nxp/lpc17xx_i2cbus2.c"}
    },
}
