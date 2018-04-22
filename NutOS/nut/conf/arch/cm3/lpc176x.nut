--
-- Copyright (C) 2011 Ole Reinhardt, Thermotemp (ole.reinhardt@embedded-it.de)
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
-- NXP LPC176x Family Devices
--
-- $Id: lpc176x.nut $
--
--

lpc176x_device_class = { "LPC1758", "LPC1768" }

--
-- ********************************************************************************
-- LCP176x Family
-- ********************************************************************************
--

nutarch_cm3_lpc176x =
{
    --
    -- LPC17xx MCU Classes
    --
    {
        name = "nutarch_cm3_lpc176x_class",
        brief = "LPC176x Device Classes",
        requires = { "HW_MCU_LPC176x" },
        options =
        {
            {
                macro = "MCU_LPC1758",
                brief = "LPC1758",
                description = "NXP LPC1758",
                flavor = "booldata",
                exclusivity = lpc176x_device_class,
                provides =
                {
                    "HW_MCU_LPC1758",
                    "HW_UART0_LPC17xx",
                    "HW_UART1_LPC17xx",
                    "HW_UART2_LPC17xx",
                    "HW_UART3_LPC17xx",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_LPC1768",
                brief = "LPC1768",
                description = "NXP LPC1768",
                flavor = "booldata",
                exclusivity = lpc176x_device_class,
                provides =
                {
                    "HW_MCU_LPC1768",
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
    -- LPC176x PLL Configuration
    --
    {
        name = "nutarch_cm3_lpc176x_pll",
        brief = "LPC175x / LPC176x PLL Setup",
        description = "PLL configuration. Currently it is hard coded. May be user configurable later.\n",
        requires = { "HW_MCU_LPC176x", "TOOL_CC_CM3", "TOOL_GCC" },
        sources =
        {
            "cm3/dev/nxp/system_lpc176x.c",
            "cm3/dev/nxp/lpc176x_clk.c"
        }
    },

    --
    -- LPC176x Device Drivers
    --

    --
    -- LPC176x GPIO Interface
    --
    {
        name = "nutarch_cm3_lpc176x_gpio",
        brief = "LPC175x / LPC176x GPIO",
        description = "Generic port I/O API.",
        requires = { "HW_GPIO_LPC176x" },
        provides = { "HW_GPIO" },
        sources =
        {
            "cm3/dev/nxp/lpc176x_gpio.c",
            "cm3/dev/nxp/lpc17xx_gpioirq.c",
        }
    },

    --
    -- LPC176x DEBUG UART Interface
    --
    {
        name = "nutarch_cm3_lpc176x_debug",
        brief = "LPC176x Debug UART Driver",
        description = "Polling UART driver, which can be used from interrupt context\n",
        requires = { "HW_UART0_LPC17xx", "HW_UART1_LPC17xx", "HW_UART2_LPC17xx", "HW_UART3_LPC17xx" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        sources =
        {
            "cm3/dev/nxp/lpc176x_debug0.c",
            "cm3/dev/nxp/lpc176x_debug1.c",
            "cm3/dev/nxp/lpc176x_debug2.c",
            "cm3/dev/nxp/lpc176x_debug3.c"
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
    -- LPC176x USART0 Interface
    --
    {
        name = "nutarch_cm3_lpc176x_usart0",
        brief = "LPC175x / LPC176x USART0 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART0_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC"
        },
        sources =
        {
            "cm3/dev/nxp/lpc176x_usart0.c",
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
                macro = "USART0_MODE_IRDA",
                brief = "Set To IrDA Mode",
                description = "When selected, the USART will run in IrDA mode.",
                flavor = "booldata",
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
    -- LPC17xx USART1 Interface
    --
    {
        name = "nutarch_cm3_lpc176x_usart1",
        brief = "LPC175x / LPC176x USART1 Driver",
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
        sources =  { "cm3/dev/nxp/lpc176x_usart1.c",
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
            },
            {
                macro = "USART1_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            }
        }
    },
    --
    -- LPC17xx USART2 Interface
    --
    {
        name = "nutarch_cm3_lpc176x_usart2",
        brief = "LPC175x / LPC176x USART2 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART2_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC"
        },
        sources =  { "cm3/dev/nxp/lpc176x_usart2.c",
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
            },
            {
                macro = "USART2_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            }
        }
    },
    --
    -- LPC17xx USART3 Interface
    --
    {
        name = "nutarch_cm3_lpc176x_usart3",
        brief = "LPC175x / LPC176x USART3 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART3_LPC17xx", "DEV_IRQ_LPC17xx", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_LPC17xx",
            "DEV_UART_SPECIFIC"
        },
        sources =  {  "cm3/dev/nxp/lpc176x_usart3.c",
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
            },
            {
                macro = "USART3_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            }
        }
    },
    --
    -- LPC176x SPI
    --
    {
        name = "nutarch_cm3_lpc176x_spi",
        brief = "SPI Bus (LPC176x)",
        description = "LPC176x SPI driver, currently implemented as polling driver \n"..
                      "without interrupt or dma usage\n",
        requires = { "HW_SPI_LPC17xx" },
        provides = { "SPIBUS_CONTROLLER" },
        sources = { "cm3/dev/nxp/lpc176x_spi.c" },
    },
}

