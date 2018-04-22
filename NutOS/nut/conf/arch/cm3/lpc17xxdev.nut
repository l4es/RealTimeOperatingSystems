--
-- Copyright (C) 2012 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

nutarch_cm3_lpc17xx_devices =
{
    -- ***********************************
    --
    -- LPC17xx Device Drivers
    --
    -- ***********************************

    --
    -- LPC176x DEBUG Macro (Low-Level Debug UART definition)
    --
    {
        name = "nutarch_cm3_lpc17xx_debugmacro",
        brief = "LPC17xx Low-Level Debug UART macros for use in exception handlers",
        description = "Polling UART function (macro) to use in exception hanlders\n",
        requires = { "HW_UART0_LPC17xx", "HW_UART1_LPC17xx", "HW_UART2_LPC17xx", "HW_UART3_LPC17xx" },
        provides = { "DEBUG_MACRO"},
		sources = { "cm3/cmsis/cortex_debug.c" },
        options =
        {
            {
                macro = "DEBUG_MACRO",
                brief = "Enabled",
                description = "Check to enable debug output of exception handlers",
                flavor = "boolean",
                file = "include/cfg/cortex_debug.h"
            },
            {
                macro = "DEBUG_BACKTRACE",
                brief = "Enable call backtrace (experimental)",
                description = "Check to enable call stack back trace in case of an exception.\n\n"..
                              "In case of an exception the last 128 words on the stack will\n"..
                              "be printed out and a call trace will be generated and printed out\n"..
                              "(Address of the function entry and offset in the function).\n"..
                              "Together with the .map file and / or the assembler listing\n"..
                              "the call trace allows you to trace back your code flow in\n"..
                              "which led to the exception. The assembler listing can be\n"..
                              "created by 'arm-none-eabi-objdump -d myfile.elf'\n\n"..
                              "This function is highly experimental and relies on a very\n"..
                              "defined function entry code and existance of a stack\n"..
                              "frame pointer\n"..
                              "To ensure the correct function layout this option disables\n"..
                              "compiler optimisation and enables a stack frame pointer\n"..
                              "generation. The following compiler options will be set:\n\n"..
                              "-O0 -mtpcs-frame -mtpcs-leaf-frame -fno-omit-frame-pointer\n\n"..
                              "This will enlarge the resulkting binary size significantly\n"..
                              "any may influence execution performance.\n\n"..
                              "The backtrace will only work for functions compiled with\n"..
                              "above compiler parameters. E.g. it will very likely not work"..
                              "for any newlib functions.\n",
                flavor = "boolean",
                makedefs = { "MDEBUG+=-O0 -mtpcs-frame -mtpcs-leaf-frame -fno-omit-frame-pointer" },
                file = "include/cfg/cortex_debug.h"
            },
            {
                macro = "DEBUG_UART_NR",
                brief = "Debug UART",
                description = "Select the UART to use for low level debugging",
                type = "enumerated",
                choices = { "LPC_UART0", "LPC_UART1", "LPC_UART2", "LPC_UART3" },
				default = "LPC_UART0",
                file = "include/cfg/cortex_debug.h"
            }
        }
    },

    --
    -- LPC17xx RTC
    --
    {
        name = "nutarch_cm3_lpc17xx_rtc",
        brief = "LPC17xx RTC Driver",
        description = "LPC17xx RTC driver.",
        requires = { "HW_RTC_LPC17xx" },
        provides = { "DEV_RTC" },
        sources = { "cm3/dev/nxp/lpc17xx_rtc.c"},
    },
    --
    -- LPC17xx Watchdog Timer
    --
    {
        name = "nutarch_cm3_lpc17xx_wdt",
        brief = "LPC17xx Watchdog Timer",
        requires = { "HW_WDT_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_wdt.c"}
    },
    --
    -- LPC17xx Flash Memory Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_iap",
        brief = "LPC17xx In Application Flash Programming API (IAP)",
        description = "Routines for setup and programming LPC17x series internal FLASH.\n",
        requires = { "HW_FLASH_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_iap.c" }
    },
    --
    -- LPC17xx General purpose DMA Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_gpdma",
        brief = "LPC17xx General purpose DMA Controller API",
        description = "Routines for setup and programming LPC17x series GPDMA controller.\n",
        requires = { "HW_GPDMA_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_gpdma.c"}
    },
    --
    -- LPC17xx SSP Controller 0, 1, 2
    --
    {
        name = "nutarch_cm3_lpc176x_ssp",
        brief = "SSP Bus 0, 1, (2)",
        description = "LPC176x SSP-SPI driver, currently implemented as polling driver \n"..
                      "without interrupt or dma usage\n",
        requires = { "HW_SSP_LPC17xx" },
        provides = { "SPIBUS_CONTROLLER" },
        sources = { "cm3/dev/nxp/lpc17xx_ssp.c" },
    },
    --
    -- LPC17xx EMAC
    --
    {
        name = "nutarch_cm3_lpc17xx_emac",
        brief = "LPC17xx EMAC Driver",
        description = "LAN driver for LPC176x, LPC177x_8x etc.",
        requires = { "HW_EMAC_LPC17xx", "NUT_EVENT", "NUT_TIMER" },
        provides = { "NET_MAC" },
        sources = { "cm3/dev/nxp/lpc17xx_emac.c"},
        options =
        {
            {
                macro = "NUT_THREAD_NICRXSTACK",
                brief = "Receiver Thread Stack",
                description = "Number of bytes to be allocated for the stack of the NIC receive thread.",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_NUM_RX_FRAG",
                brief = "Receive Buffers",
                description = "Number of 128 byte DMA receive buffers.\n"..
                              "Increase to handle high traffic situations.\n"..
                              "Decrease to handle low memory situations.\n"..
                              "Default is 64.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_NUM_TX_FRAG",
                brief = "Transmit Buffer Size",
                description = "Number of 128 byte DMA transmit buffers.\n"..
                              "Can be decreased in low memory situations. Be aware, "..
                              "that this may break your network application. Do not "..
                              "change this without exactly knowing the consequences.\n"..
                              "Default is 48.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_ALLOC_DMA_BUFFER_FROM_HEAP",
                brief = "Allocate EMAC DMA buffers from heap instead using internal SRAM",
                description = "If you enable this option, the DMA buffers for the EMAC "..
                              "RX and TX channel will be allocated from heap.\n\n"..
                              "By default, the internal AHB SRAM Bank 1 (16K) is used.\n\n"..
                              "If the configured buffer size exceeds 16K, this option "..
                              "is selected automatically\n",
                flavor = "booldata",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_LINK_LOOPS",
                brief = "Link Polling Loops",
                description = "This simple implementation runs a dumb polling loop "..
                              "while waiting for the Ethernet link status.\n"..
                              "If you experience link problems, increasing this value "..
                              "may help.\n"..
                              "Default is 10000.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
        }
    },
    --
    -- LPC17xx Signal handler
    --
    {
        name = "nutarch_cm3_lpc17xx_sig",
        brief = "LPC17xx Signals ",
        description = "Helper to create the sig_DEVXX entries",
        sources = { "cm3/dev/nxp/ih_lpc17xx.c"},
        options =
        {
            {
                macro = "HW_WDT_LPC17xx",
                brief = "WDT Availability",
                description = "WDT Availability",
                requires = { "HW_WDT_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART0_LPC17xx",
                brief = "UART 0 Availability",
                description = "UART 0 Availability",
                requires = { "HW_UART0_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART1_LPC17xx",
                brief = "UART 1 Availability",
                description = "UART 1 Availability",
                requires = { "HW_UART1_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART2_LPC17xx",
                brief = "UART 2 Availability",
                description = "UART 2 Availability",
                requires = { "HW_UART2_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART3_LPC17xx",
                brief = "UART 3 Availability",
                description = "UART 3 Availability",
                requires = { "HW_UART3_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART4_LPC17xx",
                brief = "UART 4 Availability",
                description = "UART 4 Availability",
                requires = { "HW_UART4_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C0_LPC17xx",
                brief = "I2C 0 Availability",
                description = "I2C 0 Availability",
                requires = { "HW_I2C0_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C1_LPC17xx",
                brief = "I2C 1 Availability",
                description = "I2C 1 Availability",
                requires = { "HW_I2C1_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C2_LPC17xx",
                brief = "I2C 2 Availability",
                description = "I2C 2 Availability",
                requires = { "HW_I2C2_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SSP0_LPC17xx",
                brief = "SSP0 Availability",
                description = "SSP0 Availability",
                requires = { "HW_SSP0_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SSP1_LPC17xx",
                brief = "SSP1 Availability",
                description = "SSP1 Availability",
                requires = { "HW_SSP1_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SSP2_LPC17xx",
                brief = "SSP2 Availability",
                description = "SSP2 Availability",
                requires = { "HW_SSP2_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_RTC_LPC17xx",
                brief = "RTC Availability",
                description = "RTC Availability",
                requires = { "HW_RTC_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_MCI_LPC177x_8x",
                brief = "MCI Availability for LPC177x_8x",
                description = "MCI Availability",
                requires = { "HW_MCI_LPC177x_8x" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_EMAC_LPC17xx",
                brief = "EMAC Availability",
                description = "EMAC Availability",
                requires = { "HW_EMAC_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_GPIO_LPC177x_8x",
                brief = "GPIO Availability for LPC177x_8x",
                description = "GPIO Availability for LPC177x_8x",
                requires = { "HW_GPIO_LPC177x_8x" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_GPDMA_LPC17xx",
                brief = "GPDMA Availability",
                description = "GPDMA Availability",
                requires = { "HW_GPDMA_LPC17xx" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
        }
    }
}

