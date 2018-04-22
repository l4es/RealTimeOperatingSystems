--
-- Copyright (C) 2004-2007 by egnite Software GmbH. All rights reserved.
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

-- Operating system functions
--
-- $Log$
-- Revision 1.38  2009/02/06 15:51:38  haraldkipp
-- Added NutReset support for SAM9260.
-- Removed stack size defaults.
--
-- Revision 1.37  2008/10/23 08:50:43  haraldkipp
-- Prepared AT91 UART hardware handshake.
--
-- Revision 1.36  2008/10/05 17:01:15  haraldkipp
-- Added Helix MP3 decoder and Hermite re-sampler. Do not miss to read
-- the license texts.
--
-- Revision 1.35  2008/10/03 11:31:27  haraldkipp
-- Added TWI support for the AT91SAM9260.
--
-- Revision 1.34  2008/09/23 07:24:34  haraldkipp
-- Added support for remaining AT91SAM7 familiy members.
-- Added support for AT91 SDRAM and reset controller.
--
-- Revision 1.33  2008/08/11 11:51:19  thiagocorrea
-- Preliminary Atmega2560 compile options, but not yet supported.
-- It builds, but doesn't seam to run properly at this time.
--
-- Revision 1.32  2008/08/06 12:51:08  haraldkipp
-- Added support for Ethernut 5 (AT91SAM9XE reference design).
--
-- Revision 1.31  2008/04/18 13:24:57  haraldkipp
-- Added Szemzo Andras' RS485 patch.
--
-- Revision 1.30  2008/02/15 17:05:53  haraldkipp
-- AT91SAM7SE512 support added. Global MCU list simplifies exclusivity
-- attribute. HW_EBI_AT91 added for MCUs with external bus interface.
--
-- Revision 1.29  2007/10/04 21:06:11  olereinhardt
-- Support for SAM7S256 added
--
-- Revision 1.28  2007/09/06 19:41:30  olereinhardt
-- Added HW_PDC_AT91 to AT91SAM7X256 platform
--
-- Revision 1.27  2007/04/12 09:20:00  haraldkipp
-- ATmega2561 no longer bound to ICCAVR.
--
-- Revision 1.26  2007/02/15 16:24:34  haraldkipp
-- Now the periodic interrupt timer may become the system clock.
--
-- Revision 1.25  2006/10/05 17:16:04  haraldkipp
-- Added exclusivity attribute.
--
-- Revision 1.24  2006/09/29 12:40:36  haraldkipp
-- PDC provision added for AT91 devices.
--
-- Revision 1.23  2006/09/05 12:29:59  haraldkipp
-- SPI and MCI support added for SAM9260.
--
-- Revision 1.22  2006/08/31 19:04:08  haraldkipp
-- Added support for the AT91SAM9260 and Atmel's AT91SAM9260 Evaluation Kit.
--
-- Revision 1.21  2006/07/26 11:19:06  haraldkipp
-- Defining AT91_PLL_MAINCK will automatically determine SAM7X clock by
-- reading PLL settings.
-- Added MMC/SD-Card support for AT91SAM7X Evaluation Kit.
--
-- Revision 1.20  2006/07/05 08:01:56  haraldkipp
-- SAM7 EMAC attribute added.
--
-- Revision 1.19  2006/06/28 17:22:34  haraldkipp
-- Make it compile for AT91SAM7X256.
--
-- Revision 1.18  2006/02/23 15:40:59  haraldkipp
-- GPIO provision added for all MCUs except Gameboy Advance.
--
-- Revision 1.17  2006/02/08 15:20:21  haraldkipp
-- ATmega2561 Support
--
-- Revision 1.16  2006/01/23 19:50:48  haraldkipp
-- Dummy NVMEM configuration for GBA added.
--
-- Revision 1.15  2006/01/23 17:28:30  haraldkipp
-- HW_NVMEM is now provided by specific modules.
--
-- Revision 1.14  2006/01/05 16:45:04  haraldkipp
-- ATmega2561 CPU added, but not yet supported.
--
-- Revision 1.13  2005/10/07 22:09:23  hwmaier
-- Changed HWDEF= to HWDEF+= so more defs can be added to this macros from other LUA scripts.
--
-- Revision 1.12  2005/10/04 05:47:26  hwmaier
-- Added CAN driver for AT90CAN128
--
-- Revision 1.11  2005/07/26 15:41:05  haraldkipp
-- All target dependent code is has been moved to a new library named
-- libnutarch. Each platform got its own script.
--
-- Revision 1.10  2005/02/19 22:48:12  hwmaier
-- no message
--
-- Revision 1.9  2005/01/22 19:20:21  haraldkipp
-- Fixes Ethernut 1.3G memory bug
--
-- Revision 1.8  2004/10/03 18:37:39  haraldkipp
-- GBA support
--
-- Revision 1.7  2004/09/22 08:19:42  haraldkipp
-- No ATmega103 with LAN91C111
--
-- Revision 1.6  2004/09/19 15:17:37  haraldkipp
-- ICCAVR initialization added to build
--
-- Revision 1.5  2004/09/08 10:18:54  haraldkipp
-- C startup on ARM platforms only
--
-- Revision 1.4  2004/09/07 19:07:45  haraldkipp
-- MCU type in NutConf.mk and cstartup added.
--
-- Revision 1.3  2004/08/18 16:05:26  haraldkipp
-- Use consistent directory structure
--
-- Revision 1.2  2004/08/18 13:46:08  haraldkipp
-- Fine with avr-gcc
--
-- Revision 1.1  2004/08/03 15:09:30  haraldkipp
-- Another change of everything
--
-- Revision 1.1  2004/06/07 16:32:45  haraldkipp
-- First release
--
--

nutarch =
{
    {
        name = "nutarch_mcu",
        brief = "Target CPU",
        description = "Select one only.",
        options =
        {
            {
                macro = "MCU_ATMEGA103",
                brief = "Atmel ATmega 103",
                description = "8-bit RISC microcontroller with 128K bytes flash, 4K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, UART, 3 timers, "..
                              "8-channel ADC and SPI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_AVR_HAVE_UART0",
                    "HW_AVR_HAVE_UART1",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs = { "MCU=$(MCU_ATMEGA103)", "HWDEF+=-D__HARVARD_ARCH__" }
            },
            {
                macro = "MCU_ATMEGA128",
                brief = "Atmel ATmega 128",
                description = "8-bit RISC microcontroller with 128K bytes flash, 4K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 2 USARTs, 4 timers, "..
                              "8-channel ADC, SPI and TWI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_MCU_ATMEGA128",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_AVR_HAVE_UART0",
                    "HW_AVR_HAVE_UART1",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs = { "MCU=$(MCU_ATMEGA128)", "HWDEF+=-D__HARVARD_ARCH__" }
            },
            {
                macro = "MCU_AT90CAN128",
                brief = "Atmel AT90CAN128",
                description = "8-bit RISC microcontroller with 128K bytes flash, 4K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 2 USARTs, 4 timers, "..
                              "8-channel ADC, SPI, TWI and CAN controller.\n"..
                              "\n"..
                              "Please configure appropriate stack settings in category "..
                              "Memory Management. "..
                              "Due to a bug for silicon revisions C of the AT90CAN128 MCU, "..
                              "the code stack must reside in internal RAM. Refer to "..
                              "AT90CAN128 Datasheet Rev. 4250F-CAN-04/05 - Errata Rev C ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_AVR_HAVE_UART0",
                    "HW_AVR_HAVE_UART1",
                    "HW_CAN_AVR",
                    "HW_GPIO"
                },
                makedefs = { "MCU=at90can128", "HWDEF+=-D__HARVARD_ARCH__" },
                options =
                {
                }
            },
            {
                macro = "MCU_AT90USB1287",
                brief = "Atmel AT90USB1287",
                description = "8-bit RISC microcontroller with 128K bytes flash, 8K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 1 USARTs, 4 timers, "..
                              "8-channel ADC, SPI, TWI and USB controller.\n"..
                              "\n"..
                              "Please configure appropriate stack settings in category "..
                              "Memory Management. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_AVR_HAVE_UART1",
                    "HW_GPIO"
                },
                makedefs = { "MCU=at90usb1287", "HWDEF+=-D__HARVARD_ARCH__" },
                options =
                {
                }
            },
            {
                macro = "MCU_ATMEGA2560",
                brief = "Atmel ATmega 2560",
                description = "8-bit RISC microcontroller with 256K bytes flash, 8K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 4 USARTs, 6 timers, "..
                              "8-channel ADC, SPI and TWI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_MCU_ATMEGA2560",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_AVR_HAVE_UART0",
                    "HW_AVR_HAVE_UART1",
--                    "HW_AVR_HAVE_UART2",
--                    "HW_AVR_HAVE_UART3",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs =
                {
                    "MCU = $(MCU_ATMEGA2560)",
                    "HWDEF += -D__HARVARD_ARCH__",
                    "HWDEF += -DATMega2560",
                    "HWDEF += -D__AVR_3_BYTE_PC__"
                }
            },
            {
                macro = "MCU_ATMEGA2561",
                brief = "Atmel ATmega 2561",
                description = "8-bit RISC microcontroller with 256K bytes flash, 8K bytes RAM, "..
                              "4K bytes EEPROM, 64K bytes data memory space, 2 USARTs, 6 timers, "..
                              "8-channel ADC, SPI and TWI.",
                requires = { "TOOL_CC_AVR" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR",
                    "HW_MCU_AVR_ENHANCED",
                    "HW_MCU_ATMEGA2561",
                    "HW_TIMER_AVR",
                    "HW_UART_AVR",
                    "HW_AVR_HAVE_UART0",
                    "HW_AVR_HAVE_UART1",
                    "HW_GPIO"
                },
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                makedefs =
                {
                    "MCU = $(MCU_ATMEGA2561)",
                    "HWDEF += -D__HARVARD_ARCH__",
                    "HWDEF += -DATMega2561",
                    "HWDEF += -D__AVR_3_BYTE_PC__"
                }
            },
            {
                macro = "MCU_AT91SAM9260",
                brief = "Atmel AT91SAM9260",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9260",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_MCI_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_EXT_CALYPSO"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM9G45",
                brief = "Atmel AT91SAM9G45",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9G45",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_PIT_AT91"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM9XE512",
                brief = "Atmel AT91SAM9XE512",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n"..
                              "Experimental port.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9XE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_MCI_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_PIT_AT91"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM7X128",
                brief = "Atmel AT91SAM7X128",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 128K bytes flash, "..
                              "32K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7X256",
                brief = "Atmel AT91SAM7X256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                              "64K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_EXT_CALYPSO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7X512",
                brief = "Atmel AT91SAM7X512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                              "128K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S16",
                brief = "Atmel AT91SAM7S16",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 16K bytes flash, "..
                    "4K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S32",
                brief = "Atmel AT91SAM7S32",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 32K bytes flash, "..
                    "8K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S64",
                brief = "Atmel AT91SAM7S64",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 64K bytes flash, "..
                    "16K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S128",
                brief = "Atmel AT91SAM7S128",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 128K bytes flash, "..
                    "32K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S256",
                brief = "Atmel AT91SAM7S256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                    "64K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S512",
                brief = "Atmel AT91SAM7S512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                    "64K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE32",
                brief = "Atmel AT91SAM7SE32",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 32K bytes flash, "..
                    "8K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE256",
                brief = "Atmel AT91SAM7SE256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                    "32K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE512",
                brief = "Atmel AT91SAM7SE512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                    "32K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91R40008",
                brief = "Atmel AT91R40008",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes RAM, "..
                              "64M bytes address space, 2 USARTs and 3 timers. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91R40008",
                    "HW_TIMER_AT91",
                    "HW_UART_AT91",
                    "HW_MCU_SWTWI",
                    "HW_WDOG_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_GBA",
                brief = "Nintendo GBA",
                description = "ARM7TDMI 16/32-bit RISC microcontroller",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_GBA",
                    "HW_TIMER_GBA",
                    "HW_LCD_GBA",
                    "DEV_NVMEM"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            --
            -- STM STM32F0 SERIES CONTROLLER
            --
            -- For each STM device include/cfg/arch.h should export at least
            -- Architecture   (MCU_CM3      triggered by HW_MCU_CM3)
            -- Vendor         (MCU_STM32    triggered by HW_MCU_STM32)
            -- Family    (e.g. MCU_STM32F0  triggered by HW_MCU_STM32F0)
            -- Subfamily (e.g. MCU_STM32F04 triggered by HW_MCU_STM32F04)
            -- Header    (e.g. STM32F042x6  triggered by HW_MCU_STM32F042)
            -- Subfamily may have another level of differentiation
            {
                macro = "MCU_STM32F030x8",
                brief = "STM STM32F030 64 kB",
                description = "STM STM32F030 with 64 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F0",
                    "HW_MCU_STM32F03",
                    "HW_MCU_STM32F030",
                    "HW_MCU_STM32F030x8",
                },
                makedefs = { "FLASH0_LENGTH=64K",
                             "RAM0_LENGTH=8K"},
            },
            {
                macro = "MCU_STM32F030xC",
                brief = "STM STM32F030 256 kB",
                description = "STM STM32F030 with 256 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F0",
                    "HW_MCU_STM32F03",
                    "HW_MCU_STM32F030",
                    "HW_MCU_STM32F030xC",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=32K"},
            },
            {
                macro = "MCU_STM32F042x6",
                brief = "STM STM32F042 32 kB",
                description = "STM STM32F042 with 32 kB and USB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F0",
                    "HW_MCU_STM32F04",
                    "HW_MCU_STM32F042",
                },
                makedefs = { "FLASH0_LENGTH=32K",
                             "RAM0_LENGTH=6K",
                             "SMALL_FLASH=1" },
            },
            {
                macro = "MCU_STM32F051x8",
                brief = "STM STM32F051 64 kB",
                description = "STM STM32F051 with 64 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F0",
                    "HW_MCU_STM32F05",
                    "HW_MCU_STM32F051_LD",
                },
                makedefs = { "FLASH0_LENGTH=64K",
                             "RAM0_LENGTH=8K" },
            },
            {
                macro = "MCU_STM32F072xB",
                brief = "STM STM32F072xB",
                description = "STM32F072 128kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F0",
                    "HW_MCU_STM32F07",
                    "HW_MCU_STM32F072_MD",
                },
                makedefs = { "FLASH0_LENGTH=128K",
                             "RAM0_LENGTH=16K" },
            },
            {
                macro = "MCU_STM32F091xC",
                brief = "STM STM32F091xC",
                description = "STM32F091 256 kB STM32F0.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F0",
                    "HW_MCU_STM32F09",
                    "HW_MCU_STM32F091xC",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=32K" },
            },
             --
            -- STM STM32F10X SERIES CONTROLLER
            --
            {
                macro = "MCU_STM32F100xB",
                brief = "STM STM32F100 128 kB",
                description = "STM STM32F100 128 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F1",
                    "HW_MCU_STM32F1_MD",
                    "HW_MCU_STM32F100",
                    "HW_MCU_STM32F100_MD"
                },
                makedefs = { "FLASH0_LENGTH=128K",
                             "RAM0_LENGTH=16K" },
            },
            {
                macro = "MCU_STM32F103xB",
                brief = "STM STM32F103 128 kB",
                description = "STM STM32F103 128 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F1",
                    "HW_MCU_STM32F1_MD",
                    "HW_MCU_STM32F103",
                    "HW_MCU_STM32F103_MD"
                },
                makedefs = { "FLASH0_LENGTH=128K",
                             "RAM0_LENGTH=20K" },
            },
            {
                macro = "MCU_STM32F103xC",
                brief = "STM STM32F103 257 kB",
                description = "STM STM32F103 256 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F1",
                    "HW_MCU_STM32F1_HD",
                    "HW_MCU_STM32F103",
                    "HW_MCU_STM32F103_HD"
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=48K" },
            },
            {
                macro = "MCU_STM32F103xE",
                brief = "STM STM32F103 512 kB",
                description = "STM STM32F103 512 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F1",
                    "HW_MCU_STM32F1_HD",
                    "HW_MCU_STM32F103",
                    "HW_MCU_STM32F103_HD"
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=64K" },
            },
            {
                macro = "MCU_STM32F103xG",
                brief = "STM STM32F103 1024 kB",
                description = "STM STM32F103 1024 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F1",
                    "HW_MCU_STM32F1_XL",
                    "HW_MCU_STM32F103",
                    "HW_MCU_STM32F103_XL"
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=96K" },
            },
            {
                macro = "MCU_STM32F107xC",
                brief = "STM STM32F107 256 kB",
                description = "STM STM32F107 256 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F1",
                    "HW_MCU_STM32F1_CL",
                    "HW_MCU_STM32F107",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=64K" },
            },
            {
                macro = "MCU_STM32L031x6",
                brief = "STM STM32L031 32 k",
                description = "STM32L031 32 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L0",
                    "HW_MCU_STM32L0X1",
                    "HW_MCU_STM32L031",
                },
                makedefs = { "FLASH0_LENGTH=32K",
                             "RAM0_LENGTH=8K",
                             "SMALL_FLASH=1" },
            },
            {
                macro = "MCU_STM32L053x8",
                brief = "STM STM32L053x8",
                description = "STM32L053 64 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L0",
                    "HW_MCU_STM32L0X3",
                    "HW_MCU_STM32L053",
                },
                makedefs = { "FLASH0_LENGTH=64K",
                             "RAM0_LENGTH=8K" },
            },
            {
                macro = "MCU_STM32L073xZ",
                brief = "STM STM32L073xZ",
                description = "STM32L073 192 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L0",
                    "HW_MCU_STM32L0X3",
                    "HW_MCU_STM32L073",
                },
                makedefs = { "FLASH0_LENGTH=192K",
                             "RAM0_LENGTH=20K" },
            },
            {
                macro = "MCU_STM32L151xB",
                brief = "STM STM32L151xB",
                description = "STM32L151 256 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1",
                    "HW_MCU_STM32L1_CAT1",
                    "HW_MCU_STM32L151_MD",
                },
                makedefs = { "FLASH0_LENGTH=128K",
                             "RAM0_LENGTH=16K" },
            },
            {
                macro = "MCU_STM32L152xB",
                brief = "STM STM32L152xB",
                description = "STM32L152 128 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1",
                    "HW_MCU_STM32L1_CAT1",
                    "HW_MCU_STM32L152_MD",
                },
                makedefs = { "FLASH0_LENGTH=128K",
                             "RAM0_LENGTH=16K" },
            },
            {
                macro = "MCU_STM32L152xC",
                brief = "STM STM32L152xX",
                description = "STM32L152 256 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1",
                    "HW_MCU_STM32L15",
                    "HW_MCU_STM32L1_CAT3",
                    "HW_MCU_STM32L152_C",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=32K" },
            },
            {
                macro = "MCU_STM32L152xE",
                brief = "STM STM32L152xE",
                description = "STM32L152 512 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1",
                    "HW_MCU_STM32L15",
                    "HW_MCU_STM32L1_CAT5",
                    "HW_MCU_STM32L152_E",
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=80K" },
            },
            {
                macro = "MCU_STM32L432xC",
                brief = "STM STM32L432xC",
                description = "STM32L432 256 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L4",
                    "HW_MCU_STM32L43",
                    "HW_MCU_STM32L432",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=64K"},
            },
            {
                macro = "MCU_STM32L476xG",
                brief = "STM STM32L476xG",
                description = "STM32L476 1024 kB",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L4",
                    "HW_MCU_STM32L47",
                    "HW_MCU_STM32L476",
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=96K",
                             "CCM_LENGTH=32K" },
            },
            {
                macro = "MCU_STM32F401xC",
                brief = "STM STM32F401 256 kB",
                description = "STM32F401 256 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F401",
                    "HW_MCU_STM32F401xC",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=64K" },
            },
            {
                macro = "MCU_STM32F401xE",
                brief = "STM STM32F401 512 kB",
                description = "STM32F401 512 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F401",
                    "HW_MCU_STM32F401xE",
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=96K" },
            },
            {
                macro = "MCU_STM32F405xG",
                brief = "STM STM32F405",
                description = "STM32F405 with FSMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F40",
                    "HW_MCU_STM32F405",
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=112K",
                             "RAM1_LENGTH=16K",
                             "CCM_LENGTH=64k",
                            },
            },
            {
                macro = "MCU_STM32F407xG",
                brief = "STM STM32F407",
                description = "STM32F407 with Ethernet and FSMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F40",
                    "HW_MCU_STM32F407",
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=112K",
                             "RAM1_LENGTH=16K",
                             "CCM_LENGTH=64K",
                            },
            },
            {
                macro = "MCU_STM32F411xE",
                brief = "STM STM32F411xE",
                description = "Low power 512 kB STM32F4 with USB OTG (FS).",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F41",
                    "HW_MCU_STM32F411",
                    "HW_MCU_STM32F411xE",
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=128K" },
            },
            {
                macro = "MCU_STM32F412RG",
                brief = "STM STM32F412RG",
                description = "Low power 64 Pin 1024 kB STM32F4 with USB OTG (FS).",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F41",
                    "HW_MCU_STM32F412",
                    "HW_MCU_STM32F412Rx",
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=256K" },
            },
            {
                macro = "MCU_STM32F412ZG",
                brief = "STM STM32F412ZG",
                description = "Low power 144 Pin 1024 kB STM32F4 with USB OTG (FS).",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F41",
                    "HW_MCU_STM32F412",
                    "HW_MCU_STM32F412Zx",
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=256K" },
            },
            {
                macro = "MCU_STM32F413ZH",
                brief = "STM STM32F413ZH",
                description = "Low power 144 Pin 1.5 MiB STM32F4 with USB OTG (FS).",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F41",
                    "HW_MCU_STM32F413",
                },
                makedefs = { "FLASH0_LENGTH=1536K",
                             "RAM0_LENGTH=256K",
                             "RAM1_LENGTH=64K"},
            },
            {
                macro = "MCU_STM32F429xI",
                brief = "STM STM32F429",
                description = "STM32F429 with LCD, Ethernet and FMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F42",
                    "HW_MCU_STM32F429",
                },
                makedefs = { "FLASH0_LENGTH=2048K",
                             "RAM0_LENGTH=112K",
                             "RAM1_LENGTH=16K",
                             "RAM2_LENGTH=64K",
                             "CCM_LENGTH=64K",
                            },
            },
            {
                macro = "MCU_STM32F446xE",
                brief = "STM STM32F446 512 kB",
                description = "STM32F446 512 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F44",
                    "HW_MCU_STM32F446",
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=112K",
                             "RAM1_LENGTH=16K",
                           },
            },
            {
                macro = "MCU_STM32F469xI",
                brief = "STM STM32F469 2 MB",
                description = "STM32F469 with DSI, Ethernet and FMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4",
                    "HW_MCU_STM32F46",
                    "HW_MCU_STM32F469",
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=160K",
                             "RAM1_LENGTH=32K",
                             "RAM2_LENGTH=128K",
                             "CCM_LENGTH=64K",
                            },
            },
            {
                macro = "MCU_STM32F723xE",
                brief = "STM STM32F723 512 kB",
                description = "STM32F746 with HS USB phy.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F7",
                    "HW_MCU_STM32F72",
                    "HW_MCU_STM32F723",
                },
                makedefs = { "FLASH0_LENGTH=512K"},
            },
            {
                macro = "MCU_STM32F746xG",
                brief = "STM STM32F746 1 MB",
                description = "STM32F746 with LCD, Ethernet and FMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F7",
                    "HW_MCU_STM32F74",
                    "HW_MCU_STM32F746",
                },
                makedefs = {"FLASH0_LENGTH=1024K"}
            },
            {
                macro = "MCU_STM32F756xG",
                brief = "STM STM32F756 1 MB",
                description = "STM32F746 with Crypto, LCD, Ethernet and FMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F7",
                    "HW_MCU_STM32F74",
                    "HW_MCU_STM32F746",
                },
                makedefs = {"FLASH0_LENGTH=1024K"}
            },
            {
                macro = "MCU_STM32F767xI",
                brief = "STM STM32F767 2 MB",
                description = "STM32F767 with double precession FPU, LTDC and JPEG.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F7",
                    "HW_MCU_STM32F76",
                    "HW_MCU_STM32F767",
                },
                makedefs = {"FLASH0_LENGTH=2048K"}
            },
            {
                macro = "MCU_STM32F769xI",
                brief = "STM STM32F769 2 MB",
                description = "STM32F769 with double precession FPU, LTDC, JPEG and DSI.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F7",
                    "HW_MCU_STM32F76",
                    "HW_MCU_STM32F769",
                },
                makedefs = {"FLASH0_LENGTH=2048K"}
            },
            {
                macro = "MCU_STM32F205xC",
                brief = "STM STM32F205 256 kB",
                description = "STM32F205 256 kb with FSMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2",
                    "HW_MCU_STM32F20",
                    "HW_MCU_STM32F205",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=80K",
                             "RAM1_LENGTH=16k"
                           }
            },
            {
                macro = "MCU_STM32F207xC",
                brief = "STM STM32F207 256 kB",
                description = "STM32F207 256 kb with FSMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2",
                    "HW_MCU_STM32F20",
                    "HW_MCU_STM32F207",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=80K",
                             "RAM1_LENGTH=16k",
                           }
            },
            {
                macro = "MCU_STM32F207xG",
                brief = "STM STM32F207",
                description = "STM32F207 with FSMC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2",
                    "HW_MCU_STM32F20",
                    "HW_MCU_STM32F207",
                },
                makedefs = { "FLASH0_LENGTH=1024K",
                             "RAM0_LENGTH=112K",
                             "RAM1_LENGTH=16k",
                           }
            },
            {
                macro = "MCU_STM32F303x8",
                brief = "STM STM32F303 64 kB",
                description = "STM32F303 with CCM, CAN, USB, 2 ADC, 2 DAC, 3 COMP, 1 OP.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3",
                    "HW_MCU_STM32F30",
                    "HW_MCU_STM32F303_LD",
                },
                makedefs = { "FLASH0_LENGTH=64K",
                             "RAM0_LENGTH=12K",
                             "CCM_LENGTH=4K",
                          }
            },
            {
                macro = "MCU_STM32F303xC",
                brief = "STM STM32F303 256 kB",
                description = "STM32F303 with CCM, CAN, USB, 4 ADC, 2 DAC, 7 COMP, 4 OP.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3",
                    "HW_MCU_STM32F30",
                    "HW_MCU_STM32F303_MD",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=40K",
                             "CCM_LENGTH=8K",
                           }
            },
            {
                macro = "MCU_STM32F303xE",
                brief = "STM STM32F303 512 kB",
                description = "STM32F303 with 512 kB.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3",
                    "HW_MCU_STM32F30",
                    "HW_MCU_STM32F303_HD",
                },
                makedefs = { "FLASH0_LENGTH=512K",
                             "RAM0_LENGTH=64K",
                             "CCM_LENGTH=16K",
                           }
            },
            {
                macro = "MCU_STM32F334x8",
                brief = "STM STM32F334 64 kb",
                description = "STM32F334 64 kB with HRTIM, CAN, 2 ADC, 3DAC.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3",
                    "HW_MCU_STM32F30",
                    "HW_MCU_STM32F334_LD",
                },
                makedefs = { "FLASH0_LENGTH=64K",
                             "RAM0_LENGTH=16K" },
            },
-- STM32F373 and STM32F378 are quite similar to each other and quite different
-- in pinout to other STM32F3, so introduce a subclass
            {
                macro = "MCU_STM32F373xC",
                brief = "STM STM32F373",
                description = "STM32F373 with USB, CAN, 3 SDADC, 1 ADC, 3 DAC, 2 COMP and Capsense.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3",
                    "HW_MCU_STM32F37",
                    "HW_MCU_STM32F373_MD",
                },
                makedefs = { "FLASH0_LENGTH=256K",
                             "RAM0_LENGTH=32K" },
            },
            --
            -- TI LM3S SERIES CONTROLLER
            --
            {
                macro = "MCU_LM3S9B96",
                brief = "Luminary LM3S9B96",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LM3 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LM3",
                    "HW_MCU_LM3S9B96",
                    "MCU_LM3S9B96",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            --
            -- NXP LPC17xx CONTROLLER
            --
            {
                macro = "MCU_LPC175x",
                brief = "NXP LCP175x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC175x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC175x",
                    "HW_PLL_LPC17xx",
                    "HW_RTC_LPC17xx",
                    "HW_FLASH_LPC17xx",
                    "HW_GPIO_LPC176x",
                    "HW_CRC32_LPC177x",
                    "DEV_IRQ_LPC17xx",
                    "HW_WDT_LPC17xx",
--                    "HW_GPDMA_LPC17xx",
                    "HW_EMAC_LPC17xx",
                    "HW_I2C_LPC17xx",
                    "HW_SPI_LPC17xx",
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC176x",
                brief = "NXP LCP176x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC176x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC176x",
                    "HW_PLL_LPC17xx",
                    "HW_RTC_LPC17xx",
                    "HW_FLASH_LPC17xx",
                    "HW_GPIO_LPC176x",
                    "HW_CRC32_LPC177x",
                    "DEV_IRQ_LPC17xx",
                    "HW_WDT_LPC17xx",
--                    "HW_GPDMA_LPC17xx",
                    "HW_EMAC_LPC17xx",
                    "HW_I2C_LPC17xx",
                    "HW_SPI_LPC17xx",
                    "HW_SSP_LPC17xx",
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC177x_8x",
                brief = "NXP LCP177x_8x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC177x_8x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC177x_8x",
                    "HW_MCU_LPC177x_8x_LPC407x_8x",
                    "HW_PLL_LPC17xx",
                    "HW_RTC_LPC17xx",
                    "HW_EMC_LPC177x_8x",
                    "HW_FLASH_LPC17xx",
                    "HW_EEPROM_LPC177x_8x",
                    "HW_GPIO_LPC177x_8x",
                    "HW_CRC32_LPC17xx",
                    "DEV_IRQ_LPC17xx",
                    "HW_WDT_LPC17xx",
                    "HW_GPDMA_LPC17xx",
                    "HW_MCI_LPC177x_8x",
                    "HW_EMAC_LPC17xx",
                    "HW_I2C_LPC17xx",
                    "HW_SSP_LPC17xx",
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC407x_8x",
                brief = "NXP LCP407x_8x series",
                description = "CortexM4 32-bit RISC microcontroller with HW FPU.\n\n"..
                              "Select the correct sub-type in Architecture->CM4->LPC407x_8x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC407x_8x",
                    "HW_MCU_LPC177x_8x_LPC407x_8x",
                    "HW_PLL_LPC17xx",
                    "HW_RTC_LPC17xx",
                    "HW_EMC_LPC177x_8x",
                    "HW_FLASH_LPC17xx",
                    "HW_EEPROM_LPC177x_8x",
                    "HW_GPIO_LPC177x_8x",
                    "HW_CRC32_LPC17xx",
                    "DEV_IRQ_LPC17xx",
                    "HW_WDT_LPC17xx",
                    "HW_GPDMA_LPC17xx",
                    "HW_MCI_LPC177x_8x",
                    "HW_EMAC_LPC17xx",
                    "HW_I2C_LPC17xx",
                    "HW_SSP_LPC17xx",
                },
                makedefs = { "MCU=cortex-m4" },
            },
            --
            -- UNIX EMULATION FOR NUT/OS
            --
            {
                macro = "MCU_LINUX_EMU",
                brief = "Linux Emulator",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_LINUX" },
                provides = { "HW_TARGET", "DEV_NVMEM", "HW_EMU_LINUX" }
            },
            {
                macro = "MCU_H8_3068",
                brief = "Renesas H8/3068",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_H8300" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_H8300",
                    "H83068_TIMER",
                    "H83068_UART"
                }
            },
            {
                macro = "MCU_S3C4510B",
                brief = "Samsung S3C4510B",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with Ethernet MAC,"..
                              "HDLC protocol, 64M bytes address space, I2C, 2 UARTs and "..
                              "2 timers.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_S3C45",
                    "HW_TIMER_S3C45",
                    "HW_UART_S3C45"
                }
            },

            --
            -- AVR32
            --
            {
                macro = "MCU_AVR32UC3A0512",
                brief = "Atmel AT32UC3A0512",
                description = "32-bit AVR UC3 RISC microcontroller with 512K flash, 64K SRAM,\n"..
                              "10/100 Ethernet MAC, Full-Speed + OTG USB,"..
                              "I2C, 4 UARTs and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART1_AVR32",
                    "HW_UART2_AVR32",
                    "HW_UART3_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_MACB_AVR32",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a0512" }
            },
            {
                macro = "MCU_AVR32UC3A0512ES",
                brief = "Atmel AT32UC3A0512-ES",
                description = "*** Engineering Sample, do not use this CPU for new designs ***\n\n"..
                              "32-bit AVR UC3 RISC microcontroller with 512K flash, 64K SRAM,\n"..
                              "10/100 Ethernet MAC, Full-Speed + OTG USB,"..
                              "I2C, 4 UARTs and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART1_AVR32",
                    "HW_UART2_AVR32",
                    "HW_UART3_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_MACB_AVR32",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a0512es" }
            },
            {
                macro = "MCU_AVR32UC3A3256",
                brief = "Atmel AT32UC3A3256",
                description = "32-bit AVR UC3 RISC microcontroller with 256k Flash, 64k SRAM,\n"..
                              "High-Speed + OTG USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a3256" }
            },
            {
                macro = "MCU_AVR32UC3B0256",
                brief = "Atmel AT32UC3B0256",
                description = "32-bit AVR UC3 RISC microcontroller with 256k Flash, 32k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3b0256" }
            },
            {
                macro = "MCU_AVR32UC3B164",
                brief = "Atmel AT32UC3B164",
                description = "32-bit AVR UC3 RISC microcontroller with 64k Flash, 16k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3b164" }
            },
            {
                macro = "MCU_AVR32UC3L064",
                brief = "Atmel AT32UC3L064",
                description = "32-bit AVR UC3 RISC microcontroller with 64k Flash, 16k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                   -- "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                   -- "HW_PLL_AVR32",
                    "HW_DFLL_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3l064" }
            },
            {
                macro = "MCU_MCF51CN128",
                brief = "Freescale MCF51CN128",
                description = "32-bit RISC microcontroller, V1 Coldfire Core, 128K flash, 24K SRAM"..
                              "10/100 Ethernet MAC, MII, 3xSCI(UART), 2xIIC, 2xSPI, ADC, RTC, 2xModulo Timer, 2xTimer/PWM."..
                              "Mini FlexBUS, Keyboard Interrupts, 1xExternal Interrupt, Rapid GPIO",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_M68K" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_M68K",
                    "HW_MCU_COLDFIRE",
                    "HW_MCU_MCF51CN",
                    "HW_MCU_MCF51CN128",
                },
                makedefs = { "MCU=51cn" }
            },
            {
                macro = "MCU_MCF52259",
                brief = "Freescale MCF52259",
                description = "32-bit RISC microcontroller, V2 Coldfire Core, 10/100 Ethernet MAC"..
                              "512K flash, 64K SRAM, 3xUART, 2xI2C, ADC, QSPI, DMA, FlexCAN,"..
                              "Mini FlexBUS, USB, RTC, Random Number Generator, Cryptographic Accelerator.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_M68K" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_M68K",
                    "HW_MCU_COLDFIRE",
                    "HW_MCU_MCF5225X",
                    "HW_MCU_MCF52259",
                },
                makedefs = { "MCU=52259" }
            },

            --
            -- Imaginary Zero CPU
            --
            {
                macro = "MCU_ZERO",
                brief = "Zero Dummy CPU",
                description = "Imaginary ARM9 CPU, useful as a porting template.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_TIMER_ZERO",
                    "HW_UART_ZERO"
                },
                makedefs = { "MCU=arm9" }
            }
        }
    },

    --
    -- Architecture Dependent Implementations
    --
    {
        name = "nutarch_arm",
        brief = "ARM",
        requires = { "HW_MCU_ARM" },
        provides = { "ARM_SEMIHOSTING" };
        script = "arch/arm.nut"
    },
    {
        name = "nutarch_cm3",
        brief = "CM3",
        requires = { "HW_MCU_CM3" },
        provides = { "ARM_SEMIHOSTING" };
        script = "arch/cm3.nut"
    },
    {
        name = "nutarch_avr",
        brief = "AVR",
        requires = { "HW_MCU_AVR" },
        script = "arch/avr.nut"
    },
    {
        name = "nutarch_avr32",
        brief = "AVR32",
        requires = { "HW_MCU_AVR32" },
        script = "arch/avr32.nut"
    },
    {
        name = "nutarch_h8300h",
        brief = "H8/300H",
        requires = { "HW_MCU_H8300" },
        script = "arch/h8300h.nut"
    },
    {
        name = "nutarch_m68k",
        brief = "M68K",
        requires = { "HW_MCU_M68K" },
        script = "arch/m68k.nut"
    },
    {
        name = "nutarch_unix",
        brief = "Linux Emulator",
        requires = { "HW_EMU_LINUX" },
        script = "arch/unix.nut"
    },
-- Target specific devices
    {
        name = "nutarch_cm3_stm32_pll",
        brief = "Clock settings",
        description = "Clock settings",
        requires = { "HW_MCU_STM32" },
        script = "arch/cm3/stm32pll.nut"
    },
    {
        name = "nutarch_cm3_stm32_usart_devices",
        brief = "STM32 U(S)ART devices",
        description = "STM32 U(S)ART devices.",
        requires = { "HW_MCU_STM32" },
        script = "arch/cm3/stm32usart.nut"
    },
    {
        name = "nutarch_cm3_stm32_i2c_devices",
        brief = "STM32 I2C devices",
        description = "STM32 I2C devices.",
        requires = { "HW_MCU_STM32" },
        script = "arch/cm3/stm32i2c.nut"
    },
    {
        name = "nutarch_cm3_stm32_spi_devices",
        brief = "STM32 SPI devices",
        description = "STM32 SPI devices.",
        requires = { "HW_MCU_STM32" },
        sources  = { "cm3/dev/stm/stm32_spi_cb.c" },
        script = "arch/cm3/stm32spi.nut"
    },
    {
        name = "nutarch_cm3_stm32_timer_devices",
        brief = "Stm32 Timer devices",
        description = "STM32 Timer availability and devices using timers",
        requires = { "HW_MCU_STM32" },
        script = "arch/cm3/stm32timer.nut"
    },
}
