--Pin Speed setting

function PinSpeedDesc()
    if c_is_provided("HW_MCU_STM32F1") then
        return "GPIO Pin Setting must match function speed, CPU supply "..
               "and circuit load.\n\n"..
               "GPIO_SLOW:\tmax.  2 MHz\n"..
               "GPIO_MED:\tmax. 10 Mhz\n"..
               "GPIO_FAST:\tmax. 50 MHz\n"
    end
    if c_is_provided("HW_GPIO_STM32L_0_1") then
        return "GPIO Pin Setting must match function speed, CPU supply "..
               "and circuit load.\n\n"..
               "GPIO_SLOW:\tmax. 400 kHz\n"..
               "GPIO_MED:\tmax.   2 Mhz\n"..
               "GPIO_FAST:\tmax. 10 MHz\n"..
               "GPIO_HIGH:\tmax. 50(L1)/35(L0) MHz\n"
    end
    return "GPIO Pin Setting must match function speed, CPU supply "..
           "and circuit load.\n\n"..
           "GPIO_SLOW:\tmax.   5 MHz\n"..
           "GPIO_MED:\tmax.  10 Mhz\n"..
           "GPIO_FAST:\tmax. 50 MHz\n"..
           "GPIO_HIGH:\tmax. 100 MHz\n"
end
function GetPinSpeedChoices()
    if c_is_provided("HW_MCU_STM32F1") then
        return {"GPIO_SLOW", "GPIO_MED", "GPIO_FAST"}
    end
    if c_is_provided("HW_GPIO_STM32F0") then
        return {"GPIO_SLOW", "GPIO_MED", "GPIO_FAST"}
    end
    if c_is_provided("HW_GPIO_STM32F3") then
        return {"GPIO_SLOW", "GPIO_MED", "GPIO_FAST"}
    end
    return {"GPIO_SLOW", "GPIO_MED", "GPIO_FAST", "GPIO_HIGH"}
end

nutarch_cm3_stm32_devices =
{
    -- ***********************************
    --
    -- STM32 Device Drivers
    --
    -- ***********************************

   -- STM32 RTC V2
    --
    {
        name = "nutarch_cm3_stm32v2_rtc",
        brief = "STM32 RTC Driver V2",
        description = "STM32 RTC driver V2.",
        requires = { "HW_RTC_STM32_V2" },
        provides = { "DEV_RTC", "HW_STM32_BACKUP_REG"},
        sources = { "cm3/dev/stm/stm32_rtc_v2.c" , "cm3/dev/stm/stm32_backup.c" },
        options =
        {
            {
                macro = "BACKUP_REGULATOR_OFF",
                brief = "Disable RTC/Backup SRAM in VBAT MODE",
                description = "RTC and BACKKP SRAM will loose content in"..
                " Standby and VBAT mode",
                type = "integer",
                file = "include/cfg/arch.h"
            },
            {
                macro = "BACKUP_SRAM_OFF",
                brief = "Disable Backup SRAM",
                description = "BKP SRAM access disabled",
                type = "integer",
                file = "include/cfg/arch.h"
            },
        }
    },

    --
    -- STM32 Flash Interface on STM32L0 and STM32L1
    --
    {
        name = "nutarch_cm3_stm32l_flash",
        brief = "Flash access",
        description = "STM32L0 and STM32L1 generic flash interface.",
        requires = { "HW_FLASH_STM32L" },
        options =
        {
            {
                macro = "FLASH_ERASED_PATTERN32",
                brief = "Value of erase flash",
                description = "Value of erase flash. Mostly 0xffffffff, but e.g 0 for STM32L1",
                type = "integer",
                default = "0",
                file = "include/cfg/memory.h",
             },
        }
    },
      --
    -- STM32 Flash Interface F2 and F4
    --
    {
        name = "nutarch_cm3_stm32f2_4_flash",
        brief = "Flash and parameter access",
        description = "Generic flash and parameter storage interface.",
        requires = { "HW_FLASH_STM32F2_4" },
        sources = { "cm3/dev/stm/stm32f2_4_flash.c"},
        options =
        {
            {
                macro = "FLASH_PE_PARALLELISM",
                brief = "Program/erase parallelism",
                description = "Depending on CPU voltage and eventual external programming voltage,\n"..
                "flash erase and programming can happen 8/16/32/64 Bit wide. Default is 32.",
                type = "integer",
                choices = { "8", "16", "32", "64" },
                default = "32",
                file = "include/cfg/memory.h",
             },
             {
                macro = "STM32_VRANGE",
                brief = "Supply Voltage Range",
                description = "E.g. flash access depends on voltage range. Choices are 0 (2.7-3.6V),\n"..
                " 1 (2.4-2.7V), 2 (2.1 - 2.4V) and 3 (1.8-21.1)",
                type = "integer",
                choices = { "0", "1", "2", "3" },
                default = "0",
                file = "include/cfg/memory.h",
             }
         }
    },
    --
     --
    -- STM32 GPIO Interface Version 2 on L1/F2/F4
    --
    {
        name = "nutarch_cm3_stm32_gpio",
        brief = "STM32F2 GPIO",
        description = "Generic port I/O API.",
        requires = { "HW_GPIO_STM32V2" },
        provides = { "HW_GPIO_V2", "HW_GPIO" },
        sources = {
                "cm3/dev/stm/stm32_gpio_v2.c",
        },
        options =
        {
            {
                macro = "STM32_GPIO_ANALOG_MODE",
                brief = "STM32 Pin Default Analog",
                description = "Select default pin mode as ANLOG.\n"..
                "JTAG/SWD is not touched.\n"..
                "L0 and L4 come out of reste already in ANALOG mode",
                file = "include/cfg/arch.h",
            }
        }
    },
    --
    -- STM32 CEC V2 on STM32F373
    --
    {
        name = "nutarch_cm3_stm32_cec_v2",
        brief = "STM32 CEC V2",
        description = "STM32 CEC V2 on STM32F37X.",
        requires = { "HW_CEC_STM32V3" },
        provides = { "HW_CEC_STM32" },
    },
    --
    -- STM32F EMAC Configuration
    --
    {
        name = "nutarch_cm3_stm32f_emac",
        brief = "STM32F EMAC Driver",
        description = "LAN driver for STM32Fxx7xx.",
        requires = { "HW_EMAC_STM32", "NUT_EVENT", "NUT_TIMER" },
        provides = { "NET_MAC" },
        sources = {
           "cm3/dev/stm/stm32_emac.c",
        },
        options =
        {
            {
                macro = "EMAC_REMAP",
                brief = "Use Alternate Pins",
                description = "Leaving this option unchecked, the driver will initialize the standard Pins.\n\n"..
                              "Pin Mapping is:\n RX_DV-CRS PA07\n RXD0 PC04 \n RXD1 PC05\n RXD2 PB00\n RXD3 PB01\n"..
                              "Enabling the option the driver remaps to its alternate port pins:\n\n"..
                              "Pin Mapping is:\n RX_DV-CRS PD08\n RXD0 PD09 \n RXD1 PD10\n RXD2 PD11\n RXD3 PD12\n",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/emac.h"
            },
--List all Pins, even if there are no alternate pins
            {
                macro = "EMAC_TX_EN",
                description = "Choice of STM32 EMAC TX_EN pins\n"..
                              "Possible pins: PB11, PG11.\n"..
                              "Default: PB11\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PB11",
                type = "enumerated",
                choices = { "PB11", "PG11" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TXD0",
                description = "Choice of STM32 EMAC TXD0 pins\n"..
                               "Possible pins: PB12, PG13.\n"..
                               "Default: PB12\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PB12",
                type = "enumerated",
                choices = { "PB12", "PG13" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TXD1",
                description = "Choice of STM32 EMAC TXD1 pins\n"..
                               "Possible pins: PB13, PG14.\n"..
                               "Default: PB13\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PB13",
                type = "enumerated",
                choices = { "PB13", "PG14" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TXD2",
                description = "STM32 EMAC TXD2 pins\n",
                requires = { "HW_GPIO_STM32V2", "PHY_MODE_MII" },
                default = "PC02",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TXD3",
                description = "Choice of STM32 EMAC TXD3 pins\n"..
                               "Possible pins: PB08, PE02.\n"..
                               "Default: PB08\n",
                 requires = { "HW_GPIO_STM32V2" , "PHY_MODE_MII" },
                default = "PB08",
                type = "enumerated",
                choices = { "PB08", "PE02" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MII_TX_CLK",
                description = "Stm32 EMAC MII TX_CLK pin\n",
                requires = { "HW_GPIO_STM32V2", "PHY_MODE_MII" },
                default = "PC03",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RXD0",
                description = "STM32 EMAC RXD0 pins\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PC04",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RXD1",
                description = "STM32 EMAC RXD1 pins\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PC05",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RXD2",
                description = "Choice of STM32 EMAC RXD2 pins\n"..
                               "Possible pins: PB00, PH06.\n"..
                               "Default: PB00\n",
                requires = { "HW_GPIO_STM32V2" , "PHY_MODE_MII" },
                default = "PB00",
                type = "enumerated",
                choices = { "PB00", "PH06" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RXD3",
                description = "Choice of STM32 EMAC RXD3 pins\n"..
                              "Possible pins: PB01, PH07.\n"..
                              "Default: PB01\n",
                requires = { "HW_GPIO_STM32V2" , "PHY_MODE_MII" },
                default = "PB01",
                type = "enumerated",
                choices = { "PB01", "PH07" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RX_ER",
                description = "Choice of STM32 EMAC RX_ER pins\n"..
                              "Possible pins: PIN_NONE, PB10, PI10.\n"..
                              "Default: PB10.\n"..
                              "Use PIN_NONE if normal GPIO pin is used for error detection.\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PB10",
                type = "enumerated",
                choices = { "PB10", "PI10", "PIN_NONE" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_PPS",
                description = "Choice of STM32 EMAC PPS pins\n",
                               "Possible pins: PIN_NONE, PB05, PG08.\n"..
                               "Default: PIN_NONE to disable PPS functionality\n",
                 requires = { "HW_GPIO_STM32V2" },
                default = "PIN_NONE",
                type = "enumerated",
                choices = { "PIN_NONE", "PB05", "PG08" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MII_COL",
                description = "Stm32 EMAC MII COL pin\n",
                requires = { "HW_GPIO_STM32V2" , "PHY_MODE_MII"},
                default = "PA03",
                type = "enumerated",
                choices = { "PIN_NONE", "PA03", "PH03" },
                file = "include/cfg/dev.h"
            },
             {
                macro = "EMAC_PHY_CLOCK_MCO",
                description = "25 MHz from MCU to EMAC\n",
                default = "PIN_NONE",
                type = "enumerated",
                choices = { "PIN_NONE", "PA08", "PC09" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MII_CLK_RMII_REF_CLK",
                description = "Stm32 MII CLK/RMI REF CLK pin\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PA01",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MDIO",
                description = "Stm32 EMAC MDIO pin\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PA02",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MDC",
                description = "Stm32 EMAC MDC pin\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PC01",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MII_RX_DV_RMII_CRS_DV",
                description = "Stm32 EMAC MII RX_DV/ RMII CRS_DV  pin\n",
                requires = { "HW_GPIO_STM32V2" },
                default = "PA07",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_MII_CRS",
                description = "Choice of STM32 EMAC CRS Coll pins\n"..
                              "Possible pins: PA00, PH02.\n"..
                              "Default: PA00\n",
                 requires = { "HW_GPIO_STM32V2" , "PHY_MODE_MII"},
                default = "PA00",
                type = "enumerated",
                choices = { "PA00", "PH02" },
                file = "include/cfg/dev.h"
            },
            {
                macro = "NUT_THREAD_NICRXSTACK",
                brief = "Receiver Thread Stack",
                description = "Number of bytes to be allocated for the stack of the NIC receive thread.",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RX_BUFFERS",
                brief = "Receive Buffers",
                description = "Number of 128 byte receive buffers.\n"..
                              "Increase to handle high traffic situations.\n"..
                              "Decrease to handle low memory situations.\n"..
                              "Default is 32.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TX_BUFSIZ",
                brief = "Transmit Buffer Size",
                description = "The driver will allocate two transmit buffers.\n"..
                              "Can be decreased in low memory situations. Be aware, "..
                              "that this may break your network application. Do not "..
                              "change this without exactly knowing the consequences.\n"..
                              "Default is 1536.\n",
                flavor = "booldata",
                type = "integer",
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
    -- STM32F CAN Configuration
    --
    {
        name = "nutarch_cm3_stm32f_usb_can1",
        brief = "STM32F CAN 1",
        description = "CAN support for STM32F.\n\n"..
                      "CAN bus shares pins and interrupts with USB bus!\n"..
                      "You cannot use them together!",
        requires = { "HW_USB_CAN1_STM32" },
        provides = { "CAN1_STM32" }
    },
    {
        name = "nutarch_cm3_stm32f_usb_can",
        brief = "STM32F CAN",
        description = "CAN support for STM32F.\n\n"..
                      "CAN bus shares pins and interrupts with USB bus!\n"..
                      "You cannot use them together!",
        requires = { "HW_USB_CAN_STM32" },
        provides = { "CAN1_STM32" }
    },
    {
        name = "nutarch_cm3_stm32f_can",
        brief = "STM32F CAN",
        description = "CAN support for STM32F.",
        requires = { "HW_CAN_STM32" },
        provides = { "CAN1_STM32" }
    },
    {
        name = "nutarch_cm3_stm32_can1",
        brief = "STM32F CAN",
        description = "CAN support for STM32F.",
        requires = { "HW_CAN1_STM32" },
        provides = { "CAN1_STM32" }
    },
    {
        name = "nutarch_cm3_stm32f_can1",
        brief = "STM32F CAN 1",
        description = "CAN support for STM32F.\n\n"..
                      "On some devices this bus shares pins and interrupts with USB bus!\n"..
                      "You cannot use them together!",
        requires = { "CAN1_STM32" },
        provides = { "DEV_CANBUS" },
        sources =
        {
            "cm3/dev/stm/stm32_can.c",
            "cm3/dev/stm/stm32_can1.c",
        },
        options =
        {
            {
                macro = "CAN1_REMAP",
                brief = "Use Alternate Pins",
                description = "This CAN bus can be internally connected to several sets of pins.\n\n"..
                              "Choice 0: CAN_RX PA11 CAN_TX PA12\n"..
                              "Choice 1: CAN_RX PB08 CAN_TX PB09\n"..
                              "Choice 2: CAN_RX PD00 CAN_TX PD01\n",
                requires = { "HW_GPIO_STM32V1" },
                default = "0",
                type = "enumerated",
                choices = { "0", "1", "3" },
                file = "include/cfg/can_dev.h"
            },
            {
                macro = "CAN1_TX",
                brief = "CAN1 TX Pin selection",
                description = "Choose CAN1 TX Pin, Default: PA12",
                requires = { "HW_GPIO_STM32V2" },
                default = "PA12",
                type = "enumerated",
                choices = { "PA12", "PB09", "PD01", "PH13" },
                file = "include/cfg/can_dev.h"
            },
            {
                macro = "CAN1_RX",
                brief = "CAN1 RX Pin selection",
                description = "Choose CAN1 RX Pin, Default: PA11",
                requires = { "HW_GPIO_STM32V2" },
                default = "PA11",
                type = "enumerated",
                choices = { "PA11", "PB08", "PD00", "PI00" },
                file = "include/cfg/can_dev.h"
            }
        }
    },
    {
        name = "nutarch_cm3_stm32f_can2",
        brief = "STM32F CAN 2",
        description = "CAN Bus 2 support for STM32F.",
        requires = { "HW_CAN2_STM32" },
        provides = { "DEV_CANBUS" },
        sources =
        {
            "cm3/dev/stm/stm32_can.c",
            "cm3/dev/stm/stm32_can2.c",
        },
        options =
        {
            {
                macro = "CAN2_REMAP",
                brief = "Use CAN2 Alternate Pins",
                description = "This CAN bus can be internally connected to two sets of pins.\n\n"..
                              "Choice 0: CAN_RX PB12 CAN_TX PB13\n"..
                              "Choice 1: CAN_RX PB05 CAN_TX PB06\n",
                requires = { "HW_GPIO_STM32V1" },
                default = "0",
                type = "enumerated",
                choices = { "0", "1" },
                file = "include/cfg/can_dev.h"
            },
            {
                macro = "CAN2_TX",
                brief = "CAN2 TX Pin selection",
                description = "Choose CAN2 TX Pin, Default: PB13",
                requires = { "HW_GPIO_STM32V2" },
                default = "PB13",
                type = "enumerated",
                choices = { "PB13", "PB06" },
                file = "include/cfg/can_dev.h"
            },
            {
                macro = "CAN2_RX",
                brief = "CAN2 RX Pin selection",
                description = "Choose CAN2 RX Pin, Default: PB12",
                requires = { "HW_GPIO_STM32V2" },
                default = "PB12",
                type = "enumerated",
                choices = { "PB12", "PB05" },
                file = "include/cfg/can_dev.h"
            }
        }
    },

    --
    -- STM32F ADC Configuration
    --
    {
        name = "nutarch_cm3_stm32f_adc",
        brief = "STM32F ADC Channel 1,2",
        description = "ADC channel 1 & 2 support for STM32F.",
        requires = { "HW_ADCF1_12_STM32" },
        provides = { "DEV_ADC" },
--        sources = { "cm3/dev/stm/stm32f1_adc.c" },
    },

    --
    -- STM32F MMC Interface
    --
    {
        name = "nutarch_cm3_stm32f_sdio",
        brief = "STM32F SD-Card Access",
        description = "Low level SD-Card interface for STM32F.\n\n"..
                      "Under development.",
        requires = { "HW_SPI_STM32" },
        provides = { "DEV_MMCLL" },
--        sources = { "cm3/dev/stm/stm32_sdio.c" },
    },

    --
    -- STM32F LCD Interface Drivers
    --
    {
        name = "nutarch_cm3_stm32_lcd",
        brief = "LCD glass driver",
        description = "Driver for LCD glass display.\n",
        requires = { "HW_LCD_STM32" },
        sources = { "cm3/dev/stm/stm32_lcd16seg.c" },
    },

    --
    -- STM32F0 DMA Interface
    --
    {
        name = "nutarch_cm3_stm32f0_dma",
        brief = "STM32F0 DMA1 5 ch Controller",
        description = "Generic DMA1 5 Ch Support for F0",
        requires = { "HW_MCU_STM32", "HW_DMA1_STM32F0" },
        provides = {
            "HW_DMA1_STM32F1",
            "HW_DMA1_5CH_STM32",
            "HW_DMA_COMBINED_IRQ_STM32",
        },
    },
    {
        name = "nutarch_cm3_stm32f07_dma",
        brief = "STM32F0 DMA1 7 ch Controller",
        description = "Generic DMA1 7 Ch Support for F07",
        requires = { "HW_MCU_STM32", "HW_DMA1_STM32F07" },
        provides = {
            "HW_DMA1_STM32F1",
            "HW_DMA_COMBINED_IRQ_STM32",
        },
    },
    {
        name = "nutarch_cm3_stm32f09_dma",
        brief = "STM32F0 DMA1/2 12 ch Controller",
        description = "Generic DMA 1/2 12 ch Support for F09",
        requires = { "HW_MCU_STM32", "HW_DMA1_2_STM32F09" },
        provides = {
            "HW_DMA1_STM32F1",
            "HW_DMA2_STM32F1",
            "HW_DMA_COMBINED_IRQ_STM32",
            "HW_DMA_CSELR_STM32"
        },
    },
    --
    -- STM32F1/L1 DMA Interface
    --
    {
        name = "nutarch_cm3_stm32f1_dma1",
        brief = "STM32F1/L1 DMA1 Controller",
        description = "Generic DMA Support for F1/L1",
        requires = { "HW_MCU_STM32", "HW_DMA1_STM32F1" },
        provides = { "HW_DMA1_F1" },
        sources = {
            "cm3/dev/stm/stm32f1_dma.c",
            "cm3/dev/stm/stm32_dmairq.c",
        }
    },
    {
        name = "nutarch_cm3_stm32f1_dma_combined",
        brief = "STM32F1 HD/XL DMA1 Controller",
        description = "STM32F1 HD/XL support to separate DMA CH4/CH5",
        requires = { "HW_MCU_STM32", "HW_DMA2_CH4_5_STM32F1" },
        provides = {
            "HW_DMA2_STM32F1",
            "HW_DMA_COMBINED_IRQ_STM32",
        },
    },
    {
        name = "nutarch_cm3_stm32f1_dma2",
        brief = "STM32F1/L1 DMA2 Controller",
        description = "Generic DMA Support for F1/L1",
        requires = { "HW_MCU_STM32", "HW_DMA1_STM32F1", "HW_DMA2_STM32F1" },
        provides = { "HW_DMA2_F1" },
    },

    --
    -- STM32F2/F4 DMA Interface
    --
    {
        name = "nutarch_cm3_stm32f2_dma",
        brief = "STM32F2/F4 DMA Controller",
        description = "Generic DMA Support for F2/F4",
        requires = { "HW_MCU_STM32", "HW_DMA_STM32F2" },
        sources = {
            "cm3/dev/stm/stm32f2_dma.c",
            "cm3/dev/stm/stm32_dmairq.c",
        },
    },
    --
    -- STM32 I2C1 Hardware V1 TWI
    --
    {
        name = "nutarch_cm3_stm32_twi_v1",
        brief = "I2C1 V1 TWI",
        description = "TWI support for STM32 I2C1 V1",
        requires = { "HW_I2C_STM32V1", "HW_I2C1_STM32" },
        sources =
        {
            "cm3/dev/stm/stm32_twi.c",
            "cm3/dev/stm/stm32_twi1.c",
        },
    },
    -- STM32 I2C2 Hardware V1 TWI
    --
    {
        name = "nutarch_cm3_stm32_twi_v2",
        brief = "I2C2 V1 TWI",
        description = "TWI support for STM32 I2C2 V1",
        requires = { "HW_I2C_STM32V1", "HW_I2C2_STM32" },
        sources =
        {
            "cm3/dev/stm/stm32_twi.c",
            "cm3/dev/stm/stm32_twi2.c",
        },
    },
    --
    -- STM32 I2C Hardware V1
    --
    {
        name = "nutarch_cm3_stm32_spi_v1",
        brief = "STM32 SPI V1 Controller",
        description = "Generic Suport for STM32 I2C V1",
        requires = { "HW_I2C_STM32V1" },
        sources =
        {
            "cm3/dev/stm/stm32_i2cbus_v1.c",
        },
    },
    --
    -- STM32 I2C Hardware V2
    --
    {
        name = "nutarch_cm3_stm32_spi_v2",
        brief = "STM32 SPI V2 Controller",
        description = "Generic Suport for STM32 I2C V2",
        requires = { "HW_I2C_STM32V2" },
        sources = {
            "cm3/dev/stm/stm32_i2cbus_v2.c",
        },
    },
    --
    -- STM32F0 Combined USART IRQ
    --
    {
        name = "nutarch_cm3_stm32f0_combined_irq",
        brief = "STM32F0 USART IRQ separation",
        description = "IRQ Separation for F0 devices with combined USART IRQ",
        requires = {"HW_COMBINED_IRQ_STM32F0"},
        sources = {"cm3/dev/stm/stm32f0_combined_irq.c"},
    },
    --
    -- STM32L0 Combined IRQ
    --
    {
        name = "nutarch_cm3_stm32l0_combined_irq",
        brief = "STM32L0 IRQ separation",
        description = "IRQ Separation for L0 devices with combined IRQS",
        requires = {"HW_COMBINED_IRQ_STM32L0"},
        sources = {"cm3/dev/stm/stm32l0_combined_irq.c"},
    },
    --
    -- STM32 Signal handler
    {
        name = "nutarch_cm3_stm32_sig",
        brief = "List of additional devices",
        description = "Helper to create the sig_DEVXX entries.\n\n"..
                      "No User configuration needed here.\n",
        options =
        {
            {
                macro = "HW_RTC_STM32V1",
                brief = "RTC V1 Availability",
                description = "RTC V1 Availability",
                requires = { "HW_MCU_STM32F1" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_RTC_STM32_V2",
                brief = "RTC V2 Availability",
                description = "RTC V2 Availability",
                requires = { "HW_RTC_STM32_V2" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA1_STM32F1",
                brief = "DMA1 Availability",
                description = "7 Channel DMA1 on F1/L1/F3",
                requires = { "HW_DMA1_STM32F1" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA2_STM32F1",
                brief = "DMA2 Availability",
                description = "5 Channel DMA2 on F1/L1/F3",
                requires = { "HW_DMA2_STM32F1" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA1_5CH_STM32",
                brief = "STM32F0 Devices 5 Channel DMA1",
                description = "STM32F0 Devices 5 Channel DMA1",
                requires = { "HW_DMA1_5CH_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA_COMBINED_IRQ_STM32",
                brief = "STM32 Devices with combined DMA IRQ",
                description = "RTC V1 Availability",
                requires = { "HW_DMA_COMBINED_IRQ_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA_CSELR_STM32",
                brief = "STM32 DMA CSEL",
                description = "STM32 F1 type DMA with Channel selection",
                requires = { "HW_DMA_CSELR_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_LCD_STM32",
                brief = "STM32 LCD",
                description = "LCD Availability",
                requires = { "HW_LCD_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA2_7CH_STM32",
                brief = "STM32L4 Devices 7 Channel DMA2",
                description = "STM32L4 Devices 7 Channel DMA2",
                requires = { "HW_DMA2_7CH_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_PVD_STM32",
                brief = "PVD availability",
                description = "Device provides PVD IRQ.",
                requires = { "HW_PVD_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_PVD_VDDIO2_STM32",
                brief = "PVD availability",
                description = "Device provides combined PVD/VDDIO2 IRQ.",
                requires = { "HW_PVD_VDDIO2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART_STM32V2",
                brief = "USART IP Version 2",
                description = "USART2 Version 2 availability",
                requires = { "HW_USART_STM32V2" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART1_STM32",
                brief = "USART1 Availability",
                description = "USART2 Availability",
                requires = { "HW_USART1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART2_STM32",
                brief = "USART2 Availability",
                description = "USART2 Availability",
                requires = { "HW_USART2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART3_STM32",
                brief = "USART3 Availability",
                description = "USART3 Availability",
                requires = { "HW_USART3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART4_STM32",
                brief = "UART4 Availability",
                description = "USART4 Availability",
                requires = { "HW_UART4_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART4_STM32",
                brief = "USART4 Availability",
                description = "USART4 Availability",
                requires = { "HW_USART4_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART5_STM32",
                brief = "UART5 Availability",
                description = "USART5 Availability",
                requires = { "HW_UART5_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART5_STM32",
                brief = "USART5 Availability",
                description = "USART5 Availability",
                requires = { "HW_USART5_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART6_STM32",
                brief = "USART6 Availability",
                description = "USART6 Availability",
                requires = { "HW_USART6_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART7_STM32",
                brief = "UART7 Availability",
                description = "UART7 Availability",
                requires = { "HW_UART7_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART7_STM32",
                brief = "USART7 Availability",
                description = "USART7 Availability",
                requires = { "HW_USART7_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_UART8_STM32",
                brief = "UART8 Availability",
                description = "UART8 Availability",
                requires = { "HW_UART8_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART8_STM32",
                brief = "USART8 Availability",
                description = "USART8 Availability",
                requires = { "HW_SUART8_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART3_4_STM32",
                brief = "USART3_4 Availability",
                description = "USART3_4 Availability on F0",
                requires = {"HW_USART3_4_STM32"},
                default = "1",
                provides = {
                    "HW_USART3_STM32",
                    "HW_USART4_STM32",
                    "HW_COMBINED_IRQ_STM32F0",
                    "HW_USART_COMBINED_IRQ_STM32",
                },
               type = "integer",
               file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART4_5_STM32",
                brief = "USART4_5 Availability",
                description = "USART4_5 Availability on L0",
                requires = {"HW_USART4_5_STM32"},
                default = "1",
                provides = {
                    "HW_USART4_STM32",
                    "HW_USART5_STM32",
                    "HW_COMBINED_IRQ_STM32L0",
                    "HW_COMBINED_IRQ_STM32",
                },
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART3_6_STM32",
                brief = "USART3_6 Availability",
                description = "USART3_6 Availability on F0",
                requires = {"HW_USART3_6_STM32"},
                default = "1",
                provides = {
                    "HW_USART3_STM32",
                    "HW_USART4_STM32",
                    "HW_USART5_STM32",
                    "HW_USART6_STM32",
                    "HW_COMBINED_IRQ_STM32F0",
                    "HW_USART_COMBINED_IRQ_STM32",
                },
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART3_8_STM32",
                brief = "USART3_8 Availability",
                description = "USART3_8 Availability on F0",
                requires = {"HW_USART3_8_STM32"},
                default = "1",
                provides = {
                    "HW_USART3_STM32",
                    "HW_USART4_STM32",
                    "HW_USART5_STM32",
                    "HW_USART6_STM32",
                    "HW_USART7_STM32",
                    "HW_USART8_STM32",
                    "HW_COMBINED_IRQ_STM32F0",
                    "HW_USART_COMBINED_IRQ_STM32",
                },
                type = "integer",
                file = "include/cfg/devices.h",
            },
            {
                macro = "HW_RNG_LPUART1_STM32",
                brief = "STM32 LPUSART/RNG separation",
                description = "Separate LPUART and RNG",
                requires = {"HW_RNG_LPUART1_STM32" },
                provides = {
                    "HW_LPUART1_STM32",
                    "HW_RNG_STM32",
                    "HW_COMBINED_IRQ_STM32L0",
                    "HW_USART_COMBINED_IRQ_STM32",
                    "HW_LPUART1_COMBINED_IRQ_STM32",
                    "HW_RNG_COMBINED_IRQ_STM32",
                },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_AES_LPUART1_STM32",
                brief = "STM32 LPUSART/AES separation",
                description = "Separate LPUART and AES",
                requires = {"HW_AES_LPUART1_STM32"},
                provides = {
                    "HW_LPUART1_STM32",
                    "HW_AES_STM32",
                    "HW_COMBINED_IRQ_STM32L0",
                    "HW_USART_COMBINED_IRQ_STM32",
                    "HW_LPUART1_COMBINED_IRQ_STM32",
                    "HW_AES_COMBINED_IRQ_STM32",
                },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_AES_RNG_LPUART1_STM32",
                brief = "STM32 LPUSART/RNG/AES separation",
                description = "Separate LPUART and RNG",
                requires = {"HW_AES_RNG_LPUART1_STM32"},
                provides = {
                    "HW_LPUART1_STM32",
                    "HW_RNG_STM32",
                    "HW_AES_STM32",
                    "HW_COMBINED_IRQ_STM32L0",
                    "HW_USART_COMBINED_IRQ_STM32",
                    "HW_LPUART1_COMBINED_IRQ_STM32",
                    "HW_RNG_COMBINED_IRQ_STM32",
                    "HW_AES_COMBINED_IRQ_STM32",
                },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USART_COMBINED_IRQ_STM32",
                brief = "STM32 Devices with combined USART IRQ",
                description = "STM32 Devices with combined UART IRQ",
                requires = { "HW_USART_COMBINED_IRQ_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h",
            },
            {
                macro = "HW_LPUART1_STM32",
                brief = "LPUART1 Availability",
                description = "LPUART1 Availability",
                requires = { "HW_LPUART1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_LPUART1_COMBINED_IRQ_STM32",
                brief = "LPUART1 Availability",
                description = "LPUART1 Availability",
                requires = { "HW_LPUART1_COMBINED_IRQ_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_RNG_STM32",
                brief = "RNG Availability",
                description = "RNG Availability",
                requires = { "HW_RNG_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_RNG_COMBINED_IRQ_STM32",
                brief = "RNG Availability",
                description = "RNG Availability",
                requires = { "HW_RNG_COMBINED_IRQ_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_AES_STM32",
                brief = "AES Availability",
                description = "AES Availability",
                requires = {"HW_AES_STM32"},
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_AES_COMBINED_IRQ_STM32",
                brief = "AES Availability",
                description = "AES Availability",
                requires = {"HW_AES_COMBINED_IRQ_STM32"},
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_CAN_STM32",
                brief = "CAN Availability",
                description = "CAN Availability",
                requires = { "HW_CAN_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_CAN1_STM32",
                brief = "CAN1 Availability",
                description = "CAN1 Availability",
                requires = { "HW_CAN1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_CAN2_STM32",
                brief = "CAN2 Availability",
                description = "CAN2 Availability",
                requires = { "HW_CAN2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USB_CAN_STM32",
                brief = "USB/CAN exlusive Availability",
                description = "USB/CAN Availability",
                requires = { "HW_USB_CAN_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USB_CAN1_STM32",
                brief = "USB/CAN1 exlusive Availability",
                description = "USB/CAN1 Availability",
                requires = { "HW_USB_CAN1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_USB_STM32",
                brief = "USB FS Availability",
                description = "USB Availability",
                requires = { "HW_USB_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM1_STM32",
                brief = "TIM1 Availability",
                description = "TIM1 Availability",
                requires = { "HW_TIM1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM2_STM32",
                brief = "TIM2 Availability",
                description = "TIM2 Availability",
                requires = { "HW_TIM2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM3_STM32",
                brief = "TIM3 Availability",
                description = "TIM3 Availability",
                requires = { "HW_TIM3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM4_STM32",
                brief = "TIM4 Availability",
                description = "TIM4 Availability",
                requires = { "HW_TIM4_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM5_STM32",
                brief = "TIM5 Availability",
                description = "TIM5 Availability",
                requires = { "HW_TIM5_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM6_STM32",
                brief = "TIM6 Availability",
                description = "TIM6 Availability",
                requires = { "HW_TIM6_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM7_STM32",
                brief = "TIM7 Availability",
                description = "TIM7 Availability",
                requires = { "HW_TIM7_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM8_STM32",
                brief = "TIM8 Availability",
                description = "TIM8 Availability",
                requires = { "HW_TIM8_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM9_STM32",
                brief = "TIM9 Availability",
                description = "TIM9 Availability",
                requires = { "HW_TIM9_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM10_STM32",
                brief = "TIM10 Availability",
                description = "TIM10 Availability",
                requires = { "HW_TIM10_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM11_STM32",
                brief = "TIM11 Availability",
                description = "TIM11 Availability",
                requires = { "HW_TIM11_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM12_STM32",
                brief = "TIM12 Availability",
                description = "TIM12 Availability",
                requires = { "HW_TIM12_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM13_STM32",
                brief = "TIM13 Availability",
                description = "TIM13 Availability",
                requires = { "HW_TIM13_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM14_STM32",
                brief = "TIM14 Availability",
                description = "TIM14 Availability",
                requires = { "HW_TIM14_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM15_STM32",
                brief = "TIM15 Availability",
                description = "TIM15 Availability",
                requires = { "HW_TIM15_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM16_STM32",
                brief = "TIM16 Availability",
                description = "TIM16 Availability",
                requires = { "HW_TIM16_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM17_STM32",
                brief = "TIM17 Availability",
                description = "TIM17 Availability",
                requires = { "HW_TIM17_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_TIM19_STM32",
                brief = "TIM19 Availability",
                description = "TIM19 Availability",
                requires = { "HW_TIM19_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C2_STM32",
                brief = "I2C2 Availability",
                description = "I2C2 Availability",
                requires = { "HW_I2C2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C2_STM32V2",
                brief = "I2C2 Availability",
                description = "I2C2 Availability Version 2 on STM32F3",
                requires = { "HW_I2C2_STM32V2" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C3_STM32",
                brief = "I2C3 Availability",
                description = "I2C3 Availability",
                requires = { "HW_I2C3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_I2C4_STM32",
                brief = "I2C4 Availability",
                description = "I2C4 Availability",
                requires = { "HW_I2C4_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SPI1_STM32",
                brief = "SPI1 Availability",
                description = "SPI1 Availability",
                requires = { "HW_SPI1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SPI2_STM32",
                brief = "SPI2 Availability",
                description = "SPI2 Availability",
                requires = { "HW_SPI2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SPI3_STM32",
                brief = "SPI3 Availability",
                description = "SPI3 Availability",
                requires = { "HW_SPI3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SPI4_STM32",
                brief = "SPI4 Availability",
                description = "SPI4 Availability",
                requires = { "HW_SPI4_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SPI5_STM32",
                brief = "SPI5 Availability",
                description = "SPI5 Availability",
                requires = { "HW_SPI5_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SPI6_STM32",
                brief = "SPI6 Availability",
                description = "SPI6 Availability",
                requires = { "HW_SPI6_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SDIO_STM32",
                brief = "SDIO Availability",
                description = "SDIO Availability",
                requires = { "HW_SDIO_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SAI1_STM32",
                brief = "SAI1 Availability",
                description = "SAI1 Availability",
                requires = { "HW_SAI1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC_STM32",
                brief = "ADC Global interrupt on F2/4",
                description = "ADC Global interrupt on F2/4",
                requires = { "HW_ADC_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC1_STM32F1",
                brief = "ADC1 Single Interrupt on F1",
                description = "ADC1 Single Interrupt on F1",
                requires = { "HW_ADC1_STM32F1" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC1_STM32",
                brief = "ADC1 Single Interrupt",
                description = "ADC1 Single Interrupt",
                requires = { "HW_ADC1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC1_2_STM32F1",
                brief = "ADC1_2 Interrupt on F1",
                description = "ADC1_2 Interrupt on F1",
                requires = { "HW_ADC1_2_STM32F1" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC1_2_STM32",
                brief = "ADC1_2 Interrupt",
                description = "",
                requires = { "HW_ADC1_2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC3_STM32F1",
                brief = "ADC3 Availability on F1",
                description = "ADC3 Availability on F1",
                requires = { "HW_ADC3_STM32F1" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC3_STM32",
                brief = "ADC3 Availability",
                description = "ADC3 Availability",
                requires = { "HW_ADC3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_ADC4_STM32",
                brief = "ADC4 Availability",
                description = "ADC4 Availability",
                requires = { "HW_ADC4_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SDADC1_STM32",
                brief = "SDADC1 Availability",
                description = "SDADC1 Availability",
                requires = { "HW_SDADC1_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SDADC2_STM32",
                brief = "SDADC2 Availability",
                description = "SDADC2 Availability",
                requires = { "HW_SDADC2_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_SDADC3_STM32",
                brief = "SDADC3 Availability",
                description = "SDADC3 Availability",
                requires = { "HW_SDADC3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_HASH_STM32",
                brief = "HASH Availability",
                description = "HASH Availability",
                requires = { "HW_HASH_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_HASH_RNG_STM32",
                brief = "HASH and RNG Availability",
                description = "HASH and RNG Availability",
                requires = { "HW_HASH_RNG_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DCMI_STM32",
                brief = "CAMERA Availability",
                description = "CAMERA interface Availability",
                requires = { "HW_DCMI_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_LTDC_STM32",
                brief = "LTDC Availability",
                description = "LTDC Availability",
                requires = { "HW_LTDC_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_DMA2D_STM32",
                brief = "DMA2D Availability",
                description = "DMA2D Availability",
                requires = { "HW_DMA2D_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_EMAC_STM32",
                brief = "Ethernet Availability",
                description = "Ethernet Availability",
                requires = { "HW_EMAC_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_FSMC_STM32",
                brief = "FSMC Availability",
                description = "FSMC Availability",
                requires = { "HW_FSMC_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_FMC_STM32",
                brief = "FMC Availability",
                description = "FMC Availability",
                requires = { "HW_FMC_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_COMP1_2_3_STM32",
                brief = "COMP1_2_3 Availability",
                description = "COMP1_2_3 Availability",
                requires = { "HW_COMP1_2_3_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_COMP4_5_6_STM32",
                brief = "COMP4_5_6 Availability",
                description = "COMP4_5_6 Availability",
                requires = { "HW_COMP4_5_6_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_COMP7_STM32",
                brief = "COMP7 Availability",
                description = "COMP7 Availability",
                requires = { "HW_COMP7_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_COMP_STM32",
                brief = "COMP Availability",
                description = "COMP Availability",
                requires = { "HW_COMP_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_COMP_ACQ_STM32",
                brief = "COMP_ACQ Availability",
                description = "COMP_ACQ Availability",
                requires = { "HW_COMP_ACQ_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
            {
                macro = "HW_CEC_STM32",
                brief = "CEC Availability",
                description = "Availability",
                requires = { "HW_CEC_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/devices.h"
            },
             {
                macro = "HW_CCM_STM32",
                brief = "CCM Availability",
                description = "Core coupled RAM",
                requires = { "HW_CCM_STM32" },
                default = "1",
                type = "integer",
                file = "include/cfg/arch.h"
            },
        }
    },
}