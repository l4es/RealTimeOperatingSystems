nutarch_m68k_coldfire_devices =
{
    --
    -- I2C Bus Default
    --
    {
        name = "nutarch_m68k_coldfire_devices_i2c",
        brief = "I2C Bus Default",
        description = "Choose which I2C bus will be used as a default.\n"..
                      "TwiXxx() Macros will be reconfigured to use this bus.\n"..
                      "This is for compatibility with old Nut/OS.",
        requires = { "DEV_TWI" },
        options =
        {
            {
                macro = "I2C0_AS_DEFAULT",
                brief = "I2C Bus 0 Controller",
                description = "TwiXxx() Macros will be reconfigured to use I2C Bus 0 Controller.",
                exclusivity = { "I2C0_AS_DEFAULT", "I2C1_AS_DEFAULT" },
                flavor = "booldata",
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C1_AS_DEFAULT",
                brief = "I2C Bus 1 Controller",
                description = "TwiXxx() Macros will be reconfigured to use I2C Bus 1 Controller.",
                exclusivity = { "I2C0_AS_DEFAULT", "I2C1_AS_DEFAULT" },
                flavor = "booldata",
                file = "include/cfg/twi.h"
            },
        },
    },
    
    --
    -- I2C Bus 0 Configuration
    --
    {
        name = "nutarch_m68k_coldfire_devices_i2c0",
        brief = "I2C Bus 0 Controller",
        description = "Coldfire hardware I2C support.",
        requires = { "HW_I2C_COLDFIRE", "HW_I2C0" },
        provides = { "DEV_TWI" },
        sources = { "m68k/coldfire/dev/common/mcf5_i2c.c",
                    "m68k/coldfire/dev/common/mcf5_i2c0.c" },
        options =
        {
            {
                macro = "I2C0_SCL_PORTPIN",
                brief = "I2C0 SCL Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c0_scl_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c0_scl_pins; end;
                    end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C0_SDA_PORTPIN",
                brief = "I2C0 SDA Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c0_sda_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c0_sda_pins; end;
                    end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C0_DEFAULT_SPEED",
                brief = "Default Speed",
                description = "Default speed for this bus. Different speeds can be set by software.\n",
                default = "100",
                file = "include/cfg/twi.h"
            },
        },
    },
    
    --
    -- I2C Bus 1 Configuration
    --
    {
        name = "nutarch_m68k_coldfire_devices_i2c1",
        brief = "I2C Bus 1 Controller",
        description = "Coldfire hardware I2C support.",
        requires = { "HW_I2C_COLDFIRE", "HW_I2C1" },
        provides = { "DEV_TWI" },
        sources = { "m68k/coldfire/dev/common/mcf5_i2c.c",
                    "m68k/coldfire/dev/common/mcf5_i2c1.c" },
        options =
        {
            {
                macro = "I2C1_SCL_PORTPIN",
                brief = "I2C1 SCL Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c1_scl_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c1_scl_pins; end;
                    end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C1_SDA_PORTPIN",
                brief = "I2C1 SDA Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c1_sda_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_i2c1_sda_pins; end;
                    end,
                file = "include/cfg/twi.h"
            },
            {
                macro = "I2C1_DEFAULT_SPEED",
                brief = "Default Speed",
                description = "Default speed for this bus. Different speeds can be set by software.\n",
                default = "100",
                file = "include/cfg/twi.h"
            },
        },
    },
    
    --
    -- SCI Debug Output
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci_debug",
        brief = "SCI Debug Output",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "This simple SCI output driver uses polling instead of interrupts "..
                      "and can be used within interrupt routines.\n"..
                      "It is mainly used for debugging and tracing.\n\n"..
                      "This driver uses only RXD and TXD pins which are configured in SCIn Drivers below.\n\n"..
                      "Call one of following functions:\n"..
                      "\tNutRegisterDevice(&devDebug1, 0, 0) for SCI1\n"..
                      "\tNutRegisterDevice(&devDebug2, 0, 0) for SCI2\n"..
                      "\t...\n"..
                      "\tNutRegisterDevice(&devDebugn, 0, 0) for SCIn\n\n"..
                      "Then you can use any of the stdio functions to open device sci0, sci1, ..., scin.\n".. 
                      "",
        requires = { "HW_SCI_COLDFIRE" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        sources = { "m68k/coldfire/dev/common/mcf5_sci_debug.c"},
    },

    --
    -- SCI1 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci1",
        brief = "SCI1 Driver",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic USART driver framework.",
        requires = { "HW_SCI_COLDFIRE", "HW_SCI1" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
--      sources = { "m68k/coldfire/dev/common/mcf5_sci1.c" },
        options =
        {
            {
                macro = "SCI1_TXD_PORTPIN",
                brief = "SCI1 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_txd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
            {
                macro = "SCI1_RXD_PORTPIN",
                brief = "SCI1 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci1_rxd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
        },
    },
        
    --
    -- SCI2 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci2",
        brief = "SCI2 Driver",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic USART driver framework.",
        requires = { "HW_SCI_COLDFIRE", "HW_SCI2" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
--      sources = { "m68k/coldfire/dev/common/mcf5_sci2.c" },
        options =
        {
            {
                macro = "SCI2_TXD_PORTPIN",
                brief = "SCI2 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_txd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
            {
                macro = "SCI2_RXD_PORTPIN",
                brief = "SCI2 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci2_rxd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
        },
    },
        
    --
    -- SCI3 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_sci3",
        brief = "SCI3 Driver",
        description = "In current implementation the SCI (Serial Communication Interface) is used as an UART device only.\n\n"..
                      "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic USART driver framework.",
        requires = { "HW_SCI_COLDFIRE", "HW_SCI3" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
--      sources = { "m68k/coldfire/dev/common/mcf5_sci3.c" },
        options =
        {
            {
                macro = "SCI3_TXD_PORTPIN",
                brief = "SCI3 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_txd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
            {
                macro = "SCI3_RXD_PORTPIN",
                brief = "SCI3 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF51CN") then return mcf51cn_sci3_rxd_pins end;
                    end,
                file = "include/cfg/sci.h"
            },
        },
    },
        
    --
    -- UART Debug Output
    --
    {
        name = "nutarch_m68k_coldfire_devices_uart_debug",
        brief = "UART Debug Output",
        description = "This simple UART output driver uses polling instead of interrupts "..
                      "and can be used within interrupt routines.\n"..
                      "It is mainly used for debugging and tracing.\n\n"..
                      "This driver uses only RXD and TXD pins which are configured in UARTn Drivers below.\n\n"..
                      "Call one of following functions:\n"..
                      "\tNutRegisterDevice(&devDebug0, 0, 0) for UART0\n"..
                      "\tNutRegisterDevice(&devDebug1, 0, 0) for UART1\n"..
                      "\t...\n"..
                      "\tNutRegisterDevice(&devDebugn, 0, 0) for UARTn\n\n"..
                      "Then you can use any of the stdio functions to open device uart0, uart1, ..., uartn.\n".. 
                      "",
        requires = { "HW_UART_COLDFIRE" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        sources = { "m68k/coldfire/dev/common/mcf5_uart_debug.c"},
    },

    --
    -- UART0 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_uart0",
        brief = "UART0 Driver",
        description = "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART_COLDFIRE", "HW_UART0" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/common/mcf5_uart0.c" },
        options =
        {
            {
                macro = "UART0_TXD_PORTPIN",
                brief = "UART0 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart0_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart0_txd_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RXD_PORTPIN",
                brief = "UART0 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart0_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart0_rxd_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RTS_PORTPIN",
                brief = "UART0 RTS Pin selection",
                flavor = "booldata",
                type = "enumerated",
                description = 
                    function()
                        local base_descr = "When selected, the driver will support RTS hardware handshake.\n";
                        if c_is_provided("HW_MCU_MCF5225X") then return base_descr .. mcf5225x_uart0_rts_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart0_rts_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_CTS_PORTPIN",
                brief = "UART0 CTS Pin selection",
                flavor = "booldata",
                type = "enumerated",
                description =
                    function() 
                        local base_descr = "When selected, the driver will support CTS hardware handshake.\n";
                        if c_is_provided("HW_MCU_MCF5225X") then return base_descr .. mcf5225x_uart0_cts_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart0_cts_pins end;
                    end,
                file = "include/cfg/uart.h"
            },  
            {
                macro = "UART0_HARDWARE_HDX",
                brief = "Half Duplex Mode",
                description = "When selected, the driver will be configured for half-duplex mode.\n",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },          
            {
                macro = "UART0_XONXOFF",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will support software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_HDX_CTRL",
                brief = "HDX Control (Half/Full duplex)",
                description = "When selected a GPIO pin and/or Board specific functions may be configured for Half/Full duplex control.",
                flavor = "booldata",
                provides = { "UART0_HDX_CTRL" },
                file = "include/cfg/uart.h",
            },
            {
                macro = "UART0_HDX_CTRL_BOARD_SPEC",
                brief = "HDX Control Board specific",
                description = "When selected, following board specific function may be implemented for Half/Full duplex control:\n"..
                              "TODO + path to board file",
                requires = { "UART0_HDX_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_HDX_CTRL_INV",
                brief = "HDX Control Invert",
                    description = "Sets the HDX signal as active low.\n",
                requires = { "UART0_HDX_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_HDX_CTRL_PORT",
                brief = "HDX Control Port",
                    description = "Select the port of the HDX signal.\n",
                requires = { "UART0_HDX_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_HDX_CTRL_PIN",
                brief = "HDX Control Pin",
                    description = "Select the pin for the HDX signal.\n",
                requires = { "UART0_HDX_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL",
                brief = "RS485 direction control",
                description = "When selected a GPIO pin(s) and/or Board specific functions may be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "UART0_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL_BOARD_SPEC",
                brief = "RS485 Control Board specific",
                description = "When selected, following board specific function may be implemented for RS485 direction control:\n"..
                              "TODO",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL_DE_INV",
                brief = "RS485 DE Control Invert",
                    description = "Sets the DE signal as active low.\n",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL_DE_PORT",
                brief = "RS485 DE Control Port",
                    description = "Select the port of the DE signal.\n",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL_DE_PIN",
                brief = "RS485 DE Control Pin",
                    description = "Select the pin for the DE signal.\n",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            { 
                macro = "UART0_RS485_CTRL_RE_INV",
                brief = "RS485 /RE Control Invert",
                    description = "Sets the RE signal as active high.\n",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL_RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.\n",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART0_RS485_CTRL_RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.\n",
                requires = { "UART0_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
        },
    },
    
    --
    -- UART1 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_uart1",
        brief = "UART1 Driver",
        description = "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART_COLDFIRE", "HW_UART1" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/common/mcf5_uart1.c" },
        options =
        {
            {
                macro = "UART1_TXD_PORTPIN",
                brief = "UART1 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart1_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart1_txd_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RXD_PORTPIN",
                brief = "UART1 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart1_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart1_rxd_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RTS_PORTPIN",
                brief = "UART1 RTS Pin selection",
                flavor = "booldata",
                type = "enumerated",
                description = 
                    function()
                        local base_descr = "When selected, the driver will support RTS hardware handshake.\n";
                        if c_is_provided("HW_MCU_MCF5225X") then return base_descr .. mcf5225x_uart1_rts_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart1_rts_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_CTS_PORTPIN",
                brief = "UART1 CTS Pin selection",
                flavor = "booldata",
                type = "enumerated",
                description =
                    function() 
                        local base_descr = "When selected, the driver will support CTS hardware handshake.\n";
                        if c_is_provided("HW_MCU_MCF5225X") then return base_descr .. mcf5225x_uart1_cts_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart1_cts_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_HARDWARE_HDX",
                brief = "Half Duplex Mode",
                description = "When selected, the driver will be configured for half-duplex mode.\n",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },            
            {
                macro = "UART1_XONXOFF",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will support software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_HDX_CTRL",
                brief = "HDX Control (Half/Full duplex)",
                description = "When selected a GPIO pin and/or Board specific functions may be configured for Half/Full duplex control.",
                flavor = "booldata",
                provides = { "UART1_HDX_CTRL" },
                file = "include/cfg/uart.h",
            },
            {
                macro = "UART1_HDX_CTRL_BOARD_SPEC",
                brief = "HDX Control Board specific",
                description = "When selected, following board specific function may be implemented for Half/Full duplex control:\n"..
                              "TODO + path to board file",
                requires = { "UART1_HDX_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_HDX_CTRL_INV",
                brief = "HDX Control Invert",
                    description = "Sets the HDX signal as active low.\n",
                requires = { "UART1_HDX_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_HDX_CTRL_PORT",
                brief = "HDX Control Port",
                    description = "Select the port of the HDX signal.\n",
                requires = { "UART1_HDX_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_HDX_CTRL_PIN",
                brief = "HDX Control Pin",
                    description = "Select the pin for the HDX signal.\n",
                requires = { "UART1_HDX_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL",
                brief = "RS485 direction control",
                description = "When selected a GPIO pin(s) and/or Board specific functions may be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "UART1_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL_BOARD_SPEC",
                brief = "RS485 Control Board specific",
                description = "When selected, following board specific function may be implemented for RS485 direction control:\n"..
                              "TODO",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL_DE_INV",
                brief = "RS485 DE Control Invert",
                    description = "Sets the DE signal as active low.\n",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL_DE_PORT",
                brief = "RS485 DE Control Port",
                    description = "Select the port of the DE signal.\n",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL_DE_PIN",
                brief = "RS485 DE Control Pin",
                    description = "Select the pin for the DE signal.\n",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            { 
                macro = "UART1_RS485_CTRL_RE_INV",
                brief = "RS485 /RE Control Invert",
                    description = "Sets the RE signal as active high.\n",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL_RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.\n",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART1_RS485_CTRL_RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.\n",
                requires = { "UART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
        },
    },
    
    --
    -- UART2 Interface
    --
    {
        name = "nutarch_m68k_coldfire_devices_uart2",
        brief = "UART2 Driver",
        description = "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART_COLDFIRE", "HW_UART2" },
        provides = { "DEV_UART", "DEV_UART_SPECIFIC" },
        sources = { "m68k/coldfire/dev/common/mcf5_uart2.c" },
        options =
        {
            {
                macro = "UART2_TXD_PORTPIN",
                brief = "UART2 TXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart2_txd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart2_txd_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RXD_PORTPIN",
                brief = "UART2 RXD Pin selection",
                type = "enumerated",
                description =
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart2_rxd_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart2_rxd_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RTS_PORTPIN",
                brief = "UART2 RTS Pin selection",
                flavor = "booldata",
                type = "enumerated",
                description = 
                    function()
                        local base_descr = "When selected, the driver will support RTS hardware handshake.\n";
                        if c_is_provided("HW_MCU_MCF5225X") then return base_descr .. mcf5225x_uart2_rts_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart2_rts_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_CTS_PORTPIN",
                brief = "UART2 CTS Pin selection",
                flavor = "booldata",
                type = "enumerated",
                description =
                    function() 
                        local base_descr = "When selected, the driver will support CTS hardware handshake.\n";
                        if c_is_provided("HW_MCU_MCF5225X") then return base_descr .. mcf5225x_uart2_cts_pins_descr; end;
                    end,
                choices = 
                    function() 
                        if c_is_provided("HW_MCU_MCF5225X") then return mcf5225x_uart2_cts_pins end;
                    end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_HARDWARE_HDX",
                brief = "Half Duplex Mode",
                description = "When selected, the driver will be configured for half-duplex mode.\n",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },            
            {
                macro = "UART2_XONXOFF",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will support software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_HDX_CTRL",
                brief = "HDX Control (Half/Full duplex)",
                description = "When selected a GPIO pin and/or Board specific functions may be configured for Half/Full duplex control.",
                flavor = "booldata",
                provides = { "UART2_HDX_CTRL" },
                file = "include/cfg/uart.h",
            },
            {
                macro = "UART2_HDX_CTRL_BOARD_SPEC",
                brief = "HDX Control Board specific",
                description = "When selected, following board specific function may be implemented for Half/Full duplex control:\n"..
                              "TODO + path to board file",
                requires = { "UART2_HDX_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_HDX_CTRL_INV",
                brief = "HDX Control Invert",
                    description = "Sets the HDX signal as active low.\n",
                requires = { "UART2_HDX_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_HDX_CTRL_PORT",
                brief = "HDX Control Port",
                    description = "Select the port of the HDX signal.\n",
                requires = { "UART2_HDX_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_HDX_CTRL_PIN",
                brief = "HDX Control Pin",
                    description = "Select the pin for the HDX signal.\n",
                requires = { "UART2_HDX_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL",
                brief = "RS485 direction control",
                description = "When selected a GPIO pin(s) and/or Board specific functions may be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "UART2_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL_BOARD_SPEC",
                brief = "RS485 Control Board specific",
                description = "When selected, following board specific function may be implemented for RS485 direction control:\n"..
                              "TODO",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL_DE_INV",
                brief = "RS485 DE Control Invert",
                    description = "Sets the DE signal as active low.\n",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL_DE_PORT",
                brief = "RS485 DE Control Port",
                    description = "Select the port of the DE signal.\n",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL_DE_PIN",
                brief = "RS485 DE Control Pin",
                    description = "Select the pin for the DE signal.\n",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            { 
                macro = "UART2_RS485_CTRL_RE_INV",
                brief = "RS485 /RE Control Invert",
                    description = "Sets the RE signal as active high.\n",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL_RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.\n",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioPortIds() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "UART2_RS485_CTRL_RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.\n",
                requires = { "UART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
        },
    },
}
