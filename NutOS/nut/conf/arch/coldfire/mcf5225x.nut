            
mcf5225x_i2c0_scl_pins = { "PAS0", "PQS2" }
mcf5225x_i2c0_sda_pins = { "PAS1", "PQS3" }

mcf5225x_i2c0_scl_pins_descr = "Choose I2C 0 SCL pin:\n\tPAS0 (I2C_SCL0)\n\tPQS2 (QSPI_CLK)"
mcf5225x_i2c0_sda_pins_descr = "Choose I2C 0 SDA pin:\n\tPAS1 (I2C_SDA0)\n\tPQS3 (QSPI_CS0)"

mcf5225x_i2c1_scl_pins = { "PQS0", "PUB0", "PUC3", "PTH3" }
mcf5225x_i2c1_sda_pins = { "PQS1", "PUB1", "PUC2", "PTH2" }

mcf5225x_i2c1_scl_pins_descr = "Choose I2C 1 SCL pin:\n\tPQS0 (QSPI_DOUT)\n\tPUB0 (UTXD1)\n\tPUC3 (UCTS2)\n\tPTH3 (FB_D5)"
mcf5225x_i2c1_sda_pins_descr = "Choose I2C 1 SDL pin:\n\tPQS1 (QSPI_IN)  \n\tPUB1 (URXD1)\n\tPUC2 (URTS2)\n\tPTH2 (FB_D4)"

mcf5225x_uart0_txd_pins = { "PUA0" }
mcf5225x_uart0_rxd_pins = { "PUA1" }
mcf5225x_uart0_rts_pins = { "PUA2" }
mcf5225x_uart0_cts_pins = { "PUA3" }
mcf5225x_uart0_txd_pins_descr = "Choose UART0 TXD Pin:\n\tPUA0 (UTXD0)"
mcf5225x_uart0_rxd_pins_descr = "Choose UART0 RXD Pin:\n\tPUA1 (URXD0)"
mcf5225x_uart0_rts_pins_descr = "Choose UART0 RTS Pin:\n\tPUA2 (URTS0)"
mcf5225x_uart0_cts_pins_descr = "Choose UART0 CTS Pin:\n\tPUA3 (UCTS0)"

mcf5225x_uart1_txd_pins = { "PUB0", "PQS0" }
mcf5225x_uart1_rxd_pins = { "PUB1", "PQS1" }
mcf5225x_uart1_rts_pins = { "PUB2", "PQS2" }
mcf5225x_uart1_cts_pins = { "PUB3", "PQS3" }
mcf5225x_uart1_txd_pins_descr = "Choose UART1 TXD Pin:\n\tPUB0 (UTXD1)\n\tPQS0 (QSPI_DOUT)"
mcf5225x_uart1_rxd_pins_descr = "Choose UART1 RXD Pin:\n\tPUB1 (URXD1)\n\tPQS1 (QSPI_DIN)"
mcf5225x_uart1_rts_pins_descr = "Choose UART1 RTS Pin:\n\tPUB2 (URTS1)\n\tPQS2 (QSPI_CLK)"
mcf5225x_uart1_cts_pins_descr = "Choose UART1 CTS Pin:\n\tPUB3 (UCTS1)\n\tPQS3 (QSPI_CS0)"

mcf5225x_uart2_txd_pins = { "PUC0", "PUB2", "PAS0" }
mcf5225x_uart2_rxd_pins = { "PUC1", "PUB3", "PAS1" }
mcf5225x_uart2_rts_pins = { "PUC2" }
mcf5225x_uart2_cts_pins = { "PUC3" }
mcf5225x_uart2_txd_pins_descr = "Choose UART2 TXD Pin:\n\tPUC0 (UTXD2)\n\tPUB2 (URTS1)\n\tPAS0 (I2C_SCL0)"
mcf5225x_uart2_rxd_pins_descr = "Choose UART2 RXD Pin:\n\tPUC1 (URXD2)\n\tPUB3 (UCTS1)\n\tPAS1 (I2C_SDA0)"
mcf5225x_uart2_rts_pins_descr = "Choose UART2 RTS Pin:\n\tPUC2 (URTS2)"
mcf5225x_uart2_cts_pins_descr = "Choose UART2 CTS Pin:\n\tPUC3 (UCTS2)"

nutarch_m68k_coldfire_mcf5225x =
{
   	--
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_family",
        brief = "Family",
        provides = {
                "HW_GPIO_COLDFIRE",
	        	"HW_UART_COLDFIRE",
	        	"HW_PIT_COLDFIRE",
	        	"HW_CWD_COLDFIRE",
	        	"HW_I2C_COLDFIRE",
		},
        options =
        {
            {
                macro = "MCU_MCF5225X",
                brief = "MCF5225X",
                description = "MCF5225X Coldfire Family",
                type = "integer",
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "PIT0",
                type = "integer",
                default = 1,
                provides = { "HW_PIT0" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "PIT1",
                type = "integer",
                default = 1,
                provides = { "HW_PIT1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "UART0",
                type = "integer",
                default = 1,
                provides = { "HW_UART0" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "UART1",
                type = "integer",
                default = 1,
                provides = { "HW_UART1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "UART2",
                type = "integer",
                default = 1,
                provides = { "HW_UART2" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "I2C0",
                type = "integer",
                default = 1,
                provides = { "HW_I2C0" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "I2C1",
                type = "integer",
                default = 1,
                provides = { "HW_I2C1" },
                file = "include/cfg/peripherals.h"
            },
        }
    },
    
    --
    -- Runtime Initialization
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_init",
        brief = "Initialization",
		description = "System startup code for MCF5225X family MCUs:\n"..
		              "  - Vector table\n"..
		              "  - Memories\n"..
		              "  - MCU\n"..
		              "  - Peripherals",
        sources = { 
--                  "m68k/coldfire/init/crt_common.S",
--                  "m68k/coldfire/init/crt_$(LDNAME).S",
--                  "m68k/coldfire/init/crt_mcf5225x.S",
                    "m68k/coldfire/init/crt_common_c.c", 
        			"m68k/coldfire/init/crt_mcf5225x_c.c",
        		  },
        targets = { 
--                  "m68k/coldfire/init/crt_common.o",
--                  "m68k/coldfire/init/crt_mcf5225x.o", 
--                  "m68k/coldfire/init/crt_$(LDNAME).o",
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
    },    

    --
    -- Runtime Initialization 2
    -- FIXME: Initialization code is in two groups due to problems with Configurator.
    --        If "targets" are used together with more than one .c sources, then 
    --        the configurator crashes when building. 
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_init2",
        brief = "Initialization",
        description = "System startup code for MCF5225X family MCUs:\n"..
                      "  - Vector table\n"..
                      "  - Memories\n"..
                      "  - MCU\n"..
                      "  - Peripherals",
        sources = { 
                    "m68k/coldfire/init/crt_common.S",
                    "m68k/coldfire/init/crt_$(LDNAME).S",
                    "m68k/coldfire/init/crt_mcf5225x.S",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_mcf5225x.o", 
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
    },    

    --
    -- GPIO Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_gpio",
        brief = "GPIO",
        description = "Generic port I/O API.",
        requires = { "HW_GPIO_COLDFIRE" },
        sources = { "m68k/coldfire/dev/mcf5225x/mcf5225x_gpio.c"}
    },
    
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_ihndlr",
        brief = "Interrupt Handler",
        description = "Peripheral interrupt handlers for MCF5225X family.",
        provides = { "DEV_IRQ_PIT0",
                     "DEV_IRQ_PIT1",
                     "DEV_IRQ_CWD",
                     "DEV_IRQ_I2C0",
                     "DEV_IRQ_I2C1",
                     "DEV_IRQ_UART0",
                     "DEV_IRQ_UART1",
                     "DEV_IRQ_UART2",
                   },
        sources = { "m68k/coldfire/dev/mcf5225x/ih_mcf5225x_common.c",
        			"m68k/coldfire/dev/mcf5225x/ih_mcf5225x_pit.c",
        			"m68k/coldfire/dev/mcf5225x/ih_mcf5225x_cwd.c",
        			"m68k/coldfire/dev/mcf5225x/ih_mcf5225x_i2c.c",
        			"m68k/coldfire/dev/mcf5225x/ih_mcf5225x_uart.c" },
    },

    --
    -- System Timer Hardware
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_ostimer",
        brief = "System Timer",
        requires = { "HW_PIT_COLDFIRE", "HW_PIT0", "DEV_IRQ_PIT0" },
        provides = { "NUT_OSTIMER_DEV" },
        sources = { "m68k/coldfire/dev/mcf5225x/mcf5225x_ostimer.c" },
    },
    --
    -- Reset Controller
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_reset",
        brief = "Reset Controller",
        sources = { "m68k/coldfire/dev/mcf5225x/mcf5225x_reset.c" },
    },
    --
    -- Core Watchdog
    --
    {
        name = "nutarch_m68k_coldfire_mcf5225x_cwd",
        brief = "Core Watchdog",
        requires = {"HW_CWD_COLDFIRE", "DEV_IRQ_CWD" },
        sources = { "m68k/coldfire/dev/mcf5225x/mcf5225x_cwd.c" }
    },
}
