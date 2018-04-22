
mcf51cn_sci1_txd_pins = { "PTD0" }
mcf51cn_sci1_rxd_pins = { "PTD1" }
mcf51cn_sci1_txd_pins_descr = "Choose SCI1 TXD Pin:\n\tPTD0 (RGPIO0)"
mcf51cn_sci1_rxd_pins_descr = "Choose SCI1 RXD Pin:\n\tPTD1 (RGPIO1)"

mcf51cn_sci2_txd_pins = { "PTD2" }
mcf51cn_sci2_rxd_pins = { "PTD3" }
mcf51cn_sci2_txd_pins_descr = "Choose SCI2 TXD Pin:\n\tPTD2 (RGPIO2)"
mcf51cn_sci2_rxd_pins_descr = "Choose SCI2 RXD Pin:\n\tPTD3 (RGPIO3)"

mcf51cn_sci3_txd_pins = { "PTE6", "PTA3" }
mcf51cn_sci3_rxd_pins = { "PTE7", "PTA4" }
mcf51cn_sci3_txd_pins_descr = "Choose SCI3 TXD Pin:\n\tPTE6 (KBI2P6)\n\tPTA3"
mcf51cn_sci3_rxd_pins_descr = "Choose SCI3 RXD Pin:\n\tPTE7 (KBI2P7)\n\tPTA4"

nutarch_m68k_coldfire_mcf51cn =
{
   	--
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_family",
        brief = "Family",
        provides = {
                "HW_MTIM_COLDFIRE",
                "HW_IIC_COLDFIRE",
                "HW_SCI_COLDFIRE",
--              "HW_SPI_COLDFIRE",
--              "HW_ADC_COLDFIRE",
--              "HW_RTC_COLDFIRE",
--              "HW_TMP_COLDFIRE",
--              "HW_KBI_COLDFIRE",
--              "HW_FEC_COLDFIRE",
--              "HW_MFB_COLDFIRE",
        },
        options =
        {
            {
                macro = "MCU_MCF51CN",
                brief = "MCF51CN",
                description = "MCF51CN Coldfire Family",
                type = "integer",
                default = 1,
                file = "include/cfg/arch.h"
            },
            {
                macro = "IIC1",
                type = "integer",
                default = 1,
                provides = { "HW_IIC1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "IIC2",
                type = "integer",
                default = 1,
                provides = { "HW_IIC2" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "MTIM1",
                type = "integer",
                default = 1,
                provides = { "HW_MTIM1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "MTIM2",
                type = "integer",
                default = 1,
                provides = { "HW_MTIM2" },
                file = "include/cfg/peripherals.h"
            },
            
            {
                macro = "SCI1",
                type = "integer",
                default = 1,
                provides = { "HW_SCI1" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "SCI2",
                type = "integer",
                default = 1,
                provides = { "HW_SCI2" },
                file = "include/cfg/peripherals.h"
            },
            {
                macro = "SCI3",
                type = "integer",
                default = 1,
                provides = { "HW_SCI3" },
                file = "include/cfg/peripherals.h"
            },
        }
    },
    
    --
    -- Runtime Initialization
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_init",
        brief = "Initialization",
		description = "System startup code for MCF51cn family MCUs:\n"..
		              "  - Vector table\n"..
		              "  - Memories\n"..
		              "  - MCU\n"..
		              "  - Peripherals",
        sources = { 
--                  "m68k/coldfire/init/crt_common.S",
--                  "m68k/coldfire/init/crt_$(LDNAME).S",
--                  "m68k/coldfire/init/crt_mcf51cn.S",
                    "m68k/coldfire/init/crt_common_c.c", 
        			"m68k/coldfire/init/crt_mcf51cn_c.c",
        		  },
        targets = { 
--                  "m68k/coldfire/init/crt_common.o",
--                  "m68k/coldfire/init/crt_mcf51cn.o", 
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
        name = "nutarch_m68k_coldfire_mcf51cn_init2",
        brief = "Initialization",
        description = "System startup code for MCF51CN family MCUs:\n"..
                      "  - Vector table\n"..
                      "  - Memories\n"..
                      "  - MCU\n"..
                      "  - Peripherals",
        sources = { 
                    "m68k/coldfire/init/crt_common.S",
                    "m68k/coldfire/init/crt_$(LDNAME).S",
                    "m68k/coldfire/init/crt_mcf51cn.S",
                  },
        targets = { 
                    "m68k/coldfire/init/crt_common.o",
                    "m68k/coldfire/init/crt_$(LDNAME).o",
                    "m68k/coldfire/init/crt_mcf51cn.o", 
                  },
        requires = { "TOOL_CC_M68K", "TOOL_GCC"},
    },    

    -- 
    -- Multipurpose Clock Generator
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg",
        brief = "Clock Setup",
        description = "Multipurpose Clock Generator",
        script = "arch/coldfire/mcf51cn_mcg.nut"
    },
    
    --
    -- GPIO Interface
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_gpio",
        brief = "GPIO",
        description = "Generic port I/O API.",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_gpio.c"}
    },
    

    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_ihndlr",
        brief = "Interrupt Handler",
        description = "Peripheral interrupt handlers for MCF51CN family.",
        provides = { 
                     "DEV_IRQ_MTIM1",
                     "DEV_IRQ_MTIM2",
                     "DEV_IRQ_IIC1",
                     "DEV_IRQ_IIC2",
                   },
        sources = { "m68k/coldfire/dev/mcf51cn/ih_mcf51cn_common.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_mtim.c",
          			"m68k/coldfire/dev/mcf51cn/ih_mcf51cn_iic.c",
        			},
    },

    --
    -- System Timer Hardware
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_ostimer",
        brief = "System Timer",
        requires = { "HW_MTIM_COLDFIRE", "DEV_IRQ_MTIM1" },
        provides = { "NUT_OSTIMER_DEV" },
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_ostimer.c" },
    },
    
    --
    -- Reset Controller
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_reset",
        brief = "Reset Controller",
        sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_reset.c" },
    },
}
