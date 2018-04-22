
--
-- Multipurpore Clock Generator
--
nutarch_m68k_coldfire_mcf51cn_mcg =
{
    sources = { "m68k/coldfire/dev/mcf51cn/mcf51cn_mcg.c"},
    options =
    {
        {
            macro = "MCG_BUS_DIV",
            brief = "Bus Frequency Divider",
            type = "enumerated",
            description = "Selects the amount to divide down the clock source selected by 'clock mode' below."..
                        "\n"..
                        "\n!! WARNING, the resulting bus clock frequency should not exceed the maximum specified bus clock frequency of the device."..
                        "\n"..
                        "\nClock References:"..
                        "\n  F.int = 32 kHz" ..
                        "\n  F.ext = 32 kHz - 40 MHz"..
                        "\n"..
                        "\nSetup Limitations:"..
                        "\n  FEE, FBE Mode: F.ext / 'Ref. Clock Divider' must be in the range of 31.25 kHz to 39.0625 kHz"..
                        "\n  PEE, PBE Mode: F.ext / 'Ref. Clock Divider' must be in the range of 1 MHz to 2 MHz"..
                        "\n"..
                        "\nBus Clock Calculations:"..
                        "\n  BUSCLK = CPUCLK / 2"..
                        "\n"..
                        "\n  FEI Mode: CPUCLK  = (F.int * 'FLL Multiply Factor' ) / 'Bus Frequency Divider'"..
                        "\n  FEE Mode: CPUCLK  = (F.ext / 'Ref. Clock Divider' * 'FLL Multiply Factor') / 'Bus Frequency Divider'"..
                        "\n  FBI Mode: CPUCLK  =  F.int / 'Bus Frequency Divider'"..
                        "\n  FBE Mode: CPUCLK  =  F.ext / 'Bus Frequency Divider'"..
                        "\n  PEE Mode: CPUCLK  = [(F.ext / 'Ref. Clock Divider') * 'PLL Multiply Factor'] / 'Bus Frequency Divider'"..
                        "\n  PBE Mode: CPUCLK  =  F.ext / 'Bus Frequency Divider'"..
                        "\n  BLPI Mode: CPUCLK =  F.int / 'Bus Frequency Divider'"..
                        "\n  BLPE Mode: CPUCLK =  F.ext / 'Bus Frequency Divider'",
            choices = { "1", "2", "4", "8" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_FEI",
            brief = "FLL Engaged Internal (FEI)",
            description = "In FLL engaged internal mode, the bus clock is derived from the FLL which is controlled by the internal reference clock.\n"..
                          "The Background Debug Controller (BDC) clock is supplied from the FLL.",
            provides = { "MCG_FLL", "MCG_INT_REF"},
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_FEE",
            brief = "FLL Engaged External (FEE)",
            description = "In FLL engaged external mode, the bus clock is derived from the FLL which is controlled by the external reference clock.\n"..
                          "The Background Debug Controller (BDC) clock is supplied from the FLL.",
            provides = { "MCG_FLL", "MCG_EXT_REF", "MCG_EXT_REF_CLK_MONITOR" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_FBI",
            brief = "FLL Bypassed Internal (FBI)",
            description = "In FLL bypassed internal mode, the bus clock is derived from the internal reference clock.\n"..
                          "The FLL is enabled and controlled by the internal reference clock, but is bypassed.\n"..
                          "The Background Debug Controller (BDC) clock is supplied from the FLL.",
            provides = { "MCG_FLL", "MCG_INT_REF" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_FBE",
            brief = "FLL Bypassed External (FBE)",
            description = "In FLL bypassed external mode, the bus clock is derived from the external reference clock.\n"..
                          "The FLL is enabled and controlled by the external reference clock, but is bypassed.\n"..
                          "The Background Debug Controller (BDC) clock is supplied from the FLL.",
            provides = { "MCG_FLL", "MCG_EXT_REF", "MCG_EXT_REF_CLK_MONITOR" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_PEE",
            brief = "PLL Engaged External (PEE)",
            description = "In PLL engaged mode, the bus clock is derived from the PLL which is controlled by the external reference clock.\n"..
                          "The Background Debug Controller (BDC) clock is supplied from the FLL.",
            provides = { "MCG_PLL", "MCG_EXT_REF", "MCG_EXT_REF_CLK_MONITOR" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_PBE",
            brief = "PLL Bypassed External (PBE)",
            description = "In PLL bypassed mode, the bus clock is derived from external reference clock.\n"..
                          "The PLL is enabled and controlled by the external reference clock, but is bypassed.\n"..
                          "The Background Debug Controller (BDC) clock is supplied from the FLL.",
            provides = { "MCG_PLL", "MCG_EXT_REF", "MCG_EXT_REF_CLK_MONITOR" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_BLPI",
            brief = "Bypassed Low Power Internal (BLPI)",
            description = "In bypassed low power internal mode, the bus clock is derived from the internal reference clock\n"..
                          "The FLL and PLL are disabled and bypassed.\n"..
                          "The Background Debug Controller (BDC) clock is not available.",
            provides = { "MCG_INT_REF" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
        {
            macro = "MCG_MODE_BLPE",
            brief = "Bypassed Low Power External (BLPE)",
            description = "In bypassed low power external mode, the bus clock is derived from the external reference clock.\n"..
                          "The FLL and PLL are disabled and bypassed.\n"..
                          "The Background Debug Controller (BDC) clock is not available.",
            provides = { "MCG_EXT_REF", "MCG_EXT_REF_CLK_MONITOR" },
            flavor = "boolean",
            exclusivity = { "MCG_MODE_FEI", "MCG_MODE_FEE", "MCG_MODE_FBI", "MCG_MODE_FBE", "MCG_MODE_PEE", "MCG_MODE_PBE", "MCG_MODE_BLPI", "MCG_MODE_BLPE" },
            file = "include/cfg/clock.h"
        },
    },
         
    --
    -- Internal lock
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg_clock_int_ref",
        brief = "Internal Reference Clock",
        description = "Internal Reference Clock: 32kHz",
        options = 
        {
            {
                macro = "MCG_INT_REF_CLKEN",
                brief = "Enable For Peripherals",
                description = "Enables the internal reference clock for use as MCGIRCLK.\n\n"..
                              "MCGIRCLK may be used by several peripherals as a clock source instead of BUSCLK.",
                flavor = "boolean",
                file = "include/cfg/clock.h"
            },
        },
    },
         
    --
    -- External Reference Clock
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg_clock_ext_ref",
        brief = "External Reference Clock",
        description = "Internal Reference Clock: 32kHz - 40MHz",
        options =
        {   
            {
                macro = "MCG_EXT_REF_HIGH_GAIN_OSC",
                brief = "High Gain Oscilator",
                description = "Use this mode for Crystal Oscilator\n"..
                              "This option provides a higher amplitude output for the improved noise immunity\n"..
                              "Supports following frequency ranges:\n"..
                              "    - 32 kHz to 100 kHz\n"..
                              "    - 1 MHz to 25 MHz",
                flavor = "boolean",
                provides = { "MCG_EXT_REF_HIGH_GAIN_OSC" },
                exclusivity = {"MCG_EXT_REF_HIGH_GAIN_OSC", "MCG_EXT_REF_LOW_PWR_OSC", "MCG_EXT_REF_EXT_CLOCK" },                
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_EXT_REF_LOW_PWR_OSC",
                brief = "Low Power Oscilator",
                description = "Use this mode for Crystal Oscilator\n"..
                              "This option provides the lower power consumption.\n"..
                              "Supports following frequency ranges:\n"..
                              "    - 32 kHz to 100 kHz\n"..
                              "    - 1 MHz to 8 MHz",
                flavor = "boolean",
                provides = { "MCG_EXT_REF_LOW_PWR_OSC" },
                exclusivity = {"MCG_EXT_REF_HIGH_GAIN_OSC", "MCG_EXT_REF_LOW_PWR_OSC", "MCG_EXT_REF_EXT_CLOCK" },                
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_EXT_REF_EXT_CLOCK",
                brief = "External Clock",
                description = "Use this mode for External Clock Source",
                flavor = "boolean",
                provides = { "MCG_EXT_REF_EXT_CLOCK" },
                exclusivity = {"MCG_EXT_REF_HIGH_GAIN_OSC", "MCG_EXT_REF_LOW_PWR_OSC", "MCG_EXT_REF_EXT_CLOCK" },                
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_EXT_REF_HIGH_FREQ_RANGE",
                brief = "High Frequency Range",
                description = 
                    function()
                        if c_is_provided("MCG_EXT_REF_EXT_CLOCK") then
                            return  "Select for the high frequency external clock source (1 MHz to 40 MHz).\n"..
                                    "Deselect for the low frequency external clock source (32 kHz to 1 MHz)."
                        end

                        return  "Select for the high frequency crystal oscillator (1 MHz to 25 MHz).\n"..
                                "Deselect for the low frequency crystal oscillator (32 kHz to 100 kHz).\n\n"..
                                "If you need different frequency, use external clock source instead of crystal.";
                    end,
                flavor = "boolean",
                provides = { "MCG_EXT_REF_HIGH_FREQ_RANGE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_EXT_REF_FREQ",
                brief = "Reference Clock Frequency",
                description = 
                    function()
                        local range
                        local source
                        if c_is_provided("MCG_EXT_REF_EXT_CLOCK") then
                            if c_is_provided("MCG_EXT_REF_HIGH_FREQ_RANGE") then
                                range = "1 MHz to 40 MHz"
                            else
                                range = "32 kHz to 1 MHz"
                            end
                            
                            source = "external clock"
                        else
                            if c_is_provided("MCG_EXT_REF_HIGH_FREQ_RANGE") then
                                if c_is_provided("MCG_EXT_REF_HIGH_GAIN_OSC") then
                                    range = "1 MHz to 25 MHz"
                                else
                                    range = "1 MHz to 8 MHz"
                                end
                            else
                                range = "32 kHz to 100 kHz"
                            end
                            
                            source = "crystal"
                        end
                        
                        return  "Please enter frequency of used "..source..".\n"..
                                "It MUST be in range "..range..".\n\n"..
                                "If you need another frequency, change the 'High Frequency Range' option or use different clock source."
                    end,
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_EXT_REF_CLKEN",
                brief = "Enable for peripherals",
                provides = { "MCG_EXT_REF_CLK_MONITOR" },
                description = "Enables the external reference clock for use as MCGERCLK.\n\n"..
                              "MCGERCLK may be used by several peripherals as a clock source instead of BUSCLK.",
                flavor = "boolean",
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_EXT_REF_CLK_MONITOR",
                brief = "Clock Monitor",
                requires = { "MCG_EXT_REF_CLK_MONITOR" },
                description = "The clock monitor issues reset after a loss of external clock is detected.",
                flavor = "boolean",
                file = "include/cfg/clock.h"
            },
        },
    },

    --
    -- PLL Setup
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg_clock_pll",
        brief = "PLL Setup",
        description = "PLL is used in PEE and PBE clock modes.\n"..
                      "Only external clock may be used as a refernce clock for PLL.\n",
        requires = { "MCG_PLL" },
        options =
        {
            {
                macro = "MCG_EXT_REF_DIVIDE_FACTOR",
                brief = "Ref. Clock Divider",
                requires = { "MCG_EXT_REF" },
                type = "enumerated",
                description = "Selects the amount to divide down the external reference clock.\n"..
                              "The resulting frequency MUST be in the range 1 MHz to 2 MHz.",
                choices = { "1", "2", "4", "8", "16", "32", "64", "128" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_PLL_MULTIPLY_FACTOR",
                brief = "Multiply Factor",
                type = "enumerated",
                description = "Selects the amount to multiply up the PLL reference clock.\n"..
                                    "  4 (4-8MHz)\n"..
                                    "  8 (8-16MHz)\n"..
                                    "  12 (12-24MHz)\n"..
                                    "  16 (16-32MHz)\n"..
                                    "  20 (20-40MHz)\n"..
                                    "  24 (24-48MHz)\n"..
                                    "  28 (28-56MHz) !!\n"..
                                    "  32 (32-64MHz) !!\n"..
                                    "  36 (36-72MHz) !!\n"..
                                    "  40 (40-80MHz) !!\n"..
                                    "  44 (44-88MHz) !!\n"..
                                    "  48 (48-96MHz) !!\n\n"..
                                    "!! WARNING, the resulting bus clock frequency should not exceed the maximum specified bus clock frequency of the device.",
                choices = { "4", "8", "12", "16", "20", "24", "28", "32", "36", "40", "44", "48" },
                file = "include/cfg/clock.h"
            },
        },
    },
    
    --
    -- FLL Setup
    --
    {
        name = "nutarch_m68k_coldfire_mcf51cn_mcg_clock_fll",
        brief = "FLL Setup",
        description = "FLL is used in FEI. FEE, FBI and FBE clock modes."..
                      "Either Internal or external clock may be used as a refernce clock for FLL.\n",
        requires = { "MCG_FLL" }, 
        options =
        {
            {
                macro = "MCG_EXT_REF_DIVIDE_FACTOR",
                brief = "Ref. Clock Divider",
                requires = { "MCG_EXT_REF" },
                type = "enumerated",
                description = "It selects the amount to divide down the external reference clock.\n"..
                              "The resulting frequency MUST be in the range 31.25 kHz to 39.0625 kHz.\n\n"..
                              "When internal reference clock is selected, this option is ignored (Divider = 1).",
                choices = { "1", "2", "4", "8", "16", "32", "64", "128", "256", "512", "1024" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_FLL_DCO_MODE",
                brief = "DCO Fined Tuned 32kHz",
                provides = { "MCG_FLL_DCO_MODE" },
                flavor = "boolean",
                description = 
                    function()
                        local description = "DCO is fined tuned for maximum frequency with 32.768 kHz reference.\n"..
                                            "In this mode, diferent 'Multiply Factors' are offered below."
                        if c_is_provided("MCG_EXT_REF") then                    
                            description = description.."\n\nThis mode MUST NOT be used, if (External Reference Clock Frequency / External Clock Divide factor) is not equal to 32kHz."
                        end
                        return description
                    end,
                file = "include/cfg/clock.h"
            },
            {
                macro = "MCG_FLL_MULTIPLY_FACTOR",
                brief = "Multiply Factor",
                type = "enumerated",
                description = 
                    function()
                        if c_is_provided("MCG_INT_REF") then
                            local description = "Selects the amount to multiply up the FLL reference clock.\n\n"
                            
                            if c_is_provided("MCG_FLL_DCO_MODE") then
                                return description..
                                    "  608 (19,92MHz)\n"..
                                    "  1216 (39,85MHz)\n"..
                                    "  1536 (50,33MHz)\n"..
                                    "  1824 (59,77MHz) !!\n\n"..
                                    "!! WARNING, the resulting bus clock frequency should not exceed the maximum specified bus clock frequency of the device."
                            else
                                return description..
                                    "  512 (16MHz)\n"..
                                    "  1024 (32MHz)\n"..
                                    "  1536 (50,33MHz)\n"
                            end
                            
                        end
                        if c_is_provided("MCG_EXT_REF") then
                            local description = "Selects the amount to multiply up the FLL reference clock.\n\n"
                            
                            if c_is_provided("MCG_FLL_DCO_MODE") then
                                return description..
                                          "  608 (19,92MHz)\n"..
                                          "  1216 (39,85MHz)\n"..
                                          "  1536 (50,33MHz)\n"..
                                          "  1824 (59,77MHz) !!\n\n"..
                                          "!! WARNING, the resulting bus clock frequency should not exceed the maximum specified bus clock frequency of the device."
                            else
                                return description..
                                          "  512 (16-20 MHz)\n"..
                                          "  1024 (32-40MHz)\n"..
                                          "  1536 (48-60MHz) !!\n\n"..
                                          "!! WARNING, the resulting bus clock frequency should not exceed the maximum specified bus clock frequency of the device."
                            end                            
                        end
                    end,
                choices = 
                    function()
                        if c_is_provided("MCG_FLL_DCO_MODE") then
                            return {"608", "1216", "1536", "1824" }
                        else
                            return {"512", "1024", "1536" }
                        end
                    end,
                file = "include/cfg/clock.h"
            },
        },        
    },
}
