nutarch_m68k_coldfire =
{
    --
    -- MCU Family
    --
    {
        name = "nutarch_m68k_coldfire_family",
        brief = "Family",
        options =
        {
            {
                macro = "MCU_COLDFIRE",
                brief = "COLDFIRE",
                type = "integer",
                default = 1,
                file = "include/cfg/arch.h"
            }
        }
    },
    
    --
    -- Interrupt Handler
    --
    {
        name = "nutarch_m68k_coldfire_ihndlr",
        brief = "Interrupt Handler",
        description = "CPU Interrupt handlers common for all Coldfire families.\n\n"..
                      "In addition, this package contains the default interrupt handler, which may be used by application to handle spurious interrupts.",
        sources = { "m68k/coldfire/dev/common/ih_mcf5_default.c" }
    },

    --
    -- Context Switching
    --
    {
        name = "nutarch_m68k_coldfire_context",
        brief = "Context Switching",
        description = "Context Switching (GCC)",
        provides = { "NUT_CONTEXT_SWITCH" },
        requires = { "TOOL_GCC" },
        sources = { "m68k/coldfire/os/context.c" },
        options =
        {
            {
                macro = "NUTMEM_STACKHEAP",
                brief = "Separate heap for stack",
                description = "This option enables use of a separate heap for stack.\n\n"..
                              "When a thread is created with this option enabled, "..
                              "it's stack is allocated on a special 'thread stack heap' "..
                              "which is kept in faster internal memory \ninstead of using the 'standard heap' ".. 
                              "which is typically located in slower external memory.",
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
        }
    },
    
    --
    -- Coldfire MCU Directory
    --
	{
        name = "nutarch_m68k_coldfire_mcf5225x",
        brief = "MCF5225X Family",
        requires = { "HW_MCU_MCF5225X" },
        description = "MCF5225X family",
        script = "arch/coldfire/mcf5225x.nut"
    },
	{
        name = "nutarch_m68k_coldfire_mcf51cn",
        brief = "MCF51CN Family",
        requires = { "HW_MCU_MCF51CN" },
        description = "MCF51CN family",
        script = "arch/coldfire/mcf51cn.nut"
    },
    
    --
    -- Coldfire Common Devices
    --
    {
        name = "nutarch_m68k_coldfire_devices",
        brief = "Common devices",
        description = "Common devices to the Cipfire families",
        script = "arch/coldfire/coldfire_dev.nut",
    },
}
