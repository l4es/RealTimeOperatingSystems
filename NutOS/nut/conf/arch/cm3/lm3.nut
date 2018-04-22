
nutarch_cm3_lm3 =
{
    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_lm3_family",
        brief = "MCU Family",
        options =
        {
            {
                macro = "MCU_LM3",
                brief = "Luminary LM3",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_LM3" },
                file = "include/cfg/arch.h"
            }
        }
    }
}

