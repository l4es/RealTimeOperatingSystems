nutarch_cm3_sam3 =
{
    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_sam3_family",
        brief = "MCU Family",
        options =
        {
            {
                macro = "MCU_SAM3",
                brief = "ATMEL SAM3U Series",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_SAM3" },
                file = "include/cfg/arch.h"
            }
        }
    },
    {
        name = "nutarch_cm3_sam3_irq",
        brief = "Peripheral Interrupt Handler (SAM3X)",
        requires = { "HW_MCU_AT91SAM3X" },
        provides = { "DEV_IRQ_AT91" },
        sources =
        {
            "cm3/dev/atmel/ih_sam3udbgu.c",
            "cm3/dev/atmel/ih_sam3upioa.c",
            "cm3/dev/atmel/ih_sam3upiob.c",
            "cm3/dev/atmel/ih_sam3upioc.c",
            "cm3/dev/atmel/ih_sam3uudphs.c",
        }
    }
}
