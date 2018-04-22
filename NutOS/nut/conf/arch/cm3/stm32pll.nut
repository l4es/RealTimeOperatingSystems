    --
    -- STM32 PLL Configuration
    --
--
-- Retrieve maximun system clock description
--
function GetSysClockFrequencyDesc()
    if c_is_provided("HW_MCU_STM32L0") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t2 .. 24\n"..
               "PLL output(range)\t2 .. 32\n"..
               "SYSCLK_FREQ\t\t0 .. 32"
    end
    if c_is_provided("HW_MCU_STM32L1") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t2 .. 24\n"..
               "PLL output(range)\t2 .. 32\n"..
               "SYSCLK_FREQ (max)\t0 .. 32"
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "Value (MHz)\t\tRange1\tRange2\n"..
               "PLL input (range)\t4 .. 16\t4 .. 16\n"..
               "PLL VCO output(range)\t64 .. 344\t64 .. 128\n"..
               "PLLR output(range)\t8 .. 80\t8 .. 26\n"..
               "SYSCLK_FREQ\t\t0 .. 80\t0 .. 26"
    end
    if c_is_provided("HW_MCU_STM32F0") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "SYSCLK_FREQ (max)\t16 .. 48"
    end
    if c_is_provided("HW_RCC_STM32F1_CL") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input (1 or 2).\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               "And with PLL2 as PLL1 input selected:\n"..
               " - PLL2CLK_PREDIV\t: HSE Division factor at Pll2 input.\n"..
               " - PLL2CLK_MULT\t: Pll2 multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL2 input (range)\t3 .. 5\n"..
               "PLL2 output (range)\t40 .. 74\n"..
               "PLL input (range)\t3 .. 12\n"..
               "SYSCLK_FREQ (max)\t18 .. 72"
    end
    if c_is_provided("HW_MCU_STM32F100") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input (1 or 2).\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t1 .. 24\n"..
               "SYSCLK_FREQ (max)\t16 .. 24"
    end
    if c_is_provided("HW_MCU_STM32F101") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input (1 or 2).\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t1 .. 25\n"..
               "SYSCLK_FREQ (max)\t16 .. 36"
    end
    if c_is_provided("HW_MCU_STM32F102") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input (1 or 2).\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "SYSCLK_FREQ (max)\t16 .. 48\n"..
               "PLL input (range)\t1 .. 25"
    end
    if c_is_provided("HW_MCU_STM32F103") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input (1 or 2).\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t1 .. 25\n"..
               "SYSCLK_FREQ (max)\t16 .. 72"
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               " - PLLCLK_DIV\t: Pll VCO output division factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t0.95 .. 2.10\n"..
               "PLL VCO output(range)\t192 .. 432\n"..
               "SYSCLK_FREQ (max)\t24 .. 120"
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return "System clock target frequency with SYSCLK_PLL selected.\n\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "highest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: HSE Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n\n"..
               "Value (MHz)\t\tRange1\n"..
               "PLL input (range)\t1 .. 24\n"..
               "SYSCLK_FREQ (max)\t16 .. 72"
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "PLL input (MHz)\t0.95 .. 2.10\n"..
               "PLL VCO output (MHz)\t192 .. 432\n"..
               "PLL output (MHz)\t24 .. 84\n\n"..
               "Voltage scaling\t2\t3\n"..
               "SYSCLK_FREQ (MHz)\t0 .. 84\t0 .. 60"
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "PLL input (MHz)\t0.95 .. 2.10\n"..
               "PLL VCO output (MHz)\t100 .. 432\n"..
               "PLL output (MHz)\t24 .. 100\n\n"..
               "Voltage scaling\t1\t2\t3\n"..
               "SYSCLK_FREQ (MHz)\t0 .. 100\t0 .. 84\t0 .. 64"
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "PLL input (MHz)\t0.95 .. 2.10\n"..
               "PLL VCO output (MHz)\t192 .. 432\n"..
               "PLL output (MHz)\t24 .. 168\n\n"..
               "Voltage scaling\t1\t0\n"..
               "SYSCLK_FREQ (MHz)\t0 .. 168\t0 .. 144"
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "PLL input (MHz)\t0.95 .. 2.10\n"..
               "PLL VCO output (MHz)\t192 .. 432\n"..
               "PLL output (MHz)\t24 .. 180\n\n"..
               "Voltage scaling/Overdrive\t1, on\t1, off\t2, on\t2, off\t3\n"..
               "SYSCLK_FREQ (MHz)\t0 .. 180\t0 .. 168\t0 .. 168\t0 .. 144\t0 .. 120"
    end
    if c_is_provided("HW_MCU_STM32F44") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               " - PLLCLK_DIV_R\t\t: PLL VCO output R division factor.\n\n"..
               "PLL input (MHz)\t0.95 .. 2.10\n"..
               "PLL VCO output (MHz)\t100 .. 432\n"..
               "PLL output (MHz)\t24 .. 180\n\n"..
               "Voltage scaling/Overdrive\t1, on\t1, off\t2, on\t2, off\t3\n"..
               "SYSCLK_FREQ (MHz)\t0 .. 180\t0 .. 168\t0 .. 168\t0 .. 144\t0 .. 120"
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return "System clock target frequency with SYSCLK_PLL selected.\n"..
               "With SYSCLK_FREQ undefined, setup will try to reach the\n"..
               "\thighest possible value.\n\n"..
               "If in doubt, leave undefined!\n\n"..
               "With SYSCLK_FREQ given, User must set usefull values for:\n"..
               " - PLLCLK_PREDIV\t: Division factor at Pll input.\n"..
               " - PLLCLK_MULT\t: Pll multiplier factor.\n"..
               " - PLLCLK_DIV\t\t: PLL VCO output division factor.\n\n"..
               "PLL input (MHz)\t0.95 .. 2.10\n"..
               "PLL VCO output (MHz)\t100 .. 432\n"..
               "PLL output (MHz)\t24 .. 180\n\n"..
               "Voltage scaling/Overdrive\t1, on\t1, off\t2, on\t2, off\t3\n"..
               "SYSCLK_FREQ (MHz)\t0 .. 216\t0 .. 180\t0 .. 180\t0 .. 168\t0 .. 144"
    end
    return "Unhandled"
end

--
-- Retrieve SYSCLK available on the device.
--
function GetSysClkSrc()
    if c_is_provided("HW_RCC_STM32L") then
        return {"SYSCLK_MSI", "SYSCLK_HSI", "SYSCLK_HSE", "SYSCLK_PLL"}
    end
    if c_is_provided("HW_MCU_STM32F44") then
        return {"SYSCLK_HSI", "SYSCLK_HSE", "SYSCLK_PLL", "SYSCLK_PLLR"}
    end
    return {"SYSCLK_HSI", "SYSCLK_HSE", "SYSCLK_PLL"}
end

--
-- Retrieve SYSCLK description
--
function GetSysClockSourceDesc()
    if c_is_provided("HW_HSI8_STM32F") then
-- F0/F1/F3
       return "Select where SYSCLK should get its clock from.\n\n"..
              "SYSCLK_HSI is internal 8 MHz oscillator.\n"..
              "SYSCLK_HSE is external oscillator or crystal input.\n"..
              "SYSCLK_PLL is internal PLL output.\n"..
              "\tSelect PLL source with PLLCLK_SOURCE.\n\n"..
              "Default is SYSCLK_PLL."
    end
    if c_is_provided("HW_MCU_STM32F44") then
-- F44x: Choice from two outputs of the PLL
       return "Select where SYSCLK should get its clock from.\n\n"..
              "SYSCLK_HSI is internal 16 MHz oscillator.\n"..
              "SYSCLK_HSE is external oscillator or crystal input.\n"..
              "SYSCLK_PLL is internal PLL output.\n"..
              "SYSCLK_PLLR is internal PLL R output.\n"..
              "\tSelect PLL source with PLLCLK_SOURCE.\n"..
              "\tPLLR is also used for I2S, SA1 and SPDIF.\n"..
              "\tPLL is dedicated to core.\n\n"..
              "Default is SYSCLK_PLL."
    end
    if c_is_provided("HW_HSI16_STM32F") then
-- F2/F4/F7
       return "Select where SYSCLK should get its clock from.\n\n"..
              "SYSCLK_HSI is internal 16 MHz oscillator.\n"..
              "SYSCLK_HSE is external oscillator or crystal input.\n"..
              "SYSCLK_PLL is internal PLL output.\n"..
              "\tSelect PLL source with PLLCLK_SOURCE.\n\n"..
              "Default is SYSCLK_PLL."
    end
    if c_is_provided("HW_MCU_STM32L0") then
       return "Select where SYSCLK should get its clock from.\n\n"..
              "SYSCLK_HSI is internal 16 MHz oscillator, "..
                          "eventually divided by 4.\n"..
              "\tSelect with HSICLK_DIV4.\n"..
              "SYSCLK_HSE is external oscillator or crystal input.\n"..
              "SYSCLK_PLL is internal PLL output.\n"..
              "\tSelect PLL source with PLLCLK_SOURCE.\n"..
              "PLLCLK_MSI is internal multi-speed oscillator.\n"..
              "\tSelect MSI Frequency with MSI_RANGE.\n\n"..
              "Default is SYSCLK_PLL."
    end
    if c_is_provided("HW_MCU_STM32L1") then
       return "Select where SYSCLK should get its clock from.\n\n"..
              "SYSCLK_HSI is internal 16 MHz oscillator.\n"..
              "SYSCLK_HSE is external oscillator or crystal input.\n"..
              "SYSCLK_PLL is internal PLL output.\n"..
              "\tSelect PLL source with PLLCLK_SOURCE.\n"..
              "PLLCLK_MSI is internal multi-speed oscillator.\n"..
              "\tSelect MSI Frequency with MSI_RANGE.\n\n"..
              "Default is SYSCLK_PLL."
    end
    if c_is_provided("HW_MCU_STM32L4") then
       return "Select where SYSCLK should get its clock from.\n\n"..
              "SYSCLK_HSI is internal 16 MHz oscillator.\n"..
              "SYSCLK_HSE is external oscillator or crystal input.\n"..
              "SYSCLK_PLL is internal PLL output.\n"..
              "\tSelect PLL source with PLLCLK_SOURCE.\n"..
              "PLLCLK_MSI is internal multi-speed oscillator.\n"..
              "\tSelect MSI Frequency with MSI_RANGE.\n\n"..
              "Default is SYSCLK_PLL."
    end
    return "Unhandled case"
end

--
-- HSE Value Description
--
-- With value given, SYSCLK_SOURCE will be SYSCLK_PLL with PLLCLK_HSE
--
function GetHseValueDesc()
    if c_is_provided("HW_MCU_STM32L4") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN (Range 1)\t: 0 .. 46 Mhz\n"..
               "External input at OSCIN (Range 2)\t: 0 .. 26 Mhz\n"..
               "External resonator\t\t: 4 .. 48 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_RCC_STM32L") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 32 Mhz\n"..
               "With CSS off and PLL off\t: 0 .. 32 MHz\n"..
               "External resonator\t: 4 .. 25 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F0") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 32 Mhz\n"..
               "External resonator\t: 4 .. 32 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F100") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 24 Mhz\n"..
               "External resonator\t: 4 .. 24 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F101") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 25 Mhz\n"..
               "External resonator\t: 4 .. 16 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F102") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 25 Mhz\n"..
               "External resonator\t: 4 .. 16 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F1_CL") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 50 Mhz\n"..
               "External resonator\t: 3 .. 25 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Builtin USB bootloader starts much faster with 25 MHz.\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 26 Mhz\n"..
               "External resonator\t: 4 .. 26 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 32 Mhz\n"..
               "External resonator\t: 4 .. 32 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F4") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 50 Mhz\n"..
               "External resonator\t: 4 .. 26 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return "Value of the external crystal or clock input in Hertz.\n"..
               "Allowed values:\n"..
               "External input at OSCIN\t: 1 .. 50 Mhz\n"..
               "External resonator\t: 4 .. 26 MHz\n"..
               "Typical Values is 8 MHz.\n\n"..
               "Default is undefined. This will turn off HSE.\n"
    end
    return Unhandled
end

--
-- Retrieve PLL Input CLK description for the device.
--
function GetPllClockSourceDesc()
    if c_is_provided("HW_MCU_STM32L0") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 16 MHz oscillator, "..
                        "eventually divided by 4.\n"..
            "Input clock must be in the range 2 .. 24 MHz.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
    if c_is_provided("HW_MCU_STM32L1") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 16 MHz oscillator.\n"..
            "Input clock must be in range 2 .. 24 MHz.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_NONE is PLL off.\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 16 MHz oscillator.\n"..
            "PLLCLK_MSI is internal multispeed oscillator. "..
                        "See MSI_ related values.\n"..
            "Pll input clock range is 4 .. 16 MHz.\n"..
            "Clock division by 1 .. 8 is available.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, with\n"..
            "LSE_VALUE provided PLLCLK_MSI, PLLCLK_HSI else."
   end
-- F03/F05/F100/F37
    if c_is_provided("HW_HSI8_DIV2_STM32") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 8 MHz oscillator divided by 2.\n"..
            "Input clock must be in range 1 .. 24 MHz.\n"..
            "HSE Clock division by 1 .. 16 is available.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
-- F04/F07/F09/F3_HD
    if c_is_provided("HW_HSI8_STM32") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 8 MHz oscillator.\n"..
            "Input clock must be in range 1 .. 24 MHz.\n"..
            "Clock division by 1 .. 16 is available.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
-- F105/F107
    if c_is_provided("HW_MCU_STM32F1_CL") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 8 MHz oscillator divided by 2.\n"..
            "Input clock must be in range 1 .. 12 MHz.\n"..
            "HSE Clock division by 1 .. 16 is available.\n"..
            "HSE may also be provided by PLL2.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
    if c_is_provided("HW_MCU_STM32F1") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 8 MHz oscillator divided by 2.\n"..
            "Input clock must be in range 1 .. 25 MHz.\n"..
            "HSE Clock division by 2 is available.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return "Select source for PLL input.\n\n"..
            "PLLCLK_HSE is external resonator or input. "..
                        "See HSE_ related values.\n"..
            "PLLCLK_HSI is internal 8 MHz oscillator divided by 2.\n"..
            "Input clock must be in range 1 .. 24 MHz.\n"..
            "HSE Clock division by 2 .. 16 is available.\n"..
            "PLL setup is done automatic for some default value.\n"..
            "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
            "Default with HSE_VALUE provided is PLLCLK_HSE, PLLCLK_HSI else."
    end
-- F2/F4/F7
    return "Select source for PLL input.\n\n"..
        "PLLCLK_HSE is external resonator or input. "..
                    "See HSE_ related values.\n"..
        "PLLCLK_HSI is internal 16 MHz oscillator.\n"..
        "Input clock must be in range 0.95 .. 2.1 MHz.\n"..
        "HSE Clock division by 2 .. 63 is available.\n"..
        "PLL setup is done automatic for some default value.\n"..
        "Use SYSCLK_FREQ to disable automatic setup only when needed!\n\n"..
        "Default is PLLCLK_HSI."
end

--
-- Retrieve PLL Input CLK available on the device.
--
function GetPllClkSrc()
    if c_is_provided("HW_MCU_STM32L4") then
        return {"PLLCLK_NONE", "PLLCLK_HSI", "PLLCLK_HSE", "PLLCLK_MSI"}
    end
    return {"PLLCLK_HSI", "PLLCLK_HSE"}
end

--
-- Retrieve PLL Input CLK default.
--
function GetPllClkSrcDefault()
    if c_is_provided("HW_MCU_STM32L4") then
        if c_is_provided("LSE_VALUE") then
            return "PLLCLK_MSI"
        end
    end
    if c_is_provided("HSE_VALUE") then
        return "PLLCLK_HSE"
    end
    return "PLLCLK_HSI"
end

function GetPllMult()
    if c_is_provided("HW_MCU_STM32F1_CL") then
       return { "4", "5", "6", "7", "8", "9", "6.5"}
    end
    return { "1", "2", "3", "4", "5", "6", "7", "8", "9",
             "10", "11", "12", "13", "14", "15", "16"}
end

function GetPllPrediv()
    if c_is_provided("HW_MCU_STM32F1") then
       if c_is_provided("HW_MCU_STM32F1_CL") then
          return { "1", "2", "3", "4", "5", "6", "7", "8", "9",
                   "10", "11", "12", "13", "14", "15", "16"}
       end
       if c_is_provided("HW_MCU_STM32F100") then
          return { "1", "2", "3", "4", "5", "6", "7", "8", "9",
                   "10", "11", "12", "13", "14", "15", "16"}
       end
       return { "1", "2"}
    end
    return { "1", "2", "3", "4", "5", "6", "7", "8", "9",
             "10", "11", "12", "13", "14", "15", "16"}
end

function GetLseDriveLevel()
    if c_is_provided("HW_MCU_STM32F411") then
        return { "0" , "1"}
    end
    if c_is_provided("HW_MCU_STM32F44") then
        return { "0" , "1"}
    end
    if c_is_provided("HW_MCU_STM32F46") then
        return { "0" , "1"}
    end
    if c_is_provided("HW_MCU_STM32L1") then
        return "0"
    end
    if c_is_provided("HW_MCU_STM32F1") then
        return "0"
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return "0"
    end
    if c_is_provided("HW_MCU_STM32F4") then
        return "0"
    end
    return { "0", "1", "2", "3"}
end

function GetRtcClkSrcDefault()
    if c_is_provided("LSE_VALUE") then
       return "RTCCLK_LSE"
    end
    if c_is_provided("HSE_VALUE") then
       return "RTCCLK_HSE"
    end
    return "RTCCLK_LSI"
end

function GetMsiRangeDesc()
    if c_is_provided("HW_MCU_STM32L4") then
        return "Select MSI frequency.\n"..
               "Possible values are 100/200/400/800 kHz "..
               "and 1/2/4/8/16/24/32/48 MHz.\n"..
               "Default value after reset is 4 MHz.\n",
               "In Range 2, maximum allowed frequency is 24 MHz.\n"..
               "Use MSI_OFF to disable MSI."
    end
    return "Select MSI frequency.\n"..
           "Possible values are 65/131/262/524 kHz/1.05/2.1/4.2/MHz.\n"..
           "Default value after reset is 4 MHz.\n",
          "Use MSI_OFF to disable MSI."
end

function GetMsiRange()
    if c_is_provided("HW_MCU_STM32L4") then
        return {"MSI_OFF", "MSI_100k", "MSI_200k", "MSI_400k",
               "MSI_800k", "MSI_1M", "MSI_2M", "MSI_4M", "MSI_8M",
               "MSI_16M", "MSI_24M", "MSI_32M", "MSI_48M"}
    end
    return {"MSI_OFF", "MSI_65k", "MSI_131k", "MSI_262k",
           "MSI_524k", "MSI_1050k", "MSI_2100k", "MSI_4200k"}
end
function GetMsiRangeDefault()
    if c_is_provided("HW_MCU_STM32L4") then
        return "MSI_4M"
    end
    return "MSI_4200k"
end

function GetPowerScaleRegisterDesc()
    if c_is_provided("HW_MCU_STM32L4") then
        return "Select Core voltage scaling:\n"..
               "Range\tV core\tfmax\n"..
               "1\t1.2 Volt\t80 MHz\n"..
               "2\t1.0 Volt\t26 MHz\n"..
               "Default register setting is 1."
    end
    if c_is_provided("HW_RCC_STM32L") then
        return "Select Core voltage scaling:\n"..
               "Range\tV core\tV supply(min\tfmax\n"..
               "1\t1.8 Volt\t1.71 Volt\t32 MHz\n"..
               "2\t1.5 Volt\t1.65 Volt\t16 MHz\n"..
               "3\t1.2 Volt\t1.65 Volt\t4.2 MHz\n"..
               "Default register is 1."
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return "Select Power Scale:\n"..
               "Power Scale\tfmax\n"..
               "2\t84 MHz\n"..
               "3\t60 MHz\n"..
               "Default register setting is 2.\n"..
               "With PLL off, Power scale is 3 independant from register value."
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return "Select Power Scale:\n"..
               "Power Scale\tfmax\n"..
               "1\t100 MHz\n"..
               "2\t84 MHz\n"..
               "3\t64 MHz\n"..
               "Default is 1.\n"..
               "With PLL off, Power scale is 3 independant from register value."
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return "Select Power Scale:\n"..
               "Power scale\tfmax\n"..
               "1\t168 MHz\n"..
               "2\t144 MHz\n"..
               "Default register setting is 1."
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return "Select Power Scale:\n"..
               "Power scale\tfmax Overdrive off/on\n"..
               "1\t168/180 MHz\n"..
               "2\t144/168 MHz\n"..
               "3\t144/144 MHz\n"..
               "Default register setting is 1."
    end
    if c_is_provided("HW_MCU_STM32F44") then
        return "Select Power Scale register value:\n"..
               "Power scale\tfmax Overdrive off/on\n"..
               "1\t168/180 MHz\n"..
               "2\t144/168 MHz\n"..
               "3\t120/120 MHz\n"..
               "Default register setting is 1."
    end
    if c_is_provided("HW_MCU_STM32F46") then
        return "Select Power Scale register value:\n"..
               "Power scale\tfmax Overdrive off/on\n"..
               "1\t168/180 MHz\n"..
               "2\t144/168 MHz\n"..
               "3\t120/120 MHz\n"..
               "Default register setting is 1."
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return "Select Power Scale:\n"..
               "Power scale\tfmax Overdrive off/on\n"..
               "1\t180/216 MHz\n"..
               "2\t168/180 MHz\n"..
               "3\t144/144 MHz\n"..
               "Default register setting is 1."
    end
    return "Unhandled"
end

function GetPowerScaleRegister()
    if c_is_provided("HW_MCU_STM32L4") then
        return {"1", "2"}
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return {"2", "3"}
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return {"1", "2"}
    end
    return {"1", "2", "3"}
end

function GetFlashPrefetchDefault()
    if c_is_provided("HW_MCU_STM32F7") then
        return "DISABLE"
    end
    return "ENABLE"
end

function GetPowerScaleRegisterDefault()
    if c_is_provided("HW_MCU_STM32F401") then
        return "2"
    end
    return "1"
end

nutarch_cm3_stm32_pll =
{
    name = "nutarch_cm3_stm32_rcc_common",
    brief = "STM32 Clock and system settings.",
    description = "STM32 Clock and system settings.",
    options =
    {
        {
            macro = "SYSCLK_SOURCE",
            brief = "System clock source",
            description =function() return GetSysClockSourceDesc() end,
            type = "enumerated",
            choices = function() return GetSysClkSrc() end,
            default = "SYSCLK_PLL",
            file = "include/cfg/clock.h"
        },
        {
            macro = "PLLCLK_SOURCE",
            brief = "PLL Clock Source",
            description = function() return GetPllClockSourceDesc() end,
            type = "enumerated",
            choices = function() return GetPllClkSrc() end,
            default = function() return GetPllClkSrcDefault() end,
            file = "include/cfg/clock.h"
        },
        {
            macro = "SYSCLK_FREQ",
            brief = "CM3 System Clock",
            description = function() return GetSysClockFrequencyDesc() end,
            provides = {"SYSCLK_FREQ"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "HSE_VALUE",
            brief = "External Oscillator Frequency",
            description = function() return GetHseValueDesc() end,
            type = "integer",
            provides = {"HSE_VALUE"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "HSE_BYPASS",
            brief = "HSE from external source",
            description = "Use the clock signal applied to OSC_IN.",
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h",
        },
        {
            macro = "HSI_DIVIDE_BY_FOUR",
            brief = "HSI divided by four",
            description = "HSI may be used divided by four.\n\n"..
                          "Default is DISABLE.",
            requires = {"HW_MCU_STM32L0"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h",
        },
        {
            macro = "LSE_DRIVE_LEVEL",
            brief = "LSE osc power",
            description = "Power level of LSE oscillator\n"..
                          "0 = Low drive\n"..
                          "1 = Medium high drive\n"..
                          "2 = Medium low drive\n"..
                          "3 = High drive.",
            type = "enumerated",
            choices = function() return GetLseDriveLevel() end,
            default = "0",
            file = "include/cfg/clock.h"
        },
        {
            macro = "LSE_BYPASS",
            brief = "LSE from external source",
            description = "Use clock signal applied to OSC32_IN.",
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "LSE_VALUE",
            brief = "Frequency of LSE Clock ",
            description = "Frequency of LSE quarz/external LSE input.\n"..
                          "Standard is 32768 Hz.\n"..
                          "Default is undefined.",
            type = "integer",
            provides = {"LSE_VALUE"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "LSI_ON",
            brief = "Turn LSI on",
            description = "Turn LSI on(1) or off(0).\n"..
                          "Default is on.",
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "RTCCLK_SOURCE",
            brief = "RTC(/LCD) clock source",
            description = "Clock used for RTC and LCD.\n\n"..
                          "RTCCLK_KEEPS keeps current RTC setting.\n"..
                          "RTCCLK_NONE turns off RTC.",
            type = "enumerated",
            choices = {"RTCCLK_KEEP", "RTCCLK_LSE", "RTCCLK_HSE",
                       "RTCCLK_LSI", "RTCCLK_NONE"},
            default = function() return GetRtcClkSrcDefault() end,
            file = "include/cfg/clock.h"
        },
        {
            macro = "AHB_DIV",
            brief = "AHB clock divisor.",
            description = "Divisor between SYSCLK and HCLK.\n"..
                          "Allowed values are 1, 2, 4, 8, 16, 32, "..
                          "64,  128, 256 and 512."..
                          "Default is 1.",
            type = "enumerated",
            choices =  {"1", "2", "4", "8", "16", "32", "64",
                        "128", "256", "512"},
            default = "1",
            file = "include/cfg/clock.h"
        },
        {
            macro = "APB1_DIV",
            brief = "APB1 clock divisor.",
            description = "Divisor between HCLK and PCLK1 "..
                          "for low speed APB bus.\n"..
                          "Allowed values are 1, 2, 4, 8, and 16.\n"..
                          "Leave undefined to reach highest allowed value.",
            type = "enumerated",
            choices =  {"", "1", "2", "4", "8", "16"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "APB2_DIV",
            brief = "APB2 clock divisor.",
            description = "Divisor between HCLK and PCLK2 "..
                          "for high speed APB bus.\n"..
                          "Allowed values are 1, 2, 4, 8, and 16.\n"..
                          "Leave undefined to reach highest allowed value.",
            type = "enumerated",
            choices =  {"", "1", "2", "4", "8", "16"},
            file = "include/cfg/clock.h"
        },
-- PLLCLK PREDIV, MULT and DIV vary wide between devices.
-- No sensible user guidance here!
        {
            macro = "PLLCLK_PREDIV",
            brief = "PLL Input Clock Divider",
            description =
                "In many cases HSE_VALUE and SYSCLK_FREQ\n"..
                "is enough for the code to calculate\n"..
                "PLLCLK_PREDIV, PLLCLK_MULT and PLLCLK_DIV.\n\n"..
                "For unhandled special settings, enter suitable and\n"..
                "allowed PLL Input Clock Divider value here"..
                "or better extend clock setting!",
            requires = {"SYSCLK_FREQ"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "PLLCLK_MULT",
            brief = "PLL Clock Multiplier",
            description =
                "In many cases HSE_VALUE and SYSCLK_FREQ\n"..
                "is enough for the code to calculate\n"..
                "PLLCLK_PREDIV, PLLCLK_MULT and PLLCLK_DIV.\n\n"..
                "For unhandled special settings, enter suitable and\n"..
                "allowed PLL Clock Multiplier value here"..
                "or better extend clock setting!\n"..
                "For STM32F1  CL devices enter the desired factor * 2 so\n"..
                "that the multiplication factor 6.5 becomes 13.\n",
            requires = {"SYSCLK_FREQ"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "PLLCLK_DIV",
            brief = "PLL Clock Divider",
            description =
                "In many cases HSE_VALUE and SYSCLK_FREQ\n"..
                "is enough for the code to calculate\n"..
                "PLLCLK_PREDIV, PLLCLK_MULT and PLLCLK_DIV.\n\n"..
                "For unhandled special set, enter suitable and\n"..
                "allowed PLL Clock Divider value here"..
                "or better extend clock setting!",
            requires = {"SYSCLK_FREQ"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "PLL2CLK_PREDIV",
            brief = "PLL2 Clock Predivider",
            description = "Divider between HSE and PLL2 input.\n",
            type = "enumerated",
            choices = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16"};
            requires = {"HW_MCU_STM32F1_CL", "SYSCLK_FREQ"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "PLL2CLK_MULT",
            brief = "PLL2 Clock Multiplier",
            description = "Multiplier for PLL2 clock.\n",
            requires = {"HW_MCU_STM32F1_CL", "SYSCLK_FREQ"},
            type = "enumerated",
            choices = { "8", "9", "10", "11", "12", "13", "14", "16", "20"},
            file = "include/cfg/clock.h"
        },
        {
            macro = "MSI_RANGE",
            brief = "MSI value",
            description = function() return GetMsiRangeDesc() end,
            type = "enumerated",
            requires = {"HW_RCC_STM32L"},
            choices = function() return GetMsiRange() end,
            default = function() return GetMsiRangeDefault() end,
            file = "include/cfg/clock.h"
        },
        {
            macro = "STM32_POWERSCALE",
            brief = "Power Scaling",
            description = function() return GetPowerScaleRegisterDesc() end,
            requires = {"HW_VOS_STM32"},
            type = "enumerated",
            choices = function() return GetPowerScaleRegister() end,
            default = function() return GetPowerScaleRegisterDefault() end,
            file = "include/cfg/clock.h"
        },
        {
            macro = "STM32_SUPPLY_MINIMUM",
            brief = "Minimum Supply voltage",
            description = "Flash access and flash programming parallelisme \n"..
                        "on F2/4/7 depends on supply voltage.\n"..
                        "2700 mV up to 3600 mV is fastest\n"..
                        "2400 mV up to 2700 mV is medium\n"..
                        "2100 mV up to 2400 mV is slow\n"..
                        "1800 mV up to 2100 mV is slowest\n\n"..
                        "Default is 2700 (mV)\n",
            type = "enumerated",
            requires = {"HW_FLASH_STM32F2_4"},
            choices =  {"1800", "2100", "2400", "2700"},
            default = "2700",
            file = "include/cfg/clock.h"
        },
        {
            macro = "STM32_EXTERNAL_VPP",
            brief = "External VPP",
            description = "External VPP applied to allow 64-bit "..
                        "programming.\nDefault: DISABLE.\n",
            requires = {"HW_FLASH_STM32F2_4"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "STM32_OVERDRIVE",
            brief = "Over-Drive mode",
            description = "Over-Drive Mode\n\n"..
                          "Allow higher sysclock a the expense of "..
                                 "higher power.\n"..
                          "Default is ENABLE",
            requires = {"HW_OVERDRIVE_STM32"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "ENABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_32BIT_ACCESS",
            brief = "Access flash only with 32-bit",
            description = "By default, instructions are fetched from flash "..
                          "in rows of 64 bit\n"..
                          "Needed for FLASH_PREFETCH.\n"..
                          "In special low system speed cases access with only"..
                          " 32-bit width may reduce power consumption.\n\n"..
                          "Default is DISABLE!",
            requires = {"HW_MCU_STM32L1"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_DISABLE_BUFFER",
            brief = "Don't use flash buffers",
            description = "Disables Flash prefetch and Data preread.\n"..
                          "Default is DISABLE to allow Prefetch.",
            type = "enumerated",
            requires = {"HW_MCU_STM32L0"},
            choices =  {"ENABLE", "DISABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_PREFETCH",
            brief = "Instruction prefetch",
            description = "Flash buswidth is wider than instruction width.\n"..
                          "Prefetch reads next flash row while old row\n"..
                          "is still executed. With linear code, effective\n"..
                          "no wait state is needed.\n"..
                          "On F7, program needs to access code at 0x00200000 and"..
                          " FLASH_ART_ACCELARATION must be enabled.\n"..
                          "Default is enabled(F7: disabled)!",
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = function() return GetFlashPrefetchDefault() end,
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_ART_ACCELERATION",
            brief = "Unified flash acceleration buffer",
            description = "Enable a unified cache of 64 lines of 256 bits.\n"..
                          "Needs program mapped to 0x00200000.\n"..
                          "Default is disabled!",
            requires = {"HW_MCU_STM32F7"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "ENABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_ICACHE",
            brief = "Instruction cache",
            description = "Enable instruction cache of 64/32 lines of 128/256 bits.\n"..
                          "Default is enabled!",
            requires = {"HW_ICACHE_STM32"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "ENABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_DCACHE",
            brief = "Data cache",
            description = "Enable data cache of 8 lines of 128/256 bits.\n"..
                          "Default is enabled!",
            requires = {"HW_DCACHE_STM32"},
            type = "enumerated",
            choices = {"DISABLE", "ENABLE"},
            default = "ENABLE",
            file = "include/cfg/clock.h"
        },
        {
            macro = "FLASH_PRE_READ",
            brief = "Pre-read data",
            description = "Use one flash buffers for data.\n\n"..
                          "If enabled, the memory interface stores the \n"..
                          "last address read as data and tries to read\n"..
                          " the next one when no other read or write or\n"..
                          "prefetch operation is ongoing.\n\n"..
                          "Default is DISABLE.",
            type = "enumerated",
            requires = {"HW_MCU_STM32L0"},
            choices =  {"ENABLE", "DISABLE"},
            default = "DISABLE",
            file = "include/cfg/clock.h"
        },
    },
}
