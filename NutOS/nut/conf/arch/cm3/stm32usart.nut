--
-- Copyright (C) 2015 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de.
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
-- Retrieve U(S)ARTs available on the device.
--
function GetStmUsarts()
-- STM32F03 members has different number of USARTs
    if c_is_provided("HW_MCU_STM32F030x6") then
        return { "", "USART1"}
    end
    if c_is_provided("HW_MCU_STM32F031x6") then
        return { "", "USART1"}
    end
    if c_is_provided("HW_MCU_STM32F038xx") then
        return { "", "USART1"}
    end
    if c_is_provided("HW_MCU_STM32F030xC") then
        return { "", "USART1", "USART2", "USART3", "USART4", "UART5", "UART6" }
    end
-- For other STM32F0 families, the number of USARTs is constant per familie
    if c_is_provided("HW_MCU_STM32F07") then
        return { "", "USART1", "USART2", "USART3", "USART4" }
    end
   if c_is_provided("HW_MCU_STM32F09") then
        return { "", "USART1", "USART2", "USART3", "USART4", "UART5", "UART6", "UART7", "UART8" }
    end
-- All other STM32F0 are considered to have 2 USARTs
    if c_is_provided("HW_MCU_STM32F0") then
        return { "", "USART1", "USART2"}
    end
    if c_is_provided("HW_MCU_STM32F1_LD_VL") then
        return { "", "USART1", "USART2" }
    end
    if c_is_provided("HW_MCU_STM32F1_MD") then
        return { "", "USART1", "USART2", "USART3" }
    end
    if c_is_provided("HW_MCU_STM32F1_MD_VL") then
        return { "", "USART1", "USART2", "USART3" }
    end
    if c_is_provided("HW_MCU_STM32F1_HD") then
        return { "", "USART1", "USART2", "USART3", "USART4", "USART5" }
    end
    if c_is_provided("HW_MCU_STM32F1_HD_VL") then
        return { "", "USART1", "USART2", "USART3", "USART4", "USART5" }
    end
    if c_is_provided("HW_MCU_STM32F1_XL") then
        return { "", "USART1", "USART2", "USART3", "USART4", "USART5" }
    end
    if c_is_provided("HW_MCU_STM32F1_CL") then
        return { "", "USART1", "USART2", "USART3", "USART4", "USART5" }
    end
    if c_is_provided("HW_MCU_STM32F2") then
        return { "", "USART1", "USART2", "USART3", "USART4", "USART5", "USART6" }
    end
    if c_is_provided("HW_MCU_STM32F301x8") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F302_LD") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F302_MD") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5"   }
    end
    if c_is_provided("HW_MCU_STM32F302_HD") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5"   }
    end
    if c_is_provided("HW_MCU_STM32F303_LD") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F303_MD") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5"   }
    end
    if c_is_provided("HW_MCU_STM32F303_HD") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5"   }
    end
    if c_is_provided("HW_MCU_STM32F318xX") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F328xX") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F334x8") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F358xX") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5"   }
    end
    if c_is_provided("HW_MCU_STM32F373_MD") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F378xX") then
        return { "", "USART1", "USART2", "USART3"                     }
    end
    if c_is_provided("HW_MCU_STM32F398xX") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5"   }
    end
    if c_is_provided("HW_MCU_STM32F401xx") then
        return { "", "USART1", "USART2",                               "USART6" }
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return { "", "USART1", "USART2",                               "USART6" }
    end
    if c_is_provided("HW_MCU_STM32F412") then
        return { "", "USART1", "USART2", "USART3",                   "USART6"}
    end
    if c_is_provided("HW_MCU_STM32F40") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5", "USART6"}
    end
    if c_is_provided("HW_MCU_STM32F42") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5", "USART6", "USART7", "USART8" }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5", "USART6"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5", "USART6", "USART7", "USART8"}
    end
    if c_is_provided("HW_MCU_STM32L1_MD") then
        return { "", "USART1", "USART2", "USART3" }
    end
    if c_is_provided("HW_MCU_STM32L1_MDP") then
        return { "", "USART1", "USART2", "USART3" }
    end
    if c_is_provided("HW_MCU_STM32L1_HD") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5" }
    end
    if c_is_provided("HW_MCU_STM32L1_XL") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5" }
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "", "USART1", "USART2", "USART3", "UART4", "UART5" }
    end
end

--
-- Retrieve USART1TX pins available on the device.
--
function GetTxUsart1()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PA09", "PB06", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F03") then
        return { "PA09", "PB06", "PA14", "PA02", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PA09", "PB06", "PC04", "PE00", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return { "PA09", "PB06", "PA15", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F412") then
        return { "PA09", "PB06", "PA15", "PIN_NONE" }
    end
     if c_is_provided("HW_MCU_STM32L4") then
        return { "PA09", "PB06", "PG09", "PIN_NONE" }
    end
    return { "PA09", "PB06", "PIN_NONE" }
end

--
-- Retrieve USART1RX pins available on the device.
--
function GetRxUsart1()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PA10", "PB07", "PB11", "PC04", "PC11", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F03") then
        return { "PA10", "PB07", "PA15", "PA03", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PA10", "PB07", "PC05", "PE01", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F411") then
        return { "PA10", "PB07", "PB03", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F412") then
        return { "PA10", "PB07", "PB03", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PA10", "PB07", "PG10", "PIN_NONE" }
    end
    return { "PA10", "PB07", "PIN_NONE" }
end
-- FIXME: USART1 CTS/RTS/CK Pin enummeration!

--
-- Retrieve USART2TX pins available on the device.
--
function GetTxUsart2()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PA02", "PA14", "PIN_NONE" }
    end
     if c_is_provided("HW_MCU_STM32F0") then
        return { "PA02", "PA14", "PD05", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PA02", "PA14", "PB03", "PD05", "PIN_NONE" }
    end
    return { "PA02", "PD05", "PIN_NONE" }
end

--
-- Retrieve USART2RX pins available on the device.
--
function GetRxUsart2()
    if c_is_provided("HW_MCU_STM32L0") then
        return { "PA03", "PA15", "PIN_NONE" }
    end
     if c_is_provided("HW_MCU_STM32F0") then
        return { "PA03", "PA15", "PD06", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PA03", "PA15", "PB04", "PD06", "PIN_NONE" }
    end
    return { "PA03", "PD06", "PIN_NONE" }
end
-- FIXME: USART1 CTS/RTS/CK Pin enummeration!

--
-- Retrieve USART3TX pins available on the device.
--
function GetTxUsart3()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB10", "PC10", "PC04", "PD08", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PB10", "PC10", "PD08", "PB09", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB10", "PC10", "PC04", "PD08", "PIN_NONE" }
    end
    return { "PB10", "PC10", "PD08", "PIN_NONE" }
end

--
-- Retrieve USART3RX pins available on the device.
--
function GetRxUsart3()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB11", "PC11", "PC05", "PD09", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PB11", "PC11", "PD09", "PB08", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB11", "PC11", "PC05", "PD09", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F412") then
        return { "PB11", "PC11", "PC05", "PD09", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB11", "PC11", "PC05", "PD09", "PIN_NONE" }
    end
    return { "PB11", "PC11", "PD09", "PIN_NONE" }
end

--
-- Retrieve USART3CTS pins available on the device.
--
function GetCtsUsart3()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PB13", "PD11", "PA06", "PIN_NONE"}
   end
   if c_is_provided("HW_MCU_STM32F446") then
        return { "PB13", "PD11", "PIN_NONE"}
   end
   if c_is_provided("HW_MCU_STM32F412") then
        return { "PB13", "PD11", "PIN_NONE"}
   end
   if c_is_provided("HW_MCU_STM32L4") then
        return { "PB13", "PA06", "PD11", "PIN_NONE" }
   end
   return { "PB13", "PC13", "PD13", "PIN_NONE" }
end

--
-- Retrieve USART3RTS pins available on the device.
--
function GetRtsUsart3()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PB14", "PD12", "PD02", "PB01", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F3") then
        return { "PB14", "PD12", "PF06", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB14", "PB01", "PD02", "PD12", "PIN_NONE" }
    end
    return { "PB14", "PD12", "PIN_NONE" }
end

--
-- Retrieve USART3CK pins available on the device.
--
function GetCkUsart3()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PB12", "PC12", "PB00", "PD10", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB12", "PC12", "PD10", "PB00", "PIN_NONE" }
    end
    return { "PB12", "PC12", "PD10", "PIN_NONE" }
end

--
-- Retrieve USART4TX pins available on the device.
--
function GetTxUsart4()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PC10", "PA00", "PE00", "PIN_NONE"}
    end
   if c_is_provided("HW_MCU_STM32L1") then
        return { "PC10", "PIN_NONE"}
    end
   if c_is_provided("HW_MCU_STM32F3") then
        return { "PC10", "PIN_NONE"}
    end
    return { "PC10", "PA00", "PIN_NONE" }
end

--
-- Retrieve USART4RX pins available on the device.
--
function GetRxUsart4()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PC11", "PA01", "PE09", "PIN_NONE"}
    end
   if c_is_provided("HW_MCU_STM32L1") then
        return { "PC11", "PIN_NONE"}
    end
   if c_is_provided("HW_MCU_STM32F3") then
        return { "PC11", "PIN_NONE"}
    end
    return { "PC11", "PA01", "PIN_NONE" }
end
--
-- Retrieve USART4CTS pins available on the device.
--
function GetCtsUsart4()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PB07", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PB00", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F7") then
        return { "PB00", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PB00", "PIN_NONE"}
    end
    return "PIN_NONE"
end
function GetRtsUsart4()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PIN_NONE", "PA12"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PA15", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32L4") then
        return { "PA15", "PIN_NONE"}
    end
    return "PIN_NONE"
end
--
-- Retrieve USART5TX pins available on the device.
--
function GetTxUsart5()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PC12", "PB03", "PE10", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PC12", "PE08", "PIN_NONE"}
    end
    return { "PC12", "PIN_NONE" }
end

--
-- Retrieve USART5RX pins available on the device.
--
function GetRxUsart5()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PD02", "PB04", "PE11", "PIN_NONE"}
    end
    if c_is_provided("HW_MCU_STM32F446") then
        return { "PD02", "PE07", "PIN_NONE"}
    end
    return { "PD02", "PIN_NONE" }
end

--
-- Retrieve USART5RTS pins available on the device.
--
function GetRtsUsart5()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PB05", "PE07", "PIN_NONE"}
    end
   if c_is_provided("HW_MCU_STM32F446") then
        return { "PC08", "PIN_NONE"}
    end
   if c_is_provided("HW_MCU_STM32L4") then
        return { "PB04", "PIN_NONE"}
    end
   return { "PC12", "PIN_NONE" }
end
function GetCkUsart5()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PB05", "PE07", "PIN_NONE" }
    end
   return "PIN_NONE"
end

--
-- Retrieve USART6RX pins available on the device.
--
function GetRxUsart6()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PA05", "PC01", "PF10", "PIN_NONE"}
   end
   if c_is_provided("HW_MCU_STM32F401") then
        return { "PC07", "PA12", "PIN_NONE" }
   end
   if c_is_provided("HW_MCU_STM32F412") then
        return { "PC07", "PA12", "PG09", "PIN_NONE" }
   end
   if c_is_provided("HW_MCU_STM32F446") then
        return { "PC07", "PG09", "PIN_NONE" }
   end
   return { "PC07", "PG09", "PIN_NONE" }
end
function GetRxUsart6Default()
   if c_is_provided("HW_MCU_STM32F0") then
        return "PA05"
   end
   return "PC07"
end

function GetTxUsart6()
    if c_is_provided("HW_MCU_STM32F0") then
        return { "PA04", "PC00", "PF00", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F401") then
        return { "PC06", "PA11", "PIN_NONE" }
    end
    if c_is_provided("HW_MCU_STM32F412") then
        return { "PC06", "PA11", "PG14", "PIN_NONE" }
    end
    return { "PC06", "PG14", "PIN_NONE" }
end
function GetTxUsart6Default()
    if c_is_provided("HW_MCU_STM32F0") then
        return "PA04"
    end
    return "PC06"
end
function GetRtsUsart6()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PF03", "PIN_NONE"}
   end
   return { "PG08", "PG12", "PIN_NONE" }
end
function GetCkUsart6()
   if c_is_provided("HW_MCU_STM32F0") then
        return { "PF03", "PIN_NONE"}
   end
   return { "PC08", "PG07", "PIN_NONE" }
end

-- USART1 TX DMA
function GetUsart1TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_MCU_STM32L4") then
        return {"DMA_NONE", "DMA1_CH4", "DMA2_CH6"}
        end
    if c_is_provided("HW_DMA1_STM32F0") then
        return {"DMA_NONE", "DMA1_CH2", "DMA2_CH4"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH2", "DMA2_CH4"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
    if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH2", "DMA1_CH4"}
        end
    return {"DMA_NONE", "DMA1_CH4"}
end
-- USART1 DMA RX
function GetUsart1RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL4)",
                "(DMA_CONTROL2 | DMA_STREAM5 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_MCU_STM32L4") then
        return {"DMA_NONE", "DMA1_CH5", "DMA2_CH5"}
        end
    if c_is_provided("HW_DMA1_STM32F0") then
        return {"DMA_NONE", "DMA1_CH3", "DMA2_CH5"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH3", "DMA2_CH5"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
    if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH3", "DMA1_CH5"}
        end
    return {"DMA_NONE", "DMA1_CH2"}
end

-- USART2 TX DMA
function GetUsart2TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_DMA1_STM32F0") then
        return {"DMA_NONE", "DMA1_CH4"}
        end
    if c_is_provided("HW_DMA1_STM32F0") then
        return {"DMA_NONE", "DMA1_CH4", "DMA2_CH2"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH4", "DMA2_CH7"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
     if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH4", "DMA1_CH6", "DMA1_CH2"}
        end
     return {"DMA_NONE", "DMA1_CH7"}
end
-- USART2 DMA RX
function GetUsart2RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM5 | DMA_CHANNEL4)",
                "(DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL6)"}
        end
    if c_is_provided("HW_DMA1_STM32F0") then
        return {"DMA_NONE", "DMA1_CH5"}
        end
    if c_is_provided("HW_DMA1_STM32F0") then
        return {"DMA_NONE", "DMA1_CH5", "DMA2_CH3"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH5", "DMA2_CH6"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
     if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH3", "DMA1_CH7", "DMA1_CH1"}
        end
    return {"DMA_NONE", "DMA1_CH6"}
end

-- USART3 TX DMA
function GetUsart3TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL4)",
                "(DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL7)"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH7", "DMA2_CH2"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
    if c_is_provided("HW_MCU_STM32F0") then
        return {"DMA_NONE", "DMA1_CH4"}
        end
    return {"DMA_NONE", "DMA_CH2"}
end
-- USART3 DMA RX
function GetUsart3RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH6", "DMA2_CH3"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
    return {"DMA_NONE", "DMA1_CH3"}
end

-- USART4 TX DMA
function GetUsart4TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM4 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH7"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
     if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH3", "DMA1_CH7"}
        end
    return {"DMA_NONE", "DMA2_CH3"}
end
-- USART4 DMA RX
function GetUsart4RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM2 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_DMA1_STM32F07") then
        return {"DMA_NONE", "DMA1_CH6"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
     if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH2", "DMA1_CH6"}
        end
      return {"DMA_NONE", "DMA2_CH5"}
end

-- USART5 TX DMA
function GetUsart5TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM7 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
     if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH3", "DMA1_CH7"}
        end
     return {"DMA_NONE", "DMA2_CH1"}
end
-- USART5 DMA RX
function GetUsart5RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL4)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
     if c_is_provided("HW_STM32L0") then
        return {"DMA_NONE", "DMA1_CH2", "DMA2_CH6"}
        end
     return {"DMA_NONE", "DMA2_CH2"}
end

-- USART6 TX DMA
function GetUsart6TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL2 | DMA_STREAM6 | DMA_CHANNEL5)",
                "(DMA_CONTROL2 | DMA_STREAM7 | DMA_CHANNEL5)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
    return {"DMA_NONE", "DMA2_CH1"}
end
-- USART6 DMA RX
function GetUsart6RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL2 | DMA_STREAM1 | DMA_CHANNEL5)",
                "(DMA_CONTROL2 | DMA_STREAM2 | DMA_CHANNEL5)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
    return {"DMA_NONE", "DMA2_CH2"}
end

-- USART7 TX DMA
function GetUsart7TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM1 | DMA_CHANNEL5)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
    return {"DMA_NONE", "DMA2_CH1"}
end
-- USART7 DMA RX
function GetUsart7RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM3 | DMA_CHANNEL5)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
    return {"DMA_NONE", "DMA2_CH2"}
end

-- USART8 TX DMA
function GetUsart8TxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM0 | DMA_CHANNEL5)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH2", "DMA1_CH4", "DMA1_CH7",
                "DMA2_CH1", "DMA2_CH4", "DMA2_CH5"}
        end
    return {"DMA_NONE", "DMA2_CH1"}
end
-- USART8 DMA RX
function GetUsart8RxDmaChoices()
    if c_is_provided("HW_DMA_STM32F2") then
        return {"DMA_NONE",
                "(DMA_CONTROL1 | DMA_STREAM6 | DMA_CHANNEL5)"}
        end
    if c_is_provided("HW_DMA1_2_STM32F09") then
        return {"DMA_NONE",
                "DMA1_CH1", "DMA1_CH3", "DMA1_CH5", "DMA1_CH6",
                "DMA2_CH2", "DMA2_CH3"}
        end
    return {"DMA_NONE", "DMA2_CH2"}
end

nutarch_cm3_stm32_usart_devices =
{
   -- ******************************************
    --
    -- STM32 USART Device Configuration and drivers
    --
    -- ******************************************
    --
    --
    -- STM32 DEBUG Macro (Low-Level Debug UART definition)
    --
    {
        name = "nutarch_cm3_stm32_debugmacro",
        brief = "Low-Level Debug UART macros.",
        description = "Polling UART function (macro) to use in exception handlers.",
        requires = { "DEV_UART_STM32" },
        provides = { "DEBUG_MACRO"},
		sources = { "cm3/cmsis/cortex_debug.c" },
        options =
        {
            {
                macro = "DEBUG_MACRO",
                brief = "Enabled",
                description = "Check to enable debug output of exception handlers",
                flavor = "boolean",
                file = "include/cfg/cortex_debug.h"
            },
            {
                macro = "DEBUG_BACKTRACE",
                brief = "Enable call backtrace (experimental)",
                description = "Check to enable call stack back trace in case of an exception.\n\n"..
                              "In case of an exception the last 128 words on the stack will\n"..
                              "be printed out and a call trace will be generated and printed out\n"..
                              "(Address of the function entry and offset in the function).\n"..
                              "Together with the .map file and / or the assembler listing\n"..
                              "the call trace allows you to trace back your code flow in\n"..
                              "which led to the exception. The assembler listing can be\n"..
                              "created by 'arm-none-eabi-objdump -d myfile.elf'\n\n"..
                              "This function is highly experimental and relies on a very\n"..
                              "defined function entry code and existance of a stack\n"..
                              "frame pointer\n"..
                              "To ensure the correct function layout this option disables\n"..
                              "compiler optimisation and enables a stack frame pointer\n"..
                              "generation. The following compiler options will be set:\n\n"..
                              "-O0 -mtpcs-frame -mtpcs-leaf-frame -fno-omit-frame-pointer\n\n"..
                              "This will enlarge the resulkting binary size significantly\n"..
                              "any may influence execution performance.\n\n"..
                              "The backtrace will only work for functions compiled with\n"..
                              "above compiler parameters. E.g. it will very likely not work"..
                              "for any newlib functions.\n",
                flavor = "boolean",
                makedefs = { "MDEBUG+=-O0 -mtpcs-frame -mtpcs-leaf-frame -fno-omit-frame-pointer" },
                file = "include/cfg/cortex_debug.h"
            },
            {
                macro = "DEBUG_UART_NR",
                brief = "Debug UART",
                description = "Select the UART to use for low level debugging",
                type = "enumerated",
                choices = function() return GetStmUsarts() end,
                file = "include/cfg/cortex_debug.h"
            }
        }
    },

    --
    -- STM32 DEBUG UART configuration
    --
    {
        name = "nutarch_cm3_stm32_devices_debug",
        brief = "UART Debug Output (STM32)",
        requires = { "DEV_UART_STM32" },
        provides = { "DEV_UART", "DEV_FILE", "DEV_WRITE" },
        options =
        {
            {
                macro = "DEBUG_USE_UART",
                brief = "Select debug UART channel",
                description = "Select the UART to use as debug output\n"..
                              "This is only to adapt the application examples and to tell them\n"..
                              "which USART to use as stdout.",
                type = "enumerated",
                default = "DEV_UART1",
                choices = function() return GetStmUsarts() end,
                file = "include/cfg/uart.h",
--                sources = { "cm3/dev/stm/stm32_usart.c" }
            }
        }
    },
    --
    -- STM32F USART1 Interface
    --
    {
        name = "nutarch_cm3_stm32_devices_usart1",
        brief = "STM32 USART1 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_USART1_STM32"},
        provides =
        {
            "DEV_UART_STM32",
            "DEV_UART_SPECIFIC",
            "DEV_UART1_GPIO_RTS",
            "DEV_UART1_GPIO_CTS",
            "DEV_UART1_GPIO_HDX"
        },
        sources =
        {
            "cm3/dev/stm/stm32_usart1.c",
        },
        options =
        {
            {
                macro = "USART1_REMAP",
                brief = "Use Alternate Pins",
                description = "Choose Pin remap. Default = 0 means no remap.\n\n"..
                              "Default 0 Pin Mapping is:\n TXD PA09\n RXD PA10\n CTS PB13\n RTS PB14"..
                              "Remap   1 Pin Mapping is:\n TXD PB06\n RXD PB07\n CTS PB13\n RTS PB14.",
                requires = { "HW_GPIO_STM32V1" },
                type = "enumerated",
                choices = {"0", "1"},
                default = "0",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_DMA_TX",
                brief = "Select USART1 TX DMA stream ",
                description = "Eventual select USART1 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart1TxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_DMA_RX",
                brief = "Select USART1 RX DMA stream ",
                description = "Eventual select USART1 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart1RxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "SPI1_MODE",
                brief = "Select SPIBUS 1 MODE",
                description = "Choices are IRQ(0, default), DMA(1) or polling (2)",
                choices = { "0", "1", "2"},
                default = "0",
                file = "include/cfg/spi.h"
            },
            {
                macro = "USART1_USE_CTS",
                brief = "Use USART1 CTS on F1",
                description = "Use USART1 Hardware CTS on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_USE_RTS",
                brief = "Use USART1 RTS on F1",
                description = "Use USART1 Hardware RTS on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_USE_CK",
                brief = "Use USART1 CK on F1",
                description = "Use USART1 Hardware CK on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_RX",
                brief = "USART1 RX Pin selection",
                description = "Choose USART1 RX Pin, Default: PA10. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRxUsart1() end,
                default = "PA10",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_RX_INV",
                brief = "Invert USART1 RX level.",
                description = "Invert USART1 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_TX",
                brief = "USART1 TX Pin selection",
                description = "Choose USART1 TX Pin, Default: PA09. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetTxUsart1() end,
                default = "PA09",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_TX_MULTIDRIVE",
                brief = "USART1 TX Pin as Open-Drain",
                description = "Drive USART1 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_TX_INV",
                brief = "Invert USART1 TX level.",
                description = "Invert USART1 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_TX_RX_SWAP",
                brief = "Swap USART1 TX and RX pins.",
                description = "Swap USART1 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_DATA_INV",
                brief = "USART1 Binary data inversion.",
                description = "USART1 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_MSBFIRST",
                brief = "USART1 sends MSB first.",
                description = "USART1 USART1 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_CTS",
                brief = "USART1 CTS Pin selection",
                description = "Choose USART1 CTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA11"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_RTS",
                brief = "USART1 RTS Pin selection",
                description = "Choose USART1 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA12"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_CK",
                brief = "USART1 CK Pin selection",
                description = "Choose USART1 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA08"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "USART1_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "USART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "USART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "USART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "USART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "USART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "USART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART1_PIN_SPEED",
                brief = "Pin speed setting for USART1 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
        },
    },

    --
    -- STM32F USART2 Interface
    --
    {
        name = "nutarch_cm3_stm32f_usart2",
        brief = "STM32 USART2 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_USART2_STM32" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_STM32",
            "DEV_UART_SPECIFIC",
            "DEV_UART2_GPIO_RTS",
            "DEV_UART2_GPIO_CTS",
            "DEV_UART2_GPIO_HDX"
        },
        sources =
        {
            "cm3/dev/stm/stm32_usart2.c",
        },
        options =
        {
            {
                macro = "USART2_REMAP",
                brief = "Use Alternate Pins",
                description = "Choose Pin remap. Default = 0 means no remap.\n\n"..
                              "Default 0 Pin Mapping is:\n TXD PA02\n RXD PA03\n CTS PA00\n RTS PA01\n CK PA04"..
                              "Remap   1 Pin Mapping is:\n TXD PD05\n RXD PD06\n CTS PD03\n RTS PD04\n CK PD07.",
                requires = { "HW_GPIO_STM32V1" },
                type = "enumerated",
                choices = {"0", "1"},
                default = "0",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_DMA_TX",
                brief = "Select USART2 TX DMA stream ",
                description = "Eventual select USART2 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart2TxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_DMA_RX",
                brief = "Select USART2 RX DMA stream ",
                description = "Eventual select USART2 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart2RxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_USE_CTS",
                brief = "Use USART2 CTS on F1",
                description = "Use USART2 Hardware CTS on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_USE_RTS",
                brief = "Use USART2 RTS on F1",
                description = "Use USART2 Hardware RTS on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_USE_CK",
                brief = "Use USART2 CK on F1",
                description = "Use USART2 Hardware CK on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_RX",
                brief = "USART2 RX Pin selection",
                description = "Choose USART2 RX Pin. Default is PA03. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRxUsart2() end,
                default = "PA03",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_RX_INV",
                brief = "Invert USART2 RX level.",
                description = "Invert USART2 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_TX",
                brief = "USART2 TX Pin selection",
                description = "Choose USART2 TX Pin. Default is PA02. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetTxUsart2() end,
                default = "PA02",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_TX_MULTIDRIVE",
                brief = "USART2 TX Pin as Open-Drain",
                description = "Drive USART2 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_TX_INV",
                brief = "Invert USART2 TX level.",
                description = "Invert USART2 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_TX_RX_SWAP",
                brief = "Swap USART2 TX and RX pins.",
                description = "Swap USART2 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_DATA_INV",
                brief = "USART2 Binary data inversion.",
                description = "USART2 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_MSBFIRST",
                brief = "USART2 sends MSB first.",
                description = "USART2 USART2 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_CTS",
                brief = "USART2 CTS Pin selection",
                description = "Choose USART2 CTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA00", "PD03"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_RTS",
                brief = "USART2 RTS Pin selection",
                description = "Choose USART2 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA01", "PD04"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_CK",
                brief = "USART2 CK Pin selection",
                description = "Choose USART2 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA04", "PD07"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "USART2_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "USART2_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "USART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "USART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "USART2_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "USART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "USART2_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART2_PIN_SPEED",
                brief = "Pin speed setting for USART2 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
         },
    },

    --
    -- STM32F USART3 Interface
    --
    {
        name = "nutarch_cm3_stm32_devices_usart3",
        brief = "STM32 USART3 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                  "functions for the generic driver framework.",
        requires = { "HW_USART3_STM32", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_STM32",
            "DEV_UART_SPECIFIC",
            "DEV_UART3_GPIO_RTS",
            "DEV_UART3_GPIO_CTS",
            "DEV_UART3_GPIO_HDX"
        },
        sources =
        {
            "cm3/dev/stm/stm32_usart3.c",
        },
        options =
        {
            {
                macro = "USART3_REMAP",
                brief = "Use Alternate Pins",
                description = "Choose Pin remap. Default = 0 means no remap.\n\n"..
                              "Default 0 Pin Mapping is:\n TXD PB10\n RXD PB11\n CTS PB13\n RTS PB14\n CK PC12"..
                              "Remap1  1 Pin Mapping is:\n TXD PC10\n RXD PC11\n CTS PB13\n RTS PB14\n CK PC12.",
                              "Remap3  3 Pin Mapping is:\n TXD PD08\n RXD PD09\n CTS PD11\n RTS PD12\n CK PD11.",
                requires = { "HW_GPIO_STM32V1" },
                type = "enumerated",
                choices = {"0", "1", "3"},
                default = "0",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_DMA_TX",
                brief = "Select USART3 TX DMA stream ",
                description = "Eventual select USART3 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart3TxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_TX_MULTIDRIVE",
                brief = "USART3 TX Pin as Open-Drain",
                description = "Drive USART3 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_TX_INV",
                brief = "Invert USART3 TX level.",
                description = "Invert USART3 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_TX_RX_SWAP",
                brief = "Swap USART3 TX and RX pins.",
                description = "Swap USART3 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_DATA_INV",
                brief = "USART3 Binary data inversion.",
                description = "USART3 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_MSBFIRST",
                brief = "USART3 sends MSB first.",
                description = "USART3 USART3 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_DMA_RX",
                brief = "Select USART3 RX DMA stream ",
                description = "Eventual select USART3 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart3RxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_USE_CTS",
                brief = "Use USART3 CTS on F1",
                description = "Use USART3 Hardware CTS on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_USE_RTS",
                brief = "Use USART3 RTS on F1",
                description = "Use USART3 Hardware RTS on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_USE_CK",
                brief = "Use USART3 CK on F1",
                description = "Use USART3 Hardware CK on F1",
                requires = { "HW_GPIO_STM32V1" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_RX",
                brief = "USART3 RX Pin selection",
                description = "Choose USART3 RX Pin, Default: PA11. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRxUsart3() end,
                default = "PB11",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_RX_INV",
                brief = "Invert USART3 RX level.",
                description = "Invert USART3 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_TX",
                brief = "USART3 TX Pin selection",
                description = "Choose USART3 TX Pin, Default: PB10. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetTxUsart3() end,
                default = "PB10",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_CTS",
                brief = "USART3 CTS Pin selection",
                description = "Choose USART3 CTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetCtsUsart3() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_RTS",
                brief = "USART3 RTS Pin selection",
                description = "Choose USART3 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRtsUsart3() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_CK",
                brief = "USART3 CK Pin selection",
                description = "Choose USART3 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetCkUsart3() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "USART3_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "USART3_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "USART3_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "USART3_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "USART3_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "USART3_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "USART3_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART3_PIN_SPEED",
                brief = "Pin speed setting for USART3 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
        },
     },

    --
    -- STM32F UART4 Interface
    --
    {
        name = "nutarch_cm3_stm32f_uart4",
        brief = "STM32 UART4 Driver",
        description = "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART4_STM32", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_STM32",
            "DEV_UART_SPECIFIC",
            "DEV_UART4_GPIO_HDX"
        },
        sources =
        {
            "cm3/dev/stm/stm32_uart4.c",
        },
        options =
        {
            {
                macro = "USART4_DMA_TX",
                brief = "Select USART4 TX DMA stream ",
                description = "Eventual select USART4 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart4TxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_DMA_RX",
                brief = "Select USART4 RX DMA stream ",
                description = "Eventual select USART4 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart4RxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_RX",
                brief = "USART4 RX Pin selection",
                description = "Choose USART4 RX Pin. Default is PC11. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRxUsart4() end,
                default = "PC11",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_RX_INV",
                brief = "Invert USART4 RX level.",
                description = "Invert USART4 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_TX",
                brief = "USART4 TX Pin selection",
                description = "Choose USART4 TX Pin. Default is PC10. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetTxUsart4() end,
                default = "PC10",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_TX_MULTIDRIVE",
                brief = "USART4 TX Pin as Open-Drain",
                description = "Drive USART4 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_TX_INV",
                brief = "Invert USART4 TX level.",
                description = "Invert USART4 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_TX_RX_SWAP",
                brief = "Swap USART4 TX and RX pins.",
                description = "Swap USART4 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_DATA_INV",
                brief = "USART4 Binary data inversion.",
                description = "USART4 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_MSBFIRST",
                brief = "USART4 sends MSB first.",
                description = "USART4 USART4 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_CTS",
                brief = "USART4 CTS Pin selection",
                description = "Choose USART4 CTS Pin. Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetCtsUsart4() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_RTS",
                brief = "USART4 RTS Pin selection",
                description = "Choose USART4 RTS Pin. Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PA13"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_CK",
                brief = "USART4 CK Pin selection",
                description = "Choose USART4 RTS Pin. Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetCkUsart4() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "USART4_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "USART4_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "USART4_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "USART4_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "USART4_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "USART4_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "USART4_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART4_PIN_SPEED",
                brief = "Pin speed setting for U(S)ART4 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
        },
    },

    --
    -- STM32F UART5 Interface
    --
    {
        name = "nutarch_cm3_stm32f_uart5",
        brief = "STM32 UART5 Driver",
        description = "Hardware specific UART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_UART5_STM32", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_SPECIFIC",
            "DEV_UART5_GPIO_HDX"
        },
        sources =
        {
            "cm3/dev/stm/stm32_uart5.c",
        },
        options =
        {
            {
                macro = "USART5_DMA_TX",
                brief = "Select USART5 TX DMA stream ",
                description = "Eventual select USART5 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart5TxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_DMA_RX",
                brief = "Select USART5 RX DMA stream ",
                description = "Eventual select USART5 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart5RxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_RX",
                brief = "USART5 RX Pin selection",
                description = "Choose USART5 RX Pin, Default: PD02. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRxUsart5() end,
                default = "PD02",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_RX_INV",
                brief = "Invert USART5 RX level.",
                description = "Invert USART5 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_TX",
                brief = "USART5 TX Pin selection",
                description = "Choose USART5 TX Pin, Default: PC12. Use PIN_NONE if unused, with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetTxUsart5() end,
                default = "PC12",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_TX_MULTIDRIVE",
                brief = "USART5 TX Pin as Open-Drain",
                description = "Drive USART5 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_TX_INV",
                brief = "Invert USART5 TX level.",
                description = "Invert USART5 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_TX_RX_SWAP",
                brief = "Swap USART5 TX and RX pins.",
                description = "Swap USART5 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_DATA_INV",
                brief = "USART5 Binary data inversion.",
                description = "USART5 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_MSBFIRST",
                brief = "USART5 sends MSB first.",
                description = "USART5 USART5 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_CTS",
                brief = "USART5 CTS Pin selection",
                description = "Choose USART5 CTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PC09"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_RTS",
                brief = "USART5 RTS Pin selection",
                description = "Choose USART5 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRtsUsart5() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_CK",
                brief = "USART5 CK Pin selection",
                description = "Choose USART5 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetCkUsart5() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "USART5_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "USART5_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "USART5_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "USART5_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "USART5_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "USART5_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "USART5_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART5_PIN_SPEED",
                brief = "Pin speed setting for U(S)ART5 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
        },
    },
    --
    -- STM32F USART6 Interface
    --
    {
        name = "nutarch_cm3_stm32_devices_usart6",
        brief = "STM32 USART6 Driver",
        description = "Hardware specific USART driver. Implements hardware "..
                      "functions for the generic driver framework.",
        requires = { "HW_USART6_STM32", "NUT_EVENT", "CRT_HEAPMEM" },
        provides =
        {
            "DEV_UART",
            "DEV_UART_STM32",
            "DEV_UART_SPECIFIC",
            "DEV_UART6_GPIO_RTS",
            "DEV_UART6_GPIO_CTS",
            "DEV_UART6_GPIO_HDX"
        },
        sources =
        {
            "cm3/dev/stm/stm32_usart6.c",
        },
        options =
        {
            {
                macro = "USART6_DMA_TX",
                brief = "Select USART6 TX DMA stream ",
                description = "Eventual select USART6 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart6TxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_DMA_RX",
                brief = "Select USART6 RX DMA stream ",
                description = "Eventual select USART6 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = function() return GetUsart6RxDmaChoices() end,
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_RX",
                brief = "USART6 RX Pin selection",
                description = "Choose USART6 RX Pin. Default is device specific. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRxUsart6() end,
                default = function() return GetRxUsart6Default() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_RX_INV",
                brief = "Invert USART6 RX level.",
                description = "Invert USART6 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_TX",
                brief = "USART6 TX Pin selection",
                description = "Choose USART6 TX Pin. Default is device specific. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetTxUsart6() end,
                default = function() return GetTxUsart6Default() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_TX_MULTIDRIVE",
                brief = "USART6 TX Pin as Open-Drain",
                description = "Drive USART6 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_TX_INV",
                brief = "Invert USART6 TX level.",
                description = "Invert USART6 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_TX_RX_SWAP",
                brief = "Swap USART6 TX and RX pins.",
                description = "Swap USART6 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_DATA_INV",
                brief = "USART6 Binary data inversion.",
                description = "USART6 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_MSBFIRST",
                brief = "USART6 sends MSB first.",
                description = "USART6 USART6 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_CTS",
                brief = "USART6 CTS Pin selection",
                description = "Choose USART6 CTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PG13", "PG15"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_RTS",
                brief = "USART6 RTS Pin selection",
                description = "Choose USART6 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = function() return GetRtsUsart6() end,
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_CK",
                brief = "USART6 CK Pin selection",
                description = "Choose USART6 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PC08", "PG07"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "USART6_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "USART6_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "USART6_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "USART6_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "USART6_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "USART6_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "USART6_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "USART6_PIN_SPEED",
                brief = "Pin speed setting for USART6 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
        },
    },
    -- STM32 LPUART1 Interface
    --
    {
        name = "nutarch_cm3_stm32_devices_lpuart",
        brief = "STM32 LPUART1 Driver",
        requires = { "HW_LPUART1_STM32", "NUT_EVENT", "CRT_HEAPMEM" },
         options =
        {
            {
                macro = "LPUART1_DMA_TX",
                brief = "Select LPUART1 TX DMA stream ",
                description = "FIXME: Provide selection!\n\n"..
                              "Eventual select LPUART1 TX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.",
                type = "enumerated",
                choices =  {"DMA_NONE"},
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_DMA_RX",
                brief = "Select LPUART1 RX DMA stream ",
                description = "FIXME: Provide selection!\n\n"..
                              "Eventual select LPUART1 RX stream.\n\n"..
                              "If selected, check for collision with other DMA!\n"..
                              "Default is DMA_NONE to not use DMA.\n",
                type = "enumerated",
                choices = {"DMA_NONE"},
                default = "DMA_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_RX",
                brief = "LPUART1 RX Pin selection",
                description = "FIXME: Provide selection!\n\n"..
                              "Choose LPUART1 RX Pin. Default is device specific. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = {"PB11", "PIN_NONE"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_RX_INV",
                brief = "Invert LPUART1 RX level.",
                description = "Invert LPUART1 RX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_TX",
                brief = "LPUART1 TX Pin selection",
                description = "FIXME: Provide selection!\n\n"..
                              "Choose LPUART1 TX Pin. Default is device specific. Use PIN_NONE if unused with USART used.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = {"PB10", "PIN_NONE"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_TX_MULTIDRIVE",
                brief = "LPUART1 TX Pin as Open-Drain",
                description = "Drive LPUART1 TX as Open-Drain\n\n"..
                              "Default is DISABLE for Push/Pull",
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_TX_INV",
                brief = "Invert LPUART1 TX level.",
                description = "Invert LPUART1 TX level.\n\n"..
                              "Default is DISABLE for normal level",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_TX_RX_SWAP",
                brief = "Swap LPUART1 TX and RX pins.",
                description = "Swap LPUART1 TX and RX pins.\n\n"..
                              "Default is DISABLE for no swap.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_DATA_INV",
                brief = "LPUART1 Binary data inversion.",
                description = "LPUART1 Binary data inversion.\n\n"..
                              "Default is DISABLE for no data inversion.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_MSBFIRST",
                brief = "LPUART1 sends MSB first.",
                description = "LPUART1 LPUART1 sends MSB first.\n\n"..
                              "Default is DISABLE for send LSB first.",
                requires = {"HW_USART_STM32V2"},
                type = "enumerated",
                choices = {"DISABLE", "ENABLE"},
                default = "DISABLE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_CTS",
                brief = "LPUART1 CTS Pin selection",
                description = "FIXME: Provide selection!\n\n"..
                              "Choose LPUART1 CTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = {"PIN_NONE"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_RTS",
                brief = "LPUART1 RTS Pin selection",
                description = "FIXME: Provide selection!\n\n"..
                              "Choose LPUART1 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = {"PIN_NONE"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_CK",
                brief = "LPUART1 CK Pin selection",
                description = "Choose LPUART1 RTS Pin, Default: PIN_NONE to leave unconnected.",
                requires = { "HW_GPIO_STM32V2" },
                type = "enumerated",
                choices = { "PIN_NONE", "PC08", "PG07"},
                default = "PIN_NONE",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_XONXOFF_CONTROL",
                brief = "XON/XOFF Protocol",
                description = "When selected, the driver will use software XON/XOFF protocol.",
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_INIT_BAUDRATE",
                brief = "Initial Baudrate",
                description = "Initial baudrate the Port is set to.",
                type = "integer",
                default = 115200,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_INIT_RX_BUFSIZE",
                brief = "Initial size of RX buffer",
                description = "Bytes used for RX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_INIT_TX_BUFSIZE",
                brief = "Initial size of TX buffer",
                description = "Bytes used for TX buffer.\n"..
                    "Only size - 1 bytes can be written.\n"..
                    "Guard byte is used for Full/Empty recognition",
                type = "integer",
                default = 64,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_RS485_CTRL",
                brief = "Support RS485 control",
                description = "When selected a GPIO pin must be configured for RS485 direction control.",
                flavor = "booldata",
                provides = { "LPUART1_RS485_CTRL" },
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_485DE_INV",
                brief = "RS485 DE Invert",
                description = "Sets the DE signal as active low.",
                requires = { "LPUART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_485DE_PORT",
                brief = "RS485 DE Control Port",
                description = "Select the port of the DE signal.",
                requires = { "LPUART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_485DE_PIN",
                brief = "RS485 DE Control Pin",
                description = "Select the pin for the DE signal.",
                requires = { "LPUART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_485RE_INV",
                brief = "RS485 RE Invert",
                description = "Sets the RE signal as active high.",
                requires = { "LPUART1_RS485_CTRL" },
                flavor = "booldata",
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_485RE_PORT",
                brief = "RS485 /RE Control Port",
                description = "Select the port of the /RE signal.",
                requires = { "LPUART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_485RE_PIN",
                brief = "RS485 /RE Control Pin",
                description = "Select the pin for the /RE signal.",
                requires = { "LPUART1_RS485_CTRL" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/uart.h"
            },
            {
                macro = "LPUART1_PIN_SPEED",
                brief = "Pin speed setting for LPUART1 device",
                description = function() return PinSpeedDesc() end,
                choices = function() return GetPinSpeedChoices() end,
                type = "enumerated",
                default = "GPIO_MED",
                file = "include/cfg/uart.h"
            },
        },
    },
}
