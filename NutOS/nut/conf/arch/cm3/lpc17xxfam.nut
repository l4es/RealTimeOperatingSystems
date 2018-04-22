--
-- Copyright (C) 2012 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

-- LPC Family selection
--
nutarch_cm3_lpc17xx_family =
{
    --
    -- LPC17xx Based Cpu Directory
    --
    {
        name = "nutarch_lpc17xx_fam",
        brief = "MCU Family",
        options =
        {
            {
                macro = "MCU_LPC17xx",
                brief = "LPC17xx",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_LPC17xx" },
                file = "include/cfg/arch.h"
            }
        }
    },
    {
        name = "nutarch_cm3_lpc176x",
        brief = "LPC175x / LPC176x",
        requires = { "HW_MCU_LPC176x" },
        description = "NXP LPC175x and LPC176x series",
        script = "arch/cm3/lpc176x.nut"
    },
    {
        name = "nutarch_cm3_lpc177x_8x_lpc407x_8x",
        brief = "LPC177x / LPC178x, LPC407x / LPC408x",
        requires = { "HW_MCU_LPC177x_8x_LPC407x_8x" },
        description = "NXP LPC177x/8x and LPC407x/8x series",
        script = "arch/cm3/lpc177x_8x.nut"
    },
    {
        name = "nutarch_cm3_lpc17xx_devices",
        brief = "Common devices",
        description = "Common devices to the LPC17xx families",
        requires = { "HW_MCU_LPC17xx" },
        script = "arch/cm3/lpc17xxdev.nut"
    }
}

