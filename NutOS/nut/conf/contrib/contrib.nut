--
-- Copyright (C) 2008 by egnite GmbH.
--
-- All rights reserved.
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

-- Non-BSD licensed code
--
-- $Id: contrib.nut 5257 2013-08-03 12:46:31Z olereinhardt $
--

nutcontrib =
{
    options =
    {
        {
            brief = "RealNetworks RPSL/RCSL",
            description = "Enable this option only, if you agree to the\n\n"..
                          "RealNetworks Public Source License Version 1.0\n\n"..
                          "For details see http://opensource.org/licenses/real.php",
            provides = { "LICENSE_RPSL_RCSL" },
            macro = "LICENSE_RPSL_RCSL",
            flavor = "boolean",
            makedefs = { "LICENSE_RPSL_RCSL=y" }
        },
        {
            brief = "GNU GENERAL PUBLIC LICENSE Version 3",
            description = "Enable this option only, if you agree to the\n\n"..
                          "GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007\n\n"..
                          "For details see http://www.gnu.org/licenses/gpl-3.0-standalone.html",
            provides = { "LICENSE_GPL3" },
            macro = "LICENSE_GPL3",
            flavor = "boolean",
            makedefs = { "LICENSE_GPLv3=y" }
        },
        {
            brief = "STMicroelectronics Guidance Only",
            description = "This is not an OSI approved license. Specifically "..
                          "commercial use and redistribution is unclear. Carefully "..
                          "check the copyright claimer in the individual source files.\n\n"..
                          "Enable this option only, if you agree to this.",
            provides = { "LICENSE_ST_GUIDANCE_ONLY" },
            macro = "LICENSE_ST_GUIDANCE_ONLY",
            flavor = "boolean",
            makedefs = { "LICENSE_ST_GUIDANCE=y" }
        },
        {
            brief = "MCD-ST Liberty SW License Agreement V2",
            description = "Enable this option only, if you agree to the\n\n"..
                          "MCD-ST Liberty SW License Agreement V2\n\n"..
                          "For details see http://www.st.com/software_license_agreement_liberty_v2",
            provides = { "LICENSE_MCD_ST_LIBERTY" },
            macro = "LICENSE_MCD_ST_LIBERTY",
            flavor = "boolean",
            makedefs = { "LICENSE_MCD_ST_LIBERTY=y" }
        },
    },
    {
        name = "nutcontrib_hxmp3",
        brief = "RealNetworks Decoder",
        requires = { "HW_TARGET" },
        script = "contrib/hxmp3.nut"
    },
    {
        name = "nutcontrib_armcrypto",
        brief = "ARM Crypto Library",
        requires = { "HW_TARGET" },
        script = "contrib/armcrypto.nut"
    }
}
