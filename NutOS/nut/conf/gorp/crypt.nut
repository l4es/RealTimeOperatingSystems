--
-- Copyright (C) 2010 by Ulrich Prinz
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

-- Gorp linked list modules.
--
-- $Id$
--

nutgorp_crypt =
{
    {
        name = "nutgorp_list_xtea",
        brief = "XTEA Crypto",
        description = "Functions to encrypt and decrypt with publich domain XTEA algorythm.",
        sources = { "crypt/xtea.c" }
    },
    {
        name = "nutgorp_list_aes",
        brief = "AES 128 / 192 / 256",
        description = "Byte oriented AES128 / AES192 / AES256 implementation.",
        requires = { "TOOL_GCC" },
        sources = { "crypt/aes.c" },
	options =
	{
            {
                macro = "AES_USE_TABLES",
                brief = "Use precalculated tables",
                description = "When selected, the ARS algorithm will use pre-calculated\n"..
                              "tables instead of online-calculations. Select this if\n"..
                              "code size is less important than speed.",
                flavor = "boolean",
                provides = { "AES_USE_TABLES" },
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_USE_VERSION_1",
                brief = "Use version 1 of the algorithm",
                description = "Version 1 of the AES implementation might be faster on some\n"..
                              "CPUs. Make you own speed tests.",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_ENC_PREKEYED",
                brief = "Pre-Keyed encryption",
                description = "Enable AES encryption with a precomputed key schedule",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_DEC_PREKEYED",
                brief = "Pre-Keyed decryption",
                description = "Enable AES decryption with a precomputed key schedule (standard encryption)",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_ENC_128_OTFK",
                brief = "128 Bit 'on the fly keying' encryption",
                description = "Enable AES encryption with 'on the fly' 128 bit keying",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_DEC_128_OTFK",
                brief = "128 Bit 'on the fly keying' decryption",
                description = "AES decryption with 'on the fly' 128 bit keying",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_ENC_256_OTFK",
                brief = "256 Bit 'on the fly keying' encryption",
                description = "Enable AES encryption with 'on the fly' 256 bit keying",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            },
            {
                macro = "AES_DEC_256_OTFK",
                brief = "256 Bit 'on the fly keying' decryption",
                description = "AES decryption with 'on the fly' 256 bit keying",
                flavor = "boolean",
                file = "include/cfg/aes.h"
            }
	}
    }

}
