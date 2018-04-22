--
-- Copyright (C) 2014 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

-- Cryptographic functions
--
--

bigint_reduction = {
    "CRYPTO_BIGINT_CLASSICAL",
    "CRYPTO_BIGINT_MONTGOMERY",
    "CRYPTO_BIGINT_BARRETT"
}

nutcrypto =
{
    {
        name = "nutcrypto_aes",
        brief = "AES 128/256",
        description = "AES 128 / 256 block cypher.",
        provides = { "CRYPTO_AES" },
        sources =
        {
            "aes.c"
        }
    },
    {
        name = "nutcrypto_bigint",
        brief = "Big integer mathematics",
        description = "Big integer mathematics.",
        provides = { "CRYPTO_BIGINT" },
        sources =
        {
            "bigint.c"
        },
        options =
        {
            {
                macro = "CRYPTO_BIGINT_CLASSICAL",
                brief = "Classical bigint reduction algorithm",
                description = "Classical uses standard division. It has no limitations and is "..
                              "theoretically the slowest due to the divisions used. For this particular "..
                              "implementation it is surprisingly quite fast.",
                flavor = "boolean",
                exclusivity = bigint_reduction,
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_MONTGOMERY",
                brief = "Montgomery bigint reduction algorithm",
                description = "Montgomery uses simple addition and multiplication to achieve its "..
                              "performance. It has the limitation that 0 <= x, y < m, and so is not "..
                              "used when CRT is active.",
                flavor = "boolean",
                exclusivity = bigint_reduction,
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_BARRETT",
                brief = "Barrett bigint reduction algorithm",
                description = "Barrett performs expensive precomputation before reduction and "..
                              "partial multiplies for computational speed. \n"..
                              "It is about 40% faster than Classical/Montgomery with the expense "..
                              "of about 2kB, and so this option is normally selected.",
                flavor = "boolean",
                exclusivity = bigint_reduction,
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_CRT",
                brief = "Chinese Remainder Theorem (CRT)",
                description = "Allow the Chinese Remainder Theorem (CRT) to be used.\n\n"..
                              "Uses a number of extra coefficients from the private key to improve the "..
                              "performance of a decryption. This feature is one of the most "..
                              "significant performance improvements (it reduces a decryption time by "..
                              "over 3 times).\n\n"..
                              "This option should be selected.",
                flavor = "boolean",
                default = true,
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_KARATSUBA",
                brief = "Karatsuba Multiplication",
                description = "Allow Karasuba multiplication to be used.\n\n"..
                              "Uses 3 multiplications (plus a number of additions/subtractions) "..
                              "instead of 4. Multiplications are O(N^2) but addition/subtraction "..
                              "is O(N) hence for large numbers is beneficial. For this project, the "..
                              "effect was only useful for 4096 bit keys (for 32 bit processors). For "..
                              "8 bit processors this option might be a possibility.\n\n"..
                              "It costs about 2kB to enable it.",
                provides = { "BIGINT_KARATSUBA" },
                flavor = "boolean",
                file = "include/cfg/crypto.h",
            },
            {
                macro = "MUL_KARATSUBA_THRESH",
                brief = "Karatsuba Multiplication Theshold",
                description = "The minimum number of components needed before Karasuba muliplication "..
                              "is used.\n\n"..
                              "This is very dependent on the speed/implementation of bi_add()/"..
                              "bi_subtract(). There is a bit of trial and error here and will be "..
                              "at a different point for different architectures.",
                requires = { "BIGINT_KARATSUBA" },
                default = 20,
                flavor = "integer",
                file = "include/cfg/crypto.h",
            },
            {
                macro = "SQU_KARATSUBA_THRESH",
                brief = "Karatsuba Square Threshold",
                description = "The minimum number of components needed before Karatsuba squaring "..
                              "is used.\n\n"..
                              "This is very dependent on the speed/implementation of bi_add()/"..
                              "bi_subtract(). There is a bit of trial and error here and will be "..
                              "at a different point for different architectures.",
                requires = { "BIGINT_KARATSUBA", "BIGINT_SQUARE" },
                default = 40,
                flavor = "integer",
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_SLIDING_WINDOW",
                brief = "Sliding Window Exponentiation",
                description = "Allow Sliding-Window Exponentiation to be used.\n\n"..
                              "Potentially processes more than 1 bit at a time when doing "..
                              "exponentiation. The sliding-window technique reduces the number of "..
                              "precomputations compared to other precomputed techniques.\n\n"..
                              "It results in a considerable performance improvement with it enabled"..
                              "(it halves the decryption time) and so should be selected.",
                default = true,
                flavor = "boolean",
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_SQUARE",
                brief = "Square Algorithm",
                description = "Allow squaring to be used instead of a multiplication. It uses "..
                              "1/2 of the standard multiplies to obtain its performance. "..
                              "It gives a 20% speed improvement overall and so should be selected.",
                active = "true",
                provides = { "BIGINT_SQUARE" },
                flavor = "boolean",
                file = "include/cfg/crypto.h",
            },
            {
                macro = "CRYPTO_BIGINT_CHECK_ON",
                brief = "BigInt Integrity Checking",
                description = "This is used when developing bigint algorithms. It performs a sanity "..
                              "check on all operations at the expense of speed. \n\n"..
                              "This option is only selected when developing and should normally be turned off.",
                default = false,
                flavor = "boolean",
                file = "include/cfg/crypto.h",
            },
        },
    },
    {
        name = "nutcrypto_rsa",
        brief = "RSA public key encryption / decryption",
        description = "RSA public encryption algorithm. Uses the bigint library "..
                      "to perform its calculations",
        requires = { "CRYPTO_BIGINT" },
        provides = { "CRYPTO_RSA" },
        sources =
        {
            "rsa.c"
        }
    },
    {
        name = "nutcrypto_sha1",
        brief = "SHA1 implementation",
        description = "SHA1 implementation - as defined in FIPS PUB 180-1 published April 17, 1995.",
        provides = { "CRYPTO_SHA1" },
        sources =
        {
            "sha1.c"
        }
    },
    {
        name = "nutcrypto_sha256",
        brief = "SHA256 implementation",
        description = "FIPS-180-2 compliant SHA-256 implementation",
        provides = { "CRYPTO_SHA256" },
        sources =
        {
            "sha256.c"
        }
    },
    {
        name = "nutcrypto_sha384",
        brief = "SHA384 implementation",
        description = "FIPS-180-2 compliant SHA-384 implementation",
        provides = { "CRYPTO_SHA384" },
        sources =
        {
            "sha384.c"
        }
    },
    {
        name = "nutcrypto_sha512",
        brief = "SHA512 implementation",
        description = "FIPS-180-2 compliant SHA-512 implementation",
        provides = { "CRYPTO_SHA512" },
        sources =
        {
            "sha512.c"
        }
    },
    {
        name = "nutcrypto_rc4",
        brief = "RC4 implementation",
        description = "Implementation of the RC4/ARC4 algorithm.",
        provides = { "CRYPTO_RC4" },
        sources =
        {
            "rc4.c"
        }
    },
    {
        name = "nutcrypto_md5",
        brief = "MD5 implementation",
        description = "RFC1321 compliant MD5 implementation",
        provides = { "CRYPTO_MD5" },
        sources =
        {
            "md5.c"
        }
    },
    {
        name = "nutcrypto_hmac",
        brief = "HMAC implementation",
        description = "HMAC implementation - This code was originally taken from RFC2104",
        requires = { "CRYPTO_MD5", "CRYPTO_SHA1" },
        provides = { "CRYPTO_HMAC" },
        sources =
        {
            "hmac.c"
        }
    },

}
