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

-- TLS client and server library
--
--

tls_mode = {
    "TLS_SSL_SERVER_ONLY",
    "TLS_SSL_CERT_VERIFICATION",
    "TLS_SSL_ENABLE_CLIENT",
    "TLS_SSL_FULL_MODE"
}

tls_protocol_preference = {
    "TLS_SSL_PROT_LOW",
    "TLS_SSL_PROT_MEDIUM",
    "TLS_SSL_PROT_HIGH"
}

nuttls =
{
    {
        name = "nuttls_tls1",
        brief = "SSL/TLSv1 client / server",
        description = "SSL/TLSv1 client and server implementation",
        requires = { "CRYPTO_AES", "CRYPTO_MD5", "CRYPTO_SHA1", "CRYPTO_SHA256", "CRYPTO_RSA" },
        provides = { "TLS_TLS1" },
        sources =
        {
            "tls_misc.c", "asn1.c", "tls1.c", "tls1_clnt.c", "tls1_svr.c"
        },
        options =
        {
            --
            -- TLS Mode settings (client / server)
            --
            {
                macro = "TLS_SSL_SERVER_ONLY",
                brief = "Server only - no verification",
                description = "Enable server functionality (no client functionality).\n\n"..
                              "This mode still supports sessions and chaining (which can be turned "..
                              "off in configuration).\n\n"..
                              "This is the most space efficient of the modes with the library "..
                              "about 45kB in size on ARM. Use this mode if you are doing standard SSL server "..
                              "work.",
                flavor = "boolean",
                exclusivity = tls_mode,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_CERT_VERIFICATION",
                brief = "Server only - with verification",
                description = "Enable server functionality with client authentication (no client functionality).\n\n"..
                              "This mode produces a library about 49kB in size. Use this mode if you "..
                              "have an SSL server which requires client authentication (which is "..
                              "uncommon in browser applications).",
                flavor = "boolean",
                exclusivity = tls_mode,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_ENABLE_CLIENT",
                brief = "Client & Server enabled",
                description = "Enable client/server functionality (including peer authentication)\n\n"..
                              "This mode produces a library about 51kB in size. Use this mode if you "..
                              "require axTLS to use SSL client functionality (the SSL server code "..
                              "is always enabled).",
                flavor = "boolean",
                exclusivity = tls_mode,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_FULL_MODE",
                brief = "Client/Server enabled with diagnostics",
                description = "Enable client/server functionality including diagnostics. "..
                              "Most of the extra size in this mode is due to the storage "..
                              "of various strings that are used.\n\n"..
                              "This mode produces a library about 58kB in size. It is suggested that "..
                              "this mode is used only during development, or systems that have more "..
                              "generous memory limits.\n",
                flavor = "boolean",
                exclusivity = tls_mode,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_CTX_MUTEXING",
                brief = "Enable SSL_CTX mutexing",
                description = "Normally mutexing is not required - each SSL_CTX object can deal with "..
                               "many SSL objects (as long as each SSL_CTX object is using a single "..
                               "thread).\n\n"..
                               "If the SSL_CTX object is not thread safe e.g. the case where a "..
                               "new thread is created for each SSL object, then mutexing is required.\n\n"..
                               "Select y when a mutex on the SSL_CTX object is required.",
                flavor = "boolean",
                file = "include/cfg/tls.h",
            },
        },
    },
    {
        name = "nuttls_protocol",
        brief = "SSL/TLSv1 protocol settings",
        description = "SSL/TLSv1 protocol settings",
        requires = { "TLS_TLS1"},
        provides = { "TLS_TLS1_PROTOCOL" },
        options =
        {
            --
            -- TLS protocol security preferences
            --
            {
                macro = "TLS_SSL_PROT_LOW",
                brief = "Low security mode",
                description = "Chooses the cipher in the order of RC4-SHA, AES128-SHA, AES256-SHA.\n\n"..
                              "This will use the fastest cipher(s) but at the expense of security. ",
                flavor = "boolean",
                exclusivity = tls_protocol_preference,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_PROT_MEDIUM",
                brief = "Medium security mode",
                description = "Chooses the cipher in the order of AES128-SHA, AES256-SHA, RC4-SHA.\n\n"..
                              "This mode is a balance between speed and security and is the default. ",
                flavor = "boolean",
                exclusivity = tls_protocol_preference,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_PROT_HIGH",
                brief = "High security mode",
                description = "Chooses the cipher in the order of AES256-SHA, AES128-SHA, RC4-SHA.\n\n"..
                              "This will use the strongest cipher(s) at the cost of speed. ",
                flavor = "boolean",
                exclusivity = tls_protocol_preference,
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_ENABLE_V23_HANDSHAKE",
                brief = "Enable v23 Handshake",
                description = "Some browsers use the v23 handshake client hello message "..
                              "(an SSL2 format message which all SSL servers can understand). "..
                              "It may be used if SSL2 is enabled in the browser.\n\n"..
                              "Since this feature takes a kB or so, this feature may be disabled - at "..
                              "the risk of making it incompatible with some browsers (IE6 is ok, "..
                              "Firefox 1.5 and below use it).\n\n"..
                              "Disable if backwards compatibility is not an issue (i.e. the client is "..
                              "always using TLS1.0)",
                flavor = "boolean",
                file = "include/cfg/tls.h",
            },

        },
    },
    {
        name = "nuttls_certificates",
        brief = "Certificate handling",
        description = "Certificate handling",
        requires = { "TLS_TLS1" },
        provides = { "TLS_TLS1_CERTIFICATES" },
        sources =
        {
            "gen_cert.c", "loader.c", "p12.c", "x509.c"
        },
        options =
        {
            --
            -- Certificate settings
            --
            {
                macro = "TLS_SSL_HAS_PEM",
                brief = "Enable PEM",
                description = "Enable the use of PEM format for certificates and private keys.\n\n"..
                               "PEM is not normally needed - PEM files can be converted into DER files "..
                               "quite easily. However they have the convenience of allowing multiple "..
                               "certificates/keys in the same file.\n\n"..
                               "This feature will add a couple of kB to the library.\n\n"..
                               "Disable if PEM is not used (which will be in most cases).",
                flavor = "boolean",
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_USE_PKCS12",
                brief = "Use PKCS8/PKCS12",
                description = "PKCS#12 certificates combine private keys and certificates "..
                              "together in one file.\n\n"..
                              "PKCS#8 private keys are also suppported (as it is a subset of PKCS#12).\n\n"..
                              "The decryption of these certificates uses RC4-128 (and these "..
                              "certificates must be encrypted using this cipher). The actual "..
                              "algorithm is 'PBE-SHA1-RC4-128'.\n\n"..
                              "Disable if PKCS#12 is not used (which will be in most cases).",

                flavor = "boolean",
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_EXPIRY_TIME",
                brief = "Session expiry time (in hours)",
                description = "The time (in hours) before a session expires.\n\n"..
                              "A longer time means that the expensive parts of a handshake don't "..
                              "need to be run when a client reconnects later.\n\n"..
                              "The default is 1 day.",
                default = 24,
                flavor = "integer",
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_X509_MAX_CA_CERTS",
                brief = "Maximum number of certificate authorites",
                description = "Determines the number of CA's allowed.\n\n"..
                              "Increase this figure if more trusted sites are allowed. Each "..
                              "certificate adds about 300 bytes (when added).\n\n"..
                              "The default is to allow the Debian cert bundle to be parsed.",
                default = 150,
                flavor = "integer",
                file = "include/cfg/tls.h",
            },
            {
                macro = "TLS_SSL_MAX_CERTS",
                brief = "Maximum number of chained certificates",
                description = "Determines the number of certificates used in a certificate "..
                              "chain. The chain length must be at least 1.\n\n"..
                              "Increase this figure if more certificates are to be added to the "..
                              "chain. Each certificate adds about 300 bytes (when added).\n\n"..
                              "The default is to allow one certificate + 2 certificates in the chain.",
                default = 3,
                flavor = "integer",
                file = "include/cfg/tls.h",
            },
        },
    },
}
