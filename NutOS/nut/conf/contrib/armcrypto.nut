
nutcontrib_armcrypto =
{
    {
        name = "nutcontrib_arm_crypto_aes",
        brief = "AES",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/aes/aes_dec.c",
            "arm-crypto-lib/aes/aes_enc.c",
            "arm-crypto-lib/aes/aes_invsbox.c",
            "arm-crypto-lib/aes/aes_keyschedule.c",
            "arm-crypto-lib/aes/aes_sbox.c",
            "arm-crypto-lib/aes/aes128_dec.c",
            "arm-crypto-lib/aes/aes128_enc.c",
            "arm-crypto-lib/aes/aes192_dec.c",
            "arm-crypto-lib/aes/aes192_enc.c",
            "arm-crypto-lib/aes/aes256_dec.c",
            "arm-crypto-lib/aes/aes256_enc.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_arcfour",
        brief = "ARCFOUR (RC4)",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/arcfour/arcfour.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_base64",
        brief = "Base64",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/base64/base64_dec.c",
            "arm-crypto-lib/base64/base64_enc.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_bcal",
        brief = "Block Cipher Abstraction Layer",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/bcal/bcal-basic.c",
            "arm-crypto-lib/bcal/bcal_aes128.c",
            "arm-crypto-lib/bcal/bcal_aes192.c",
            "arm-crypto-lib/bcal/bcal_aes256.c",
            "arm-crypto-lib/bcal/bcal_cast5.c",
            "arm-crypto-lib/bcal/bcal_cast6.c",
            "arm-crypto-lib/bcal/bcal_cscipher.c",
            "arm-crypto-lib/bcal/bcal_des.c",
            "arm-crypto-lib/bcal/bcal_khazad.c",
            "arm-crypto-lib/bcal/bcal_noekeon.c",
            "arm-crypto-lib/bcal/bcal_present128.c",
            "arm-crypto-lib/bcal/bcal_present80.c",
            "arm-crypto-lib/bcal/bcal_rc5.c",
            "arm-crypto-lib/bcal/bcal_rc6.c",
            "arm-crypto-lib/bcal/bcal_seed.c",
            "arm-crypto-lib/bcal/bcal_serpent.c",
            "arm-crypto-lib/bcal/bcal_tdes.c",
            "arm-crypto-lib/bcal/bcal_tdes2.c",
            "arm-crypto-lib/bcal/bcal_xtea.c",
            "arm-crypto-lib/bcal/keysize_descriptor.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_bigint",
        brief = "Big Integers",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/bigint/bigint.c",
            "arm-crypto-lib/bigint/bigint_io.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_blake",
        brief = "BLAKE",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/blake/blake_common.c",
            "arm-crypto-lib/blake/blake_large.c",
            "arm-crypto-lib/blake/blake_small.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_bmw",
        brief = "BMW",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/bmw/bmw_large.c",
            "arm-crypto-lib/bmw/bmw_small.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_cast5",
        brief = "CAST5",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/cast5/cast5.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_cast6",
        brief = "CAST6",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/cast6/cast6.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_cscipher",
        brief = "CS-Cipher",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/cscipher/cscipher_sbox.c",
            "arm-crypto-lib/cscipher/cscipher_small.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_cubehash",
        brief = "CubeHash",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/cubehash/cubehash.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_des",
        brief = "DES",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/des/des.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_dsa",
        brief = "DSA",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/dsa/dsa_key_blob.c",
            "arm-crypto-lib/dsa/dsa_sign.c",
            "arm-crypto-lib/dsa/dsa_verify.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_echo",
        brief = "ECHO",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/echo/aes_enc_round.c",
            "arm-crypto-lib/echo/echo.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_entropium",
        brief = "Entropium",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/entropium/entropium.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_gf256mul",
        brief = "GF(256) Multiplication",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/gf256mul/gf256mul.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_grain",
        brief = "Grain",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/grain/grain.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_groestl",
        brief = "Grostl",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/groestl/groestl_large.c",
            "arm-crypto-lib/groestl/groestl_small.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_hfal",
        brief = "Hash Function Abstraction Layer",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/hfal/hfal-basic.c",
            "arm-crypto-lib/hfal/hfal-hmac.c",
            "arm-crypto-lib/hfal/hfal_blake_large.c",
            "arm-crypto-lib/hfal/hfal_blake_small.c",
            "arm-crypto-lib/hfal/hfal_bmw_large.c",
            "arm-crypto-lib/hfal/hfal_bmw_small.c",
            "arm-crypto-lib/hfal/hfal_cubehash.c",
            "arm-crypto-lib/hfal/hfal_echo.c",
            "arm-crypto-lib/hfal/hfal_groestl_large.c",
            "arm-crypto-lib/hfal/hfal_groestl_small.c",
            "arm-crypto-lib/hfal/hfal_jh.c",
            "arm-crypto-lib/hfal/hfal_keccak.c",
            "arm-crypto-lib/hfal/hfal_md5.c",
            "arm-crypto-lib/hfal/hfal_sha1.c",
            "arm-crypto-lib/hfal/hfal_sha224.c",
            "arm-crypto-lib/hfal/hfal_sha256.c",
            "arm-crypto-lib/hfal/hfal_sha384.c",
            "arm-crypto-lib/hfal/hfal_sha512.c",
            "arm-crypto-lib/hfal/hfal_shabal.c",
            "arm-crypto-lib/hfal/hfal_skein1024.c",
            "arm-crypto-lib/hfal/hfal_skein256.c",
            "arm-crypto-lib/hfal/hfal_skein512.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_jh",
        brief = "JH",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/jh/jh_simple_aux.c",
            "arm-crypto-lib/jh/jh_simple_small_core.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_keccak",
        brief = "Keccak",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/keccak/keccak.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_khazad",
        brief = "KHAZAD",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/khazad/khazad.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_md5",
        brief = "MD5",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/md5/md5.c",
            "arm-crypto-lib/md5/md5_sbox.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_memxor",
        brief = "Memory XOR",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/memxor/memxor.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_mgf1",
        brief = "MFG1",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/mgf1/mgf1.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_mickey128",
        brief = "MICKEY-128",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/mickey128/mickey128.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_noekeon",
        brief = "NOEKEON",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/noekeon/noekeon.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_present",
        brief = "PRESENT",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/present/present_common.c",
            "arm-crypto-lib/present/present128.c",
            "arm-crypto-lib/present/present80.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_prf_tls12",
        brief = "TLS 1.2 PRF",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/prf_tls12/prf_tls12.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_rabbit",
        brief = "Rabbit",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/rabbit/rabbit_c.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_rc5",
        brief = "RC5",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/rc5/rc5.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_rc6",
        brief = "RC6",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/rc6/rc6.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_rsa",
        brief = "RSA",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/rsa/rsa_basic.c",
            "arm-crypto-lib/rsa/rsaes_oaep.c",
            "arm-crypto-lib/rsa/rsaes_pkcs1v15.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_salsa20",
        brief = "Salsa20",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/salsa20/salsa20.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_scal",
        brief = "Stream Cipher Abstraction Layer",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/scal/scal-basic.c",
            "arm-crypto-lib/scal/scal_arcfour.c",
            "arm-crypto-lib/scal/scal_grain.c",
            "arm-crypto-lib/scal/scal_mickey128.c",
            "arm-crypto-lib/scal/scal_rabbit.c",
            "arm-crypto-lib/scal/scal_salsa20.c",
            "arm-crypto-lib/scal/scal_trivium.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_seed",
        brief = "SEED",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/seed/seed_c.c",
            "arm-crypto-lib/seed/seed_sbox.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_serpent",
        brief = "Serpent",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/serpent/serpent-sboxes-bitslice.c",
            "arm-crypto-lib/serpent/serpent.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_sha1",
        brief = "SHA-1",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/sha1/sha1.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_sha2",
        brief = "SHA-2",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/sha2/sha2_large_common.c",
            "arm-crypto-lib/sha2/sha2_small_common.c",
            "arm-crypto-lib/sha2/sha224.c",
            "arm-crypto-lib/sha2/sha256.c",
            "arm-crypto-lib/sha2/sha384.c",
            "arm-crypto-lib/sha2/sha512.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_shabal",
        brief = "Shabal",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/shabal/shabal.c",
            "arm-crypto-lib/shabal/shabal192.c",
            "arm-crypto-lib/shabal/shabal224.c",
            "arm-crypto-lib/shabal/shabal256.c",
            "arm-crypto-lib/shabal/shabal384.c",
            "arm-crypto-lib/shabal/shabal512.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_skein",
        brief = "Skein",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/skein/skein1024.c",
            "arm-crypto-lib/skein/skein256.c",
            "arm-crypto-lib/skein/skein512.c",
            "arm-crypto-lib/skein/threefish_mix_c.c",
            "arm-crypto-lib/skein/threefish1024_enc.c",
            "arm-crypto-lib/skein/threefish256_enc.c",
            "arm-crypto-lib/skein/threefish512_enc.c",
            "arm-crypto-lib/skein/ubi1024.c",
            "arm-crypto-lib/skein/ubi256.c",
            "arm-crypto-lib/skein/ubi512.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_trivium",
        brief = "Trivium",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/trivium/trivium.c"
        },
    },
    {
        name = "nutcontrib_arm_crypto_xtea",
        brief = "XTEA",
        requires = { "LICENSE_GPL3" },
        sources =
        {
            "arm-crypto-lib/xtea/xtea.c"
        },
    },
}
