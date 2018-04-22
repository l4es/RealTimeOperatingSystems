/*
 * Copyright (c) 2007-2015, Cameron Rich
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the axTLS project nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file crypto.h
 */

#ifndef HEADER_CRYPTO_H
#define HEADER_CRYPTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cfg/arch.h>
#include <stdint.h>
#include <toolchain.h>
#include <sys/types.h>

#if defined(__AVR__)
#define CRYPTO_INTEGER_8BIT
#else
#define CRYPTO_INTEGER_32BIT
#endif

#include <cfg/crypto.h>
#include <crypto/bigint_impl.h>
#include <crypto/bigint.h>

/**************************************************************************
 * AES declarations
 **************************************************************************/

#define AES_MAXROUNDS           14
#define AES_BLOCKSIZE           16
#define AES_IV_SIZE             16

typedef struct aes_key_st
{
    uint16_t rounds;
    uint16_t key_size;
    uint32_t ks[(AES_MAXROUNDS+1)*8];
    uint8_t iv[AES_IV_SIZE];
} AES_CTX;

typedef enum
{
    AES_MODE_128,
    AES_MODE_256
} AES_MODE;

extern void AES_set_key(AES_CTX *ctx, const uint8_t *key,
        const uint8_t *iv, AES_MODE mode);
extern void AES_cbc_encrypt(AES_CTX *ctx, const uint8_t *msg,
        uint8_t *out, int length);
extern void AES_cbc_decrypt(AES_CTX *ks, const uint8_t *in, uint8_t *out, int length);
extern void AES_convert_key(AES_CTX *ctx);

/**************************************************************************
 * RC4 declarations
 **************************************************************************/

typedef struct
{
    uint8_t x, y, m[256];
} RC4_CTX;

extern void RC4_setup(RC4_CTX *s, const uint8_t *key, int length);
extern void RC4_crypt(RC4_CTX *s, const uint8_t *msg, uint8_t *data, int length);

/**************************************************************************
 * SHA1 declarations
 **************************************************************************/

#define SHA1_SIZE   20

/*
 *  This structure will hold context information for the SHA-1
 *  hashing operation
 */
typedef struct
{
    uint32_t Intermediate_Hash[SHA1_SIZE/4]; /* Message Digest */
    uint32_t Length_Low;            /* Message length in bits */
    uint32_t Length_High;           /* Message length in bits */
    uint16_t Message_Block_Index;   /* Index into message block array   */
    uint8_t Message_Block[64];      /* 512-bit message blocks */
} SHA1_CTX;

extern void SHA1_Init(SHA1_CTX *);
extern void SHA1_Update(SHA1_CTX *, const uint8_t * msg, int len);
extern void SHA1_Final(uint8_t *digest, SHA1_CTX *);

/**************************************************************************
 * SHA256 declarations
 **************************************************************************/

#define SHA256_SIZE   32

typedef struct
{
    uint32_t total[2];
    uint32_t state[8];
    uint8_t buffer[64];
} SHA256_CTX;

void SHA256_Init(SHA256_CTX *c);
void SHA256_Update(SHA256_CTX *, const uint8_t *input, int len);
void SHA256_Final(uint8_t *digest, SHA256_CTX *);

 /**************************************************************************
 * SHA512 declarations
 **************************************************************************/

#define SHA512_SIZE   64

typedef struct
{
    union
    {
        uint64_t h[8];
        uint8_t digest[64];
    } h_dig;
    union
    {
        uint64_t w[80];
        uint8_t buffer[128];
    } w_buf;
    size_t size;
    uint64_t totalSize;
} SHA512_CTX;

void SHA512_Init(SHA512_CTX *c);
void SHA512_Update(SHA512_CTX *, const uint8_t *input, int len);
void SHA512_Final(uint8_t *digest, SHA512_CTX *);

/**************************************************************************
 * SHA384 declarations
 **************************************************************************/

#define SHA384_SIZE   48

typedef SHA512_CTX SHA384_CTX;
void SHA384_Init(SHA384_CTX *c);
void SHA384_Update(SHA384_CTX *, const uint8_t *input, int len);
void SHA384_Final(uint8_t *digest, SHA384_CTX *);


/**************************************************************************
 * MD5 declarations
 **************************************************************************/

#define MD5_SIZE    16

typedef struct
{
  uint32_t state[4];        /* state (ABCD) */
  uint32_t count[2];        /* number of bits, modulo 2^64 (lsb first) */
  uint8_t buffer[64];       /* input buffer */
} MD5_CTX;

void MD5_Init(MD5_CTX *);
void MD5_Update(MD5_CTX *, const uint8_t *msg, int len);
void MD5_Final(uint8_t *digest, MD5_CTX *);

/**************************************************************************
 * HMAC declarations
 **************************************************************************/
extern void hmac_md5(const uint8_t *msg, int length, const uint8_t *key,
        int key_len, uint8_t *digest);
extern void hmac_sha1(const uint8_t *msg, int length, const uint8_t *key,
        int key_len, uint8_t *digest);

/**************************************************************************
 * RSA declarations
 **************************************************************************/

typedef struct
{
    bigint *m;              /* modulus */
    bigint *e;              /* public exponent */
    bigint *d;              /* private exponent */
#ifdef CRYPTO_BIGINT_CRT
    bigint *p;              /* p as in m = pq */
    bigint *q;              /* q as in m = pq */
    bigint *dP;             /* d mod (p-1) */
    bigint *dQ;             /* d mod (q-1) */
    bigint *qInv;           /* q^-1 mod p */
#endif
    int num_octets;
    BI_CTX *bi_ctx;
} RSA_CTX;

extern void RSA_priv_key_new(RSA_CTX **rsa_ctx,
        const uint8_t *modulus, int mod_len,
        const uint8_t *pub_exp, int pub_len,
        const uint8_t *priv_exp, int priv_len
#ifdef CRYPTO_BIGINT_CRT
      , const uint8_t *p, int p_len,
        const uint8_t *q, int q_len,
        const uint8_t *dP, int dP_len,
        const uint8_t *dQ, int dQ_len,
        const uint8_t *qInv, int qInv_len
#endif
        );
extern void RSA_pub_key_new(RSA_CTX **rsa_ctx,
        const uint8_t *modulus, int mod_len,
        const uint8_t *pub_exp, int pub_len);
extern void RSA_free(RSA_CTX *ctx);
extern int RSA_decrypt(const RSA_CTX *ctx, const uint8_t *in_data, uint8_t *out_data,
        int out_len, int is_decryption);
extern bigint *RSA_private(const RSA_CTX *c, bigint *bi_msg);
extern bigint *RSA_sign_verify(BI_CTX *ctx, const uint8_t *sig, int sig_len,
        bigint *modulus, bigint *pub_exp);
extern bigint *RSA_public(const RSA_CTX * c, bigint *bi_msg);
extern int RSA_encrypt(const RSA_CTX *ctx, const uint8_t *in_data, uint16_t in_len,
        uint8_t *out_data, int is_signing);
extern void RSA_print(const RSA_CTX *ctx);

/**************************************************************************
 * RNG declarations
 **************************************************************************/
extern void RNG_initialize(void);
extern void RNG_custom_init(const uint8_t *seed_buf, int size);
extern void RNG_terminate(void);
extern int get_random(int num_rand_bytes, uint8_t *rand_data);
extern int get_random_NZ(int num_rand_bytes, uint8_t *rand_data);

# ifdef __cplusplus
}
# endif

#endif
