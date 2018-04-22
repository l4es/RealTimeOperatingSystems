#ifndef _AES_H_
#define _AES_H_

/*
 * Copyright (c) 1998-2008, Brian Gladman, Worcester, UK.
 *
 * Copyright 2012, Ole Reinhardt <ole.reinhardt@embedded-it.de>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*
 * \file include/gorp/aes.h
 *
 * \brief Brian Gladmans byte oriented implementation of the AES crypto algorithm
 *
 * Issue 09/09/2006
 *
 * This is an AES implementation that uses only 8-bit byte operations on the
 * cipher state (there are options to use 32-bit types if available).
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/aes.h>

#define N_ROW                   4
#define N_COL                   4
#define N_BLOCK   (N_ROW * N_COL)
#define N_MAX_ROUNDS           14

typedef uint8_t uint_8t;

typedef uint_8t return_type;

/*  Warning: The key length for 256 bit keys overflows a byte
    (see comment below)
*/

typedef struct
{   uint_8t ksch[(N_MAX_ROUNDS + 1) * N_BLOCK];
    uint_8t rnd;
} aes_context;

/*  The following calls are for a precomputed key schedule

    NOTE: If the length_type used for the key length is an
    unsigned 8-bit character, a key length of 256 bits must
    be entered as a length in bytes (valid inputs are hence
    128, 192, 16, 24 and 32).
*/

#if defined( AES_ENC_PREKEYED ) || defined( AES_DEC_PREKEYED )

return_type aes_set_key( const uint8_t key[],
                         int keylen,
                         aes_context ctx[1] );
#endif

#if defined( AES_ENC_PREKEYED )

return_type aes_encrypt( const uint8_t in[N_BLOCK],
                         uint8_t out[N_BLOCK],
                         const aes_context ctx[1] );

return_type aes_cbc_encrypt( const uint8_t *in,
                         uint8_t *out,
                         int n_block,
                         uint8_t iv[N_BLOCK],
                         const aes_context ctx[1] );
#endif

#if defined( AES_DEC_PREKEYED )

return_type aes_decrypt( const uint8_t in[N_BLOCK],
                         uint8_t out[N_BLOCK],
                         const aes_context ctx[1] );

return_type aes_cbc_decrypt( const uint8_t *in,
                         uint8_t *out,
                         int n_block,
                         uint8_t iv[N_BLOCK],
                         const aes_context ctx[1] );
#endif

/*  The following calls are for 'on the fly' keying.  In this case the
    encryption and decryption keys are different.

    The encryption subroutines take a key in an array of bytes in
    key[L] where L is 16, 24 or 32 bytes for key lengths of 128,
    192, and 256 bits respectively.  They then encrypts the input
    data, in[] with this key and put the reult in the output array
    out[].  In addition, the second key array, o_key[L], is used
    to output the key that is needed by the decryption subroutine
    to reverse the encryption operation.  The two key arrays can
    be the same array but in this case the original key will be
    overwritten.

    In the same way, the decryption subroutines output keys that
    can be used to reverse their effect when used for encryption.

    Only 128 and 256 bit keys are supported in these 'on the fly'
    modes.
*/

#if defined( AES_ENC_128_OTFK )
void aes_encrypt_128( const uint8_t in[N_BLOCK],
                      uint8_t out[N_BLOCK],
                      const uint8_t key[N_BLOCK],
                      uint_8t o_key[N_BLOCK] );
#endif

#if defined( AES_DEC_128_OTFK )
void aes_decrypt_128( const uint8_t in[N_BLOCK],
                      uint8_t out[N_BLOCK],
                      const uint8_t key[N_BLOCK],
                      uint8_t o_key[N_BLOCK] );
#endif

#if defined( AES_ENC_256_OTFK )
void aes_encrypt_256( const uint8_t in[N_BLOCK],
                      uint8_t out[N_BLOCK],
                      const uint8_t key[2 * N_BLOCK],
                      uint8_t o_key[2 * N_BLOCK] );
#endif

#if defined( AES_DEC_256_OTFK )
void aes_decrypt_256( const uint8_t in[N_BLOCK],
                      uint8_t out[N_BLOCK],
                      const uint8_t key[2 * N_BLOCK],
                      uint8_t o_key[2 * N_BLOCK] );
#endif

#endif
