/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#ifndef _MCF5_H
#define _MCF5_H

#include <stdint.h>
#include <cfg/arch.h>

#if defined (MCU_MCF5225X)
#include <arch/m68k/coldfire/mcf5225x/mcf5225x.h>
#elif defined (MCU_MCF51CN)
#include <arch/m68k/coldfire/mcf51cn/mcf51cn.h>
#else
#warning "Unknown Coldfire MCU Family defined"
#endif

/*
 * Exception Stack Frame Definition
 *
 *        3322222222221111 111111
 *        1098765432109876 5432109876543210
 *       +----------------+----------------+
 * A7--> |Fmt/FS/Vector/FS|      SR        |
 *       +----------------+----------------+
 * +0x04 |         Program Counter         |
 *       +----------------+----------------+
 */
#define MCF5_RD_SF_FORMAT(PTR)   \
    ((*((uint16_t *)(PTR)) >> 12) & 0x00FF)

#define MCF5_RD_SF_VECTOR(PTR)   \
    ((*((uint16_t *)(PTR)) >>  2) & 0x00FF)

#define MCF5_RD_SF_FS(PTR)       \
    ( ((*((uint16_t *)(PTR)) & 0x0C00) >> 8) | (*((uint16_t *)(PTR)) & 0x0003) )

#define MCF5_SF_SR(PTR)  *((uint16_t *)(PTR)+1)
#define MCF5_SF_PC(PTR)  *((uint32_t *)(PTR)+1)

#endif  /* _MCF5_H */

