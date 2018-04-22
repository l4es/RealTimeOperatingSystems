#ifndef _ARCH_M68K_H_
#define _ARCH_M68K_H_

/*
 * Copyright (C) 2001-2004 by egnite Software GmbH. All rights reserved.
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

#include <cfg/arch.h>

#if defined (MCU_COLDFIRE)
#include <arch/m68k/coldfire/mcf5.h>
#else
#warning "Unknown M68K MCU Family defined"
#endif

#if !defined(__BIG_ENDIAN__)
#define __BIG_ENDIAN__
#endif

#define CONST   const
#define INLINE  inline

#define PSTR(p)    (p)
#define PRG_RDB(p) (*((const char *)(p)))

#define prog_char  const char
#define PGM_P      prog_char *

#define SIGNAL(x)  __attribute__((interrupt_handler)) void x(void)

#define strlen_P(x)             strlen(x)
#define strcpy_P(x,y)           strcpy(x,y)
#define strncpy_P(x,y,z)        strncpy(x,y,z)
#define strcmp_P(x, y)          strcmp(x, y)
#define strcasecmp_P(x,y)       strcasecmp(x,y)
#define strstr_P(x,y)           strstr(x,y)
#define memcpy_P(x,y,z)         memcpy(x,y,z)

#define outb(_reg, _val)  (*((volatile uint8_t *)(_reg)) = (_val))
#define outw(_reg, _val)  (*((volatile uint16_t *)(_reg)) = (_val))
#define outr(_reg, _val)  (*((volatile uint32_t *)(_reg)) = (_val))

#define inb(_reg)   (*((volatile uint8_t *)(_reg)))
#define inw(_reg)   (*((volatile uint16_t *)(_reg)))
#define inr(_reg)   (*((volatile uint32_t *)(_reg)))

#define _BV(bit)    (1 << (bit))

#ifndef _NOP
#define _NOP() __asm__ __volatile__ ("nop")
#endif

#endif
