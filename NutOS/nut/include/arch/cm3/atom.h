/*
 * Copyright (C) 2001-2005 by egnite Software GmbH. All rights reserved.
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
 *
 */

/*
 * $Log$
 * Revision 1.0  2010/08/04 11:26:47  ulrichprinz
 * Added Nikolays Cortex M3 port
 *
 *
 */

#ifndef _SYS_ATOM_H_
#error "Do not include this file directly. Use sys/atom.h instead!"
#endif

#ifdef __GNUC__

#define NutEnterCritical() \
{ \
    asm volatile (  \
        "@ NutEnterCritical"    "\n\t" \
        "mrs     r0, PRIMASK"   "\n\t" \
        "cpsid   i"             "\n\t" \
        :::"r0" \
    ); \
}

#define NutExitCritical() \
    {\
        asm volatile ( \
        "@ NutExitCritical"     "\n\t" \
        "mrs     r0, PRIMASK"   "\n\t" \
        "cpsie   i"             "\n\t" \
        :::"r0" \
    ); \
}

#define NutJumpOutCritical()    NutExitCritical()

#else /* __IMAGECRAFT__ */

#define NutEnterCritical() \
    asm("; NutEnterCritical\n" \
        "mrs r12, apsr\n" \
        "orr r12, r12, #0xC0\n" \
        "msr psr_c, r12")

#define NutExitCritical() \
    asm("; NutExitCritical\n" \
        "mrs r12, apsr\n" \
        "bic r12, r12, #0xC0\n" \
        "msr psr_c, r12")

#endif
