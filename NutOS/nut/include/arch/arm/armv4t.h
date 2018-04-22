#ifndef _ARCH_ARM_ARMV4T_H_
#define _ARCH_ARM_ARMV4T_H_

/*
 * Copyright 2011 by egnite GmbH
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

/*!
 * \file arch/arm/armv4t.h
 * \brief ARMv4T architecture definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmV4t
 */
/*@{*/

#if defined (MCU_AT91)
#include <arch/arm/at91.h>
#elif defined (MCU_GBA)
#include <arch/arm/gba.h>
#elif defined (MCU_LPC2XXX)
#include <arch/arm/lpc2xxx.h>
#endif

#ifndef __ASSEMBLER__
#include <stdint.h>
#include <dev/mweeprom.h>
#endif

#define ARM_MODE_USER       0x10
#define ARM_MODE_FIQ        0x11
#define ARM_MODE_IRQ        0x12
#define ARM_MODE_SVC        0x13
#define ARM_MODE_ABORT      0x17
#define ARM_MODE_UNDEF      0x1B
#define ARM_MODE_SYS        0x1F
#define ARM_MODE_MASK       0x1F

#define I_BIT               0x80
#define ARM_CPSR_I_BIT      0x80
#define F_BIT               0x40
#define ARM_CPSR_F_BIT      0x40
#define T_BIT               0x20
#define ARM_CPSR_T_BIT      0x20

#ifndef __ASSEMBLER__
#ifdef __GNUC__
#define ARM_SET_CP15_CR(val) __asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0" :: "r"(val) : "cc")
#define ARM_GET_CP15_CR() ( \
    { \
        unsigned int val; \
        __asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0" : "=r"(val) :: "cc"); \
        val; \
    } \
)
#endif /* __GNUC__ */
#endif /* __ASSEMBLER__ */


#endif
