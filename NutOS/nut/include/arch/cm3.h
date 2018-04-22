#ifndef _ARCH_CM3_H_
#define _ARCH_CM3_H_

/*
 * Copyright (C) 2001-2006 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2012-2014 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de).
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
 * $Log$
 * Revision 1.00  2010/08/06 09:34:34  ulrichprinz
 * Initial version.
 */

#include <stddef.h>
#include <cfg/arch.h>
#include <arch/cm3/cortexM3.h>

#ifndef __ASSEMBLER__
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

#ifdef __GNUC__
#ifndef CONST
#define CONST      const
#endif
#ifndef INLINE
#define INLINE     inline
#endif
#else
#ifndef CONST
#define CONST      const
#endif
#ifndef INLINE
#define INLINE
#endif
#endif

#define PSTR(p)    (p)
#define PRG_RDB(p) (*((const char *)(p)))

#define prog_char  const char
#define PGM_P      prog_char *

#define SIGNAL(x)  __attribute__((interrupt_handler)) void x(void)

#if !defined(__arm__) && !defined(__cplusplus)
#define main       NutAppMain
#endif

#define strlen_P(x)             strlen((char *)(x))
#define strcpy_P(x,y)           strcpy(x,(char *)(y))

#define strcmp_P(x, y)          strcmp((char *)(x), (char *)(y))
#define memcpy_P(x, y, z)       memcpy(x, y, z)

#ifndef __ASSEMBLER__
/*!
 * \brief End of uninitialised data segment. Defined in the linker script.
 */
extern void *__bss_end;

/*!
 * \brief Begin of the stack segment. Defined in the linker script.
 */
extern void *__stack;
#endif

#ifndef _NOP
#ifdef __GNUC__
#define _NOP() __asm__ __volatile__ ("mov r0, r0  @ _NOP")
#else
#define _NOP() asm("mov r0, r0")
#endif
#endif

#define outb(_reg, _val)  (*((volatile unsigned char *)(_reg)) = (_val))
#define outw(_reg, _val)  (*((volatile unsigned short *)(_reg)) = (_val))
#define outr(_reg, _val)  (*((volatile unsigned int *)(_reg)) = (_val))

#define inb(_reg)   (*((volatile unsigned char *)(_reg)))
#define inw(_reg)   (*((volatile unsigned short *)(_reg)))
#define inr(_reg)   (*((volatile unsigned int *)(_reg)))

#define _BV(bit)    (1 << (bit))

#define sbi(_reg, _bit)         outr(_reg, inr(_reg) | _BV(_bit))
#define cbi(_reg, _bit)         outr(_reg, inr(_reg) & ~_BV(_bit))
#define bit_is_set(_reg, _bit)  ((inr(_reg) & _BV(_bit)) != 0)
#define bit_is_clear(_reg, _bit) ((inr(_reg) & _BV(_bit)) == 0)

/*!
 * \brief Get the Bit position index of the highest bit from a bit value
 *
 */
#define  _BI2(arg) (((arg) & 0x00000002) ? 1: 0)
#define  _BI4(arg) (((arg) & 0x0000000c) ? ( _BI2(arg>> 2) +  2) :  _BI2(arg))
#define  _BI8(arg) (((arg) & 0x000000f0) ? ( _BI4(arg>> 4) +  4) :  _BI4(arg))
#define _BI16(arg) (((arg) & 0x0000ff00) ? ( _BI8(arg>> 8) +  8) :  _BI8(arg))
#if defined __builtin_clz
#define _BI32(arg) (31 - __builtin_clz(arg))
#else
#define _BI32(arg) (((arg) & 0xffff0000) ? (_BI16(arg>>16) + 16) : _BI16(arg))
#endif

/*!
 * \brief Get the address of a device register by its base and the offset  of the register
 * in its register structure
 *
 * CortexM specific:
 */
#define CM3ADDR(base, regstruct, reg) ((base) + offsetof(regstruct, reg))

/*!
 * \brief Atomic access via register address of CortexM devices.
 *
 * CortexM specific:
 * Translates a register address into a volatile single cycle
 * read or write access of the register.
 *
 * Constant base part of address allows room for compiler optimization
 */
#define CM3MEM(addr)   *((volatile unsigned long *)(addr))
#define CM3MEM16(addr) *((volatile uint16_t *)     (addr))
#define CM3MEM8(addr)  *((volatile uint8_t *)      (addr))

#define CM3REG(base, regstruct, reg)   ((regstruct *)(base))->reg

/*!
 * \brief Atomic bit access via bitband address of CortexM devices.
 *
 * CortexM specific:
 * Translates a register address into a volatile single cycle
 * read or write access of the register.
 *
 * Constant base part of address allows room for compiler optimization.
 *
 * Be sure to handle bit values >> 32!
 */
#if defined (MCU_CM_NO_BITBAND)
/* We can't map this macro, to lets abort compilation here if we ever use it */
#else
#define CM3BBREG(base, regstruct, reg, bit) *((volatile uint32_t *) &(((uint8_t *) ((base & 0xF0000000) + 0x02000000 + ((base & 0xFFFFF)<<5))) [(offsetof(regstruct, reg) <<5) + ((bit) <<2)] ) )
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BBSET(base, regstruct, reg, bit) (*(volatile uint32_t *)((base) + offsetof(regstruct, reg) + ((bit)/32) * 4) |=  (1<<((bit)%32)))
#else
#define CM3BBSET(base, regstruct, reg, bit) (*((volatile uint32_t *) &(((uint8_t *) ((base & 0xF0000000) + 0x02000000 + ((base & 0xFFFFF)<<5))) [(offsetof(regstruct, reg) <<5) + ((bit) <<2)] ) ) = 1)
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BBCLR(base, regstruct, reg, bit) (*(volatile uint32_t *)((base) + offsetof(regstruct, reg) + ((bit)/32) * 4) &= ~(1<<((bit)%32)))
#else
#define CM3BBCLR(base, regstruct, reg, bit) (*((volatile uint32_t *) &(((uint8_t *) ((base & 0xF0000000) + 0x02000000 + ((base & 0xFFFFF)<<5))) [(offsetof(regstruct, reg) <<5) + ((bit) <<2)] ) ) = 0)
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BBGET(base, regstruct, reg, bit) ((*(volatile uint32_t *)((base) + offsetof(regstruct, reg) + ((bit)/32) * 4) & (1<<((bit)%32))) == (1<<((bit)%32)))
#else
#define CM3BBGET(base, regstruct, reg, bit) (*((volatile uint32_t *) &(((uint8_t *) ((base & 0xF0000000) + 0x02000000 + ((base & 0xFFFFF)<<5))) [(offsetof(regstruct, reg) <<5) + ((bit) <<2)] ) ))
#endif

#if defined (MCU_CM_NO_BITBAND)
/* We can't map this macro, to lets abort compilation here if we ever use it */
#else
#define CM3BBADDR(base, regstruct, reg, bit) ((volatile uint32_t *) &(((uint8_t *) ((base & 0xF0000000) + 0x02000000 + ((base & 0xFFFFF)<<5))) [(offsetof(regstruct, reg) <<5) + ((bit) <<2)] ) )
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BBSETVAL(base, regstruct, reg, bit, value) (value)?CM3BBSET(base, regstruct, reg, bit):CM3BBCLR(base, regstruct, reg, bit)
#else
#define CM3BBSETVAL(base, regstruct, reg, bit, value) (*((volatile uint32_t *) &(((uint8_t *) ((base & 0xF0000000) + 0x02000000 + ((base & 0xFFFFF)<<5))) [(offsetof(regstruct, reg) <<5) + ((bit) <<2)] ) ) = (value)?1:0)
#endif

/*!
 * \brief Get Base Address of the Bitband region belonging to Device Register structrure
 *
 */
#if defined (MCU_CM_NO_BITBAND)
/* Reproduce the base address for the following macros */
#define CM3BB_BASE(base) ((volatile uint32_t *)base)
#else
#define CM3BB_BASE(base) (volatile uint32_t *) (((uint32_t)base & 0xF0000000) + 0x02000000 + (((uint32_t)base & 0xFFFFF)<<5))
#endif

/*!
 * \brief Get Offset of Bitband Bit in the (uint32_t*) Bitband Array
 *
 * CM3/4 can do immediate offset access for -255, +4095 bytes around a base. So bits in the first 32 32-bit
 * registers above the base can be reached witout loading an absolute address in a bitband access
 *
 */
#if defined (MCU_CM_NO_BITBAND)
/* We can't map this macro, to lets abort compilation here if we ever use it */
#else
#define CM3BB_OFFSET(regstruct, reg, bit) ((offsetof(regstruct, reg) <<3) + bit)
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BB_OFFSETSET(bb_base, regstruct, reg, flag) ((regstruct*)bb_base)->reg |= (flag)
#else
#define CM3BB_OFFSETSET(bb_base, regstruct, reg, flag) (((volatile uint32_t *)bb_base)[CM3BB_OFFSET(regstruct, reg, _BI32(flag))] = 1)
/* Without bitband support set bb_base to base of register structure and use
 * ((regstruct*)bb_base)->reg |= (bit)
 */
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BB_OFFSETCLR(bb_base, regstruct, reg, flag) ((regstruct*)bb_base)->reg &= ~(flag)
#else
#define CM3BB_OFFSETCLR(bb_base, regstruct, reg, flag) (((volatile uint32_t *)bb_base)[CM3BB_OFFSET(regstruct, reg,  _BI32(flag))] = 0)
/* Without bitband support set bb_base to base of register structure and use
 * ((regstruct*)bb_base)->reg &= ~(bit)
 */
#endif

#if defined (MCU_CM_NO_BITBAND)
#define CM3BB_OFFSETGET(bb_base, regstruct, reg, flag) ((((regstruct*)bb_base)->reg & (flag)) == (flag))
#else
#define CM3BB_OFFSETGET(bb_base, regstruct, reg, flag) (((volatile uint32_t *)bb_base)[CM3BB_OFFSET(regstruct, reg, _BI32(flag))])
/* Without bitband support set bb_base to base of register structure and use
 * ((regstruct*)bb_base)->reg & (bit)
 */
#endif

#ifdef __IMAGECRAFT__
#define __attribute__(x)
#endif

#define _SFR_MEM8(addr)     (addr)
#define _SFR_MEM16(addr)    (addr)

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

#if !defined (__ASSEMBLER__) && defined(__CROSSWORKS_ARM)
/*!
 * \brief Case insensitive string comparisions.
 *
 * Not supported by CrossWorks and temporarly redirected
 * to the stricmp and strnicmp routines.
 *
 */
#define strcasecmp(s1, s2)      stricmp(s1, s2)
#define strncasecmp(s1, s2, n)  strnicmp(s1, s2, n)

/*
 * Not supported by CrossWorks, added prototypes here.
 */
int   stricmp(const char *s1, const char *s2);
int   strnicmp(const char *s1, const char *s2, size_t n);
char *strdup(const char *str);

/*
 * If "Enforce ANSI Checking" is enabled, which is
 * the default from the v2.x version of CrossWorks
 * the keyword asm will not be recognize. Therefore
 * the next define is needed to solve the problem.
 */
#define asm __asm__
#endif

#endif /* _ARCH_CM3_H_ */
