#ifndef _TOOLCHAIN_GENERIC_H_
#define _TOOLCHAIN_GENERIC_H_

/*
 * Copyright (C) 2012 by egnite GmbH
 * Copyright (C) 2001-2005 by egnite Software GmbH
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

#ifndef _TOOLCHAIN_H_
#error Do not include this file directly, include <toolchain.h> instead.
#endif

#include <stdint.h>

/*!
 * \name Convenience Macros
 */
/*@{*/

#ifndef _BV
/*!
 * \brief Get bit mask for a given bit number.
 *
 * Using the convenience macro _BV(bit) is equivalent to (1 << bit),
 * but often makes the code more readable.
 *
 * Initially it had been provided by avr-libc, but was declared
 * deprecated and finally removed in later versions. Among Nut/OS
 * developers it is still preferred to the bitwise left shift and
 * used on all platforms.
 */
#define _BV(bit)    (1 << (bit))
#endif

#ifndef BV
/* Deprecated version. */
#define BV(bit) _BV(bit)
#endif

/*@}*/

#ifndef INLINE
/*!
 * \brief Deprecated inline function macro.
 *
 * This macro is functionally equivalent to \ref NUT_INLINE_FUNC and
 * kept for backward compatibility.
 */
#define INLINE  NUT_INLINE_FUNC
#endif

/*!
 * \name Traditional Register Access Macros
 */
/*@{*/

#ifndef _SFR_MEM8
#define _SFR_MEM8(addr)     (addr)
#endif

#ifndef _SFR_MEM16
#define _SFR_MEM16(addr)    (addr)
#endif

#ifndef outb
/*!
 * \brief Write an 8-bit value to an I/O register.
 *
 * The default version assumes memory mapped registers and therefore
 * writes to a memory location. This default behavior may be overridden
 * by a target specific implementation.
 *
 * A special version is provided for 8-bit AVR targets to generate
 * optimized instruction code for registers in IO space. However,
 * to make this work, the register address must be a constant.
 *
 * \todo Although heavily used in Nut/OS, it should be replaced in long
 *       term by XXX to avoid problems with compiler optimization.
 */
#define outb(_reg, _val)  (*((volatile unsigned char *)(_reg)) = (_val))
#endif

#ifndef outw
/*!
 * \brief Write an 16-bit value to an I/O register.
 *
 * The default version assumes memory mapped registers and therefore
 * writes to a memory location. This default behavior may be overridden
 * by a target specific implementation.
 *
 * Note, that this instruction is not atomic on 8-bit targets. It may
 * be required to disable interrupts first.
 *
 * A special version is provided for AVR targets to make sure that
 * the low byte is written first. Also see the AVR specific note
 * for \ref outb.
 *
 * \todo Although heavily used in Nut/OS, it should be replaced in long
 *       term by XXX to avoid problems with compiler optimization.
 */
#define outw(_reg, _val)  (*((volatile unsigned short *)(_reg)) = (_val))
#endif

#ifndef outr
/*!
 * \brief Write to an I/O register.
 *
 * The default version assumes memory mapped registers and therefore
 * writes to a memory location. This default behavior may be overridden
 * by a target specific implementation.
 *
 * The size of the value should be equal to the native number of register
 * bits on the target. However, this is currently implemented on 32-bit
 * targets only. Do not use this function on other architectures.
 *
 * \todo Although heavily used in Nut/OS, it should be replaced in long
 *       term by XXX to avoid problems with compiler optimization.
 */
#define outr(_reg, _val)  (*((volatile unsigned long *)(_reg)) = (_val))
#endif

#ifndef inb
/*!
 * \brief Read an 8-bit value from an I/O register.
 *
 * The default version reads from a memory location, but may be
 * overridden by a target specific implementation.
 *
 * A special version is provided for 8-bit AVR targets to generate
 * optimized instruction code for registers in IO space. However,
 * to make this work, the register address must be a constant.
 *
 * \todo Although heavily used in Nut/OS, it should be replaced in long
 *       term by XXX to avoid problems with compiler optimization.
 */
#define inb(_reg)   (*((volatile unsigned char *)(_reg)))
#endif

#ifndef inw
/*!
 * \brief Read a 16-bit value from an I/O register.
 *
 * The default version reads from a memory location, but may be
 * overridden by a target specific implementation.
 *
 * See the target specific note for \ref outw.
 *
 * \todo Although heavily used in Nut/OS, it should be replaced in long
 *       term by XXX to avoid problems with compiler optimization.
 */
#define inw(_reg)   (*((volatile unsigned short *)(_reg)))
#endif

#ifndef inr
/*!
 * \brief Read from an I/O register.
 *
 * The default version reads from a memory location, but may be
 * overridden by a target specific implementation.
 *
 * The size of the value should be equal to the native number of register
 * bits on the target. However, this is currently implemented on 32-bit
 * targets only. Do not use this function on other architectures.
 *
 * \todo Although heavily used in Nut/OS, it should be replaced in long
 *       term by XXX to avoid problems with compiler optimization.
 */
#define inr(_reg)   (*((volatile unsigned long *)(_reg)))
#endif

#ifndef sbi
/*!
 * \brief Set bit in an I/O register.
 *
 * The default version assumes memory mapped 32-bit registers and reads
 * the current value from a memory location, sets the given bit to 1 and
 * writes the 32-bit value back to the same memory location. This default
 * behavior may be overridden by a target specific implementation.
 *
 * A special version is provided for 8-bit AVR targets to generate
 * an atomic bit set instruction for registers in IO space. However, to
 * make this work, the register address must be a constant.
 */
#define sbi(_reg, _bit) outr(_reg, inr(_reg) | _BV(_bit))
#endif

#ifndef cbi
/*!
 * \brief Clear bit in an I/O register.
 *
 * The default version assumes memory mapped 32-bit registers and reads
 * the current value from a memory location, clears the given bit to 0
 * and writes the 32-bit value back to the same memory location. This
 * default behavior may be overridden by a target specific
 * implementation.
 *
 * A special version is provided for 8-bit AVR targets to generate
 * an atomic bit set instruction for registers in IO space. However,
 * to make this work, the register address must be a constant.
 */
#define cbi(_reg, _bit) outr(_reg, inr(_reg) & ~_BV(_bit))
#endif

#ifndef bit_is_set
/*!
 * \brief Test bit in an I/O register.
 *
 * Returns 1 if the bit is set to 1, or 0 if the bit is cleared to 0.
 *
 * The default version assumes memory mapped 32-bit registers and reads
 * the current value from a memory location to check the given bit. This
 * default behavior may be overridden by a target specific
 * implementation.
 *
 * A special version is provided for 8-bit AVR targets to generate a
 * bit test instruction for registers in IO space. However, to make
 * this work, the register address must be a constant.
 */
#define bit_is_set(_reg, _bit)  ((inr(_reg) & _BV(_bit)) != 0)
#endif

#ifndef mem_wr
/*!
 * \brief Write unsigned integer value to memory location.
 *
 * The function typically is used to access memory mapped hardware
 * registers.
 *
 * \note Although fairly unlikely, the compiler may re-order memory
 *       accesses. If the order is important, you must use mem_wr_mb()
 *       instead.
 */
static NUT_INLINE_FUNC void mem_wr(unsigned int reg, unsigned int val)
{
    *(volatile unsigned int *) reg = val;
}
#endif

#ifndef mem_wr8
/*!
 * \brief Write unsigned 8-bit value to memory location.
 *
 * See \ref mem_wr
 */
static NUT_INLINE_FUNC void mem_wr8(unsigned int reg, uint8_t val)
{
    *(volatile uint8_t *) reg = val;
}
#endif

#ifndef mem_wr16
/*!
 * \brief Write unsigned 16-bit value to memory location.
 *
 * See \ref mem_wr
 */
static NUT_INLINE_FUNC void mem_wr16(unsigned int reg, uint16_t val)
{
    *(volatile uint16_t *) reg = val;
}
#endif

#ifndef mem_wr32
/*!
 * \brief Write unsigned 32-bit value to memory location.
 *
 * See \ref mem_wr
 */
static NUT_INLINE_FUNC void mem_wr32(unsigned int reg, uint32_t val)
{
    *(volatile uint32_t *) reg = val;
}
#endif

#ifndef mem_rd
/*!
 * \brief Read unsigned integer value from memory location.
 *
 * The function typically is used to access memory mapped hardware
 * registers.
 *
 * \note Although fairly unlikely, the compiler may re-order memory
 *       accesses. If the order is important, you must use mem_rd_mb()
 *       instead.
 */
static NUT_INLINE_FUNC unsigned int mem_rd(unsigned int reg)
{
    return *(const volatile unsigned int *) reg;
}
#endif

#ifndef mem_rd8
/*!
 * \brief Read unsigned 8-bit value from memory location.
 *
 * See \ref mem_rd.
 */
static NUT_INLINE_FUNC uint8_t mem_rd8(unsigned int reg)
{
    return *(const volatile uint8_t *) reg;
}
#endif

#ifndef mem_rd16
/*!
 * \brief Read unsigned 16-bit value from memory location.
 *
 * See \ref mem_rd.
 */
static NUT_INLINE_FUNC uint16_t mem_rd16(unsigned int reg)
{
    return *(const volatile uint16_t *) reg;
}
#endif

#ifndef mem_rd32
/*!
 * \brief Read unsigned 32-bit value from memory location.
 *
 * See \ref mem_rd.
 */
static NUT_INLINE_FUNC uint32_t mem_rd32(unsigned int reg)
{
    return *(const volatile uint32_t *) reg;
}
#endif

#ifndef mem_wr_mb
/*!
 * \brief Sequentially write unsigned integer value to memory location.
 *
 * The function typically is used to access memory mapped hardware
 * registers in a serialized way. It makes sure, that all preceding
 * reads and writes have completed before writing to the specified
 * address.
 */
static NUT_INLINE_FUNC void mem_wr_mb(unsigned int reg, unsigned int val)
{
    mem_barrier();
    mem_wr(reg, val);
}
#endif

#ifndef mem_wr8_mb
/*!
 * \brief Sequentially write unsigned 8-bit value to memory location.
 *
 * See \ref mem_wr_mb.
 */
static NUT_INLINE_FUNC void mem_wr8_mb(unsigned int reg, uint8_t val)
{
    mem_barrier();
    mem_wr8(reg, val);
}
#endif

#ifndef mem_wr16_mb
/*!
 * \brief Sequentially write unsigned 16-bit value to memory location.
 *
 * See \ref mem_wr_mb.
 */
static NUT_INLINE_FUNC void mem_wr16_mb(unsigned int reg, uint16_t val)
{
    mem_barrier();
    mem_wr16(reg, val);
}
#endif

#ifndef mem_wr32_mb
/*!
 * \brief Sequentially write unsigned 32-bit value to memory location.
 *
 * See \ref mem_wr_mb.
 */
static NUT_INLINE_FUNC void mem_wr32_mb(unsigned int reg, uint32_t val)
{
    mem_barrier();
    mem_wr32(reg, val);
}
#endif

#ifndef mem_rd_mb
/*!
 * \brief Immediately read unsigned integer value from memory location.
 *
 * The function typically is used to access memory mapped hardware
 * registers in a serialized way. It makes sure, that this read and
 * all preceding reads and writes have completed before the function
 * returns.
 */
static NUT_INLINE_FUNC unsigned int mem_rd_mb(unsigned int reg)
{
    unsigned int rc = mem_rd(reg);
    mem_barrier();

    return rc;
}
#endif

#ifndef mem_rd8_mb
/*!
 * \brief Immediately read unsigned 8-bit value from memory location.
 *
 * See \ref mem_rd_mb.
 */
static NUT_INLINE_FUNC uint8_t mem_rd8_mb(unsigned int reg)
{
    uint8_t rc = mem_rd8(reg);
    mem_barrier();

    return rc;
}
#endif

#ifndef mem_rd16_mb
/*!
 * \brief Immediately read unsigned 16-bit value from memory location.
 *
 * See \ref mem_rd_mb.
 */
static NUT_INLINE_FUNC uint16_t mem_rd16_mb(unsigned int reg)
{
    uint16_t rc = mem_rd16(reg);
    mem_barrier();

    return rc;
}
#endif

#ifndef mem_rd32_mb
/*!
 * \brief Immediately read unsigned 32-bit value from memory location.
 *
 * See \ref mem_rd_mb.
 */
static NUT_INLINE_FUNC uint32_t mem_rd32_mb(unsigned int reg)
{
    uint32_t rc = mem_rd32(reg);
    mem_barrier();

    return rc;
}
#endif

#ifndef PSTR
/*!
 * \brief Declare static pointer to a string in program space.
 *
 * This macro is used to place string literals in code space. This
 * makes a lot of sense on Harvard architectures, where program space
 * is typically much larger than data space.
 *
 * Example:
 * \code
 * printf_P(PSTR("Hello world!\n"));
 * \endcode
 *
 * Unfortunately this will only work, if the compiler allows to
 * declare static pointers inside expressions, which is a non-
 * standard C extension.
 *
 * On other architectures, this macro will simply return a pointer
 * to the given string literal. Because printf_P will be directly
 * mapped to printf, on non-Harvard architectures the example above
 * will become
 *
 * \code
 * printf(("Hello world!\n"));
 * \endcode
 */
#define PSTR(p)    (p)
#endif

#ifndef PRG_RDB
/*!
 * \brief Read data byte from program space.
 *
 * This macro simply returns the data byte located at the given address.
 * However, if used on Harvard architectures, the memory address is
 * expected to be located in code space.
 */
#define PRG_RDB(p) (*((const char *)(p)))

#ifndef prog_char
/*!
 * \brief Declare a character variable located in program space.
 *
 * This macro affects variables on Harvard architectures only. On other
 * architectures it simply declares a constant character variable.
 */
//#define prog_char  const char
#endif

#ifndef PGM_P
/*!
 * \brief Declare a pointer variable to a string in program space.
 *
 * This macro affects variables on Harvard architectures only. On other
 * architectures it simply declares a pointer to a constant character.
 */
#define PGM_P      prog_char *
#endif

#ifndef __GNUC__
/*!
 * \brief Remove object attributes.
 *
 * Unfortunately the Nut/OS code uses GCC attributes. Until we replaced
 * them entirely by our new feature macros, this macro removes them for
 * all other compilers.
 */
#define __attribute__(x)

#endif

#endif

#ifndef __HARVARD_ARCH__

#ifndef strlen_P
#define strlen_P(x) strlen(x)
#endif

#ifndef strcpy_P
#define strcpy_P(x, y) strcpy(x,y)
#endif

#ifndef strcat_P
#define strcat_P(x, y) strcat(x, y)
#endif

#ifndef strcmp_P
#define strcmp_P(x, y) strcmp(x, y)
#endif

#ifndef memcpy_P
#define memcpy_P(x, y, z) memcpy(x, y, z)
#endif

#endif

#endif
