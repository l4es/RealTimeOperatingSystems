#ifndef _TOOLCHAIN_GCC_H_
#define _TOOLCHAIN_GCC_H_
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
#error "Do not include this file directly, include <toolchain.h> instead."
#endif

/*!
 * \name General Compiler Feature Macros
 */
/*@{*/

#ifndef NUT_INLINE_FUNC
/*!
 * \brief Inline function macro.
 *
 * Although not specified in ANSI C89, the inline keyword is essential
 * in embedded programming and had been supported by many compilers long
 * before its standardization in C99.
 *
 * In order to support compilers, which do not follow ANSI C99, Nut/OS
 * code must use this macro instead of the inline keyword to declare
 * inline functions.
 *
 * Note, that functions may not be inlined if compiler optimizations
 * had been disabled.
 */
#define NUT_INLINE_FUNC  inline
#endif

#ifndef NUT_FORCE_INLINE
/*!
 * \brief Forced inline function attribute.
 *
 * Always inlines a function, even if optimization has been disabled.
 */
#define NUT_FORCE_INLINE __attribute__((__always_inline__))
#endif

#ifndef NUT_FORCE_NO_INLINE
/*!
 * \brief Forced inline function attribute.
 *
 * Always inlines a function, even if optimization has been disabled.
 */
#define NUT_PREVENT_INLINE __attribute__((__noinline__))
#endif

#if !defined(NUT_NAKED_FUNC)
/*!
 * \brief Naked function attribute.
 *
 * The compiler will not generate prolog and epilog code for functions
 * with this attribute.
 */
#define NUT_NAKED_FUNC __attribute__((__naked__))
#endif

#ifndef NUT_PURE_FUNC
/*!
 * \brief Pure function attribute.
 *
 * The result returned by a pure function depends only on their
 * parameters and global variables. Thus, it may be called less often
 * because the compiler may re-use the result from a previous call.
 */
#define NUT_PURE_FUNC __attribute__((__pure__))
#endif

#ifndef NUT_CONST_FUNC
/*!
 * \brief Constant function attribute.
 *
 * A constant function returns a value, which depends only on their
 * parameters. Furthermore, it will not have any other effect.
 */
#define NUT_CONST_FUNC __attribute__((__const__))
#endif

#ifndef NUT_NORETURN_FUNC
#define NUT_NORETURN_FUNC __attribute__((__noreturn__))
#endif

#ifndef NUT_USED_FUNC
/*!
 * \brief Used function attribute.
 *
 * When this attribute is attached to a function, then the compiler
 * generates the function code, even if it appears to be unreferenced.
 */
#define NUT_USED_FUNC __attribute__((__used__))
#endif

#ifndef NUT_ALLOC_FUNC
/*!
 * \brief Memory allocation function attribute.
 *
 * Used for functions returning a pointer, which doesn't alias anything.
 */
#define NUT_ALLOC_FUNC __attribute__((__malloc__))
#endif

#ifndef NUT_IRQ_HANDLER
#if defined(__arm__)
#define NUT_IRQ_HANDLER __attribute__((__interrupt__("IRQ")))
#endif
#endif

#ifndef NUT_FIQ_HANDLER
#if defined(__arm__)
#define NUT_FIQ_HANDLER __attribute__((__interrupt__("FIQ")))
#endif
#endif

#ifndef NUT_SWI_HANDLER
#if defined(__arm__)
#define NUT_SWI_HANDLER __attribute__((__interrupt__("SWI")))
#endif
#endif

#ifndef NUT_ABORT_HANDLER
#if defined(__arm__)
#define NUT_ABORT_HANDLER __attribute__((__interrupt__("ABORT")))
#endif
#endif

#ifndef NUT_UNDEF_HANDLER
#if defined(__arm__)
#define NUT_UNDEF_HANDLER __attribute__((__interrupt__("UNDEF")))
#endif
#endif

#ifndef RAMFUNC
/*!
 * \brief Function running in RAM.
 *
 * This section will be copied to RAM during runtime initialization.
 */
#if defined(__arm__)
#define RAMFUNC __attribute__((__long_call__, __section__(".ramfunc")))
#else
#define RAMFUNC __attribute__((__section__(".ramfunc")))
#endif
#endif

#ifndef NUT_PACKED_TYPE
/*!
 * \brief Packed type attribute.
 *
 * Specifies, that the minimum required memory be used to represent the
 * type.
 */
#define NUT_PACKED_TYPE __attribute__((__packed__))
#endif

#ifndef NUT_ALIGNED_TYPE
/*!
 * \brief Aligned type attribute.
 *
 * Specifies a minimum alignment for variables in bytes.
 *
 * \note The actual alignment is not guaranteed, but limited by the
 * capabilities of the linker.
 */
#define NUT_ALIGNED_TYPE(bytes) __attribute__((__aligned__(bytes)))
#endif

#ifndef NUT_LINKER_SECT
/*!
 * \brief Linker section attribute.
 *
 * By default, the compiler will put code in the .text segment,
 */
#define NUT_LINKER_SECT(name) __attribute__((__section__(name)))
#endif

#ifndef NUT_WEAK_SYMBOL
/*!
 * \brief Weak symbol attribute.
 *
 * Weak functions or variables may be replaced by the linker with
 * non-weak functions or variables specified elsewhere, including
 * application code.
 */
#define NUT_WEAK_SYMBOL __attribute__((__weak__))
#endif

#ifndef NUT_DEPRECATED
/*!
 * \brief Deprecated attribute.
 */
#define NUT_DEPRECATED __attribute__((__deprecated__))
#endif

#define COMPRESS_DISABLE
#define COMPRESS_REENABLE

/*@}*/

/*!
 * \name General Compiler-Specific Workarounds
 */
/*@{*/

#ifndef CONST
/*!
 * \brief Constant qualifier.
 *
 * Compilers interpret const variables in a different way. Most notably,
 * the ImageCraft compilers up to version 6 use it to specify data in
 * program space for Harvard architectures. Therefore, const was
 * replaced by this macro throughout the code.
 *
 * Since Nut/OS version 5 such abuse of the const qualifier is no longer
 * supported. In order not to break existing applications, we keep this
 * deprecated macro.
 */
#define CONST   const
#endif

/*@}*/

/*!
 * \brief Compiler memory barrier.
 *
 * This function does not generate any code by itself, but forbids the
 * compiler to reorder read and write commands around it.
 */
#define mem_barrier() __asm__ __volatile__("":::"memory")


/*
 * Include the runtime library header here. It may override any of the
 * following definitions.
 */
#if defined(__AVR__)

/* Only avr-libc is currently supported for GCC. */
#include <toolchain/avrlibc.h>

#elif defined(__AVR32__)

#include <arch/avr32.h>
#include <toolchain/generic.h>

#elif defined(__CORTEX__)
#include <arch/cm3.h>
#include <toolchain/generic.h>

#elif defined(__arm__)

/* Note, that newlib is the default. */
#if defined(__CROSSWORKS_ARM)
#include <toolchain/crossworks.h>
#include <toolchain/newlib.h>
#else
#include <toolchain/newlib.h>
#endif

#elif defined(__m68k__)

#include <arch/m68k.h>
#include <toolchain/generic.h>

#endif

#ifndef PROGMEM
#define PROGMEM
#endif

/*!
 * \brief NOP instruction.
 *
 * Several Nut/OS drivers use one or more single cycle operations
 * for very short delays. It's implementation depends on the inline
 * assembler and the target family.
 */
#ifndef _NOP
#if defined(__arm__)
#define _NOP() __asm__ __volatile__("mov r0, r0  @ _NOP")
#elif defined(__AVR__) || defined(__AVR32__)
#define _NOP() __asm__ __volatile__("nop")
#endif
#endif

#endif
