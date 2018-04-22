#ifndef TOOLCHAIN_ICC_H_
#define TOOLCHAIN_ICC_H_
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

/*
 * Include some basic header files of the ImageCraft runtime library.
 */
#include <stddef.h>
#include <macros.h>
#include <eeprom.h>

/* ICCV7 incompatibility nightmare. */
#if defined(_MCU_Enhanced) && !defined(_MCU_enhanced)
#define _MCU_enhanced
#endif

#if defined(_MCU_Extended) && !defined(_MCU_extended)
#define _MCU_extended
#endif

/*!
 * \brief Specify enhanced AVR target.
 *
 * For backward compatibility this macro is automatically defined if
 * _MCU_enhanced is defined.
 *
 */
#if defined(_MCU_enhanced) || defined(_MCU_extended)
#undef __AVR_ENHANCED__
#define __AVR_ENHANCED__ /* Generic test for enhanced AVRs like ATMEGA128, AT09CAN128 */
#endif

/*!
 * \name General Compiler Feature Macros
 */
/*@{*/

#ifndef NUT_INLINE_FUNC
#define NUT_INLINE_FUNC
#endif

#ifndef NUT_FORCE_INLINE
#define NUT_FORCE_INLINE
#endif

#ifndef NUT_NAKED_FUNC
#define NUT_NAKED_FUNC
#endif

#ifndef NUT_PURE_FUNC
#define NUT_PURE_FUNC
#endif

#ifndef NUT_CONST_FUNC
#define NUT_CONST_FUNC
#endif

#ifndef NUT_NORETURN_FUNC
#define NUT_NORETURN_FUNC
#endif

#ifndef NUT_USED_FUNC
#define NUT_USED_FUNC
#endif

#ifndef NUT_ALLOC_FUNC
#define NUT_ALLOC_FUNC
#endif

#ifndef RAMFUNC
#define RAMFUNC
#endif

#ifndef NUT_PACKED_TYPE
#define NUT_PACKED_TYPE
#endif

#ifndef NUT_ALIGNED_TYPE
#define NUT_ALIGNED_TYPE(bytes)
#endif

#ifndef NUT_LINKER_SECT
#define NUT_LINKER_SECT(name)
#endif

#ifndef NUT_WEAK_SYMBOL
#define NUT_WEAK_SYMBOL
#endif

#ifndef NUT_DEPRECATED
#define NUT_DEPRECATED
#endif

#define COMPRESS_DISABLE
#define COMPRESS_REENABLE

/*@}*/

/*!
 * \brief Redefined standard library routines.
 *
 * ImageCraft has a multipass linker, which is fine for complicated
 * dependencies in most cases. However, there is the potential risk,
 * that standard library calls with the same name are linked from
 * the wrong library. To avoid this, an additional postfix is added
 * to routines, which are implemented in Nut/OS libraries.
 */
#define printf      printf_M
#define puts        puts_M
#define sprintf     sprintf_M
#define vprintf     vprintf_M
#define scanf       scanf_M
#define gets        gets_M
#define malloc      malloc_M
#define calloc      calloc_M
#define realloc     realloc_M
#define free        free_M

/* device specific functions map to special name forms so the linker
 * can give more meaningful error messages
 */
#pragma device_specific_function putchar getchar

/*!
 * \brief Redirected stdio routines.
 *
 * Native stdio routines with format strings in program space are
 * redirected to their Nut/OS implementation.
 */
#define cprintf     printf_P
#define csprintf    sprintf_P
#define cscanf      scanf_P
#define csscanf     sscanf_P

#define memcpy_P(dst, src_P, n) cmemcpy(dst, src_P, n)
#define strcat_P(s1, s2_P)      cstrcat(s1, s2_P)
#define strcmp_P(s1, s2_P)      (-cstrcmp(s2_P, s1))
#define strlen_P(s_P)           cstrlen(s_P)
#define strncat_P(s1, s2_P, n)  cstrncat(s1, s2_P, n)
#define strncmp_P(s1_P, s2, n)  cstrncmp(s1_P, s2, n)
#define strcpy_P(dst, src_P)    cstrcpy(dst, src_P)
#define strncpy_P(x,y,z)        cstrncpy(x,y,z)

/*!
 * \brief Case insensitive string comparisions.
 *
 * Not supported by ICCAVR and temporarly redirected to
 * the case sensitive routines.
 *
 * \bug Case insensitive string comparisions fail with ICCAVR.
 */
#define strcasecmp(s1, s2)  strcmp(s1, s2)
#define strncasecmp(s1, s2, n)  strncmp(s1, s2, n)

/*!
 * \brief Start of heap area.
 */
#define __heap_start   _bss_end


#define PRG_RDB(p)  (*((__flash char *)(p)))
#define prog_char __flash char
#define prog_int __flash int

#define wdt_enable(tmo) \
{ \
    register unsigned char s = _BV(WDCE) | _BV(WDE); \
    register unsigned char r = tmo | _BV(WDE); \
    asm("in R0, 0x3F\n"     \
        "cli\n"             \
        "wdr\n"             \
        "out 0x21, %s\n"    \
        "out 0x21, %r\n"    \
        "out 0x3F, R0\n");  \
}

#define wdt_disable() \
{ \
    register unsigned char s = _BV(WDCE) | _BV(WDE); \
    register unsigned char r = 0;  \
    asm("in R0, $3F\n"      \
        "cli\n"             \
        "out 0x21, %s\n"    \
        "out 0x21, %r\n"    \
        "out 0x3F, R0\n");  \
}

#define wdt_reset() \
{ \
    _WDR(); \
}

#define __SFR_OFFSET 0
#define SFR_IO_ADDR(sfr) ((sfr) - __SFR_OFFSET)
#define SFR_MEM_ADDR(sfr) (sfr)
#define SFR_IO_REG_P(sfr) ((sfr) < 0x40 + __SFR_OFFSET)

#define _SFR_MEM8(addr)     (addr)
#define _SFR_MEM16(addr)    (addr)

#define BV(x)       BIT(x)
#define _BV(x)      BIT(x)

#define cli()           CLI()
#define sei()           SEI()
#define cbi(reg, bit)   (reg &= ~BIT(bit))
#define sbi(reg, bit)   (reg |= BIT(bit))


#define loop_until_bit_is_set(reg, bit) while((reg & BIT(bit)) == 0)

#define bit_is_clear(reg, bit)  ((reg & BIT(bit)) == 0)
#define bit_is_set(reg, bit)    ((reg & BIT(bit)) != 0)

/* FIXME */
#define parity_even_bit(x)  (0)

/* FIXME */
#define SIGNAL(x)   void x(void)

#define outb(reg, val)  (reg = val)

#define inb(reg)        (reg)

#include <eeprom.h>

#if defined(_MCU_enhanced) || defined(_MCU_extended)

#ifdef ATMega2560
#include <iom2560v.h>
#define __AVR_ATmega2560__
#ifndef _EE_EXTIO
#error "Looks like wrong platform. Select avrext-icc, not avr-icc."
#endif
#elif defined(ATMega2561)
#include <iom2561v.h>
#define __AVR_ATmega2561__
#ifndef _EE_EXTIO
#error "Looks like wrong platform. Select avrext-icc, not avr-icc."
#endif
#else
#include <iom128v.h>
#define __AVR_ATmega128__
#endif

#ifndef RAMEND
#if defined(ATMega2560) || defined(ATMega2561)
#define RAMEND  0x21FF
#else
#define RAMEND  0x10FF
#endif
#endif

#ifndef SRW
#define SRW  6
#endif

/* ICC doesn't define generic ports and flags. */
#ifndef TXC
#define TXC     TXC0
#endif
#ifndef ADCSR
#define ADCSR   ADCSRA
#endif

/* Master */
#define TW_START            0x08
#define TW_REP_START        0x10
/* Master Transmitter */
#define TW_MT_SLA_ACK       0x18
#define TW_MT_SLA_NACK      0x20
#define TW_MT_DATA_ACK      0x28
#define TW_MT_DATA_NACK     0x30
#define TW_MT_ARB_LOST      0x38
/* Master Receiver */
#define TW_MR_ARB_LOST      0x38
#define TW_MR_SLA_ACK       0x40
#define TW_MR_SLA_NACK      0x48
#define TW_MR_DATA_ACK      0x50
#define TW_MR_DATA_NACK     0x58
/* Slave Transmitter */
#define TW_ST_SLA_ACK       0xA8
#define TW_ST_ARB_LOST_SLA_ACK  0xB0
#define TW_ST_DATA_ACK      0xB8
#define TW_ST_DATA_NACK     0xC0
#define TW_ST_LAST_DATA     0xC8
/* Slave Receiver */
#define TW_SR_SLA_ACK       0x60
#define TW_SR_ARB_LOST_SLA_ACK  0x68
#define TW_SR_GCALL_ACK     0x70
#define TW_SR_ARB_LOST_GCALL_ACK 0x78
#define TW_SR_DATA_ACK      0x80
#define TW_SR_DATA_NACK     0x88
#define TW_SR_GCALL_DATA_ACK    0x90
#define TW_SR_GCALL_DATA_NACK   0x98
#define TW_SR_STOP      0xA0
/* Misc */
#define TW_NO_INFO      0xF8
#define TW_BUS_ERROR        0x00


#else                           /* ATmega103 */

#include <iom103v.h>
#define __AVR_ATmega103__

#ifndef DOR
#define DOR  OVR
#endif

#ifndef RAMEND
#define RAMEND  0x0FFF
#endif

#ifndef WDCE
#define WDCE    WDTOE
#endif

#endif

#define eeprom_read_block(dst, addr, size)  EEPROMReadBytes((int)addr, dst, size)
#define eeprom_write_byte(addr, src)        EEPROMwrite((int)addr, src)
#define eeprom_read_byte(addr)              EEPROMread((int)addr)

extern void *calloc(size_t num, size_t size);
extern char *strdup(const char *str);

#include <string.h>

#endif

