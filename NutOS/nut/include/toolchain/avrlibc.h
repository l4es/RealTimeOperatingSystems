#ifndef _TOOLCHAIN_AVRLIBC_H_
#define _TOOLCHAIN_AVRLIBC_H_
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

/*
 * Include library specific header files, which are required by the
 * kernel here. Avoid polluting kernel code with target specific ifdefs.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10400UL
#include <avr/signal.h>
#endif
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdlib.h>

/*
 * test for a macro added in avr-libc 1.2.0, if yes use different path for twi.h
 * note: has to be after #include <eeprom.h>
 */
#ifdef eeprom_rb
#include <avr/twi.h>
#else
#include <compat/twi.h>
#endif

/*!
 * \name Library-Specific Workarounds
 */
/*@{*/

/*!
 * \brief Main function entry.
 *
 * In order to save a few bytes of code on very tiny AVR devices, in
 * some versions of AVR Libc the main function entry differs form normal
 * function entries in that the stack pointer is set at the beginning of
 * main(). Actually this is done by the compiler, but tightly coupled to
 * the library.
 *
 * For Nut/OS this is fatal, because main is started as a separate
 * thread by the idle thread, assuming the stack pointer at the main
 * thread's local stack. As a dirty workaround, main() is redefined as
 * NutAppMain(). Dirty, because some debuggers (and developers) may
 * become confused.
 *
 * \todo Limit redefinition of main to specific library versions.
 */
#define main    NutAppMain

/*@}*/


/*!
 * \name Register Compatibility Macros
 */
/*@{*/

/*!
 * \brief Special function register offset.
 *
 * Automatically subtracts 0x20 from I/O space addresses, but according
 * to the AVR Libc documentation, this is a hack, and it is recommended
 * to change the source code, wrapping such addresses in macros.
 */
#ifndef __SFR_OFFSET
#define __SFR_OFFSET    0
#endif

/*
 * Since version 1.1.0 of avr-libc, some former deprecated macros are deleted.
 * But we need them futher on, so they are defined here.
 */

#ifndef inb
#define inb(sfr) _SFR_BYTE(sfr)
#endif
#ifndef outb
#define outb(sfr, val) (_SFR_BYTE(sfr) = (val))
#endif
#ifndef inw
#define inw(sfr) _SFR_WORD(sfr)
#endif
#ifndef outw
#define outw(sfr, val) (_SFR_WORD(sfr) = (val))
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#if __AVR_LIBC_VERSION__ >= 10800UL
typedef const char PROGMEM prog_char;
#endif
#ifndef PGM_P
#define  PGM_P      prog_char *
#endif
#ifndef PRG_RDB
#define PRG_RDB(addr)       pgm_read_byte(addr)
#endif

/*@}*/

#include <toolchain/generic.h>

#define __bss_end   __heap_start
extern void *__heap_start;

#ifndef atof
#define atof(s)     strtod(s, 0)
#endif

/*!
 * \name EEPROM Access Compatibility Macros
 */
/*@{*/

#define EEPROMReadBytes(addr, ptr, size)    eeprom_read_block((char *)(addr), ptr, size)
/*!
 * \brief Read multibyte types from the EEPROM.
 */
#define EEPROM_READ(addr, dst)  eeprom_read_block((char *)(addr), &dst, sizeof(dst))
#define EEPROMread(addr)        eeprom_read_byte((char *)(addr))

/*!
 * \brief Write multibyte types to the EEPROM.
 */
#define EEPROM_WRITE(addr, src)                         \
{                                       \
    unsigned short __i;                             \
    for(__i = 0; __i < sizeof(src); __i++)                  \
    eeprom_write_byte(((char *)(addr)) + __i, *(((char *)(&(src))) + __i)); \
}

#define EEPROMWriteBytes(addr, ptr, size)                   \
{                                       \
    unsigned short __i;                             \
    for(__i = 0; __i < size; __i++)                     \
    eeprom_write_byte(((char *)(addr)) + __i, *(((char *)(ptr)) + __i));    \
}

/*@}*/

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#if !defined(TXC)
#define TXC     TXC0
#endif
#if !defined(TXB8)
#define TXB8     TXB80
#endif
#if !defined(UMSEL)
#define UMSEL     UMSEL00
#endif
#if !defined(U2X)
#define U2X     U2X0
#endif
#if !defined(UCSZ0)
#define UCSZ0     UCSZ00
#endif
#if !defined(UCSZ1)
#define UCSZ1     UCSZ01
#endif
#if !defined(UCSZ2)
#define UCSZ2     UCSZ02
#endif
#if !defined(UPM0)
#define UPM0     UPM00
#endif
#if !defined(UPM1)
#define UPM1     UPM01
#endif

#if !defined(USBS)
#define USBS     USBS0
#endif
#if !defined(UPE)
#define UPE     UPE0
#endif
#if !defined(MPCM)
#define MPCM     MPCM0
#endif
#if !defined(UCPOL)
#define UCPOL     UCPOL0
#endif
#elif defined(__AVR_AT90USB1287__)
#if !defined(TXC)
#define TXC     TXC1
#endif
#if !defined(TXB8)
#define TXB8     TXB81
#endif
#if !defined(UMSEL)
#define UMSEL     UMSEL10
#endif
#if !defined(U2X)
#define U2X     U2X1
#endif
#if !defined(UCSZ0)
#define UCSZ0     UCSZ10
#endif
#if !defined(UCSZ1)
#define UCSZ1     UCSZ11
#endif
#if !defined(UCSZ2)
#define UCSZ2     UCSZ12
#endif
#if !defined(UPM0)
#define UPM0     UPM10
#endif
#if !defined(UPM1)
#define UPM1     UPM11
#endif

#if !defined(USBS)
#define USBS     USBS1
#endif
#if !defined(UPE)
#define UPE     UPE1
#endif
#if !defined(MPCM)
#define MPCM     MPCM1
#endif
#if !defined(UCPOL)
#define UCPOL     UCPOL1
#endif
#endif

#ifdef __AVR_ENHANCED__

/* Nut/OS is still using the original ATmega103 register names for
   backward compatibility. */
#if  defined(__AVR_AT90USB1287__)
/* AT90USB1287 without USART0 */
#ifndef UDR
#define UDR     UDR1
#endif
#ifndef UBRR
#define UBRR    UBRR1L
#endif
#ifndef USR
#define USR     UCSR1A
#endif
#ifndef UCR
#define UCR     UCSR1B
#endif
#ifndef EICR
#define EICR    EICRB
#endif
#ifndef RXC
#define RXC     RXC1
#endif
#ifndef UDRE
#define UDRE    UDRE1
#endif
#ifndef FE
#define FE      FE1
#endif
#ifndef DOR
#define DOR     DOR1
#endif
#ifndef RXCIE
#define RXCIE   RXCIE1
#endif
#ifndef TXCIE
#define TXCIE   TXCIE1
#endif
#ifndef UDRIE
#define UDRIE   UDRIE1
#endif
#ifndef RXEN
#define RXEN    RXEN1
#endif
#ifndef TXEN
#define TXEN    TXEN1
#endif
#else
#ifndef UDR
#define UDR     UDR0
#endif
#ifndef UBRR
#define UBRR    UBRR0L
#endif
#ifndef USR
#define USR     UCSR0A
#endif
#ifndef UCR
#define UCR     UCSR0B
#endif
#ifndef EICR
#define EICR    EICRB
#endif
#ifndef RXC
#define RXC     RXC0
#endif
#ifndef UDRE
#define UDRE    UDRE0
#endif
#ifndef FE
#define FE      FE0
#endif
#ifndef DOR
#define DOR     DOR0
#endif
#ifndef RXCIE
#define RXCIE   RXCIE0
#endif
#ifndef TXCIE
#define TXCIE   TXCIE0
#endif
#ifndef UDRIE
#define UDRIE   UDRIE0
#endif
#ifndef RXEN
#define RXEN    RXEN0
#endif
#ifndef TXEN
#define TXEN    TXEN0
#endif
#endif

/* Some ATC90CAN128 SFR names are different to ATMEGA128. Define some
   compatibilty macros. */
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) ||  defined(__AVR_AT90USB1287__)
#ifndef ADCW
#define ADCW    ADC
#endif
#ifndef ADCSR
#define ADCSR   ADCSRA
#endif
#ifndef ADFR
#define ADFR    ADATE
#endif
#ifndef OCIE0
#define OCIE0   OCIE0A
#endif
#ifndef TCCR0
#define TCCR0   TCCR0A
#endif
#ifndef TCCR2
#define TCCR2   TCCR2A
#endif
#ifndef OCR0
#define OCR0    OCR0A
#endif
#ifndef TIMSK
#define TIMSK   TIMSK0
#endif
#ifndef TIFR
#define TIFR   TIFR0
#endif
#endif /* __AVR_AT90CAN128__ || __AVR_ATmega2560__ || __AVR_ATmega2561__ || __AVR_AT90USB1287__)*/

#endif /* __AVR_ENHANCED__ */

/* Define internal _SLEEP_MODE_MASK that is no longer public in avrlibc. */
#ifndef _SLEEP_MODE_MASK
#if defined(SM) && !defined(SM0) && !defined(SM1) && !defined(SM2)
#define _SLEEP_MODE_MASK _BV(SM)
#elif !defined(SM) && defined(SM0) && defined(SM1) && !defined(SM2)
#define _SLEEP_MODE_MASK (_BV(SM0) | _BV(SM1))
#elif !defined(SM) && defined(SM0) && defined(SM1) && defined(SM2)
#define _SLEEP_MODE_MASK (_BV(SM0) | _BV(SM1) | _BV(SM2))
#endif
#endif

#endif

