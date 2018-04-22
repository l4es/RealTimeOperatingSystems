/*
 * Copyright (C) 2001-2006 by egnite Software GmbH
 * Copyright (C) 2013 by egnite GmbH
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
 *
 */

/*
 * $Id: nutinit.c 5217 2013-06-28 18:43:40Z haraldkipp $
 */

#include <sys/thread.h>
#include <sys/heap.h>
#include <cfg/memory.h>
#include <cfg/os.h>
#include <cfg/arch/avr.h>
#include <cfg/arch.h>

#include <dev/board.h>
#include <dev/gpio.h>

/*!
 * \addtogroup xgNutArchAvrInit
 */
/*@{*/

#ifdef NUTXMEM_SIZE
/*!
 * \brief Last memory address using external SRAM.
 */
#define NUTMEM_END (uint16_t)(NUTXMEM_START + (uint16_t)NUTXMEM_SIZE - 1U)

#else
/*!
 * \brief Last memory address without using external SRAM.
 *
 * \todo Shall we support NUTRAMEND for backward compatibility? If, then
 *       let's do it in cfg/memory.h.
 */
#define NUTMEM_END (uint16_t)(NUTMEM_START + (uint16_t)NUTMEM_SIZE - 1U)

#endif

#ifndef NUT_THREAD_MAINSTACK
#define NUT_THREAD_MAINSTACK    1024
#endif
#define MAINTHREAD_STACK_SIZE   (((NUT_THREAD_MAINSTACK) * (NUT_THREAD_STACK_MULT)) + (NUT_THREAD_STACK_ADD))

#ifndef NUT_THREAD_IDLESTACK
#if defined(__GNUC__)
/* avr-gcc optimized code used 36 bytes. */
#define NUT_THREAD_IDLESTACK    128
#else
/* icc-avr v7.19 used 132 bytes. */
#define NUT_THREAD_IDLESTACK    256
#endif
#endif
#define IDLETHREAD_STACK_SIZE   (((NUT_THREAD_IDLESTACK) * (NUT_THREAD_STACK_MULT)) + (NUT_THREAD_STACK_ADD))

/* Running with avr-gccdbg and only internal stack may result in hard to
 * find errors where the stack got overwritten
 *
 * Do some sanity check!
 */
#ifndef NUTXMEM_SIZE
#if MAINTHREAD_STACK_SIZE + IDLETHREAD_STACK_SIZE + 1024 > NUTMEM_SIZE
#warning "Potential stack overflow, reduce stack sizes of main or idle thread."
#endif
#endif

#ifdef NUTMEM_RESERVED
/*!
 * \brief Number of bytes reserved in on-chip memory.
 *
 * AVR offers the option to temporarily use address and data bus
 * lines as general purpose I/O. If such drivers need data memory,
 * this must be located at internal memory addresses.
 *
 * \todo Not a nice implementation but works as long as this module
 *       is linked first. Should be made a linker option.
 */
uint8_t nutmem_onchip[NUTMEM_RESERVED];
#endif

/* sleep mode to put avr in idle thread, SLEEP_MODE_NONE is used for for non sleeping */
#if defined(__GNUC__) && defined(__AVR_ENHANCED__)
uint8_t idle_sleep_mode = SLEEP_MODE_NONE;

/* AT90CAN128 uses a different register to enter sleep mode */
#if defined(SMCR)
#define AVR_SLEEP_CTRL_REG    SMCR
#else
#define AVR_SLEEP_CTRL_REG    MCUCR
#endif

#endif

#ifdef __GNUC__
/*
 * Some special declarations for AVRGCC. The runtime library
 * executes section .init8 before finally jumping to main().
 * We never let it do that jump, but start main() as a
 * separate thread. This introduces new problems:
 * 1. The compiler reinitializes the stack pointer when
 *    entering main, at least version 3.3 does it.
 * 2. The compiler doesn't allow to redeclare main to make
 *    it fit for NutThreadCreate().
 * 3. The runtime library requires, that main is defined
 *    somewhere.
 * Solutions:
 * 1. We do not use main at all, but let the preprocessor
 *    redefine it to NutAppMain() in the application code.
 *    See compiler.h. Note, that the declaration of NutAppMain
 *    in this module differs from the one in the application
 *    code. Fortunately the linker doesn't detect this hack.
 * 2. We use a linker option to set the symbol main to zero.
 *
 * Thanks to Joerg Wunsch, who helped to solve this.
 */
void NutInit(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init8");
extern void NutAppMain(void *arg) NUT_NORETURN_FUNC;
#else
extern void main(void *);
#endif

/*
 * External memory interface for GCC.
 */
#if defined(__GNUC__) && defined(NUTXMEM_SIZE)

/*
 * At the very beginning enable extended memory interface.
 */
static void NutInitXRAM(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init1") NUT_USED_FUNC;
void NutInitXRAM(void)
{
#if defined(__AVR_AT90CAN128__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(MCU_AT90USB1287)
/*
 * Note: Register structure of ATCAN128 differs from ATMEGA128 in regards
 * to wait states.
 */
#ifdef NUT_3WAITSTATES /* One wait state 1 for low, 3 for high memory range */
    XMCRA = _BV(SRE) | _BV(SRL2) | _BV(SRW00) | _BV(SRW10) | _BV(SRW11);
#else
    XMCRA = _BV(SRE) | _BV(SRW10); /* One wait state for the whole memory range */
#endif

#elif defined(__AVR_ATmega128__)

    MCUCR = _BV(SRE) | _BV(SRW10);

/* Configure two sectors, lower sector = 0x1100 - 0x7FFF,
 * Upper sector = 0x8000 - 0xFFFF and run 3 wait states for the
 * upper sector (NIC), 1 wait state for lower sector (XRAM).
 */
#ifdef NUT_3WAITSTATES
    XMCRA |= _BV(SRL2) | _BV(SRW00) | _BV(SRW11); /* SRW10 is set in MCUCR */
    XMCRB = 0;
#endif

#else  /* ATmega103 */
    MCUCR = _BV(SRE) | _BV(SRW);
#endif
}

#endif

/*! \fn NutThreadSetSleepMode(uint8_t mode)
 * \brief Sets the sleep mode to enter in Idle thread.
 *
 * If the idle thread is running, no other thread is active
 * so we can safely put the mcu to sleep.
 *
 * \param mode one of the sleep modes defined in avr/sleep.h or
 *             sleep_mode_none (don't enter sleep mode)
 *
 * \return previous sleep mode
 */
#if defined(__GNUC__) && defined(__AVR_ENHANCED__)
uint8_t NutThreadSetSleepMode(uint8_t mode)
{
    uint8_t old_mode = idle_sleep_mode;
    idle_sleep_mode = mode;
    return old_mode;
}
#endif

static NutIdleCallback IdleCall;
NutIdleCallback NutRegisterIdleCallback(NutIdleCallback func)
{
    NutIdleCallback last = IdleCall;
    IdleCall = func;
    return last;
}

/*!
 * \brief AVR Idle thread.
 *
 * Running at priority 254 in an endless loop.
 */
THREAD(NutIdle, arg)
{
#if defined(__GNUC__) && defined(__AVR_ENHANCED__)
    uint8_t sleep_mode;
#endif

#ifdef NUT_INIT_IDLE
    NutIdleInit();
#endif

    /* Initialize system timers. */
    NutTimerInit();

#ifdef NUT_INIT_MAIN
    NutMainInit();
#endif

    /* Create the main application thread. */
    NutThreadCreate("main", main, 0, MAINTHREAD_STACK_SIZE);

    /*
     * Run in an idle loop at the lowest priority. We can still
     * do something useful here, like killing terminated threads
     * or putting the CPU into sleep mode.
     */
    NutThreadSetPriority(254);
    for (;;) {
        NutThreadYield();
        NutThreadDestroy();
        if (IdleCall) {
            IdleCall();
        }

#if defined(__GNUC__) && defined(__AVR_ENHANCED__)
        if (idle_sleep_mode != SLEEP_MODE_NONE) {
            sleep_mode = AVR_SLEEP_CTRL_REG & _SLEEP_MODE_MASK;
            set_sleep_mode(idle_sleep_mode);
#ifdef IDLE_THREAD_ADC_OFF
            uint8_t adc = bit_is_set(ADCSR, ADEN);
            cbi(ADCSR, ADEN); // disable ADC
#endif
#ifdef IDLE_THREAD_BUSKEEPER_OFF
            uint8_t bitkeeper = bit_is_set(XMCRB, XMBK);
            cbi(XMCRB, XMBK); // disable buskeeper
#endif
            /* Note:  avr-libc has a sleep_mode() function, but it's broken for
            AT90CAN128 with avr-libc version earlier than 1.2 */
            AVR_SLEEP_CTRL_REG |= _BV(SE);
            __asm__ __volatile__ ("sleep" "\n\t" :: );
            AVR_SLEEP_CTRL_REG &= ~_BV(SE);
#ifdef IDLE_THREAD_ADC_OFF
            if (bitkeeper) {
                sbi(XMCRB, XMBK); // re-enable buskeeper
            }
#endif
#ifdef IDLE_THREAD_BUSKEEPER_OFF
            if (adc) {
                sbi(ADCSR, ADEN); // re-enable ADC
            }
#endif
            set_sleep_mode(sleep_mode);
        }
#endif
    }
}

#if defined(__GNUC__)
static void NutInitSP(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init5") NUT_USED_FUNC;
void NutInitSP(void)
{
#if defined (NUTMEM_STACKHEAP)
    /* Stack must remain in internal RAM where avr-libc's runtime lib init placed it */
#else
   /* Initialize stack pointer to end of external RAM while starting up the system
    * to avoid overwriting .data and .bss section.
    */
    SP = (uint16_t)(NUTMEM_END);
#endif
}
#endif

#if defined(__GNUC__)
static void NutInitHeap(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init5") NUT_USED_FUNC;
#endif
void NutInitHeap()
{
#if defined (NUTMEM_STACKHEAP) /* Stack resides in internal memory */
    NutStackAdd((void *) NUTMEM_START, NUTMEM_STACKHEAP);
#endif
    /* Then add the remaining RAM to heap.
     *
     * 20.Aug.2004 haraldkipp: This had been messed up somehow. It's nice to have
     * one continuous heap area, but we lost the ability to have systems with
     * a gap between internal and external RAM.
     */
    if ((uint16_t)NUTMEM_END - (uint16_t) (&__heap_start) > 384) {
        NutHeapAdd(&__heap_start, (uint16_t) NUTMEM_END - 256 - (uint16_t) (&__heap_start));
    }
}

/*!
 * \brief Nut/OS Initialization.
 *
 * Initializes the memory management and the thread system and starts
 * an idle thread, which in turn initializes the timer management.
 * Finally the application's main() function is called.
 *
 * Depending on the compiler, different methods are used to execute this
 * function before main() is called.
 *
 * For ICCAVR the default crtatmega.o startup file is replaced by
 * crtnut.o, which calls NutInit instead of main(). This is done
 * by adding the following compiler options in the project:
 * \code -ucrtnut.o nutinit.o \endcode
 *
 * crtnut.o should be replaced by crtnutram.o, if the application's
 * variable space exceeds 4kB. For boards with RTL8019AS and EEPROM
 * emulation (like Ethernut 1.3 Rev-G) use crtenut.o or crtenutram.o.
 *
 * For AVRGCC this function is located in section .init8, which is
 * called immediately before jumping to main(). NutInit is defined
 * as:
 * \code
 * void NutInit(void) NUT_NAKED_FUNC NUT_LINKER_SECT(".init8");
 * \endcode
 *
 * \todo Make heap threshold configurable, currently hardcoded at 384.
 *
 * \todo Make wait states for external memory access configuratble.
 *
 * \todo Make early UART initialization for kernel debugging configurable.
 */
void NutInit(void)
{
    /*
     * We can't use local variables in naked functions.
     */
#ifdef NUTDEBUG
    /* Note: The platform's default baudrate will be set in NutCustomInit() */
    outb(UCR, BV(RXEN) | BV(TXEN));
#endif

#ifdef NUT_INIT_BOARD
    NutBoardInit();
#endif

#ifndef __GNUC__
    /* Initialize stack pointer to end of external RAM while starting up the system
     * to avoid overwriting .data and .bss section.
     */
    SP = (uint16_t)(NUTMEM_END);

    /* Initialize the heap memory
     */
    NutInitHeap();
#endif /* __GNUC__ */

    /* Read OS configuration from non-volatile memory. */
    NutLoadConfig();

    /* Create idle thread
     */
    NutThreadCreate("idle", NutIdle, 0, IDLETHREAD_STACK_SIZE);
}

/*@}*/
