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
 * $Id: nutinit.c 5595 2014-03-21 09:48:21Z u_bonnes $
 */

#include <cfg/arch.h>
#include <cfg/memory.h>
#include <cfg/os.h>
#ifdef MCU_AT91
#include <arch/arm/at91.h>
#endif

#include <dev/board.h>
#include <dev/gpio.h>

#ifdef EARLY_STDIO_DEV
#include <sys/device.h>
#include <stdio.h>
#include <fcntl.h>
#include <dev/debug.h>
struct __iobuf {
    int     iob_fd;
    uint16_t iob_mode;
    uint8_t iob_flags;
    int     iob_unget;
};
#endif

/*!
 * \addtogroup xgNutArchArmInit
 */
/*@{*/

#ifndef NUT_THREAD_MAINSTACK
#define NUT_THREAD_MAINSTACK    1024
#endif

#ifndef NUT_THREAD_IDLESTACK
/* arm-elf-gcc optimized code used 160 bytes. */
#define NUT_THREAD_IDLESTACK    256
#endif

#ifdef __CROSSWORKS_ARM

/*
 * A CrossWorks MemoryMap file will be used. Here the memory
 * between __heap_start__ and __External_RAM_segment_end__
 * can be used for NutOS.
 */
extern void *__heap_start__;
extern void *__RAM_END_segment_end__;

#define HEAP_START  &__heap_start__
#define HEAP_SIZE  ((uintptr_t)(&__RAM_END_segment_end__ - 1) - (uintptr_t)(HEAP_START) - 256)

#else   /* GCC */

/*!
 * \brief Last memory address.
 */
#define NUTMEM_END (uintptr_t)(NUTMEM_START + NUTMEM_SIZE - 1U)
extern void *__heap_start;

#define HEAP_START  &__heap_start
#define HEAP_SIZE  ((uintptr_t) (NUTMEM_END - 256 - (uintptr_t) (&__heap_start)))
#endif

#if !defined(__arm__) && !defined(__cplusplus)
extern void NutAppMain(void *arg) NUT_NORETURN_FUNC;
#else
extern void main(void *);
#endif

/*!
 * \brief Idle thread.
 *
 * \param arg Ignored by the idle thread.
 *
 * This function runs in an endless loop as a lowest priority thread.
 */
THREAD(NutIdle, arg)
{
#ifdef NUT_INIT_IDLE
    NutIdleInit();
#endif
    /* Initialize system timers. */
    NutTimerInit();

#if defined(HEARTBEAT_IDLE_PORT) && defined(HEARTBEAT_IDLE_PIN)
    GpioPinConfigSet(HEARTBEAT_IDLE_PORT, HEARTBEAT_IDLE_PIN,
PIO_CFG_OUTPUT);
    GpioPinSetHigh(HEARTBEAT_IDLE_PORT, HEARTBEAT_IDLE_PIN);
#if defined(HEARTBEAT_IDLE_INVERT)
#define HEARTBEAT_ACTIVE() \
    GpioPinSetLow(HEARTBEAT_IDLE_PORT, HEARTBEAT_IDLE_PIN)
#define HEARTBEAT_IDLE()  \
    GpioPinSetHigh(HEARTBEAT_IDLE_PORT, HEARTBEAT_IDLE_PIN)
#else
#define HEARTBEAT_ACTIVE()  \
    GpioPinSetHigh(HEARTBEAT_IDLE_PORT, HEARTBEAT_IDLE_PIN)
#define HEARTBEAT_IDLE() \
    GpioPinSetLow(HEARTBEAT_IDLE_PORT, HEARTBEAT_IDLE_PIN)
#endif
#else
#define HEARTBEAT_ACTIVE()
#define HEARTBEAT_IDLE()
#endif

    /* Read OS configuration from non-volatile memory. We can't do this
    ** earlier, because the low level driver may be interrupt driven. */
    NutLoadConfig();

#ifdef NUT_INIT_MAIN
    NutMainInit();
#endif

    /* Create the main application thread. Watch this carefully when
    ** changing compilers and compiler versions. Some handle main()
    ** in a special way, like setting the stack pointer and other
    ** weird stuff that may break this code. */
    NutThreadCreate("main", main, 0,
        (NUT_THREAD_MAINSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD);

    /* Enter an idle loop at the lowest priority. This will run when
    ** all other threads are waiting for an event. */
    NutThreadSetPriority(254);
    for (;;) {
	HEARTBEAT_ACTIVE();
        /* Check if other threads became ready to run. */
        NutThreadYield();
        /* Remove terminated threads. */
        NutThreadDestroy();
        /* We could do some power management. */
        HEARTBEAT_IDLE();
    }
}

/*!
 * \brief Nut/OS Initialization.
 *
 * Initializes the memory management and the thread system and starts
 * an idle thread, which in turn initializes the timer management.
 * Finally the application's main() function is called.
 */
void NutInit(void)
{
    /* Do some basic hardware initialization first. Frankly, these
    ** are all hacks and could be done in a more general way. */
#if defined(MCU_AT91)
    McuInit();
#endif
#if defined(MCU_AT91SAM7X) || defined (MCU_AT91SAM7S) || defined(MCU_AT91SAM7SE)
    {
        uint32_t freq = NutGetCpuClock();
        /* Set Flash Waite state. */
        outr(MC_FMR_EFC0, ((((freq + freq / 2) / 1000000UL) & 0xFF) << 16) | MC_FWS_2R3W);
#if defined(MCU_AT91SAM7X512) || defined(MCU_AT91SAM9XE512) || defined(MCU_AT91SAM7SE512)
        outr(MC_FMR_EFC1, ((((freq + freq / 2) / 1000000UL) & 0xFF) << 16) | MC_FWS_2R3W);
#endif
    }
#endif
#ifdef EARLY_STDIO_DEV
    /* We may optionally initialize stdout as early as possible.
    ** Be aware, that no heap is available and no threads are
    ** running. We need a very basic driver here, which won't
    ** use interrupts or call malloc, NutEventXxx, NutSleep etc. */
    {
        extern NUTFILE *__fds[FOPEN_MAX];
        extern FILE *__iob[FOPEN_MAX];
 
        extern NUTDEVICE EARLY_STDIO_DEV;
        static struct __iobuf early_stdout;
        /* Initialize the output device. */
        EARLY_STDIO_DEV.dev_init(&EARLY_STDIO_DEV);
        /* Assign a static iobuf. */
        stdout = &early_stdout;
        /* Open the device. */
        __fds[1] = (int) EARLY_STDIO_DEV.dev_open(&EARLY_STDIO_DEV, "", 0, 0);
        __iob[1] = stdout;

        stdout->iob_fd = 1;
        /* Set the mode. No idea if this is required. */
        stdout->iob_mode = _O_WRONLY | _O_CREAT | _O_TRUNC;

        /* A first trial. */
        puts("\nStarting Nut/OS");
    }
#endif
#ifdef NUT_INIT_BOARD
    NutBoardInit();
#endif
    /* Initialize our heap memory. */
    NutHeapAdd(HEAP_START, HEAP_SIZE & ~3);

    /* Create idle thread. Note, that the first call to NutThreadCreate
    ** will never return. */
    NutThreadCreate("idle", NutIdle, 0,
        (NUT_THREAD_IDLESTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD);
}

/*@}*/
