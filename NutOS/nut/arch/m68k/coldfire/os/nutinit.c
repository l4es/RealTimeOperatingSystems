/*
 * Copyright 2012 by Embedded Technologies s.r.o
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
#include <arch/m68k.h>
#include <dev/board.h>

#ifdef EARLY_STDIO_DEV
#include <sys/device.h>
#include <stdio.h>
#include <fcntl.h>
#include <dev/debug.h>
struct __iobuf {
    int         iob_fd;
    uint16_t    iob_mode;
    uint8_t     iob_flags;
    int         iob_unget;
};
#endif

#ifndef NUT_THREAD_MAINSTACK
#define NUT_THREAD_MAINSTACK    1024
#endif

#ifndef NUT_THREAD_IDLESTACK
#define NUT_THREAD_IDLESTACK    512
#endif

/*!
 * \brief Memory address and sizes.
 */
extern void *__heap_start;
extern void *__heap_size;
extern void *__heap2_start;
extern void *__heap2_size;
//extern void *__stack_init_start;
//extern void *__stack_init_size;

#define HEAP_START          &__heap_start
#define HEAP_SIZE           ((size_t)&__heap_size)
#define HEAP2_START         &__heap2_start
#define HEAP2_SIZE          ((size_t)&__heap2_size)
//#define STACK_INIT_START    &__stack_init_start
//#define STACK_INIT_SIZE     ((size_t)&__stack_init_size)

#ifdef NUTMEM_STACKHEAP
extern void *__stack_heap_start;
extern void *__stack_heap_size;

#define STACK_HEAP_START    &__stack_heap_start
#define STACK_HEAP_SIZE     ((size_t)&__stack_heap_size)
#endif

#define NUT_THREAD_CONFIGSTACK  NUT_THREAD_IDLESTACK

extern void main(void *arg) __attribute__ ((noreturn));

/*!
 * \brief M68K Start thread.
 *
 * Configures OS and starts the main application thread.
 *
 */

THREAD( NutStart, arg)
{
    /*
     * Read OS configuration from non-volatile memory.
     * This cannot be called sooner, because NonVolatile memory driver
     * may use interrupts, timers and thread switching.
     */
    NutLoadConfig();

#ifdef NUT_INIT_MAIN
    /*
     * Call custom initialization.
     * NOTE: All OS parts are ready. Whole Nut/OS API may be used.
     */
    NutMainInit();
#endif

    /*
     * Create the main application thread.
     */
    NutThreadCreate("main", main, 0, (NUT_THREAD_MAINSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD);

    /*
     * Terminate the thread
     */
    NutThreadExit();
    while(1)
        NutThreadYield();
}

/*!
 * \brief Idle thread.
 *
 * Running at priority 254 in an endless loop.
 */

THREAD( NutIdle, arg)
{
    /*
     * Enable interrupts.
     * SR[IPL] = 0; (See NutThreadCreate(): ef->sr = 0x2000;)
     * IMRL[0] = 0;
     */
#ifdef MCU_MCF5225X // JS TODO
    MCF_INTC_IMRL(0) &= ~MCF_INTC_IMRL_MASKALL;
#endif

#ifdef NUT_INIT_IDLE
    /*
     * Call custom initialization
     * NOTE: context switching and timers cannot be used yet.
     */
    NutIdleInit();
#endif

    /*
     * Initialize system timers.
     */
    NutTimerInit();

    /*
     * Create the Start thread.
     */
    NutThreadCreate("config", NutStart, 0, (NUT_THREAD_CONFIGSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD);

    /*
     * Run in an idle loop at the lowest priority. We can still
     * do something useful here, like killing terminated threads
     * or putting the CPU into sleep mode.
     */
    NutThreadSetPriority(254);
    for (;;) {
        NutThreadYield();
        NutThreadDestroy();
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
    size_t heap2_size = HEAP2_SIZE;
#ifdef NUTMEM_STACKHEAP
    size_t stackheap_size = STACK_HEAP_SIZE;
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

    /* Initialize heap memory. */
    NutHeapAdd(HEAP_START, HEAP_SIZE);

    /* Add next heap region if defined */
    if (heap2_size)
        NutHeapAdd(HEAP2_START, heap2_size);

    /* Initialize stack heap (fast internal memory) if defined */
#ifdef NUTMEM_STACKHEAP
    if (stackheap_size)
        NutStackAdd(STACK_HEAP_START, stackheap_size);
#endif

    /* Create idle thread */
    NutThreadCreate("idle", NutIdle, 0, (NUT_THREAD_IDLESTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD);
}

/*@}*/
