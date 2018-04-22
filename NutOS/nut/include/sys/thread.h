#ifndef _SYS_THREAD_H_
#define _SYS_THREAD_H_

/*
 * Copyright (C) 2001-2006 by egnite Software GmbH. All rights reserved.
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
 * -
 * Portions Copyright (C) 2000 David J. Hudson <dave@humbug.demon.co.uk>
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You can redistribute this file and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software Foundation;
 * either version 2 of the License, or (at your discretion) any later version.
 * See the accompanying file "copying-gpl.txt" for more details.
 *
 * As a special exception to the GPL, permission is granted for additional
 * uses of the text contained in this file.  See the accompanying file
 * "copying-liquorice.txt" for details.
 */

/*!
 * \file sys/thread.h
 * \brief Multi-threading support.
 *
 * \verbatim File version $Id: thread.h 6648 2017-05-16 13:01:55Z u_bonnes $ \endverbatim
 */

#include <compiler.h>
#include <sys/types.h>
#include <cfg/memory.h>

#include <stdint.h>

#define DEADBEEF    0xDEADBEEF

/*!
 * \addtogroup xgThread Thread Management
 * \ingroup xgNutOS
 * \anchor xrThread
 * \brief Cooperative multi-threading support.
 *
 * Typically Nut/OS is at its most useful where there are several
 * concurrent tasks that need to be undertaken at the same time.
 * To support this requirement, Nut/OS offers some kind of light
 * processes called threads. In this context a thread is a sequence
 * of executing software that can be considered to be logically
 * independent from other software that is running on the same CPU.
 *
 * All threads are executing in the same address space using the
 * same hardware resources, which significantly reduces task switching
 * overhead. Therefore it is important to stop them from causing
 * each other problems. This is particularly an issue where two or
 * more threads need to share a resources like memory locations or
 * peripheral devices.
 *
 * The system works on the principle that the most urgent thread
 * always runs. One exception to this is if a CPU interrupt arrives
 * and the interrupt has not been disabled. Each thread has a
 * priority which is used to determine how urgent it is. This
 * priority ranges from 0 to 255, with the lowest value indicating
 * the most urgent.
 *
 * Nut/OS implements cooperative multithreading. That means,
 * that threads are not bound to a fixed timeslice. Unless
 * they are waiting for specific event or explicitly yielding
 * the CPU, they can rely on not being stopped unexpectedly.
 * However, they may be interrupted by hardware interrupt
 * signals. In opposite to pre-emptive multithreading,
 * cooperative multithreading simplifies resource sharing
 * and results in faster and smaller code.
 *
 * \todo Using the special priority 255 to kill a thread is not
 * required and should be removed.
 *
 * To specify a function named Back as an independent thread, one can write
 * \code
 * #include <sys/thread.h>
 *
 * THREAD(Back, arg)
 * {
 *     for (;;) {
 *         NutSleep(1000);
 *     }
 * }
 * \endcode
 *
 * To start this thread, use
 * \code
 * #include <sys/thread.h>
 *
 * // Other code here...
 * NutThreadCreate("Bg", Back, NULL, 512);
 * // Execution continues here and concurrently in the background thread.
 * \endcode
 *
 * The functions listed below are hardware independent. Additional
 * API calls are located in the architecture dependant sections.
 * - \ref xgNutArchAvrOsContextGcc "AVR with GNU Compiler"
 * - \ref xgNutArchAvrOsContextIcc "AVR with ImageCraft Compiler"
 * - \ref xgNutAvr32OsContext "AVR32 with GNU Compiler"
 * - \ref xgNutArchArmOsContext "ARM with GNU Compiler"
 * - \ref xgNutArchCm3OsContext "Cortex M3"
 * - \ref xgNutArchUnixOsContext "Emulator (UNIX)"
 */

/*@{*/

/*!
 * Thread information structure type.
 */
typedef struct _NUTTHREADINFO NUTTHREADINFO;

/*!
 * \struct _NUTTHREADINFO thread.h sys/thread.h
 * \brief Thread information structure.
 *
 * \todo Sort items while considering alignment.
 */
struct _NUTTHREADINFO {
    NUTTHREADINFO *td_next;     /*!< \brief Linked list of all threads. */
    NUTTHREADINFO *td_qnxt;     /*!< \brief Linked list of all queued thread. */
    volatile unsigned int td_qpec;     /*!< \brief Pending event counter. */
    char td_name[9];            /*!< \brief Name of this thread. */
    uint8_t td_state;            /*!< \brief Operating state. One of TDS_ */
    uintptr_t td_sp;               /*!< \brief Stack pointer. */
    uint8_t td_priority;         /*!< \brief Priority level. 0 is highest priority. */
    uint8_t *td_memory;          /*!< \brief Pointer to heap memory used for stack. */
    HANDLE td_timer;            /*!< \brief Event timer. */
    volatile HANDLE td_queue;   /*!< \brief Root entry of the waiting queue. */
#ifdef __NUT_EMULATION__
    pthread_t td_pthread;       /*!< \brief pthread for unix emulations. */
    void (*td_fn) (void *);     /*!< \brief thread function */
    void *td_arg;               /*!< \brief args given to NutCreateThread */
    pthread_cond_t td_cv;       /*!< \brief conditional variable for unix emulations. */
    uint16_t td_cs_level;        /*! \brief number critical sections has been entered without leaving */
#endif
};

/*!
 * \name Thread States
 */
/*@{*/
#define TDS_TERM        0       /*!< Thread has exited. */
#define TDS_RUNNING     1       /*!< Thread is running. */
#define TDS_READY       2       /*!< Thread is ready to run. */
#define TDS_SLEEP       3       /*!< Thread is sleeping. */
/*@}*/

#define SLEEP_MODE_NONE 0xff

/*!
 * \brief Stack size factor.
 *
 * Configured stack sizes are multiplied with this value.
 *
 * All stack size settings of internal Nut/OS threads had been calculated
 * for size optimized code. Probably more stack space is required with
 * other compiler settings.
 *
 * For example, when GCC generates non-optimized code for source code
 * debugging, a factor of 3 should be applied to all stack sizes.
 *
 * Application code may also make use of this macro.
 *
 * \code
 * #include <sys/thread.h>
 *
 * #define MY_THREAD_STACK  ((384 * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD)
 *
 * NutThreadCreate("myth", ThreadFunc, 0, MY_THREAD_STACK);
 * \endcode
 *
 * See also \ref NUT_THREAD_STACK_ADD.
 */
#ifndef NUT_THREAD_STACK_MULT
#define NUT_THREAD_STACK_MULT   1
#endif

/*!
 * \brief Stack size summand.
 *
 * The specified value will be added to all configured stack sizes.
 *
 * See \ref NUT_THREAD_STACK_MULT.
 */
#ifndef NUT_THREAD_STACK_ADD
#define NUT_THREAD_STACK_ADD    0
#endif

/*!
 * \brief Currently running thread.
 *
 * Pointer to the NUTTHREADINFO structure of the currently
 * running thread.
 */
extern NUTTHREADINFO *runningThread;

/*!
 * \brief List of all created threads.
 *
 * Linked list of NUTTHREADINFO structures of all threads.
 * New threads are put in front. This list contains at
 * least two threads, the main application thread followed
 * by the idle thread.
 */
extern NUTTHREADINFO *nutThreadList;

/*!
 * \brief List of ready-to-run threads.
 *
 * Priority ordered linked list of NUTTHREADINFO structures
 * of all threads which are ready to run. The idle thread
 * will always remain at the end of this list.
 */
extern NUTTHREADINFO *runQueue;

extern volatile int total_pending;

#ifndef __NUT_EMULATION__
/*!
 * \private
 * \brief Initialize thread handling in the Linux emulation.
 *
 * Used in the Liux emulation only. Has to be initialized once.
 */
extern void NutThreadInit(void);
#endif

#if defined(__GNUC__) && defined (__AVR_ENHANCED__)
extern uint8_t NutThreadSetSleepMode(uint8_t mode);
#endif


/*!
 * \brief Create a new thread.
 *
 * If the current thread's priority is lower or equal than the default
 * priority (64), then the current thread is stopped and the new one
 * is started.
 *
 * \param name      String containing the symbolic name of the new thread,
 *                  up to 8 characters long.
 * \param fn        The thread's entry point, typically created by the
 *                  THREAD macro.
 * \param arg       Argument pointer passed to the new thread.
 * \param stackSize Number of bytes of the stack space allocated for
 *                  the new thread.
 *
 * \note On ARM some targets the thread must run in ARM mode.
 *       Thumb mode is not supported.
 *
 * \return Pointer to the NUTTHREADINFO structure or NULL to indicate an
 *         error.
 */
extern HANDLE NutThreadCreate(char *name, void (*fn) (void *), void *arg, size_t stackSize);

/*!
 * \brief Set the current thread's priority.
 *
 * The priority of newly created threads is set to 64,
 * but may be changed when the thread starts running.
 *
 * Changing the priority level to 255 will kill the
 * calling thread.
 *
 * When another thread with a higher or equal priority
 * is ready to run, the current thread will be stopped
 * and control of the CPU is passed to the other thread.
 *
 * The function returns the old priority, which makes it
 * easy to temporarily switch to another priority and
 * later set back the old one.
 *
 * \param level New priority level or 255 to kill the thread. Zero
 *              specifies the highest priority. The idle thread is
 *              running at level 254 (lowest priority). Application
 *              threads should use levels from 32 to 253.
 *
 * \return The old priority of this thread.
 *
 * \todo Using a specific priority level for killing a thread is actually
 *       not the best idea. NutThreadKill() can be used instead.
 */
extern uint8_t NutThreadSetPriority(uint8_t level);

/*!
 * \brief Kill the running thread.
 *
 * The thread is moved from the schedule que and
 *
 * Applications generally do not call this function.
 */
extern void NutThreadKill(void);

/*!
 * \brief Free a thread that was previously killed and release memory
 *        back to the OS.
 *
 * Called when another thread is killed and by the idle thread.
 *
 * Applications generally do not call this function, however you could
 * call it to try to reclaim memory.
 */
extern void NutThreadDestroy(void);

/*!
 * \brief End the current thread
 *
 * Terminates the current thread, in due course the memory associated
 * with the thread will be released back to the OS this is done by the
 * idle thread.
 *
 * \todo NutThreadKill() can be used instead of setting the priority level
 *       to 255.
 */
extern void NutThreadExit(void);

/*!
 * \private
 * \brief Continue with the highest priority thread, which is ready to run.
 *
 * If the currently running thread lost its top position in the queue
 * of ready-to-run threads, then the context will be switched.
 *
 * \todo Removing a single thread from a wait queue only improves context
 *       switching, but may result in an event time-out for remaining
 *       threads, although events had been posted already.
 */
extern void NutThreadResume(void);

/*!
 * \private
 * \brief Resume a previously suspended thread.
 *
 * This routine is called by the system when a
 * sleep timer elapses.
 *
 * \note This routine is used as a timer callback
 *       for NutSleep implementation
 *       Applications typically do not call this
 *       function.
 *
 * \param timer Handle of the elapsed timer.
 * \param th    Handle of the thread to wake up.
 *
 * \todo Used by the timer module. Should be moved there, because not all
 *       applications will use of NutSleep().
 */
extern void NutThreadWake(HANDLE timer, HANDLE th);

/*!
 * \brief Give up the CPU.
 *
 * If another thread within the same or higher priority
 * is ready to run, then the current thread is stopped
 * and the other one is started.
 *
 */
extern void NutThreadYield(void);

/*!
 * \private
 * \brief Add a thread to a prioritiy ordered queue.
 *
 * Insert the thread into a specified queue behind
 * the last thread with lower or equal priority.
 *
 * \param td   Pointer to NUTTHREADINFO of the thread to be
 *             inserted in the queue.
 * \param tqpp Pointer to the root of the queue.
 */
extern void NutThreadAddPriQueue(NUTTHREADINFO * td, NUTTHREADINFO * volatile *tqpp);

/*!
 * \private
 * \brief Remove a thread from a specified queue.
 *
 * \note Depending on the given queue, CPU interrupts must have
 * been disabled before calling this function.
 *
 * \param td   Pointer to NUTTHREADINFO of the thread to be
 *             removed from the queue.
 * \param tqpp Pointer to the root of the queue.
 */
extern void NutThreadRemoveQueue(NUTTHREADINFO * td, NUTTHREADINFO * volatile *tqpp);

/*!
 * \private
 * \brief Switch to another thread.
 *
 * Stop the current thread, saving its context. Then start the
 * one with the highest priority, which is ready to run.
 *
 * Application programs typically do not call this function.
 *
 * \note CPU interrupts must be disabled before calling this function.
 *
 */
extern void NutThreadSwitch(void);

/*!
 * \brief Query handle of a thread with a specific name.
 *
 * \param name Case sensitive symbolic name of the thread.
 *
 * \return Handle of the thread, if it exists. Otherwise NULL is returned.
 *
 * \todo Rarely used helper function. Should be placed in a seperate module.
 */
extern HANDLE GetThreadByName(char *name);

/*!
 * \brief Check all Nut/OS threads for sufficient stack space.
 *
 * See NutThreadStackAvailable() for further informations.
 *
 * \param minleft Number of bytes that should have been unused.
 *
 * \return Pointer to the first thread that used too much stack
 *         space or NULL, if enough stack space has been available
 *         so far in all threads.
 */
extern NUTTHREADINFO *NutThreadStackCheck(size_t minsiz);

/*!
 * \brief Return the size of unused stack space.
 *
 * The stack will be treated as an array of 32-bit values, which
 * are initially set to \ref DEADBEEF. Starting at the stack's
 * bottom, this function will simply count the number of array
 * members, which still contain the original value.
 *
 * This implicates at least three limitations:
 *
 * - Overflows may be undetected, if some local variables are unmodified.
 * - The result may be wrong, if local variables contain the DEADBEEF value.
 * - The result is a multiple of 4 bytes.
 *
 * This function is available only if NUTDEBUG_CHECK_STACK has
 * been defined during system build.
 *
 * \param name Symbolic name of the thread.
 *
 * \return The number of bytes which never had been touched.
 */
extern size_t NutThreadStackAvailable(char *name);

/*!
 * \brief Macro for thread entry definitions.
 */
#define THREAD(threadfn, arg) \
void threadfn(void *arg) __attribute__ ((noreturn)); \
void threadfn(void *arg)

/*@}*/

#endif
