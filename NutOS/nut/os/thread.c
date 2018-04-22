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
 * \file os/thread.c
 * \brief Multi-threading support.
 *
 * This kernel module implements the platform independent part of the Nut/OS
 * cooperative multi-threading.
 *
 * \verbatim File version $Id: thread.c 6648 2017-05-16 13:01:55Z u_bonnes $ \endverbatim
 */

#include <cfg/os.h>
#include <cfg/memory.h>

#include <string.h>

#include <sys/types.h>
#include <sys/heap.h>
#include <sys/atom.h>
#include <sys/timer.h>
#include <sys/event.h>
#include <sys/thread.h>
#include <sys/nutdebug.h>

#include <sys/osdebug.h>

#ifdef NUTTRACER
#include <sys/tracer.h>
#endif

/*!
 * \addtogroup xgThread
 */
/*@{*/

#if defined(NUT_CRITICAL_NESTING) && !defined(NUT_CRITICAL_NESTING_STACK)
unsigned int critical_nesting_level;
#endif

#ifdef __NUT_EMULATION__
// prototype
extern void NutUnixThreadYieldHook(void);  // from unix_nutinit.c
#endif

NUTTHREADINFO * runningThread;

/*!
 * \brief Thread to be killed.
 *
 * Pointer to the NUTTHREADINFO structure of the latest
 * killed thread.
 *
 * \todo Should be static, because it's locally used only.
 */
NUTTHREADINFO * killedThread;

NUTTHREADINFO * nutThreadList;

NUTTHREADINFO * runQueue;

volatile int total_pending;

void NutThreadAddPriQueue(NUTTHREADINFO * td, NUTTHREADINFO * volatile *tqpp)
{
    NUTTHREADINFO *tqp;

    NUTASSERT(td != NULL);

    td->td_queue = (HANDLE) tqpp;
    td->td_qpec = 0;            // start with clean event count

    /*
     * Be most careful not to override an intermediate event from interrupt
     * context, which may change a queue from empty to signaled state. Many
     * thanks to Michael Jones, who detected and corrected this bug.
     */
    NutEnterCritical();
    tqp = *tqpp;

    if (tqp == SIGNALED) {
        tqp = 0;
        td->td_qpec++;          // transfer the signaled state
        total_pending ++;
    } else if (tqp) {
        NutExitCritical();      // there are other threads in queue
                        // so its save to leave critical.

        while (tqp && tqp->td_priority <= td->td_priority) {
            tqpp = &tqp->td_qnxt;
            tqp = tqp->td_qnxt;
        }

        NutEnterCritical();     // back into critical
    }

    td->td_qnxt = tqp;

    *tqpp = td;
    if (td->td_qnxt && td->td_qnxt->td_qpec) {
        td->td_qpec += td->td_qnxt->td_qpec; // don't overwrite count
        td->td_qnxt->td_qpec = 0;
    }
    NutExitCritical();
}

void NutThreadRemoveQueue(NUTTHREADINFO * td, NUTTHREADINFO * volatile *tqpp)
{
    NUTTHREADINFO *tqp;

    NutEnterCritical();
    tqp = *tqpp;
    NutExitCritical();

    if (tqp != SIGNALED) {
        while (tqp) {
            if (tqp == td) {
                NutEnterCritical();
                *tqpp = td->td_qnxt;
                if (td->td_qpec) {
                    if (td->td_qnxt) {
                        td->td_qnxt->td_qpec = td->td_qpec;
                    }
                    td->td_qpec = 0;
                }
                NutExitCritical();

                td->td_qnxt = 0;
                td->td_queue = 0;
                break;
            }
            tqpp = &tqp->td_qnxt;
            tqp = tqp->td_qnxt;
        }
    }
}

void NutThreadResume(void)
{
    NUTTHREADINFO *td;
    NUTTHREADINFO **qhp;
    NUTTHREADINFO *tqp;
    unsigned int cnt;

    /*
     * Process events that have been posted from interrupt context.
     */
    td = nutThreadList;
    while (td) {
        NutEnterCritical();
        cnt = td->td_qpec;
        NutExitCritical();
        if (cnt) {
            /* In order to reduce context switching time, it is sufficient
             * to remove the thread on top of the priority ordered list. */
            qhp = (NUTTHREADINFO **)(td->td_queue);
            NutEnterCritical();
            td->td_qpec--;
            total_pending--;
            tqp = *qhp;
            NutExitCritical();
            if (tqp != SIGNALED) {
                NutEventPostAsync((HANDLE *)qhp);
            }
        }
        td = td->td_next;
    }

    /*
     * Process elapsed timers. Must be done after processing the
     * events from interupt routines.
     */
    NutTimerProcessElapsed();

    /* Check for context switch. */
    if (runningThread != runQueue) {
#ifdef NUTTRACER
        TRACE_ADD_ITEM(TRACE_TAG_THREAD_YIELD,(int)runningThread);
#endif

        if (runningThread->td_state == TDS_RUNNING) {
            runningThread->td_state = TDS_READY;
        }
        NutEnterCritical();
        NutThreadSwitch();
        NutExitCritical();
    }
}

void NutThreadWake(HANDLE timer, HANDLE th)
{
    NUTASSERT(th != NULL);

    /* clear pointer on timer and waiting queue */
    ((NUTTHREADINFO *) th)->td_timer = 0;
    ((NUTTHREADINFO *) th)->td_state = TDS_READY;
    NutThreadAddPriQueue(th, (NUTTHREADINFO **) & runQueue);
}

void NutThreadYield(void)
{

#ifdef __NUT_EMULATION__
    NutEnterCritical();
    NutUnixThreadYieldHook();
    NutExitCritical();
#endif

    /*
     * Remove current thread from runQueue and reinsert it.
     * The idle thread is the last one in the queue and will
     * never be removed.
     */
    if (runningThread->td_qnxt) {
        NutThreadRemoveQueue(runningThread, (NUTTHREADINFO **) & runQueue);
        NutThreadAddPriQueue(runningThread, (NUTTHREADINFO **) & runQueue);
    }

    /* Continue with the highest priority thread, which is ready to run. */
    NutThreadResume();
}

uint8_t NutThreadSetPriority(uint8_t level)
{
    uint8_t last = runningThread->td_priority;

    /*
     * Remove the thread from the run queue and re-insert it with a new
     * priority, if this new priority level is below 255. A priotity of
     * 255 will kill the thread.
     */
    NutThreadRemoveQueue(runningThread, &runQueue);
    runningThread->td_priority = level;
    if (level < 255) {
        NutThreadAddPriQueue(runningThread, (NUTTHREADINFO **) & runQueue);
    } else {
        NutThreadKill();
    }

    /*
     * Are we still on top of the queue? If yes, then change our status
     * back to running, otherwise do a context switch.
     */
    if (runningThread == runQueue) {
        runningThread->td_state = TDS_RUNNING;
    } else {
        runningThread->td_state = TDS_READY;
#ifdef NUTTRACER
        TRACE_ADD_ITEM(TRACE_TAG_THREAD_SETPRIO,(int)runningThread);
#endif

        NutEnterCritical();
        NutThreadSwitch();
        NutExitCritical();
    }

    return last;
}

void NutThreadExit(void)
{
    NutThreadSetPriority(255);
}

void NutThreadDestroy(void)
{
    if (killedThread) {
        NutStackFree(killedThread->td_memory);
        killedThread = 0;
    }
}

void NutThreadKill(void)
{

    NUTTHREADINFO *pCur = nutThreadList;
    NUTTHREADINFO **pp = (NUTTHREADINFO **) & nutThreadList;

    /* Free up any unfinished already killed threads. */
    NutThreadDestroy();

    /* Remove from the thread list. */
    while (pCur) {
        if (pCur == runningThread) {
            *pp = pCur->td_next;
            break;
        }

        pp = (NUTTHREADINFO **) & pCur->td_next;
        pCur = pCur->td_next;
    }

    /* Schedule for cleanup. */
    killedThread = runningThread;
}

HANDLE GetThreadByName(char * name)
{
    NUTTHREADINFO *tdp;

    if (name) {
        for (tdp = nutThreadList; tdp; tdp = tdp->td_next) {
            if (strcmp(tdp->td_name, name) == 0)
                return tdp;
        }
    } else {
        return runningThread;
    }
    return NULL;
}


/* Calculate the size if untouched stack space. */
static size_t StackAvail(NUTTHREADINFO *td)
{
    uint32_t *sp = (uint32_t *)td->td_memory;

    while(*sp++ == DEADBEEF);

    return (size_t)((uintptr_t)sp - (uintptr_t)td->td_memory);
}

size_t NutThreadStackAvailable(char *name)
{
    NUTTHREADINFO *tdp = (NUTTHREADINFO *)GetThreadByName(name);

    return tdp ? StackAvail(tdp) : 0;
}

#if defined(NUTDEBUG_CHECK_STACKMIN) || defined(NUTDEBUG_CHECK_STACK)
NUTTHREADINFO *NutThreadStackCheck(size_t minsiz)
{
    NUTTHREADINFO *tdp;

    for (tdp = nutThreadList; tdp; tdp = tdp->td_next) {
        if (StackAvail(tdp) < minsiz) {
            break;
        }
    }
    return tdp;
}
#endif

/*@}*/
