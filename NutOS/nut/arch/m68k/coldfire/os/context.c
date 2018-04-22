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

#include <cfg/os.h>
#include <string.h>
#include <stdio.h>
#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>

/*!
 * \addtogroup xgNutArchM68kOsContext
 */
/*@{*/

/*!
 * \brief M68k GCC context switch frame layout.
 *
 * This is the layout of the stack after a thread's context has been
 * switched-out. The stack pointer is stored in the thread info and
 * points to this structure.
 */
typedef struct {
    uint32_t d[6];  // d3-d7 (NOTE: d0-d2, a0-a1 are working registers for C language)
    uint32_t a[4];   // a2-a5
    uint16_t pad;    // alignment to 32 bit
    uint16_t sr;     // status register
} SWITCHFRAME;

/*!
 * \brief Thread entry frame layout.
 *
 * This is the stack layout being build to enter a new thread.
 */
typedef struct {
    uint32_t fp_entry;  // value fp register
    uint32_t pc_entry;  // function NutThreadEntry
    uint16_t pad;       // alignment to 32 bit
    uint16_t sr;        // status register
    uint32_t d0;        // arg function
    uint32_t pc_fn;     // thread's entry funciton
} ENTERFRAME;

/*!
 * \ Remove thread from thread list, set tread to be killed and yield
 * After this call function NutThreadDestroy in NutIdle to release memory back to the OS.
 */
void NutThreadExitAndYield(void)
{
    NutThreadExit();
    NutThreadYield();
}

/*!
 * \brief Enter a new thread.
 */
void NutThreadEntry(void)
{
    /* The thread's entry point ef->pc_fn to jsr */
    __asm__ volatile("move.l    12(%sp), %a0");
    /* Move sp and fp to save place in stack. */
    __asm__ volatile("add.l     #12, %sp");
    __asm__ volatile("move.l    %sp, %fp");
    /* The thread's arg copy ef->d0 to use in sub rutine */
    __asm__ volatile("move.l    -4(%sp), (%sp)");
    /* NutExitCritical - load sr */
    __asm__ volatile("move.l    -8(%sp), %d0");
    __asm__ volatile("move.w    %d0, %sr");
    /* Jump to thread's entry point ef->pc_fn */
    __asm__ volatile("jsr       (%a0)");
    /* If returned from subrutine terminate thread */
    NutThreadExitAndYield();
}

/*!
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
void NutThreadSwitch(void)
{
#define LOCAL_VARIABLES_SIZE (sizeof(SWITCHFRAME))
    SWITCHFRAME sf;

    /* Save CPU context. */
    __asm__ volatile("movem.l   %%d2-%%d7/%%a2-%%a5, %[sf_d0]":[sf_d0] "=m" (sf.d[0]));
    __asm__ volatile("move.w    %sr,%d0");
    __asm__ volatile("move.w    %%d0,%[sf_sr]" :[sf_sr] "=m" (sf.sr));
    __asm__ volatile("move.l    %%sp,%[td_sp]" :[td_sp] "=m" (runningThread->td_sp));

    /*
     * This defines a global label, which may be called
     * as an entry point into this function (used in NutThreadCreate())
     */
    asm volatile(".global thread_start\n"
            "thread_start:\n\t"::);

    /* Select thread on top of the run queue. */
    runningThread = runQueue;
    runningThread->td_state = TDS_RUNNING;

    /* Restore context. */
    /* load stack pointer from NUTTHREADINFO */

    // NOTE: "sf" variable shows garbage in Eclipse Variables View (frame pointer is not configured yet)
    __asm__ volatile("move.l    %[td_sp],%%sp" ::[td_sp] "m" (runningThread->td_sp));
    __asm__ volatile("move.l    %sp,%a0");
    __asm__ volatile("adda.l    %[size],%%a0" ::[size] "i" (LOCAL_VARIABLES_SIZE));
    __asm__ volatile("move.l    %a0,%fp");
    // now you can use the "sf" variable (local variables)
    __asm__ volatile("move.w    %[sf_sr],%%d0" ::[sf_sr] "m" (sf.sr));
    __asm__ volatile("move.w    %d0,%sr");
    __asm__ volatile("movem.l   %[sf_d0], %%d2-%%d7/%%a2-%%a5"::[sf_d0] "m" (sf.d[0]));
}

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
 * \note The thread must run in ARM mode. Thumb mode is not supported.
 *
 * \return Pointer to the NUTTHREADINFO structure or 0 to indicate an
 *         error.
 */
HANDLE NutThreadCreate(char * name, void(*fn)(void *), void *arg, size_t stackSize)
{
    uint8_t *threadMem;
    SWITCHFRAME *sf;
    ENTERFRAME *ef;
    NUTTHREADINFO *td;
    size_t alloc_size;

    /*
     * Allocate stack and thread info structure in one block.
     * We sill setup the following layout:
     *
     * Upper memory addresses.
     *
     *              +--------------------+
     *              I                    I
     *              I   NUTTHREADINFO    I
     *              I                    I
     * td ->        +-----+--------------+ <- Stack top
     *              I     I              I
     *              I  T  I   ENTERFRAME I
     *              I  H  I              I
     * ef ->        I  R  +--------------+
     *              I  E  I              I    ^
     *              I  A  I  SWITCHFRAME I    I
     *              I  D  I              I    I  pop moves up
     * sf ->        I     +--------------+ <- Initial stack pointer
     *              I  S  I              I    I  push moves down
     *              I  T  I Application  I    I
     *              I  A  I Stack        I    V
     *              I  C  I              I
     *              I  K  I              I
     * threadMem -> +-----+--------------+ <- Stack bottom
     *
     * Lower memory addresses.
     */

    /* Align to the 4 byte boundary */
    alloc_size = stackSize + sizeof(NUTTHREADINFO);
    alloc_size += 3;
    alloc_size &= ~3;

    if ((threadMem = NutStackAlloc(alloc_size)) == 0) {
        if ((threadMem = NutHeapAlloc(alloc_size)) == 0) {
            return 0;
        }
    }
    td = (NUTTHREADINFO *) ((threadMem + stackSize) & ~3);
    ef = (ENTERFRAME *) ((uintptr_t) td - sizeof(ENTERFRAME));
    sf = (SWITCHFRAME *) ((uintptr_t) ef - sizeof(SWITCHFRAME));

    /* 
     * Set predefined values at the stack bottom. May be used to detect
     * stack overflows.
     */
#if defined(NUTDEBUG_CHECK_STACKMIN) || defined(NUTDEBUG_CHECK_STACK)
    {
        uint32_t *fip = (uint32_t *)threadMem;
        while (fip < (uint32_t *)sf) {
            *fip++ = DEADBEEF;
        }
    }
#else
    *((uint32_t *) threadMem) = DEADBEEF;
    *((uint32_t *) (threadMem + 4)) = DEADBEEF;
    *((uint32_t *) (threadMem + 8)) = DEADBEEF;
    *((uint32_t *) (threadMem + 12)) = DEADBEEF;
#endif

    /*
     * Setup the entry frame to simulate C function entry.
     */
    ef->fp_entry = (uintptr_t) &ef->pc_entry; // no local variables fp = sp
    ef->pc_entry = (uintptr_t) NutThreadEntry;
    ef->sr = 0x2000; // supervisor mode + allow all interrupts (exit critical section)
    ef->d0 = (uintptr_t) arg;
    ef->pc_fn = (uintptr_t) fn;

    /*
     * Setup the switch frame.
     */
    sf->sr = 0x2700; // supervisor mode + disallow interrupts (enter critical section)

    /*
     * Initialize the thread info structure and insert it into the
     * thread list and the run queue.
     */
    memcpy(td->td_name, name, sizeof(td->td_name) - 1);
    td->td_name[sizeof(td->td_name) - 1] = 0;
    td->td_state = TDS_READY;
    td->td_sp = (uintptr_t) sf;
    td->td_priority = 64;
    td->td_memory = threadMem;
    td->td_timer = 0;
    td->td_queue = 0;

    NutEnterCritical();
    td->td_next = nutThreadList;
    nutThreadList = td;
    NutThreadAddPriQueue(td, (NUTTHREADINFO **) &runQueue);

    /*
     * If no thread is active, switch to new thread.
     */
    if (runningThread == NULL) {
        NutEnterCritical();
        asm volatile ("jmp thread_start\n\t"::);
        /* we will never come back here .. */
    }

    /*
     * If current context is not in front of the run queue (highest
     * priority), then switch to the thread in front.
     */
    if (runningThread != runQueue) {
        runningThread->td_state = TDS_READY;
        NutThreadSwitch();
    }
    NutExitCritical();

    return td;
}

/*@}*/
