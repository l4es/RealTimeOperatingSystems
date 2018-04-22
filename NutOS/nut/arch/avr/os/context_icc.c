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

/*!
 * \file arch/avr/os/context_icc.c
 * \brief Context switching ported to ImageCraft AVR.
 *
 * \verbatim File version $Id: context_icc.c 4477 2012-08-20 17:50:01Z haraldkipp $ \endverbatim
 */

#include <cfg/os.h>
#include <cfg/memory.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>

/*!
 * \addtogroup xgNutArchAvrOsContextIcc ImageCraft Context Switching for AVR CPUs
 * \ingroup xgNutArchAvrOs
 * \brief Thread context switching for ImageCraft ICCAVR.
 */
/*@{*/

/*!
 * \brief ICC AVR context switch frame layout.
 *
 * This is the layout of the stack after a thread's context has been
 * switched-out.
 */
typedef struct {
    uint8_t csf_r29;
    uint8_t csf_r28;
    uint8_t csf_r23;
    uint8_t csf_r22;
    uint8_t csf_r21;
    uint8_t csf_r20;
    uint8_t csf_r15;
    uint8_t csf_r14;
    uint8_t csf_r13;
    uint8_t csf_r12;
    uint8_t csf_r11;
    uint8_t csf_r10;
#ifdef __AVR_ATmega2561__
    uint8_t csf_pcex;
#endif
    uint8_t csf_pchi;
    uint8_t csf_pclo;
} SWITCHFRAME;

/*!
 * \brief Thread entry frame layout.
 *
 * This is the stack layout being build to enter a new thread.
 */
typedef struct {
    uint8_t cef_arghi;
    uint8_t cef_arglo;
    /*!
     * \brief ImageCraft software stack.
     */
    uint8_t cef_yhi;
    uint8_t cef_ylo;
    uint8_t cef_rampz;
    uint8_t cef_sreg;
    uint8_t cef_r1;
#ifdef __AVR_3_BYTE_PC__
    uint8_t cef_pcex;
#endif
    uint8_t cef_pchi;
    uint8_t cef_pclo;
} ENTERFRAME;


/*
 * This code is executed when entering a thread.
 */
static void NutThreadEntry(void)
{
    asm("pop r17");             // first parameter for ICC
    asm("pop r16");
    asm("pop r29");             // SW-Stack; Y-Register
    asm("pop r28");
    asm("pop r0");              // r0 = _tmp_reg_
    asm("out 0x3B, r0");        // RAMPZ; replace with define later
    asm("pop r0");
    asm("pop r1");              // r1 = _zero_reg_
    asm("out 0x3F, r0");        // SREG; replace with define later
    asm("reti");
}

void NutThreadSwitch(void)
{
    /*
     * Save all CPU registers.
     */
    register uint8_t i = 0;
    register uint8_t j = 0;

    asm("push r10");
    asm("push r11");
    asm("push r12");
    asm("push r13");
    asm("push r14");
    asm("push r15");
    asm("push r20");
    asm("push r21");
    asm("push r22");
    asm("push r23");
    asm("push r28");
    asm("push r29");
    asm("in %i, $3D");          // SPL
    asm("in %j, $3E");          // SPH

    runningThread->td_sp = (((uint16_t) j) << 8) & 0xFF00 | (i & 0xFF);

    /*
     * This defines a global label, which may be called
     * as an entry point into this function.
     */
    asm(".globl thread_start");
    asm("thread_start:");

    /*
     * Reload CPU registers from the thread in front
     * of the run queue.
     */
    runningThread = runQueue;
    runningThread->td_state = TDS_RUNNING;

    i = (uint8_t) (runningThread->td_sp & 0xFF);
    j = (uint8_t) ((runningThread->td_sp >> 8) & 0xFF);

    asm("out $3D, %i");         // SPL
    asm("out $3E, %j");         // SPH
    asm("pop r29");
    asm("pop r28");
    asm("pop r23");
    asm("pop r22");
    asm("pop r21");
    asm("pop r20");
    asm("pop r15");
    asm("pop r14");
    asm("pop r13");
    asm("pop r12");
    asm("pop r11");
    asm("pop r10");
}

HANDLE NutThreadCreate(uint8_t * name, void (*fn) (void *), void *arg, size_t stackSize)
{
    uint8_t *threadMem;
    SWITCHFRAME *sf;
    ENTERFRAME *ef;
    NUTTHREADINFO *td;
    uint16_t yreg;
    const uint8_t *paddr;

    /*
     * Allocate stack and thread info structure in one block.
     */
    if ((threadMem = NutStackAlloc(stackSize + sizeof(NUTTHREADINFO))) == 0) {
        return 0;
    }

    td = (NUTTHREADINFO *) (threadMem + stackSize);
    ef = (ENTERFRAME *) ((uint16_t) td - sizeof(ENTERFRAME));
    sf = (SWITCHFRAME *) ((uint16_t) ef - sizeof(SWITCHFRAME));


    memcpy(td->td_name, name, sizeof(td->td_name) - 1);
    td->td_name[sizeof(td->td_name) - 1] = 0;
    td->td_sp = (uint16_t) sf - 1;
    td->td_memory = threadMem;
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
    td->td_priority = 64;

    /*
     * Setup entry frame to simulate C function entry.
     */
    paddr = (const uint8_t *) fn;
    ef->cef_pclo = *paddr;
    ef->cef_pchi = *(paddr + 1);
#ifdef __AVR_3_BYTE_PC__
    ef->cef_pcex = *(paddr + 2);
#endif
    ef->cef_sreg = 0x80;
    ef->cef_rampz = 0;
    ef->cef_r1 = 0;

    ef->cef_arglo = (uint8_t) (((uint16_t) arg) & 0xff);
    ef->cef_arghi = (uint8_t) (((uint16_t) arg) >> 8);

    yreg = td->td_sp - 40;
    ef->cef_yhi = (uint8_t) (yreg >> 8);
    ef->cef_ylo = (uint8_t) (yreg & 0xFF);

    paddr = (const uint8_t *) NutThreadEntry;
    sf->csf_pclo = *paddr;
    sf->csf_pchi = *(paddr + 1);
#ifdef __AVR_3_BYTE_PC__
    sf->csf_pcex = *(paddr + 2);
#endif

    /*
     * Insert into the thread list and the run queue.
     */

    td->td_next = nutThreadList;
    nutThreadList = td;
    td->td_state = TDS_READY;
    td->td_timer = 0;
    td->td_queue = 0;
#ifdef NUTDEBUG
    if (__os_trf)
        fprintf(__os_trs, "Cre<%04x>", (uintptr_t) td);
#endif

    NutThreadAddPriQueue(td, (NUTTHREADINFO **) & runQueue);

#ifdef NUTDEBUG
    if (__os_trf) {
        NutDumpThreadList(__os_trs);
        //NutDumpThreadQueue(__os_trs, runQueue);
    }
#endif

    /*
     * If no thread is active, switch to new thread.
     */
    if (runningThread == 0) {
        NutEnterCritical();
        asm("rjmp thread_start");
        /* we will never come back here .. */
    }

    /*
     * If current context is not in front of
     * the run queue (highest priority), then
     * switch to the thread in front.
     */
    if (runningThread != runQueue) {
        runningThread->td_state = TDS_READY;
#ifdef NUTDEBUG
        if (__os_trf)
            fprintf(__os_trs, "New<%04x %04x>", (uintptr_t) runningThread, (uintptr_t) runQueue);
#endif
        NutEnterCritical();
        NutThreadSwitch();
        NutExitCritical();
    }

    return td;
}

/*@}*/
