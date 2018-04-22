/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

/* Parts taken from 
 * http://www.opensource.apple.com/source/lldb/lldb-159/source/Plugins/Process/Utility/ARMUtils.h?txt
 *
 * University of Illinois/NCSA
 * Open Source License
 * 
 * Copyright (c) 2010 Apple Inc.
 * All rights reserved.
 * 
 * Developed by:
 * 
 *     LLDB Team
 * 
 *     http://lldb.llvm.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal with
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 *    * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimers.
 * 
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *       this list of conditions and the following disclaimers in the
 *       documentation and/or other materials provided with the distribution.
 * 
 *     * Neither the names of the LLDB Team, copyright holders, nor the names of 
 *       its contributors may be used to endorse or promote products derived from 
 *       this Software without specific prior written permission.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE
 * SOFTWARE.
 * 
 */ 


/*!
 * \file arch/cm3/cmsis/cortex_debug.c
 *
 * \verbatim
 * $Id: cortex_debug.c $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <sys/types.h>
#include <sys/thread.h>
#include <inttypes.h>
#include <cfg/cortex_debug.h>
#include <arch/cm3/cortex_debug.h>
#include <toolchain.h>
#if defined (MCU_LPC17xx)
#include <arch/cm3/nxp/lpc17xx_debug_macro.h>
#elif defined (MCU_STM32)
#include <arch/cm3/stm/stm32_debug_macro.h>
#endif

extern void * _evect;           /* End of vector table / start of text segment in FLASH */
extern void * _etext;           /* Start of constants in FLASH */

#define OPCODE_PUSH_R7         0xB480
#define OPCODE_PUSH_R7_LR      0xB580
#define OPCODE_PUSH_MASK       0xFE00
#define OPCODE_PUSH            0xB400

#define OPCODE_SUB_SP          0xB080
#define OPCODE_SUB_SP_MASK     0xFF80

#define OPCODE_SUB_W_SP        0xF1AD
#define OPCODE_SUB_W_SP_MASK   0xFBEF

#define OPCODE_ADD_R7_SP       0xAF00
#define OPCODE_ADD_R7_SP_MASK  0xFF00

static char *exception_names[6] = {"Default", "NMI", "Hard Fault", "Mem Fault", "Bus Fault", "Usage Fault"};

static RAMFUNC void DebugPrint(const char *str)
{
#ifdef DEBUG_MACRO
    while (*str != 0) {
        if (*str == '\n') DebugPut('\r');
        DebugPut(*str);
        str++;
    }
#endif
}

static RAMFUNC void DebugPrintU32Hex(uint32_t val) {
#ifdef DEBUG_MACRO
    int i;
    for (i = 7; i >= 0; i--) {
        register int nibble = (val >> (4 * i)) & 0x0F;
        DebugPut(nibble < 10 ? '0' + nibble : 'a' + nibble - 10);
    }
#endif
}

#ifdef DEBUG_BACKTRACE

static inline RAMFUNC uint32_t bits(const uint32_t val, const uint32_t msbit, const uint32_t lsbit)
{
    return (val & ~(0xFFFFFFFF << (msbit + 1))) >> lsbit;
}

static inline RAMFUNC uint32_t bit(const uint32_t val, const uint32_t msbit)
{
    return bits(val, msbit, msbit);
}

static RAMFUNC uint32_t ror(uint32_t val, uint32_t N, uint32_t shift)
{
    uint32_t m = shift % N;
    return (val >> m) | (val << (N - m));
}

// (imm32, carry_out) = ThumbExpandImm_C(imm12, carry_in)
static inline RAMFUNC uint32_t ThumbExpandImm_C(uint32_t opcode, uint32_t carry_in, uint32_t *carry_out)
{
    uint32_t imm32; // the expaned result
    const uint32_t i = bit(opcode, 26);
    const uint32_t imm3 = bits(opcode, 14, 12);
    const uint32_t abcdefgh = bits(opcode, 7, 0);
    const uint32_t imm12 = i << 11 | imm3 << 8 | abcdefgh;

    if (bits(imm12, 11, 10) == 0)
    {
        switch (bits(imm12, 9, 8)) {
        case 0:
            imm32 = abcdefgh;
            break;

        case 1:
            imm32 = abcdefgh << 16 | abcdefgh;
            break;

        case 2:
            imm32 = abcdefgh << 24 | abcdefgh << 8;
            break;

        case 3:
            imm32 = abcdefgh  << 24 | abcdefgh << 16 | abcdefgh << 8 | abcdefgh; 
            break;
        }
        *carry_out = carry_in;
    }
    else
    {
        const uint32_t unrotated_value = 0x80 | bits(imm12, 6, 0);
        imm32 = ror(unrotated_value, 32, bits(imm12, 11, 7));
        *carry_out = bit(imm32, 31);
    }
    return imm32;
}

static inline RAMFUNC uint32_t ThumbExpandImm(uint32_t opcode)
{
    // 'carry_in' argument to following function call does not affect the imm32 result.
    uint32_t carry_in = 0;
    uint32_t carry_out;
    return ThumbExpandImm_C(opcode, carry_in, &carry_out);
}

// imm32 = ZeroExtend(i:imm3:imm8, 32)
static inline RAMFUNC uint32_t ThumbImm12(uint32_t opcode)
{
  const uint32_t i = bit(opcode, 26);
  const uint32_t imm3 = bits(opcode, 14, 12);
  const uint32_t imm8 = bits(opcode, 7, 0);
  const uint32_t imm12 = i << 11 | imm3 << 8 | imm8;
  return imm12;
}

static inline RAMFUNC uint32_t ThumbImm7(uint32_t opcode)
{
  return bits(opcode, 6, 0);
}

static inline RAMFUNC uint32_t ThumbImm8(uint32_t opcode)
{
  return bits(opcode, 7, 0);
}

#endif

void RAMFUNC CortexRegDump(uint32_t *arg, int exception_type, uint32_t *fp)
{
    uint32_t  stacked_r0;
    uint32_t  stacked_r1;
    uint32_t  stacked_r2;
    uint32_t  stacked_r3;
    uint32_t  stacked_r12;
    uint32_t  stacked_lr;
    uint32_t  stacked_pc;
    uint32_t  stacked_psr;

    DebugPrint("\n---------------------------------------------------\n");
    stacked_r0 = ((uint32_t) arg[0]);
    stacked_r1 = ((uint32_t) arg[1]);
    stacked_r2 = ((uint32_t) arg[2]);
    stacked_r3 = ((uint32_t) arg[3]);

    stacked_r12 = ((uint32_t) arg[4]);
    stacked_lr  = ((uint32_t) arg[5]);
    stacked_pc  = ((uint32_t) arg[6]);
    stacked_psr = ((uint32_t) arg[7]);

    DebugPrint("["); DebugPrint(exception_names[exception_type]); DebugPrint(" handler - all numbers in hex]\n");

#if defined(SCB_HFSR_FORCED_Msk)
    if (exception_type == 2) {
        /* This is a hard fault handler */
        if ((SCB->HFSR & SCB_HFSR_FORCED_Msk) != 0) {
            DebugPrint("Forced Hard Fault!\n");
        }
    }
#endif

    DebugPrint("\nR0        = 0x"); DebugPrintU32Hex(stacked_r0);
    DebugPrint("\nR1        = 0x"); DebugPrintU32Hex(stacked_r1);
    DebugPrint("\nR2        = 0x"); DebugPrintU32Hex(stacked_r2);
    DebugPrint("\nR3        = 0x"); DebugPrintU32Hex(stacked_r3);
    DebugPrint("\nR12       = 0x"); DebugPrintU32Hex(stacked_r12);
    DebugPrint("\nLR [R14]  = 0x"); DebugPrintU32Hex(stacked_lr);
    DebugPrint("\nPC [R15]  = 0x"); DebugPrintU32Hex(stacked_pc);
    DebugPrint("\nPSR       = 0x"); DebugPrintU32Hex(stacked_psr);
    DebugPrint("\nBFAR      = 0x"); DebugPrintU32Hex(*((volatile unsigned long *)(0xE000ED38)));
    DebugPrint("\nCFSR      = 0x"); DebugPrintU32Hex(*((volatile unsigned long *)(0xE000ED28)));
    DebugPrint("\nHFSR      = 0x"); DebugPrintU32Hex(*((volatile unsigned long *)(0xE000ED2C)));
    DebugPrint("\nDFSR      = 0x"); DebugPrintU32Hex(*((volatile unsigned long *)(0xE000ED30)));
    DebugPrint("\nAFSR      = 0x"); DebugPrintU32Hex(*((volatile unsigned long *)(0xE000ED3C)));
    DebugPrint("\nSCB_SHCSR = 0x"); DebugPrintU32Hex(SCB->SHCSR);
    DebugPrint("\n---------------------------------------------------\n");

#ifdef DEBUG_BACKTRACE
    /* These are experimental enhancements to the exception handler which
     * allow to print out a stack dump and a call trace.
     * 
     * For these extensions to work properly, we need frame pointer support.
     * Compiler optimisation must be disabled (-O0)
     *
     * At a minimum, we need the following compiler flags to be enabled:
     * 
     * -O0 -mtpcs-frame -mtpcs-leaf-frame -fno-omit-frame-pointer
     *
     * Please consider this code highly experimental. Even if the frame pointer 
     * support is enabled in the compiler, gcc does not create correct frame
     * pointers when compiling for CortexM. Instead of base address of the
     * current stack frame, the framepointer register (r7 for CortexM) holds 
     * the stack pointer address after allocating stack for local variables etc...
     * This value is quite useless, as we need to know the start of the stack 
     * frame, where the lr register value was pushed.
     *
     * Usually the following function entry code is generated (with a few
     * variations):
     *  
     * push    {r7, lr}
     * sub     sp, #32
     * add     r7, sp, #0
     *
     * The below code now trys to detect and decode these three instructions.
     * The code searches backwards from the last known PC value.
     * If the above instructions are found, it is assumed, that this is the 
     * entry of the current function, and the instructions are decoded
     * to back calculate the real stack pointer value at entry into the function.
     *
     * In the above example:
     * 
     * The "frame pointer" is r7. We have to substract 0 and add 32 to the 
     * framepointer to get the stack pointer value right after pushing LR to 
     * the stack. We then read LR from the stack memory to get the address
     * where the current functions has been called from. Further more we read 
     * R7 from the stack to also get the next "frame pointer".
     * The above steps are repeated until we reached the base of the current
     * stack or got an invalid stack pointer value.
     * 
     * The backtrace only works for functions that are compiled with exactly 
     * the above compiler options. E.g. it will not work for included
     * binary only libraries like newlib.                      
     */
    uint32_t  offs;
    uint32_t *sp;
    uint16_t *pc;
    uint16_t *lr;
    uint16_t *saved_pc;
    
    DebugPrint("\nStack:\n\n");
    /* Print out stack, beginning with the current frame pointer. Frame pointer 
     * (r7) is only available, if compiled with above compiler options.
     */
    sp = fp;
    for (offs = 0; offs < 128; offs += 4) {
        DebugPrintU32Hex((uint32_t)sp); DebugPrint(": "); 
        DebugPrintU32Hex(*(sp++)); DebugPut(' '); DebugPrintU32Hex(*(sp++)); DebugPut(' '); DebugPrintU32Hex(*(sp++)); DebugPut(' '); DebugPrintU32Hex(*(sp++)); DebugPut(' '); 
        DebugPrint("\n");
    }
    
    DebugPrint("\n---------------------------------------------------\n");

    /* We try to trace back the last function calls and print out a stack trace */
    DebugPrint("\nCall Trace:\n\n");

    DebugPrint("\nInitial fp: 0x");DebugPrintU32Hex((uint32_t) fp); DebugPrint("\n\n");

    pc = (uint16_t*)(stacked_pc & 0xFFFFFFFE);

    if (fp <= (uint32_t*)runningThread->td_memory) {
        DebugPrint("\nInvalid frame pointer! Out of thread stack bounds:\n\n");
        DebugPrint("PC: ");DebugPrintU32Hex((uint32_t)pc);  DebugPrint("\n");
        DebugPrint("FP: ");DebugPrintU32Hex((uint32_t)fp);  DebugPrint("\n");
        DebugPrint("Thread stack base: ");DebugPrintU32Hex((uint32_t)runningThread->td_memory);  DebugPrint("\n");
    } else {
        DebugPrint("\n   Addr:        Function:      Offset:      Frame pointer:\n");
        lr = pc;
        int count = 0;
        while (((uint32_t)pc > (stacked_pc & 0xF8000000) + 2) &&
               ((uint32_t)fp > 0) && (fp > (uint32_t*)runningThread->td_memory)) {
            /* Probe for the instruction entry "signature": push {r7, lr, more regs}; sub sp, #x; add r7, sp, #y; */
            if ((((*pc & OPCODE_PUSH_MASK) == OPCODE_PUSH) && ((*(pc + 1) & OPCODE_SUB_SP_MASK) == OPCODE_SUB_SP) && ((*(pc + 2) & OPCODE_ADD_R7_SP_MASK) == OPCODE_ADD_R7_SP)) ||
                (((*pc & OPCODE_PUSH_MASK) == OPCODE_PUSH) && ((*(pc + 1) & OPCODE_SUB_W_SP_MASK) == OPCODE_SUB_W_SP) && ((*(pc + 3) & OPCODE_ADD_R7_SP_MASK) == OPCODE_ADD_R7_SP)) ||
                (((*pc & OPCODE_PUSH_MASK) == OPCODE_PUSH) && ((*(pc + 1) & OPCODE_ADD_R7_SP_MASK) == OPCODE_ADD_R7_SP))) {
                /* Found the function entry. Minimum R7 is pushed. LR only on node functions */
                DebugPrint("\n   0x"); DebugPrintU32Hex((uint32_t) lr); DebugPrint(" = [0x"); DebugPrintU32Hex((uint32_t) pc); DebugPrint("] + 0x"); DebugPrintU32Hex((uint32_t) (lr - pc) * 2); DebugPrint(": ");
                /* Increment pc to read next opcode. Very likely 'sub pc, #x' */
                saved_pc = pc;
                pc ++;
                if ((*pc & OPCODE_SUB_SP_MASK) == OPCODE_SUB_SP) {
                    /* sub pc, #x */
                    DebugPrint(" Sub: 0x"); DebugPrintU32Hex(ThumbImm7(*pc)); DebugPrint("\n");
                    
                    fp = fp + ThumbImm7(*pc); /* Add value which was substracted and function entry + 1, for the LR and the FP address (R7) itself */
                    pc ++;
                } else 
                if ((*pc & OPCODE_SUB_W_SP_MASK) == OPCODE_SUB_SP) {
                    /* sub.w sp, sp, #x */
                    fp = fp + ThumbExpandImm((uint32_t)*pc << 16 | (uint32_t)*(pc + 1)) / sizeof(uint32_t);
                    pc += 2;
                }

                if ((*pc & OPCODE_ADD_R7_SP_MASK) == OPCODE_ADD_R7_SP) {
                    /* add r7, sp, #x */
                    fp = fp - ThumbImm8(*pc);

                    DebugPrint(" fp: 0x");
                    DebugPrintU32Hex((uint32_t) fp);

//                    if (*(pc - 2) == OPCODE_PUSH_R7) {
                    if (*(saved_pc) == OPCODE_PUSH_R7) {
                        /* Set pc to stacked_lr value, which is the calling function */
                        pc = (uint16_t*)(stacked_lr & 0xFFFFFFFE);
                    } else {
                        pc = (uint16_t*)((*(fp + 1)) & 0xFFFFFFFE);
                    }
                    lr = pc;

                    fp = (uint32_t*)*fp;
                    
                    count ++;

                    if (((uint32_t)fp == 0) || (fp < (uint32_t*)runningThread->td_memory)) {
                        DebugPrint("\n\nInvalid next frame pointer! Out of thread stack bounds:"); 
                        DebugPrint("\nfp: 0x"); DebugPrintU32Hex((uint32_t) fp);
                        DebugPrint("\npc: 0x"); DebugPrintU32Hex((uint32_t) pc);
                        DebugPrint("\n");
                    }
                }
            }
            pc --;
        }
    }
#endif    
    DebugPrint("\n\nHalted...");
    
    __asm("BKPT #0\n") ; // Break into the debugger
    while(1);
}
