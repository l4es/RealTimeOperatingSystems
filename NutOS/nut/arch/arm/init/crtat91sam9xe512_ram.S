/*
 * Copyright (C) 2012 by egnite GmbH.
 * Copyright (C) 2006-2007 by egnite Software GmbH.
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
 * $Id: crtat91sam9xe512_ram.S 4053 2012-03-26 14:41:44Z haraldkipp $
 */

        .nolist

#include <cfg/clock.h>
#include <cfg/memory.h>

#include <arch/arm.h>

        .psize      0
        .list

/*
 * This file contains the runtime initialization for AT91SAM9XE512
 * based systems.
 *
 * The related linker file is at91sam9xe512_ram.ld.
 *
 * Code and data is located in external RAM and must be loaded by a boot
 * loader or debugger, which initializes the basic clocks. If this code
 * runs in SDRAM, it is assumed that it has been initialized as well.
 */

/*
 * Stack sizes.
 *
 * Interrupt nesting is currently not supported. Therefore dedicated
 * stacks are used for all exceptions.
 */
#ifndef INI_STACK_SIZE
#define INI_STACK_SIZE      (256 * 4)
#endif

#ifndef IRQ_STACK_SIZE
#define IRQ_STACK_SIZE      (128 * 4)
#endif

#ifndef FIQ_STACK_SIZE
#define FIQ_STACK_SIZE      (64 * 4)
#endif

#ifndef EXC_STACK_SIZE
#define EXC_STACK_SIZE      (32 * 4)
#endif

#define TOTAL_STACK_SIZE    (INI_STACK_SIZE + IRQ_STACK_SIZE + FIQ_STACK_SIZE + EXC_STACK_SIZE)

        /* We start in ARM mode. */
        .arm

/*
 * Vector Section: Must be moved to address 0.
 */
        .section .vectors,"ax",%progbits

        .global __vectors
__vectors:
        /* Reset entry. */
        ldr     pc, [pc, #___init_vect - . - 8]
        /* Undefined instruction entry. */
        ldr     pc, [pc, #___undef_vect - . - 8]
        /* Software interrupt entry. */
        ldr     pc, [pc, #___swi_vect - . - 8]
        /* Prefetch abort entry. */
        ldr     pc, [pc, #___pref_vect - . - 8]
        /* Data abort entry. */
        ldr     pc, [pc, #___data_vect - . - 8]
        /*
         * Reserved entry, used by some boot loaders to store the
         * size of the binary image.
         */
        .word   0
        /* Interrupt request entry, auto vectoring. */
        ldr     pc, [pc, #__vectors + AIC_IVR - . - 8]
        /* Fast interrupt request entry, auto vectoring. */
        ldr     pc, [pc, #__vectors + AIC_FVR - . - 8]

___init_vect:
        .weak   __init1
        .set    __init1, _start
        .word   __init1

        /*
         * Exceptions are weakly linked to endless loops.
         *
         * This may look a bit confusing. The following words contain
         * the jump addresses that are loaded at each entry, see above.
         * They contain weakly defined addresses pointing to executable
         * code. Each of them may be overridden by the operating system
         * or the application, if a function with the same name exist.
         * By default they simply point to an endless loop.
         */
___undef_vect:
        .weak   __undef
        .set    __undef, __undef_stop
        .word   __undef
___swi_vect:
        .weak   __swi
        .set    __swi, __undef_stop
        .word   __swi
___pref_vect:
        .weak   __prefetch_abort
        .set    __prefetch_abort, __undef_stop
        .word   __prefetch_abort
___data_vect:
        .weak   __data_abort
        .set    __data_abort, __undef_stop
        .word   __data_abort

/*
 * Init Section 0: Exception Dummies.
 *
 * All exceptions without a handler will end up in the same loop.
 * Alternatively, we may use a dedicated loop for each exception.
 * If an unhandled exception occurs, we can then use a simple
 * debugger (like OpenOCD) to retrieve the current program counter
 * and check the linker map to determine the type of the exception.
 */
        .section .init0, "ax", %progbits

        .global __undef_stop
__undef_stop:
        b       __undef_stop

/*
 * Init section 1: Memory remapping.
 *
 * Additional code may be added here by defining a naked function
 * named __init1 and placing it in section .init1.user.
 */
        .section .init1, "ax", %progbits

        .global	_start
_start:

        /* Copy 8 vectors and 5 handler addresses to internal RAM. */
        ldr     r0, =__vectors
        mov     r1, #0x00300000
        ldmia   r0!, {r2-r9}
        stmia   r1!, {r2-r9}
        ldmia   r0!, {r2-r6}
        stmia   r1!, {r2-r6}

        /* Remap internal RAM to address 0. */
        mov     r0, #(MATRIX_MRCR_RCB0 | MATRIX_MRCR_RCB1)
        ldr     r1, =MATRIX_BASE
        str     r0, [r1, #MATRIX_MRCR_OFF]

        .weak   __init2
        .set    __init2, __set_stacks
        ldr	pc, =__init2

/*
 * Init section 2: Set stack pointers.
 */
        .section .init2,"ax",%progbits
        .global __set_stacks
__set_stacks:
        /*
         * Set stack pointers with disabled interrupts.
         */
        ldr     r0, =__stack

        msr     CPSR_c, #ARM_MODE_SVC | ARM_CPSR_I_BIT | ARM_CPSR_F_BIT
        mov     sp, r0
        sub     r0, r0, #INI_STACK_SIZE

        msr     CPSR_c, #ARM_MODE_FIQ | ARM_CPSR_I_BIT | ARM_CPSR_F_BIT
        mov     sp, r0
        sub     r0, r0, #FIQ_STACK_SIZE

        msr     CPSR_c, #ARM_MODE_IRQ | ARM_CPSR_I_BIT | ARM_CPSR_F_BIT
        mov     sp, r0
        sub     r0, r0, #IRQ_STACK_SIZE

        msr     CPSR_c, #ARM_MODE_ABORT | ARM_CPSR_I_BIT | ARM_CPSR_F_BIT
        mov     sp, r0

        msr     CPSR_c, #ARM_MODE_UNDEF | ARM_CPSR_I_BIT | ARM_CPSR_F_BIT
        mov     sp, r0

        /*
         * Nut/OS system and application are running in system mode.
         * Note, that we re-use the supervisory stack.
         */
        msr     CPSR_c, #ARM_MODE_SYS | ARM_CPSR_I_BIT | ARM_CPSR_F_BIT
        ldr     r0, =__stack
        mov     sp, r0

#if 0
        /* Enable the instruction cache. */
        mrc     p15, 0, r0, c1, c0, 0
        ldr     r3, =0xC0001000
        orr     r0, r0, r3
        mcr     p15, 0, r0, c1, c0, 0
#endif

        .weak   __init3
        .set    __init3, __clear_bss
        ldr	pc, =__init3

/*
 * Init section 3: Initialize C variables.
 */
        .section .init3, "ax", %progbits
        .global __clear_bss
__clear_bss:

        ldr     r1, =__bss_start
        ldr     r2, =__bss_end
        mov     r3, #0

1:      cmp     r1, r2
        strne   r3, [r1], #4
        bne     1b

        .weak   __init4
        .set    __init4, __call_rtos
        ldr	pc, =__init4

/*
 * Init section 4: Start Nut/OS.
 */
        .section .init4, "ax", %progbits
        .global __call_rtos
__call_rtos:

        /* Jump to Nut/OS initialization. */
        ldr     r0, =NutInit
        .weak   __exit0
        .set    __exit0, __stop_rtos
        ldr     lr, =__exit0
        bx      r0

/*
 * Exit Section 0: Endless loop.
 *
 * May be overridden by defining a naked function named __exit0.
 */
        .section .exit0, "ax", %progbits

        .global __stop_rtos
__stop_rtos:
        b       __stop_rtos

/*
 * Stack Section.
 */
	.section .stack, "w", %nobits
        .space  TOTAL_STACK_SIZE
