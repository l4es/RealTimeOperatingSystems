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

//------------------------------------------------------------------------------
// CPU vectors
//------------------------------------------------------------------------------
.section .vectors,"ax",%progbits

INITSP:      .long    __stack_init  /* Initial %sp          */
INITPC:      .long    _start        /* Initial PC           */
vector02:    .long    IH_DEFAULT    /* Access Error         */
vector03:    .long    IH_DEFAULT    /* Address Error        */
vector04:    .long    IH_DEFAULT    /* Illegal Instruction  */
vector05:    .long    IH_DEFAULT    /* Reserved             */
vector06:    .long    IH_DEFAULT    /* Reserved             */
vector07:    .long    IH_DEFAULT    /* Reserved             */
vector08:    .long    IH_DEFAULT    /* Privilege Violation  */
vector09:    .long    IH_DEFAULT    /* Trace                */
vector0A:    .long    IH_DEFAULT    /* Unimplemented A-Line */
vector0B:    .long    IH_DEFAULT    /* Unimplemented F-Line */
vector0C:    .long    IH_DEFAULT    /* Debug Interrupt      */
vector0D:    .long    IH_DEFAULT    /* Reserved             */
vector0E:    .long    IH_DEFAULT    /* Format Error         */
vector0F:    .long    IH_DEFAULT    /* Unitialized Int.     */
vector10:    .long    IH_DEFAULT    /* Reserved             */
vector11:    .long    IH_DEFAULT    /* Reserved             */
vector12:    .long    IH_DEFAULT    /* Reserved             */
vector13:    .long    IH_DEFAULT    /* Reserved             */
vector14:    .long    IH_DEFAULT    /* Reserved             */
vector15:    .long    IH_DEFAULT    /* Reserved             */
vector16:    .long    IH_DEFAULT    /* Reserved             */
vector17:    .long    IH_DEFAULT    /* Reserved             */
vector18:    .long    IH_DEFAULT    /* %spurious Interrupt  */
vector19:    .long    IH_DEFAULT    /* Autovector Level 1   */
vector1A:    .long    IH_DEFAULT    /* Autovector Level 2   */
vector1B:    .long    IH_DEFAULT    /* Autovector Level 3   */
vector1C:    .long    IH_DEFAULT    /* Autovector Level 4   */
vector1D:    .long    IH_DEFAULT    /* Autovector Level 5   */
vector1E:    .long    IH_DEFAULT    /* Autovector Level 6   */
vector1F:    .long    IH_DEFAULT    /* Autovector Level 7   */
vector20:    .long    IH_DEFAULT    /* TRAP #0              */
vector21:    .long    IH_DEFAULT    /* TRAP #1              */
vector22:    .long    IH_DEFAULT    /* TRAP #2              */
vector23:    .long    IH_DEFAULT    /* TRAP #3              */
vector24:    .long    IH_DEFAULT    /* TRAP #4              */
vector25:    .long    IH_DEFAULT    /* TRAP #5              */
vector26:    .long    IH_DEFAULT    /* TRAP #6              */
vector27:    .long    IH_DEFAULT    /* TRAP #7              */
vector28:    .long    IH_DEFAULT    /* TRAP #8              */
vector29:    .long    IH_DEFAULT    /* TRAP #9              */
vector2A:    .long    IH_DEFAULT    /* TRAP #10             */
vector2B:    .long    IH_DEFAULT    /* TRAP #11             */
vector2C:    .long    IH_DEFAULT    /* TRAP #12             */
vector2D:    .long    IH_DEFAULT    /* TRAP #13             */
vector2E:    .long    IH_DEFAULT    /* TRAP #14             */
vector2F:    .long    IH_DEFAULT    /* TRAP #15             */
vector30:    .long    IH_DEFAULT    /* Reserved             */
vector31:    .long    IH_DEFAULT    /* Reserved             */
vector32:    .long    IH_DEFAULT    /* Reserved             */
vector33:    .long    IH_DEFAULT    /* Reserved             */
vector34:    .long    IH_DEFAULT    /* Reserved             */
vector35:    .long    IH_DEFAULT    /* Reserved             */
vector36:    .long    IH_DEFAULT    /* Reserved             */
vector37:    .long    IH_DEFAULT    /* Reserved             */
vector38:    .long    IH_DEFAULT    /* Reserved             */
vector39:    .long    IH_DEFAULT    /* Reserved             */
vector3A:    .long    IH_DEFAULT    /* Reserved             */
vector3B:    .long    IH_DEFAULT    /* Reserved             */
vector3C:    .long    IH_DEFAULT    /* Reserved             */
vector3D:    .long    IH_DEFAULT    /* Reserved             */
vector3E:    .long    IH_DEFAULT    /* Reserved             */
vector3F:    .long    IH_DEFAULT    /* Reserved             */

//------------------------------------------------------------------------------
// Section 0 - Basic CPU initialization (family dependant code)
//           - The code starts here after reset
//           - After this section, C stack memory must be initialized
//------------------------------------------------------------------------------
    .globl _start
    .type _start, %function
    .section .init0,"ax",%progbits
_start:
_reset:
    /* disable interrupts */
    /* move.w        #0x2700,%sr */

//------------------------------------------------------------------------------
// Section 2 - Set up stack pointer
//           - After this section you may call C functions without global
//             variables
//------------------------------------------------------------------------------
    .section .init2,"ax",%progbits

    /* Initialize stack pointer */
    move.l #(__stack_init), %sp

    /* Clear the back chain to stop unwinding */
    move.l #0, %fp

    /* Push a dummy return address. */
    move.l %fp,%sp@-
    // TODO: zkusit: link %fp, #0

//------------------------------------------------------------------------------
// Section 4 - External Memories initialization (board dependant code)
//           - After this section all memories which need data initialization
//             MUST be initialized.
//------------------------------------------------------------------------------
    .section .init4,"ax",%progbits

//------------------------------------------------------------------------------
// Section 6 - Initialize memory regions
//           - After this code you may call C functions and use global variables
//------------------------------------------------------------------------------
    .section .init6,"ax",%progbits

    /* Init .data .bss .dataint .bssint and other regions defined in ldscript */
    jsr InitRegions

//------------------------------------------------------------------------------
// Section 8 - Relocate vectors
//           - Enable interrupts
//           - After this code you may use interrupts.
//           - NOTE: the interrupts are still disabled because the Interrupt
//                   priority mask in the status register is still 0b111
//------------------------------------------------------------------------------
    .section .init8,"ax",%progbits

    move.l    #__vectors_start,%d0
    movec.l    %d0,%VBR

//------------------------------------------------------------------------------
// Section 10 - CPU and Peripherals Initialization (family dependant code)
//------------------------------------------------------------------------------
    .section .init12,"ax",%progbits

//------------------------------------------------------------------------------
// Section 12 - C++ constructors (NOT implemented yet)
//------------------------------------------------------------------------------
    .section .init10,"ax",%progbits

//------------------------------------------------------------------------------
// Section 14 - Enter Nut/OS Initialization
//------------------------------------------------------------------------------
    .section .init10,"ax",%progbits

    jmp NutInit
