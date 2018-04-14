/***************************************************************************
 * $Source$: AvRtos.c
 * $Rev$: 2.0.0
 * $Author$: Harald
 * $Date$: 2016/01/12
 *
 * Module: AvRtosHw
 *
 * Copyright (c) 2016, Harald Baumeister, Döggingen
 * All rights reserved.
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation. Either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY. Without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 ****************************************************************************/

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
#include "AvRtos.h"

/*****************************************************************************
 * 2. file local constants/definitions
 *****************************************************************************/
/* The lowest priority. */
#define KERNEL_INTERRUPT_PRIORITY         0xFFUL
#define SYSCALL_INTERRUPT_PRIORITY        0xFFUL

/*****************************************************************************
 * 3. global variables
 *****************************************************************************/

/*****************************************************************************
 * 7. file local function prototypes
 *****************************************************************************/

/*****************************************************************************
 * 9. exported functions
 *****************************************************************************/
//----------------------------------------------------------------------------
ISR(SysTick_Handler)
{
   /************************************************************************
    * local variables
    *************************************************************************/

   /************************************************************************
    * function code
    *************************************************************************/

   _kernelScheduleTick();
   _kernelSwitchIsrContext();
}

//----------------------------------------------------------------------------
ISR(SVC_Handler)
{
   /************************************************************************
    * local variables
    *************************************************************************/

   /************************************************************************
    * function code
    *************************************************************************/

   _kernelSwitchContext();
}

//----------------------------------------------------------------------------
void kernelSetupTimerInterrupt( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/

   /************************************************************************
    * function code
    *************************************************************************/

	/* Enable and setup SysTick Timer at a periodic rate */
	SysTickConfig(KERNEL_CPU_CLOCK_HZ / KERNEL_TICK_HZ);
}

//----------------------------------------------------------------------------
void kernelSetupHardware( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/

   /************************************************************************
    * function code
    *************************************************************************/

   /* Make SVC and SysTick the lowest priority interrupts. */
   NVICSetPriority(SVCallIRQn, SYSCALL_INTERRUPT_PRIORITY);
   NVICSetPriority(SysTickIRQn, KERNEL_INTERRUPT_PRIORITY);
}

//----------------------------------------------------------------------------
tStack* kernelSetupTaskStack( void* taskFunction, tStack* stackPointer, uint16_t stackSize )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tStack* stack;
   tStack* sp;
   uint16_t i;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Initialize complete stack with defined values.
    * This is only done for debugging the stack. */
   stack = stackPointer;
   for( i = 0; i < stackSize; i++ )
      stack[i] = (tStack)0x55555555;

   /* Place some defined values to the start of the stack.
    * This is only done for debugging the stack. */
   sp = stackPointer + stackSize - 1;
   *sp-- = 'T';
   *sp-- = 'R';
   *sp-- = 'A';
   *sp-- = 'T';
   *sp-- = 'S';
   *sp-- = 0;

   /* Setup basic stack */
   /* Hardware Stack */
   *sp-- = 0x01000000;               /*xPSR*/
   *sp-- = (uint32_t) taskFunction;  /*PC  */
   *sp-- = 0xFFFFFFFDL;              /*LR  */
   *sp-- = 0x12121212L;              /*R12 */
   *sp-- = 0x03030303L;              /*R3  */
   *sp-- = 0x02020202L;              /*R2  */
   *sp-- = 0x01010101L;              /*R1  */
   *sp-- = 0x00000000L;              /*R0  */

   /* Software Stack */
   *sp-- = 0x11111111L;              /*R11 */
   *sp-- = 0x10101010L;              /*R10 */
   *sp-- = 0x09090909L;              /*R9  */
   *sp-- = 0x08080808L;              /*R8  */
   *sp-- = 0x07070707L;              /*R7  */
   *sp-- = 0x06060606L;              /*R6  */
   *sp-- = 0x05050505L;              /*R5  */
   *sp   = 0x04040404L;              /*R4  */

   return sp;
}

//----------------------------------------------------------------------------
inline void kernelStartFirstTask( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/

   /************************************************************************
    * function code
    *************************************************************************/

   __asm volatile
   (
   " ldr r2, =kernelRunningTask    \n" // Get the location of the current TCB.
   " ldr r1, [r2]                  \n" // Get first item in TCB (Stack Pointer)
   " ldr r0, [r1]                  \n" // Write Stack Pointer to R0
   " ldmia r0!, {r4-r11}           \n" // Restore the registers.
   " push {r4}                     \n" // Save R4 + R5 to stack. We use it after.
   " push {r5}                     \n"
   " movs r4, r0                   \n" // Move R0 to R4 to restore other registers
   " ldmia r4!, {r0-r3, r12}       \n" // Restore the register
   " add r4, #4                    \n" // Ignore LR
   " ldr r14, [r4]                 \n" // Write content of PC from stack to LR
   " add r4, #4                    \n" // Next entry
   " ldr r5, [r4]                  \n" // Write XPSR from stack to R5
   " add r4, #4                    \n" // Next entry
   " msr APSR_nzcvq, r5            \n" // Write XPSR
   " pop {r5}                      \n" // Restore R5 from stack
   " str r5, [r4]                  \n" // Save R5 to PSP Stack. We switch the stack now
   " msr psp, r4                   \n" // Set PSP stack
   " pop {r4}                      \n" // Restore R4 from stack
   " movs r5, #2                   \n" // Switch the stack to PSP
   " msr control, r5               \n"
   " pop {r5}                      \n" // Restore R5 from current stack
   " cpsie i                       \n"
   " cpsie f                       \n"
   " dsb                           \n"
   " isb                           \n"
   " bx r14                        \n"
   );
}
