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
#include "AT91SAM7S256.h"
#include "AvRtos.h"

/*****************************************************************************
 * 2. file local constants/definitions
 *****************************************************************************/

/*****************************************************************************
 * 3. global variables
 *****************************************************************************/

/*****************************************************************************
 * 7. file local function prototypes
 *****************************************************************************/
/* ISR to handle manual context switches. */
void SWI_Handler( void ) __attribute__((interrupt("SWI"), naked));

/*****************************************************************************
 * 9. exported functions
 *****************************************************************************/
void SWI_Handler( void )
{
   /* Within an IRQ ISR the link register has an offset from the true return
   address, but an SWI ISR does not.  Add the offset manually so the same
   ISR return code can be used in both cases. */
   asm volatile ( "ADD     LR, LR, #4" );

   /* Perform the context switch.  First save the context of the current task. */
   SAVE_CONTEXT();

   /* Find the highest priority task that is ready to run. */
   _kernelSwitchContext();

   /* Restore the context of the new task. */
   RESTORE_CONTEXT();
}

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
void kernelSetupTimerInterrupt( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   AT91PS_PITC pxPIT = AT91C_BASE_PITC;
   unsigned int mask;

   /************************************************************************
    * function code
    *************************************************************************/

   mask = 0x1 << AT91C_ID_SYS;
   //* Disable the interrupt on the interrupt controller
   AT91C_BASE_AIC->AIC_IDCR = mask;
   //* Save the interrupt handler routine pointer and the interrupt priority
   AT91C_BASE_AIC->AIC_SVR[AT91C_ID_SYS] = (unsigned int) SysTick_Handler;
   //* Store the Source Mode Register
   AT91C_BASE_AIC->AIC_SMR[AT91C_ID_SYS] = AT91C_AIC_PRIOR_HIGHEST;
   //* Clear the interrupt on the interrupt controller
   AT91C_BASE_AIC->AIC_ICCR = mask;

   /* Configure the PIT period. */
   pxPIT->PITC_PIMR = (1 << 24) | (1 << 25) |
                      ((( KERNEL_CPU_CLOCK_HZ / 16 ) / 1000UL ) * KERNEL_TICK_MS );

   /* Enable the interrupt.  Global interrupts are disables at this point so
    this is safe. */
   AT91C_BASE_AIC->AIC_IECR = 0x1 << AT91C_ID_SYS;
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
   stackPointer = sp + 1;

   /* Software Stack */
   *sp-- = (uint32_t) taskFunction + 4;
   *sp-- = 0x14141414L;              /*R14 LR */
   *sp-- = (uint32_t) stackPointer;  /*R13 SP */
   *sp-- = 0x12121212L;              /*R12 */
   *sp-- = 0x11111111L;              /*R11 */
   *sp-- = 0x10101010L;              /*R10 */
   *sp-- = 0x09090909L;              /*R9  */
   *sp-- = 0x08080808L;              /*R8  */
   *sp-- = 0x07070707L;              /*R7  */
   *sp-- = 0x06060606L;              /*R6  */
   *sp-- = 0x05050505L;              /*R5  */
   *sp-- = 0x04040404L;              /*R4  */
   *sp-- = 0x03030303L;              /*R3  */
   *sp-- = 0x02020202L;              /*R2  */
   *sp-- = 0x01010101L;              /*R1  */
   *sp-- = 0x00000000L;              /*R0  */
   *sp   = 0x0000001FL;              /*SPSR System mode, ARM mode, interrupts enabled. */;              /*R0  */

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

   RESTORE_CONTEXT();
}
