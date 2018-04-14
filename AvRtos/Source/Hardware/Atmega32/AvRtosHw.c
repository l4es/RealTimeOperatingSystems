/***************************************************************************
 * $Source$: AvRtos.c
 * $Rev$: 2.0.0
 * $Author$: Harald
 * $Date$: 2014/01/03
 *
 * Module: AvRtosHw
 *
 * Copyright (c) 2013, Harald Baumeister, Döggingen
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
ISR(TIMER0_COMP_vect)
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
   uint32_t compareMatch;

   /************************************************************************
    * function code
    *************************************************************************/

   compareMatch = (KERNEL_CPU_CLOCK_HZ / 64UL) / KERNEL_TICK_HZ;
   if( compareMatch > 0xFFUL )
   {
      compareMatch = (KERNEL_CPU_CLOCK_HZ / 256UL) / KERNEL_TICK_HZ;
      if( compareMatch > 0xFFUL )
      {
         compareMatch = (KERNEL_CPU_CLOCK_HZ / 1024UL) / KERNEL_TICK_HZ;

         /* Setup compare match value for compare match A.  Interrupts are disabled
          before this is called so we need not worry here. */
         OCR0 = (uint8_t)( compareMatch & (uint32_t) 0xff );

         /* Setup clock source and compare match behavior. */
         TCCR0 = 0x08 | 0x05;
      }
      else
      {
         /* Setup compare match value for compare match A.  Interrupts are disabled
          before this is called so we need not worry here. */
         OCR0 = (uint8_t)( compareMatch & (uint32_t) 0xff );

         /* Setup clock source and compare match behavior. */
         TCCR0 = 0x08 | 0x04;
      }
   }
   else
   {
      /* Setup compare match value for compare match A.  Interrupts are disabled
       before this is called so we need not worry here. */
      OCR0 = (uint8_t)( compareMatch & (uint32_t) 0xff );

      /* Setup clock source and compare match behavior. */
      TCCR0 = 0x08 | 0x03;
   }

   /* Enable the interrupt - this is okay as interrupt are currently globally
    disabled. */
   TIMSK |= 0x02;
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
   sp = stack + stackSize - 1;
   *sp-- = 'T';
   *sp-- = 'R';
   *sp-- = 'A';
   *sp-- = 'T';
   *sp-- = 'S';

   /* Setup basic stack */
   *sp-- = ((tStack)( (uint16_t) taskFunction )); // store PC(lo)
   *sp-- = ((tStack)( (uint16_t) taskFunction >> 8 )); // store PC(lo)
   *sp-- = 0; // r0
   *sp-- = 0x80; // SREG
   *sp-- = 0; // r1
   for( i = 2; i < 32; i++ )
      *sp-- = i; // store r2-r31

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

   CONTEXT_SWITCH_RESTORE_CONTEXT();
   CONTEXT_SWITCH_RETURN();
}
