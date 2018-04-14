/***************************************************************************
 * $Source$: AvRtos.h
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

#ifndef AVRTOSHW_H
#define AVRTOSHW_H

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * 2. exported constants/definitions
 *****************************************************************************/
/*****************************************************************************
 * 3. global variables/function prototypes
 *****************************************************************************/
extern void*   kernelRunningTask;
extern uint8_t kernelCurrentContext;

extern void _kernelScheduleTick( void );
extern void _kernelSwitchIsrContext( void );
extern void _kernelSwitchContext( void );

/*****************************************************************************
 * 4. exported typedefs
 *****************************************************************************/
typedef uint8_t tStack;

/*****************************************************************************
 * 6. exported macros
 *****************************************************************************/
#define __ATTRIBUTE__      __attribute__ (( naked ))

/**
 * Macro to save the current processor context
 */
#define CONTEXT_SWITCH_SAVE_CONTEXT()                 \
 asm volatile ("push r0                         \n\t" \
               "in      r0, __SREG__            \n\t" \
               "cli                             \n\t" \
               "push r0                         \n\t" \
               "push r1                         \n\t" \
               "clr  r1                         \n\t" \
               "push r2                         \n\t" \
               "push r3                         \n\t" \
               "push r4                         \n\t" \
               "push r5                         \n\t" \
               "push r6                         \n\t" \
               "push r7                         \n\t" \
               "push r8                         \n\t" \
               "push r9                         \n\t" \
               "push r10                        \n\t" \
               "push r11                        \n\t" \
               "push r12                        \n\t" \
               "push r13                        \n\t" \
               "push r14                        \n\t" \
               "push r15                        \n\t" \
               "push r16                        \n\t" \
               "push r17                        \n\t" \
               "push r18                        \n\t" \
               "push r19                        \n\t" \
               "push r20                        \n\t" \
               "push r21                        \n\t" \
               "push r22                        \n\t" \
               "push r23                        \n\t" \
               "push r24                        \n\t" \
               "push r25                        \n\t" \
               "push r26                        \n\t" \
               "push r27                        \n\t" \
               "push r28                        \n\t" \
               "push r29                        \n\t" \
               "push r30                        \n\t" \
               "push r31                        \n\t" \
               "lds  r26, kernelRunningTask     \n\t" \
               "lds  r27, kernelRunningTask + 1 \n\t" \
               "in      r0, 0x3d                \n\t" \
               "st      x+, r0                  \n\t" \
               "in      r0, 0x3e                \n\t" \
               "st      x+, r0                  \n\t" \
            )

/**
 * Macro to restore a new processor context and maybe to switch to a new task
 */
#define CONTEXT_SWITCH_RESTORE_CONTEXT()              \
 asm volatile ("lds  r26, kernelRunningTask     \n\t" \
               "lds  r27, kernelRunningTask + 1 \n\t" \
               "ld      r28, x+                 \n\t" \
               "out  __SP_L__, r28              \n\t" \
               "ld      r29, x+                 \n\t" \
               "out  __SP_H__, r29              \n\t" \
               "pop  r31                        \n\t" \
               "pop  r30                        \n\t" \
               "pop  r29                        \n\t" \
               "pop  r28                        \n\t" \
               "pop  r27                        \n\t" \
               "pop  r26                        \n\t" \
               "pop  r25                        \n\t" \
               "pop  r24                        \n\t" \
               "pop  r23                        \n\t" \
               "pop  r22                        \n\t" \
               "pop  r21                        \n\t" \
               "pop  r20                        \n\t" \
               "pop  r19                        \n\t" \
               "pop  r18                        \n\t" \
               "pop  r17                        \n\t" \
               "pop  r16                        \n\t" \
               "pop  r15                        \n\t" \
               "pop  r14                        \n\t" \
               "pop  r13                        \n\t" \
               "pop  r12                        \n\t" \
               "pop  r11                        \n\t" \
               "pop  r10                        \n\t" \
               "pop  r9                         \n\t" \
               "pop  r8                         \n\t" \
               "pop  r7                         \n\t" \
               "pop  r6                         \n\t" \
               "pop  r5                         \n\t" \
               "pop  r4                         \n\t" \
               "pop  r3                         \n\t" \
               "pop  r2                         \n\t" \
               "pop  r1                         \n\t" \
               "pop  r0                         \n\t" \
               "out  __SREG__, r0               \n\t" \
               "pop  r0                         \n\t" \
            )

/**
 * Macro to return from a context switch
 */
#define CONTEXT_SWITCH_RETURN()  asm volatile ( "reti" )

/**
 * Macro to define a critical system section
 */
#define CRITICAL_SECTION         uint8_t sreg

/**
 * Macro to enter a critical system section
 */
#define ENTER_CRITICAL()         sreg = SREG; \
                                 asm volatile ( "cli" )

/**
 * Macro to exit from a critical system section
 */
#define EXIT_CRITICAL()          SREG = sreg

/**
 * Macro to switch the context
 */
#define RESCHEDULE()             _kernelSwitchContext();

/*
 * Redefine the ISR macro. If you use some RTOS function inside an interrupt
 * service routine use this macro to declare the interrupt function.
 */
#ifdef ISR
#undef ISR
#endif

#ifdef __cplusplus
#  define ISR(vector, ...)            \
    extern "C" void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __attribute__ ( naked ); \
    void vector ## _func(void); \
    void vector (void) \
    { \
       CONTEXT_SWITCH_SAVE_CONTEXT(); \
       kernelCurrentContext = 1; \
       vector ## _func(); \
       kernelCurrentContext = 0; \
       CONTEXT_SWITCH_RESTORE_CONTEXT(); \
       CONTEXT_SWITCH_RETURN(); \
    } \
    void vector ## _func(void)
#else
#  define ISR(vector, ...)            \
    void vector (void) __attribute__ ((signal,__INTR_ATTRS))  __attribute__ (( naked )); \
    void vector ## _func(void); \
    void vector (void) \
    { \
       CONTEXT_SWITCH_SAVE_CONTEXT(); \
       kernelCurrentContext = 1; \
       vector ## _func(); \
       kernelCurrentContext = 0; \
       CONTEXT_SWITCH_RESTORE_CONTEXT(); \
       CONTEXT_SWITCH_RETURN(); \
    } \
    void vector ## _func(void)
#endif

/*****************************************************************************
 * 9. exported function prototypes
 *****************************************************************************/
/*************************************************************************//**
 * \fn       void kernelSetupTimerInterrupt( void )
 *
 * \brief    Function to initialize the system tick interrupt.
 ****************************************************************************/
void kernelSetupTimerInterrupt( void );

/*************************************************************************//**
 * \fn       void kernelSetupHardware( void )
 *
 * \brief    Function to initialize some CPU hardware.
 ****************************************************************************/
void kernelSetupHardware( void );

/*************************************************************************//**
 * \fn      void kernelSetupTaskStack( void* taskFunction, tStack* stackPointer, uint16_t stackSize )
 *
 * \param   taskFunction Pointer to current task function
 *
 * \param   stackPointer Pointer to the tasks stack buffer.
 *
 * \param   stackSize    Size of the task stack in sizeof(tStack) bytes.
 *
 * \brief   Function to initialize the task stack.
 ****************************************************************************/
tStack* kernelSetupTaskStack( void* taskFunction, tStack* stackPointer, uint16_t stackSize );

/*************************************************************************//**
 * \fn       void kernelSetupHardware( void )
 *
 * \brief    Function to start the first task.
 *           This function will not return.
 ****************************************************************************/
void kernelStartFirstTask( void );


#ifdef __cplusplus
}
#endif
#endif // AVRTOSHW_H
