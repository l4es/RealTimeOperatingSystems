/***************************************************************************
 * $Source$: AvRtos.h
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

#ifndef AVRTOSHW_H
#define AVRTOSHW_H

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
#include <sys/types.h>
#include "cortex-m3.h"

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
typedef uint32_t tStack;

/*****************************************************************************
 * 6. exported macros
 *****************************************************************************/
#define __ATTRIBUTE__      //__attribute__ (( naked ))

/**
 * Macro to save the current processor context
 * Used by _kernelSwitchContext
 */
#define CONTEXT_SWITCH_SAVE_CONTEXT()

/**
 * Macro to restore a new processor context and maybe to switch to a new task
 * Used by _kernelSwitchContext
 */
#define CONTEXT_SWITCH_RESTORE_CONTEXT()

/**
 * Macro to return from a context switch
 * Used by _kernelSwitchContext
 */
#define CONTEXT_SWITCH_RETURN()


/**
 * Macro to save the current processor context
 */
#define SAVE_CONTEXT()                                                                               \
   asm volatile                                                                                         \
   (                                                                                                    \
   " cpsid i                       \n"                                                                  \
   " mrs r0, psp                   \n" /* Get current task stack */                                     \
   " isb                           \n"                                                                  \
   " ldr   r3, =kernelRunningTask  \n" /* Get the location of the current TCB. */                       \
   " ldr   r2, [r3]                \n"                                                                  \
   " stmdb r0!, {r4-r11}           \n" /* Save the remaining registers. */                              \
   " str r0, [r2]                  \n" /* Save the new top of stack into the first member of the TCB. */\
   " push {r14}                    \n"                                                                  \
   );

#define RESTORE_CONTEXT()                                                                            \
   asm volatile                                                                                         \
   (                                                                                                    \
   " pop {r14}                     \n"                                                                  \
   " ldr   r3, =kernelRunningTask  \n" /* Get the location of the current TCB. */                       \
   " ldr   r2, [r3]                \n"                                                                  \
   " ldr r0, [r2]                  \n"                                                                  \
   " ldmia r0!, {r4-r11}           \n" /* Pop the registers. */                                         \
   " msr psp, r0                   \n"                                                                  \
   " isb                           \n"                                                                  \
   " cpsie i                       \n"                                                                  \
   " bx r14                        \n"                                                                  \
   );

/**
 * Macro to call the scheduler
 */
#define RESCHEDULE()                         \
   asm volatile(" cpsie i");                 \
   asm volatile(" svc 0");

/**
 * Macro to define a critical system section
 */
#define CRITICAL_SECTION

/**
 * Macro to enter a critical system section
 */
#define ENTER_CRITICAL()         asm volatile ( "CPSID i" )

/**
 * Macro to exit from a critical system section
 */
#define EXIT_CRITICAL()          asm volatile ( "CPSIE i" )

/*
 * Redefine the ISR macro. If you use some RTOS function inside an interrupt
 * service routine use this macro to declare the interrupt function.
 */
#ifdef ISR
#undef ISR
#endif

#ifdef __cplusplus
#  define ISR(vector, ...)            \
    extern "C" void vector (void) __attribute__ (( isr, naked )); \
    void vector ## _func(void); \
    void vector (void) \
    { \
       SAVE_CONTEXT(); \
       kernelCurrentContext = 1; \
       vector ## _func(); \
       kernelCurrentContext = 0; \
       RESTORE_CONTEXT(); \
    } \
    void vector ## _func(void)
#else
#  define ISR(vector, ...)            \
    void vector (void) __attribute__ (( isr, naked )); \
    void vector ## _func(void); \
    void vector (void) \
    { \
       SAVE_CONTEXT(); \
       kernelCurrentContext = 1; \
       vector ## _func(); \
       kernelCurrentContext = 0; \
       RESTORE_CONTEXT(); \
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
