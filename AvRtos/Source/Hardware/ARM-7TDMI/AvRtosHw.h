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
#include "AT91SAM7S256.h"

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
#define __ATTRIBUTE__     // __attribute__ (( naked ))

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

#define SAVE_CONTEXT()                     \
   asm volatile (                          \
   /* Push R0 as we are going to use the register. */\
   "STMDB   SP!, {R0}                \n\t" \
                                           \
   /* Set R0 to point to the task stack pointer. */\
   "STMDB   SP,{SP}^                 \n\t" \
   "NOP                              \n\t" \
   "SUB  SP, SP, #4                  \n\t" \
   "LDMIA   SP!,{R0}                 \n\t" \
                                           \
   /* Push the return address onto the stack. */\
   "STMDB   R0!, {LR}                \n\t" \
                                           \
   /* Now we have saved LR we can use it instead of R0. */\
   "MOV  LR, R0                      \n\t" \
                                           \
   /* Pop R0 so we can save it onto the system mode stack. */\
   "LDMIA   SP!, {R0}                \n\t" \
                                           \
   /* Push all the system mode registers onto the task stack. */\
   "STMDB   LR,{R0-LR}^              \n\t" \
   "NOP                              \n\t" \
   "SUB  LR, LR, #60                 \n\t" \
                                           \
   /* Push the SPSR onto the task stack. */\
   "MRS  R0, SPSR                    \n\t" \
   "STMDB   LR!, {R0}                \n\t" \
                                           \
   /* Store the new top of stack for the task. */\
   "LDR  R0, =kernelRunningTask      \n\t" \
   "LDR  R0, [R0]                    \n\t" \
   "STR  LR, [R0]                    \n\t" \
   );                                      \

#define RESTORE_CONTEXT()                  \
   asm volatile (                          \
   "LDR     R0, =kernelRunningTask   \n\t" \
   "LDR     R0, [R0]                 \n\t" \
   "LDR     LR, [R0]                 \n\t" \
                                           \
   /* Get the SPSR from the stack. */      \
   "LDMFD   LR!, {R0}                \n\t" \
   "MSR     SPSR, R0                 \n\t" \
                                           \
   /* Restore all system mode registers for the task. */\
   "LDMFD   LR, {R0-R14}^            \n\t" \
   "NOP                              \n\t" \
                                           \
   /* Restore the return address. */       \
   "LDR     LR, [LR, #60]            \n\t" \
                                           \
   /* And return */                        \
   "SUBS PC, LR, #4                  \n\t" \
   );                                      \

/**
 * Macro to call the scheduler
 */
#define RESCHEDULE()             asm volatile ("SWI 0");

/**
 * Macro to define a critical system section
 */
#define CRITICAL_SECTION

/**
 * Macro to enter a critical system section
 */
#define ENTER_CRITICAL()                                                 \
asm volatile ("STMDB   SP!, {R0}   \n\t" /* Push R0.                  */ \
              "MRS  R0, CPSR       \n\t" /* Get CPSR.                 */ \
              "ORR  R0, R0, #0xC0  \n\t" /* Disable IRQ, FIQ.         */ \
              "MSR  CPSR, R0       \n\t" /* Write back modified value.*/ \
              "LDMIA   SP!, {R0}  " );   /* Pop R0.                   */

/**
 * Macro to exit from a critical system section
 */
#define EXIT_CRITICAL()                                                  \
asm volatile ("STMDB   SP!, {R0}  \n\t" /* Push R0.                   */ \
              "MRS  R0, CPSR      \n\t" /* Get CPSR.                  */ \
              "BIC  R0, R0, #0xC0 \n\t" /* Enable IRQ, FIQ.           */ \
              "MSR  CPSR, R0      \n\t" /* Write back modified value. */ \
              "LDMIA   SP!, {R0}  " )   /* Pop R0.                    */

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
       AT91C_BASE_AIC->AIC_EOICR = AT91C_BASE_PITC->PITC_PIVR;\
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
       AT91C_BASE_AIC->AIC_EOICR = AT91C_BASE_PITC->PITC_PIVR;\
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
