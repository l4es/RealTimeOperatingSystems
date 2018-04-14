/************************************************************************//**
 * \mainpage  Documentation for the AvRtos project
 * \author    Harald Baumeister
 * \date      2016/01/12
 * \version   2.0.0
 *
 * \section intro_sec Introduction
 * It was my intention to develop a new real time operating system for AVR microcontrollers
 * because current existing RTOS's do not really fit my expectation of
 * a tiny and easy to use RTOS.\n
 * This is because most RTOS's for AVR are written in assembler. This makes it
 * really difficult to read and understand the written source code and it is
 * not easy to find bugs.\n
 * Also my expectation is to make it easier to integrate it into an existing project
 * so that the whole RTOS fits into a small amount source and header files.\n
 * I tried to develop a RTOS that fits a 1 kByte source code and to minimize the
 * overhead of RAM for task control structures and task synchronization elements.
 * As a result AvRtos uses the following system resources:\n
 * - 1 kByte of flash memory for code
 * - 11 Byte RAM for each task control block
 * - 5 Byte RAM for each semaphore control block
 * - 13 Byte RAM for each queue control block
 *
 * Currently AvRtos supports following task synchronization elements:\n
 * - Mutexes
 * - Semaphores
 * - Queues
 *
 * \section sourcecode_organisation_sec Source code organization
 * All declarations in the header file AvRtos.h could be used and represent the
 * API of the kernel.\n
 * All declarations in the header file AvRtosConfig.h can be adapted by the user and represent the
 * the kernel configuration.\n
 * All functions beginning with an underscore are internal kernel functions
 * and shouldn't be used outside the kernel.\n
 *
 * \section customization Customization
 * The kernel needs a system tick interrupt. Currently this interrupt is
 * configured for a 10ms tick rate. For most systems this tick rate is fast enough
 * but you can increase this tick rate to about 1kHz if desired. While the
 * overall system load will be increased a fast reaction to events could be
 * achieved.\n
 * The system tick interrupt configuration is platform dependent. For AVR controllers
 * TIMER0 is used but you could also use this timer for some other system functions.
 * In this case you must provide another tick
 * interrupt. For example you can also use TIMER1. Therefore you have to change
 * the interrupt vector from TIMER0_COMP_vect to TIMER1_COMP_vect but do not change
 * the source code of the interrupt function. The interrupt function MUST be declared
 * naked because the processor context will be saved and restored by the macros
 * SAVE_CONTEXT() and RESTORE_CONTEXT().\n
 * In the function kernelSetupTimerInterrupt you have to configure TIMER1.\n
 * If you change the tick interrupt please make sure that this interrupt is working
 * correctly. If this interrupt is not working correctly, no context switch
 * can be performed.
 *
 * \section isr Interrupt service routines
 * If you use interrupts in your software, you should use the redefined macro from
 * AvRtosHw.h.\n
 * The benefit to use this macro is that OS functions can recognize the current
 * running context and perform a context switch if necessary.\n
 * If you declare the interrupt service routine directly a possible context switch can
 * only be performed at the next system tick. This increases the latency maybe.\n
 * Please see the example in the file main.c, how you can do this.
 *
 * \section copyright Copyright and license information
 * \copyright Copyright (c) 2013, Harald Baumeister\n
 *            All rights reserved.\n
 * \n
 * This program is a free software. You can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation. Either version 2 of the License, or
 * any later version.\n
 * \n
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY. Without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.\n
 ****************************************************************************/

#ifndef AVRTOS_H
#define AVRTOS_H

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
#include "AvRtosConfig.h"
#include "AvRtosHw.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * 2. exported constants/definitions
 *****************************************************************************/
/*****************************************************************************
 * 3. global variables
 *****************************************************************************/
/*****************************************************************************
 * 4. exported typedefs
 *****************************************************************************/
/**
 * Type definition of a system task function.
 */
typedef void (*tKernelTaskFunction)( void );

/**
 * Type definition of a system task handle. The handle will be returned from
 * taskCreate and can be used to suspend or resume the task.
 */
typedef void* tTaskHandle;

#if KERNEL_CONFIG_USE_SEMAPHORES == 1
/**
 * Type definition of a system semaphore. The handle will be returned from
 * semaphoreCreate and must be used to manipulate the semaphore.
 */
typedef void* tSemaphoreHandle;
#endif

#if KERNEL_CONFIG_USE_EVENT == 1
/**
 * Type definition of a system events. The handle will be returned from
 * eventCreate and must be used to manipulate the events.
 */
typedef void* tEventHandle;

/**
 * Type definition of a system events option.
 */
typedef enum
{
   EVENT_OPTION_OR,
   EVENT_OPTION_AND,
   EVENT_OPTION_OR_CONSUME,
   EVENT_OPTION_AND_CONSUME

}tEventOption;

#endif

#if KERNEL_CONFIG_USE_MUTEXES == 1
/**
 * Type definition of a system mutex. The handle will be returned from
 * mutexCreate and must be used to manipulate the mutex.
 */
typedef void* tMutexHandle;
#endif

#if KERNEL_CONFIG_USE_QUEUES == 1
/**
 * Type definition of a system queue. The handle will be returned from
 * queueCreate and must be used to manipulate the queue.
 */
typedef void* tQueueHandle;
#endif

/*****************************************************************************
 * 6. exported macros
 *****************************************************************************/
/**
 * \def KERNEL_VERSION
 * Current kernel version
 */
#define KERNEL_VERSION        "2.0.3"

/**
 * \def KERNEL_IDLE_TASK_PRIORITY
 * Definition for kernel idle task priority. This is the basic priority.
 * Every system task should have a higher priority than the idle task.
 */
#define KERNEL_IDLE_TASK_PRIORITY      0

/**
 * \def KERNEL_WAIT_FOREVER
 * Timeout value definition.
 */
#define KERNEL_WAIT_FOREVER            0xFFFF

/**
 * \def  Kernel error definitions
 */
#define KERNEL_NO_ERROR                0x00
#define KERNEL_MUTEX_OWNER_ERROR       0xFC
#define KERNEL_CONTEXT_ERROR           0xFD
#define KERNEL_PARAMETER_ERROR         0xFE
#define KERNEL_TIMEOUT_ERROR           0xFF

/*****************************************************************************
 * 9. exported function prototypes
 *****************************************************************************/
/*************************************************************************//**
 * \fn       uint8_t kernelStartScheduler( void )
 *
 * \return   1 If this function returns there was an error. This function
 *           should never return.
 *
 * \brief    This function will initialize the kernel tick timer and creates
 *           the idle task. The function checks for the highest priority task
 *           in the system and perform a context switch to execute this task.
 *           It is useful to create all needed tasks before calling this function.
 ****************************************************************************/
uint8_t kernelStartScheduler( void );

/*************************************************************************//**
 * \fn       tTaskHandle taskCreate( tKernelTaskFunction taskFunction, char* taskName, tStack* stack, uint16_t stackSize, uint8_t priority )
 *
 * \param    taskFunction Pointer to the task entry function. A task function must
 *           be implemented to never return.
 *
 * \param    taskName Pointer to the task name. The task name will be stored
 *           to the task control block. So it is easier to debug the system.
 *
 * \param    stack Pointer to the task stack. The task stack must be allocated
 *           in a static way. At development time it is easier to check the stack
 *           if you now where you have to lock for.
 *
 * \param    stackSize The size in bytes of the stack.
 *
 * \param    priority The task priority. It should be higher than KERNEL_IDLE_TASK_PRIORITY
 *
 * \return   tTaskHandle Pointer to the created task. This handle can be used to
 *           suspend and resume the task manually.
 *
 * \brief    This function will initialize the task control block and prepares
 *           the task stack for the first usage.
 ****************************************************************************/
tTaskHandle taskCreate( tKernelTaskFunction taskFunction, char* taskName, tStack* stack, uint16_t stackSize, uint8_t priority );

/*************************************************************************//**
 * \fn       void taskSleep( uint16_t ticks )
 *
 * \param    ticks The minimum number of system ticks the task will sleep.
 *           It is very useful to define the timeout value like this.
 *           <pre> taskSleep( 100/KERNEL_TICK_MS ); </pre>
 *           This will suspend the task for 100 mills.
 *
 * \brief    This function will suspend the current running task for the
 *           given number of system ticks.
 ****************************************************************************/
void taskSleep( uint16_t ticks );

/*************************************************************************//**
 * \fn       void taskSuspend( tTaskHandle taskToSuspend )
 *
 * \param    taskToSuspend Handle of the task that should be suspended.
 *
 * \brief    This function will suspend the given task. The task an be
 *           awaken by calling taskResume.
 ****************************************************************************/
void taskSuspend( tTaskHandle taskToSuspend );

/*************************************************************************//**
 * \fn       void taskResume( tTaskHandle taskToResume )
 *
 * \param    taskToResume Handle of the task that should be resumed.
 *
 * \brief    This function will resume a previously suspended task.
 ****************************************************************************/
void taskResume( tTaskHandle taskToResume );

#if KERNEL_CONFIG_USE_SEMAPHORES == 1
/*************************************************************************//**
 * \fn       tSemaphoreHandle semaphoreCreate( uint8_t initValue )
 *
 * \param    initValue Initial semaphore value.
 *
 * \return   semaphore Pointer to the semaphore handle.
 *
 * \brief    This function will create a semaphore and initialize it
 *           with the given value.
 *           A handle will be returned that must be used by the semaphore
 *           manipulation functions.
 ****************************************************************************/
tSemaphoreHandle semaphoreCreate( uint8_t initValue );

/*************************************************************************//**
 * \fn       uint8_t semaphoreGet( tSemaphoreHandle semaphore, uint16_t timeoutTicks )
 *
 * \param    semaphore Handle to a previously created semaphore.
 *
 * \param    timeoutTicks Timeout value to wait before this function
 *           returns. If the timeout value is KERNEL_WAIT_FOREVER
 *           this function will only return if the semaphore was set
 *           by an other task or in an interrupt.
 *           If the value is 0 this function will only do a short check
 *           if the semaphore was set and will return immediately.
 *
 * \return   KERNEL_PARAMETER_ERROR If the semaphore handle is not valid\n
 *           KERNEL_TIMEOUT_ERROR   If there was an timeout.
 *
 * \brief    This function will try to get the given semaphore.\n
 *           This function can be called from an interrupt service routine.
 *           In this case the timeoutTicks MUST be 0
 ****************************************************************************/
uint8_t semaphoreGet( tSemaphoreHandle semaphore, uint16_t timeoutTicks );

/*************************************************************************//**
 * \fn       void semaphoreSet( tSemaphoreHandle semaphore )
 *
 * \param    semaphore Handle to a previously created semaphore.
 *
 * \brief    This function will set the given semaphore.\n
 *           This function can be called from an interrupt service routine.
 *           In this case the timeoutTicks MUST be 0
 ****************************************************************************/
void semaphoreSet( tSemaphoreHandle semaphore );
#endif

#if KERNEL_CONFIG_USE_MUTEXES == 1
/*************************************************************************//**
 * \fn       tMutexHandle mutexCreate( void )
 *
 * \return   mutex Pointer to the mutex handle.
 *
 * \brief    This function will create a mutex.
 *           A handle will be returned that must be used by the mutex
 *           manipulation functions.
 ****************************************************************************/
tMutexHandle mutexCreate( void );

/*************************************************************************//**
 * \fn       uint8_t mutexGet( tMutexHandle mutex, uint16_t timeoutTicks )
 *
 * \param    mutex Handle to a previously created mutex.
 *
 * \param    timeoutTicks Timeout value to wait before this function
 *           returns. If the timeout value is KERNEL_WAIT_FOREVER
 *           this function will only return if the mutex was unlocked
 *           by the owner task.
 *
 * \return   KERNEL_PARAMETER_ERROR    If the mutex handle is not valid\n
 *           KERNEL_TIMEOUT_ERROR   If there was an timeout.\n
 *           KERNEL_CONTEXT_ERROR      If this function is called from an interrupt.
 *
 * \brief    This function will try to lock the given mutex.\n
 *           This function can NOT be called from an interrupt service routine.
 ****************************************************************************/
uint8_t mutexGet( tMutexHandle mutex, uint16_t timeoutTicks );

/*************************************************************************//**
 * \fn       uint8_t mutexGive( tMutexHandle mutex )
 *
 * \return   KERNEL_PARAMETER_ERROR If the mutex handle is not valid\n
 *           KERNEL_MUTEX_OWNER_ERROR  If current task is not the owner of the mutex.\n
 *           KERNEL_CONTEXT_ERROR   If this function is called from an interrupt.
 *
 * \brief    This function will unlock the given mutex, if the current
 *           running task is the owner of the mutex.\n
 *           This function can NOT be called from an interrupt service routine.
 ****************************************************************************/
uint8_t mutexGive( tMutexHandle mutex );
#endif

#if KERNEL_CONFIG_USE_EVENT == 1
/*************************************************************************//**
 * \fn       tEventHandle eventCreate( void )
 *
 * \return   event Pointer to the event handle.
 *
 * \brief    This function will create a event group.
 *           A handle will be returned that must be used by the event
 *           manipulation functions.
 ****************************************************************************/
tEventHandle eventCreate( void );

/*************************************************************************//**
 * \fn       uint8_t eventRetrieve( tEventHandle event, uint32_t reqEvents, uint32_t* retEvents, tEventOption option, uint16_t timeoutTicks )
 *
 * \param    event Handle to a previously created event group.
 *
 * \param    reqEvents The requested event flag combination
 *
 * \param    option Event manipulation option
 *                  AND option: All event flags must be present
 *                  OR option:  Only one of the event flags must be present
 *                  CONSUME option: The requested event flags will be cleared
 *
 * \param    timeoutTicks Timeout value to wait before this function
 *           returns. If the timeout value is KERNEL_WAIT_FOREVER
 *           this function will only return if the correct event combination
 *           was set.
 *
 * \return   retEvents Event return register. This can maybe 0 if the result of the
 *                     operation is not needed.
 *
 * \return   KERNEL_PARAMETER_ERROR If the event handle is not valid\n
 *           KERNEL_TIMEOUT_ERROR   If there was an timeout.\n
 *
 * \brief    This function will try to retrieve the requested event flags.\n
 *           This function can be called from an interrupt service routine.
 *           In this case the timeoutTicks MUST be 0
 ****************************************************************************/
uint8_t eventRetrieve( tEventHandle event, uint32_t reqEvents, uint32_t* retEvents, tEventOption option, uint16_t timeoutTicks );

/*************************************************************************//**
 * \fn       uint8_t eventSet( tEventHandle event, uint32_t eventFalgs, tEventOption option )
 *
 * \param    event Handle to a previously created event group.
 *
 * \param    eventFalgs Event flag combination to set
 *
 * \param    option Event manipulation option
 *                  AND option: All eventFalgs will be ANDed with the event register.
 *                              This can be used to clear some flags.
 *                  OR option:  All eventFalgs will be ORed with the event register.
 *                              This is the normal option to set some flags.
 *                  CONSUME option: Has no meaning for this function.
 *
 * \return   KERNEL_PARAMETER_ERROR If the event handle is not valid\n
 *
 * \brief    This function will set or clear the given event fags.\n
 *           This function can be called from an interrupt service routine.
 ****************************************************************************/
uint8_t eventSet( tEventHandle event, uint32_t eventFalgs, tEventOption option );
#endif

#if KERNEL_CONFIG_USE_QUEUES == 1
/*************************************************************************//**
 * \fn       tQueueHandle queueCreate( uint8_t *buff, uint8_t size, uint8_t num_msgs )
 *
 * \param    buff Buffer that will be used as message buffer.
 *
 * \param    size Size of the one message in bytes
 *
 * \param    num_msgs Number of messages that can be stored to this queue
 *
 * \return   queue Pointer to the queue handle.
 *
 * \brief    This function will create a queue and initialize the queue buffer.
 *           A handle will be returned that must be used by the queue
 *           manipulation functions.
 ****************************************************************************/
tQueueHandle queueCreate( uint8_t *buff, uint8_t size, uint8_t num_msgs );

/*************************************************************************//**
 * \fn      uint8_t queueGet( tQueueHandle queue, void* msgBuffer, uint16_t timeoutTicks )
 *
 * \param   queue Handle to a previously created queue.
 *
 * \param   msgBuffer Pointer to the buffer where a message will be
 *          stored to.
 *
 * \param   timeoutTicks Timeout value to wait before this function
 *          returns. If the timeout value is KERNEL_WAIT_FOREVER
 *          this function will only return if a new message was put
 *          by an other task.
 *          If the value is 0 this function will only do a short check
 *          if a message was inserted and will return immediately.
 *
 * \return  KERNEL_PARAMETER_ERROR If the queue handle or buffer is not valid\n
 *          KERNEL_TIMEOUT_ERROR   If there was an timeout.
 *
 * \brief   This function will try to get a message from the queue buffer.
 *          If there is a message, it will be stored to the given
 *          message buffer.\n
 *          This function can be called from an interrupt service routine.
 *          In this case the timeoutTicks MUST be 0
 ****************************************************************************/
uint8_t queueGet( tQueueHandle queue, void* msgBuffer, uint16_t timeoutTicks );

/*************************************************************************//**
 * \fn      uint8_t queuePut( tQueueHandle queue, void* msgBuffer, uint16_t timeoutTicks )
 *
 * \param   queue Handle to a previously created queue.
 *
 * \param   msgBuffer Pointer to the message buffer.
 *
 * \param   timeoutTicks Timeout value to wait before this function
 *          returns. If the timeout value is KERNEL_WAIT_FOREVER
 *          this function will only return if there is room for a new message.
 *          If the value is 0 this function will only do a short check
 *          if the message can be inserted to the queue and will return immediately.
 *
 * \return  KERNEL_PARAMETER_ERROR If the queue handle or buffer is not valid\n
 *          KERNEL_TIMEOUT_ERROR   If there was an timeout.
 *
 * \brief   This function will try to put a message from the given message buffer.
 *          If the queue is already full this function waits for the given
 *          timeout value to put the message to the queue.\n
 *          This function can be called from an interrupt service routine.
 *          In this case the timeoutTicks MUST be 0
 ****************************************************************************/
uint8_t queuePut( tQueueHandle queue, void* msgBuffer, uint16_t timeoutTicks );
#endif

#ifdef __cplusplus
}
#endif
#endif // AVRTOS_H
