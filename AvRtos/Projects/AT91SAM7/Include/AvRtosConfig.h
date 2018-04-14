/***************************************************************************
 * $Source$: AvRtosConfig.h
 * $Rev$: 2.0.0
 * $Author$: Harald
 * $Date$: 2016/01/23
 *
 * Module: AvRtosConfig
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

#ifndef AVRTOSCONFIG_H
#define AVRTOSCONFIG_H

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
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
/*****************************************************************************
 * 6. exported macros
 *****************************************************************************/
/**
 * General definitions for kernel tick rate
 */
#define KERNEL_CPU_CLOCK_HZ   50000000UL                 /**< Basic system frequency */
#define KERNEL_TICK_HZ        100UL                      /**< Kernel tick rate in Hz. For most systems a tick rate of 100 Hz is fast enough. A tick rate of 1000 Hz is also possible but increases the interrupt rate and system load. */
#define KERNEL_TICK_MS        (1000UL/KERNEL_TICK_HZ)    /**< Kernel tick rate in mills. This definition should be used to define system timeout values. */

/**
 * \def KERNEL_IDLE_TASK_STACK_SIZE
 * Definition for kernel idle task stack. Change this value with care, because a
 * stack overflow will result in unpredictable system behavior.
 * Each system stack will be filled with 0x55. At development time this is very
 * useful to debug the stack and to see if there was a stack overflow.
 */
#define KERNEL_IDLE_TASK_STACK_SIZE    150

/**
 * Definitions for system task synchronization objects that will be used.
 * If you do not need one of them, set the value to 0.
 */
#define KERNEL_CONFIG_USE_EVENT        1
#define KERNEL_CONFIG_USE_MUTEXES      1
#define KERNEL_CONFIG_USE_SEMAPHORES   1
#define KERNEL_CONFIG_USE_QUEUES       1

/**
 * Definitions for system resources. Define your needed system recourses here.
 * On small systems it is not very useful to allocate the needed recourses
 * dynamically. So a static allocation will be used here.
 * But you have to take care to increase this numbers every time you define a
 * new task, semaphore or queue.
 */
#define KERNEL_NUMBER_OF_TASKS         5
#define KERNEL_NUMBER_OF_MUTEXES       3
#define KERNEL_NUMBER_OF_EVENTS        2
#define KERNEL_NUMBER_OF_SEMAPHORES    3
#define KERNEL_NUMBER_OF_QUEUES        2

/**
 * \def  Kernel idle task hook function
 * Can be used to do some work while the kernel is idle.
 */
#define KERNEL_IDLE_HOOK_FUNCTION()

/*****************************************************************************
 * 9. exported function prototypes
 *****************************************************************************/

#ifdef __cplusplus
}
#endif
#endif // AVRTOSCONFIG_H
