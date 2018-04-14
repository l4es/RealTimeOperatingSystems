/*
 * kernel.h
 *
 * Created: 13/4/2013 10:13:52 AM
 *  Author: dcstanc
 *
 * kernel.c - Kernel main routines
 *
   	Copyright (C) 2013 Colin Tan
   	
   	
   	This file is part of ArdOS.

   	ArdOS is free software: you can redistribute it and/or modify
   	it under the terms of the GNU Lesser General Public License as published by
   	the Free Software Foundation, either version 3 of the License, or
   	(at your option) any later version.

   	ArdOS is distributed in the hope that it will be useful,
   	but WITHOUT ANY WARRANTY; without even the implied warranty of
   	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   	GNU Lesser General Public License for more details.

   	You should have received a copy of the GNU Lesser General Public License
   	along with ArdOS.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef KERNEL_H_
#define KERNEL_H_

#include <stdlib.h>
#include <avr/interrupt.h>
#include <Arduino.h>

/* =====================================================================================

	CONFIGURATION SECTION:
	
	Use this section to configure various parts of ArdOS.
	
	====================================================================================== */

// OS Scheduler Types
#define OS_PRIORITY	0
#define OS_RR		1
#define OS_RMS		2
#define OS_EDF		3

// Supported CPU Types
#define AT168		0
#define AT328		1
#define AT1280		2
#define AT2560		3

// Configure the CPU Type
// Valid values are AT168, AT328, AT1280 and AT2560
// Support in ardio.c for the AT1280 and AT2560 is limited. If you are
// compiling for these processors, you will need to use the Arduino provided
// libraries to access I/O.

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define OSCPU_TYPE	AT2560
#else
#define OSCPU_TYPE	AT328
#endif

// ABSOLUTE MAXIMUM number of tasks that your system can host
#define OSMAX_TASKS		8

// Pre-emptive or cooperative
#define OS_PREEMPTIVE		1

// Scheduler type: 0 = Fixed priority, 1 = Priority RR, 2 = EDF
//					Currently only fixed priority is available.

#define OSSCHED_TYPE		OS_RMS

// Default task stack size in unsigned longs (32 bits per element)
#define OSSTACK_SIZE		50

// Use sleep
#define OSUSE_SLEEP			1

// Use semaphores
#define OSUSE_SEMA			1

// Use queues
#define OSUSE_QUEUES		1

// Use priority queues
#define OSUSE_PRIOQUEUES	1

// Mutexes and conditionals
#define OSUSE_MUTEXES		1

#define OSUSE_CONDITIONALS	1

// Profiler
#define OSUSE_PROFILER		1


#if OSUSE_PROFILER
	#include "profiler.h"
#endif


/* =====================================================================================

	TASK MANAGEMENT SECTION:
	
	This section contains task management data structures and routines.
	
	====================================================================================== */

// Definition for task control block
typedef struct tc
{
	uint8_t pid;	// Process ID
	uint8_t prio;	// Task priority
	#if (OSSCHED_TYPE == OS_RMS) || (OSSCHED_TYPE == OS_EDF)
	unsigned long c;	// Task execution time
	unsigned long t;	//Task period
	#endif
	
	unsigned char status; // bit 0 = first run flag, bit 1 = blocked flag
	unsigned long *stack; // The task stack
	unsigned long sp;	// Stack pointer
	void (*taskptr)(void*); // Task pointer
	void *rarg;
} tTCB;


// Definition for task queues
typedef struct tq
{
	unsigned char *qptr, head, tail, len, ctr;
} tQueue;

#if OS_DEBUG == 1
void printProcQ(tQueue *q, tTCB *tasklist);
#endif

void initQ(unsigned char *, unsigned char len, tQueue *q);
void prioEnq(int pid, tTCB *tasklist, tQueue *q);

// Initializes task priorities based on the priority queue to ensure properness
#if (OSSCHED_TYPE == OS_RMS) 
void initialPrioAssign(tTCB *tasklist, tQueue *q);
#endif

void enq(int pid, tQueue *q);
void procEnq(int pid, tTCB *tasklist, tQueue *q);
unsigned char procPeek(tQueue *q);
unsigned char procDeq(tQueue *q);

/* =====================================================================================

	PRIVATE ARDOS SECTION:
	
	This section contains private ArdOS data structures and routines.
	
	====================================================================================== */


// Kernel variables
extern unsigned char _procCount;
extern tTCB _tasks[OSMAX_TASKS];
extern unsigned char _csreg;

// Task Ready Queue
extern tQueue _ready;

// Task running index
extern unsigned char _running;

// Scheduler suspension flag
extern unsigned char _suspend;

// Task counts
extern unsigned char _numTasks, _maxTasks;
// Task Status flags
#define _OS_FIRSTRUN	0b1
#define _OS_BLOCKED		0b10


// Error codes
#define OS_NO_ERR			0	// No error
#define OS_ERR_MAX_PROCS	1	// Reached maximum processes
#define OS_ERR_BAD_PRIO		2	// Bad priority number (RR)
#define OS_ERR_DUP_PRIO		3	// Duplicate priority number (Fixed Priority)
#define OS_ERR_NOMEM		4
#define OS_ERR_NOT_SCHED	5	// Tasks not schedulable

/* Private OS Functions. Do not call these */

// Atomicity Control
#define OSMakeAtomic(a) \
	*a = SREG;	\
	cli();

#define OSExitAtomic(a) \
	SREG = a;

unsigned int OSGetError();
void OSSetError(unsigned int);

void _OSSwap(unsigned char forcedSwap) __attribute__ ((naked));

/* =====================================================================================

	PUBLIC ARDOS SECTION:
	
	This section contains public ArdOS data structures and routines available to users.
	
	====================================================================================== */


/* Public OS Functions */

void OSInit(unsigned char numTasks);
void OSRun();

// Set task stack size
void OSSetStackSize(unsigned char);

// prio = Task priority, taskStack = stack for task, rptr = Pointer to task routines, rarg = Arguments to pass to task
unsigned int OSCreateTask(int prio, void (*rptr)(void *), void *rarg);

// c = task running time, t = task period
#if (OSSCHED_TYPE == OS_RMS) || (OSSCHED_TYPE == OS_EDF)
unsigned int OSCreateTimedTask(unsigned long c, unsigned long t, void (*rptr)(void *), void *rarg);
#endif

// Swaps task. Causes current task to relinquish control of the CPU. Scheduler selects next task to run.
void OSSwap() __attribute__ ((naked));

// Priority swap: Swap takes place only if new task has a higher priority
void OSPrioSwap() __attribute__ ((naked));

// Makes the current task sleep for the given number of milliseconds
void OSSleep(unsigned long millis);

// Returns the number of milliseconds since the OS was started up
unsigned long OSticks();

// Routines to be called within ISRs

void OSSwapFromISR() __attribute__ ((naked));
void OSPrioSwapFromISR() __attribute__ ((naked));

#endif /* KERNEL_H_ */
