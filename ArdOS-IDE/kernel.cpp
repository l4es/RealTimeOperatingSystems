/*
 * kernel.c
 *
 * Created: 13/4/2013 10:13:40 AM
 *  Author: dcstanc
 *
 * Contains main OS codes
 *
 *
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

#include "kernel.h"


// Store SREG
unsigned char _csreg;

// Stack pointer temp variable
unsigned long pxCurrentTCB;

// Pointer to routine start
unsigned long pxFuncPtr, pxFuncArg;

// Number of tasks
unsigned char _numTasks=0;
unsigned char _maxTasks=0;

// Task stack size
unsigned char _taskStackSize=OSSTACK_SIZE;

// Task lists
unsigned char _procCount=0;
tTCB _tasks[OSMAX_TASKS];

// Task Ready Queue
static unsigned char _readybuf[OSMAX_TASKS];
tQueue _ready;

// Tick counter
volatile unsigned long _osticks=0;

// Task running index
unsigned char _running=255;
unsigned int _errno=0;

// Forced Swap flag
unsigned char _forcedSwap=0;

// Scheduler suspension flag
unsigned char _suspend=0;

// Macros to save and restore context
#define portSAVE_CONTEXT()\
asm volatile (\
	"push r0	\n\t"\
	"mov r0, %0	\n\t"\
	"cli	\n\t"\
	"push r0	\n\t"\
	"push r1	\n\t"\
	"clr r1	\n\t"\
	"push r2	\n\t"\
	"push r3	\n\t"\
	"push r4	\n\t"\
	"push r5	\n\t"\
	"push r6	\n\t"\
	"push r7	\n\t"\
	"push r8	\n\t"\
	"push r9	\n\t"\
	"push r10	\n\t"\
	"push r11	\n\t"\
	"push r12	\n\t"\
	"push r13	\n\t"\
	"push r14	\n\t"\
	"push r15	\n\t"\
	"push r16	\n\t"\
	"push r17	\n\t"\
	"push r18	\n\t"\
	"push r19	\n\t"\
	"push r20	\n\t"\
	"push r21	\n\t"\
	"push r22	\n\t"\
	"push r23	\n\t"\
	"push r24	\n\t"\
	"push r25	\n\t"\
	"push r26	\n\t"\
	"push r27	\n\t"\
	"push r28	\n\t"\
	"push r29	\n\t"\
	"push r30	\n\t"\
	"push r31	\n\t"\
	"in r26, __SP_L__	\n\t"\
	"in r27, __SP_H__	\n\t"\
	"sts pxCurrentTCB+1, r27	\n\t"\
	"sts pxCurrentTCB, r26	\n\t"\
	: : "r" (_csreg));

#define portRESTORE_CONTEXT()\
asm volatile (\
	"cli	\n\t"\
	"out __SP_L__, %A1	\n\t"\
	"out __SP_H__, %B1	\n\t"\
	"pop r31	\n\t"\
	"pop r30	\n\t"\
	"pop r29	\n\t"\
	"pop r28	\n\t"\
	"pop r27	\n\t"\
	"pop r26	\n\t"\
	"pop r25	\n\t"\
	"pop r24	\n\t"\
	"pop r23	\n\t"\
	"pop r22	\n\t"\
	"pop r21	\n\t"\
	"pop r20	\n\t"\
	"pop r19	\n\t"\
	"pop r18	\n\t"\
	"pop r17	\n\t"\
	"pop r16	\n\t"\
	"pop r15	\n\t"\
	"pop r14	\n\t"\
	"pop r13	\n\t"\
	"pop r12	\n\t"\
	"pop r11	\n\t"\
	"pop r10	\n\t"\
	"pop r9	\n\t"\
	"pop r8	\n\t"\
	"pop r7	\n\t"\
	"pop r6	\n\t"\
	"pop r5	\n\t"\
	"pop r4	\n\t"\
	"pop r3	\n\t"\
	"pop r2	\n\t"\
	"pop r1	\n\t"\
	"pop r0	\n\t"\
	"out __SREG__, r0\n\t"\
	"mov %0, r0	\n\t"\
	"pop r0	\n\t"\
	: "=r" (_csreg) : "r" (pxCurrentTCB));

// Sets up SP to point to the thread stack
#define portSetStack()\
asm volatile(\
	"OUT __SP_L__, %A0	\n\t"\
	"OUT __SP_H__, %B0	\n\t"\
	: : "r" (pxCurrentTCB))

// Loads the starting address of the thread function onto the stack and
// puts in the passed parameter into R25 and R24 as expected by the function.

#if OSCPU_TYPE==AT168 || OSCPU_TYPE==AT328
	#define portPushRetAddress()\
	asm volatile(\
		"mov r0, %A0	\n\t"\
		"push r0	\n\t"\
		"mov r0, %B0	\n\t"\
		"push r0	\n\t"\
		"mov R25, %B1	\n\t"\
		"mov R24, %A1	\n\t"\
		: : "r" (pxFuncPtr), "r" (pxFuncArg))
#elif OSCPU_TYPE==AT1280 || OSCPU_TYPE==AT2560
	#define portPushRetAddress()\
	asm volatile(\
		"mov r0, %A0	\n\t"\
		"push r0	\n\t"\
		"mov r0, %B0	\n\t"\
		"push r0	\n\t"\
		"mov r0, %C0	\n\t"\
		"push r0	\n\t"\
		"mov R25, %B1	\n\t"\
		"mov R24, %A1	\n\t"\
		: : "r" (pxFuncPtr), "r" (pxFuncArg))
#endif


#if (OSSCHED_TYPE == OS_RMS)
float LL_LIMIT[] = {1, 0.8284271247, 0.7797631497, 0.75682846, 0.743491775, 0.7347722899, 0.7286265957, 0.7240618613};
#endif

// Error handling
unsigned int OSGetError()
{
	return _errno;
}

void OSSetError(unsigned int errno)
{
	_errno=errno;
	Serial.begin(115200);
	Serial.print("ERROR: ");
	Serial.println(_errno);
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
}


#if OSUSE_SLEEP==1

	// Sleep routine. Records number of milliseconds to sleep. sleep_flag tells you if process is indeed sleeping. 1 bit per process
	// maximum 16 processes.
	unsigned long _sleepTime[OSMAX_TASKS];
	int _sleepFlag=0;

	// Sleep routine. Sleeps in milliseconds
	void OSSleep(unsigned long millis)
	{
		unsigned char sreg;
		OSMakeAtomic(&sreg);
		// Set sleep time
		_sleepTime[_running]=millis-1;
		
		if(_sleepTime[_running]<0)
			_sleepTime[_running]=0;
			
		_sleepFlag |= (1<<_running);
	
		// Set blocked flag
		_tasks[_running].status|=_OS_BLOCKED;
	
		// Note: No need to remove from READY queue because a _running process would have already been de-queued from there.
		// So just call scheduler to swap.
	
		OSExitAtomic(sreg);
		OSSwap();
	}
#endif

// OS Task Management Routines

// Set task stack
void OSSetStackSize(unsigned char stackSize)
{
	_taskStackSize=stackSize;
}

// Task handling

// prio = Task priority, taskStack = stack for task, rptr = Pointer to task routines, rarg = Arguments to pass to task
unsigned int OSCreateTask(int prio, void (*rptr)(void *), void *rarg)
{
	unsigned char sreg;
	
	OSMakeAtomic(&sreg);
	if(_procCount>_maxTasks)
	{
		OSSetError(OS_ERR_MAX_PROCS);
		return OSGetError();
	}

	// For fixed priority we must have unique priorities and priorities must be between 0 and n-1 for n processes.
	#if OSSCHED_TYPE == OS_PRIORITY
	
		// Check if current priority level already exists
		uint8_t i, found=0;
	
		for(i=0; i<_procCount && !found; i++)
			found=(_tasks[i].prio==prio);
	
		if(found)
		{
			OSSetError(OS_ERR_DUP_PRIO);
			return OSGetError();
		}
		// Check for BAD_PRIO. Should be between 0 and _maxTasks
		if(prio != 255 && (prio <0 || prio > _maxTasks))
		{
			OSSetError(OS_ERR_BAD_PRIO);
			return OSGetError();
		}
		
	#endif
	
	// Insert task
	_tasks[_procCount].prio=prio;
	_tasks[_procCount].taskptr=rptr;
	_tasks[_procCount].rarg=rarg;
	_tasks[_procCount].stack=(unsigned long *) calloc((size_t) _taskStackSize, sizeof(unsigned long));
	
	if(_tasks[_procCount].stack==NULL)
	{
		OSSetError(OS_ERR_NOMEM);
		return OSGetError();
	}
	
	_tasks[_procCount].sp=(unsigned long) &(_tasks[_procCount].stack[_taskStackSize-1]);
	_tasks[_procCount].pid=_procCount;
	_tasks[_procCount].status|=_OS_FIRSTRUN;

	
	// Insert into ready queue
	procEnq(_procCount, _tasks, &_ready);
	_procCount++;

	OSExitAtomic(sreg);
	return OS_NO_ERR;
}

#if (OSSCHED_TYPE == OS_RMS) || (OSSCHED_TYPE == OS_EDF)
unsigned int OSCreateTimedTask(unsigned long c, unsigned long t, void (*rptr)(void *), void *rarg)
{
	unsigned char sreg;
	
	OSMakeAtomic(&sreg);
	if(_procCount>_maxTasks)
	{
		OSSetError(OS_ERR_MAX_PROCS);
		return OSGetError();
	}
	
	// Insert task
	_tasks[_procCount].prio=0;
	_tasks[_procCount].c=c;
	_tasks[_procCount].t=t;
	_tasks[_procCount].taskptr=rptr;
	_tasks[_procCount].rarg=rarg;
	_tasks[_procCount].stack=(unsigned long *) calloc((size_t) _taskStackSize, sizeof(unsigned long));
	
	if(_tasks[_procCount].stack==NULL)
	{
		OSSetError(OS_ERR_NOMEM);
		return OSGetError();
	}
	
	_tasks[_procCount].sp=(unsigned long) &(_tasks[_procCount].stack[_taskStackSize-1]);
	_tasks[_procCount].pid=_procCount;
	_tasks[_procCount].status|=_OS_FIRSTRUN;

	
	// Insert into ready queue
	procEnq(_procCount, _tasks, &_ready);
	_procCount++;

	OSExitAtomic(sreg);
	return OS_NO_ERR;
}
#endif

void OSScheduler()
{
	// Remove first item from queue
	unsigned char _nextRun=procPeek(&_ready);
	
	// Check to see that it is a proper process
	if(_nextRun != 255)
	#if (OSSCHED_TYPE==OS_PRIORITY )|| (OSSCHED_TYPE==OS_RMS) || (OSSCHED_TYPE==OS_EDF )
	if(_running==255 || _tasks[_nextRun].prio < _tasks[_running].prio || _forcedSwap)
	#endif
	{
		_nextRun=procDeq(&_ready);
		if(_running!=255 && _nextRun !=  _running)
		{
			_tasks[_running].sp=pxCurrentTCB;
			
			// Push to READY queue if not blocked
			if(!(_tasks[_running].status & _OS_BLOCKED))
				procEnq(_running, _tasks, &_ready);
		}

		pxCurrentTCB=_tasks[_nextRun].sp;
		_running=_nextRun;
		
	}
	
}


#if (OSSCHED_TYPE == OS_RMS) 
unsigned int OSTestSchedulability()
{
	float utilization;
	int i;
	for (i = 0; i< _numTasks; i++){
		utilization += (float)_tasks[i].c / (float) _tasks[i].t;
	}
	if (utilization > LL_LIMIT[_numTasks-1])
	{
		OSSetError(OS_ERR_NOT_SCHED);
		return OSGetError();
	}
	else
		return 0;
		
}
#endif
	
inline void runTask()
{
	// Check if this is the first run of this process
	if(_tasks[_running].status & _OS_FIRSTRUN)
	{
		_tasks[_running].status &= ~(_OS_FIRSTRUN);
		portSetStack();
		// Run the function by setting (R25,R24) = function argument and pushing the start
		// address of the function onto the stack
		pxFuncPtr=(unsigned long) _tasks[_running].taskptr;
		pxFuncArg=(unsigned long) _tasks[_running].rarg;
		portPushRetAddress();
		sei();
	}
	else
	{
		portRESTORE_CONTEXT();
	}
}

void _OSSwap(unsigned char forcedSwap)
{
	portSAVE_CONTEXT();
	cli();
	
	if(!_suspend)
	{
		_forcedSwap=forcedSwap;
		OSScheduler();
	}

	runTask();
}

// Swaps task. Causes current task to relinquish control of the CPU. Scheduler selects next task to run.
void OSSwap()
{
	_csreg = SREG;
	_OSSwap(1);
	asm("ret");
}

void OSSwapFromISR()
{
	_csreg = SREG;
	_OSSwap(1);
	asm("reti");
}

void OSPrioSwap()
{
	_csreg = SREG;
	_OSSwap(0);
	asm("ret");
}

void OSPrioSwapFromISR()
{
	_csreg = SREG;
	_OSSwap(0);
	asm("reti");
}


ISR(TIMER2_OVF_vect, ISR_NAKED)
{
	portSAVE_CONTEXT();
	OSMakeAtomic(&_csreg);
	// Increment tick counter
	_osticks++;
	
	#if OSUSE_PROFILER == 1
	
	setCurrentProfiled(_tasks[_running].pid);
	
	
		#if OSUSE_SLEEP == 1
		if (_sleepTime[_running]==0)
			tickTask();
		#else
		tickTask();
		#endif
		
	#endif
	
	#if OSUSE_SLEEP==1
		// Decrement wait counters
		unsigned char i;
	
		for(i=0; i<_maxTasks; i++)
			if(_sleepTime[i]>0)
				_sleepTime[i]--;
			else
				if(_sleepFlag & (0b1 << i))
				{
					// Clear the flag
					_sleepFlag &= ~(0b1 << i);
				
					// Unblock the task
					_tasks[i].status &= ~(_OS_BLOCKED);
					// Put onto ready queue, call scheduler
					procEnq(i, _tasks, &_ready);
				
					// Call scheduler if this is fixed priority
				
					#if (OSSCHED_TYPE==OS_PRIORITY || OSSCHED_TYPE == OS_RMS) && OS_PREEMPTIVE==1
						portRESTORE_CONTEXT();
						OSExitAtomic(_csreg);
						_OSSwap(0);
						asm("reti");
					#endif
				}
	#endif
	portRESTORE_CONTEXT();
	OSExitAtomic(_csreg);
	asm("reti");
}

unsigned long OSticks()
{
	return _osticks;
}

void configureTimer()
{
	// Set fast PWM, OC2A and OC2B disconnected.
	TCCR2A=0b00000011;
	TCNT2=0;
	
	// Enable TOV2
	TIMSK2|=0b1;
	
}

void startTimer()
{
	// Start timer giving frequency of approx 1000 Hz
	TCCR2B=0b00000100;
}

// OS Initialization and Starting Routines
void OSInit(unsigned char numTasks)
{
	OSMakeAtomic(&_csreg);
	configureTimer();

	_numTasks=numTasks;
	_maxTasks=numTasks+1;
	_suspend=0;
	
	OSExitAtomic(_csreg);
	// Initialization code for fixed priority. +1 for idle task
	initQ(_readybuf, _maxTasks, &_ready);
}

// The idle task. Just wastes CPU cycles
void _OSIdle(void *p)
{
	while(1);
}

void OSRun()
{
	OSMakeAtomic(&_csreg);
	// Create idle task
	#if OSSCHED_TYPE == OS_RMS
	OSTestSchedulability();
	initialPrioAssign( _tasks, &_ready);
	#endif
	
	
	OSCreateTask(255, _OSIdle, NULL);
	//#endif
	
	#if OSUSE_PROFILER == 1
	resetAllProfileBins();
	#endif
	
	startTimer();
	
	OSExitAtomic(_csreg);
	OSSwap();
}
