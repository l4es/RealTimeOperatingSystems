/*
 * mutex.c
 * Provides mutex and conditional variable support
 * Created: 8/5/2013 8:47:44 AM
 *  Author: Colin Tan

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

#include "mutex.h"

#if OSUSE_MUTEXES==1
void OSCreateMutex(OSMutex *mutex)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	mutex->val=1;
	initQ(mutex->procList, _maxTasks, &mutex->blocked);
	OSExitAtomic(sreg);
}

void OSTakeMutex(OSMutex *mutex)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	if(!mutex->val)
	{
		procEnq(_running, _tasks, &mutex->blocked);
		_tasks[_running].status |= _OS_BLOCKED;
		OSExitAtomic(sreg);
		OSSwap();
	}
	else
		mutex->val=0;
		
	OSExitAtomic(sreg);
}

void OSGiveMutex(OSMutex *mutex)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	unsigned char wakeProc=procDeq(&mutex->blocked);
	
	if(wakeProc!=255)
	{
		_tasks[wakeProc].status &= ~_OS_BLOCKED;
		procEnq(wakeProc, _tasks, &_ready);
		OSExitAtomic(sreg);
		OSPrioSwap();
	}
	else
		mutex->val=1;
	
	OSExitAtomic(sreg);
}

#if OSUSE_CONDITIONALS==1
void OSCreateConditional(OSCond *cond)
{
	cond->blockedProcess=255;
}

// Can only be used by one process.
void OSWait(OSCond *cond, OSMutex *mutex)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	if(!cond->pendingWake)
	{
		cond->blockedProcess=_running;
		_tasks[_running].status |= _OS_BLOCKED;
		OSGiveMutex(mutex);
		// If val is 1 it means no swap took place.
		if(mutex->val)
			OSSwap();
	
		// Retake the mutex when we resume
		OSTakeMutex(mutex);
	}
	else
		cond->pendingWake=0;
		
	OSExitAtomic(sreg);
}

void OSSignal(OSCond *cond)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	if(cond->blockedProcess != 255)
	{
		_tasks[cond->blockedProcess].status &= ~_OS_BLOCKED;
		procEnq(cond->blockedProcess, _tasks, &_ready);
		cond->blockedProcess=255;
		cond->pendingWake=0;
		OSExitAtomic(sreg);
		OSPrioSwap();
	}
	else
		cond->pendingWake=1;
	OSExitAtomic(sreg);
}
#endif
#endif
