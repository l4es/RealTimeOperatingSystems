/*
 * queue.c
 *
 * Created: 4/5/2013 9:36:36 AM
 *  Author: dcstanc
 
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

#include "queue.h"

#if OSUSE_QUEUES==1 || OSUSE_PRIOQUEUES==1

// Unblock a blocked process and call the scheduler
void OSQueueUnblock(TMsgQ *queue, unsigned char sreg)
{
	_tasks[queue->blockproc].status &= ~_OS_BLOCKED;

	// Enqueue blocked process
	procEnq(queue->blockproc, _tasks, &_ready);
	queue->blockproc=255;
	
	OSExitAtomic(sreg);
	// Make a priority swap
	OSPrioSwap();
}

int OSDequeue(TMsgQ *queue)
{
	unsigned char sreg;
	int ret=255;
	OSMakeAtomic(&sreg);

	if(!queue->count)
	{
		// Block the current task
		queue->blockproc=_running;
		_tasks[_running].status|=_OS_BLOCKED;
		
		OSExitAtomic(sreg);
		
		// Make a task swap
		OSSwap();
	}
	
	#if OSUSE_QUEUES==1
	
		if(!queue->prioQ)
			ret=queue->qbuf[queue->head];

	#endif

	#if OSUSE_PRIOQUEUES==1

		if(queue->prioQ)
			ret=queue->pqbuf[queue->head].data;

	#endif

	queue->head = (queue->head+1) % queue->len;
	queue->count--;
	
	OSExitAtomic(sreg);
	return ret;
}
#endif

#if OSUSE_QUEUES==1
void OSCreateQueue(int *buffer, unsigned char length, TMsgQ *queue)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	queue->blockproc=255;
	queue->count=0;
	queue->head=0;
	queue->tail=0;
	queue->qbuf=buffer;
	queue->len=length;
	queue->prioQ=0;
	OSExitAtomic(sreg);
}

void OSEnqueue(int data, TMsgQ *queue)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	if(queue->count>=queue->len)
	{
		OSExitAtomic(sreg);
		return;
	}
	
	if(!queue->prioQ)
	{
		queue->qbuf[queue->tail]=data;
		queue->tail = (queue->tail+1)  % queue->len;
		queue->count++;
	}
	
	// Check if any process is blocked. Unblock if there is
	if(queue->blockproc!=255)
		OSQueueUnblock(queue, sreg);
				
	OSExitAtomic(sreg);
}

#endif

#if OSUSE_PRIOQUEUES==1
void OSCreatePrioQueue(TPrioNode *buffer, unsigned char length, TMsgQ *queue)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	queue->blockproc=255;
	queue->count=0;
	queue->head=0;
	queue->len=length;
	queue->pqbuf=buffer;
	queue->prioQ=1;
	queue->tail=0;
	OSExitAtomic(sreg);
}

extern unsigned long _sleepTime[];
extern int _sleepFlag;

void OSPrioEnqueue(int data, unsigned char prio, TMsgQ *queue)
{
		unsigned char i;
		unsigned int iter=queue->head;
		unsigned char flag=0, sreg;
		
		OSMakeAtomic(&sreg);
		if(queue->count >= queue->len)
		{
			OSExitAtomic(sreg);
			return;
		}
		
				
		// Locate our insertion point
		while(iter != queue->tail && !flag)
		{
			flag=(queue->pqbuf[iter].prio > prio);
			
			if(!flag)
				iter=(iter+1) % queue->len;
		}
		

		// If we have found our spot, shift the rest down and insert. Otherwise insert at the end
		if(flag)
		{
			if(queue->tail > queue->head)
				for(i=queue->tail-1; i>=iter && i != 255; i--)
					queue->pqbuf[(i+1)%queue->len]=queue->pqbuf[i];
			else
			{
				for(i=(queue->tail > 0 ? queue->tail-1 : queue->len-1); i!=iter; i=(i>0 ? i-1 : queue->len-1))
					queue->pqbuf[(i+1)%queue->len]=queue->pqbuf[i];

				// Last case
				queue->pqbuf[(i+1)%queue->len]=queue->pqbuf[i];
			}
		}
		else
			iter=queue->tail;
		
		queue->tail=(queue->tail+1)%queue->len;
		queue->pqbuf[iter].data=data;
		queue->pqbuf[iter].prio=prio;
		queue->count++;
		
		if(queue->blockproc!=255)
			OSQueueUnblock(queue, sreg);

		/* Block of code below is a work around that resolves the data corruption issue */
		// Set sleep time
		_sleepTime[_running]=0;
		_sleepFlag |= (1<<_running);
		
		// Set blocked flag
		_tasks[_running].status|=_OS_BLOCKED;
		/* Block of code above is a work around that resolves the data corruption issue */
		
		// Note: No need to remove from READY queue because a _running process would have already been de-queued from there.
		// So just call scheduler to swap.
		
		OSExitAtomic(sreg);
		OSSwap();
}
#endif
