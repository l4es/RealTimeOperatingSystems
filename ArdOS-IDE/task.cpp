/*
 * task.c
 *
 * Created: 12/4/2013 9:28:57 AM
 *  Author: dcstanc
 *
 * Task handling routines for the kernel. Provides task creation and queue management services.
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
// Priority queue routines
void prioEnq(int pid, tTCB *tasklist, tQueue *q)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);

	unsigned char i;
	unsigned int iter=q->head;
	unsigned char flag=0;
	
	if(q->ctr >= q->len)
	{
		OSExitAtomic(sreg);
		return;
	}
			
	while(iter != q->tail && !flag)
	{
		#if  OSSCHED_TYPE == OS_PRIORITY
		flag=(tasklist[q->qptr[iter]].prio > tasklist[pid].prio);
		#elif (OSSCHED_TYPE == OS_RMS) || (OSSCHED_TYPE == OS_EDF)
		flag=((tasklist[q->qptr[iter]].t >  tasklist[pid].t ||(tasklist[q->qptr[iter]].t == tasklist[pid].t && tasklist[q->qptr[iter]].c >  tasklist[pid].c )) && !(tasklist[pid].t == 0 && tasklist[pid].c == 0));
		#endif
		if(!flag)
			iter=(iter+1) % q->len;
	}

	// If we have found our spot, shift the rest down and insert. Otherwise insert at the end
	if(flag)
	{
		
		if(q->tail > q->head)
			for(i=q->tail-1; i>=iter && i != 255; i--)
				q->qptr[(i+1)%q->len]=q->qptr[i];
		else
		{
				for(i=(q->tail > 0 ? q->tail-1 : q->len-1); i!=iter; i=(i>0 ? i-1 : q->len-1))
					q->qptr[(i+1)%q->len]=q->qptr[i];

				// Last case
				q->qptr[(i+1)%q->len]=q->qptr[i];
		}
	}
	else
		iter=q->tail;
		
	q->tail=(q->tail+1)%q->len;
	

	
	
	q->qptr[iter]=pid;
	q->ctr++;
	OSExitAtomic(sreg);
}


// Initializes task priorities based on the priority queue to ensure properness
#if (OSSCHED_TYPE == OS_RMS) 
void initialPrioAssign(tTCB *tasklist, tQueue *q)
{
	unsigned int iter=q->head;
	uint8_t prio = 0;
	while(iter != q->tail)
	{
		// Only update the priority if it is not a dummy task
		if (tasklist[q->qptr[iter]].c != 0 || tasklist[q->qptr[iter]].t != 0)
		{
			tasklist[q->qptr[iter]].prio=prio;
			prio++;
		}
		iter=(iter+1) % q->len;
	}
}
#endif

void procEnq(int pid, tTCB *tasklist, tQueue *q)
{
		prioEnq(pid, tasklist, q);
}

unsigned char procPeek(tQueue *q)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	if(!q->ctr)
	{
		OSExitAtomic(sreg);
		return 255;
	}
	else
	{
		OSExitAtomic(sreg);
		return q->qptr[q->head];
	}
}

unsigned char procDeq(tQueue *q)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);
	unsigned char ret=255;
	if(q->ctr>0)
	{
		ret=q->qptr[q->head];
		q->head=(q->head+1)%q->len;
		q->ctr--;
	}
	OSExitAtomic(sreg);
	return ret;
}

void initQ(unsigned char *qbuf, unsigned char len, tQueue *q)
{
	unsigned char sreg;
	OSMakeAtomic(&sreg);

	unsigned char i;
		
	q->head=0;
	q->tail=0;
	q->qptr=qbuf;
	q->len=len;
	q->ctr=0;

	for(i=0; i<len; i++)
		q->qptr[i]=255;
	OSExitAtomic(sreg);
}
	
