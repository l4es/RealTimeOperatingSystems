/*
 * queue.h
 *
 * Created: 3/5/2013 7:53:09 AM
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


#ifndef QUEUE_H_
#define QUEUE_H_

#include "kernel.h"

// Provides queue operations. Due to space constraints queues can have multiple writers but only one reader.

#if OSUSE_QUEUES==1 || OSUSE_PRIOQUEUES==1
typedef struct tpn
{
		int data;
		unsigned char prio;
} TPrioNode;

typedef struct tmq
{
	#if OSUSE_QUEUES==1
	// Pointer to queue buffer
	int *qbuf;
	#endif
	
	#if OSUSE_PRIOQUEUES==1
	TPrioNode *pqbuf;
	#endif
		
	// # of items, length of queue, head and tail pointers, process blocking on this queue, flag indicating if this is
	// a priority queue
	unsigned char count, len, head, tail, blockproc, prioQ;
	
} TMsgQ;

typedef TMsgQ OSQueue;

int OSDequeue(TMsgQ *queue);
#endif

#if OSUSE_QUEUES==1
void OSCreateQueue(int *buffer, unsigned char length, TMsgQ *queue);
void OSEnqueue(int data, TMsgQ *queue);
#endif

#if OSUSE_PRIOQUEUES==1
void OSCreatePrioQueue(TPrioNode *buffer, unsigned char length, TMsgQ *queue);
void OSPrioEnqueue(int data, unsigned char prio, TMsgQ *queue);
#endif




#endif /* QUEUE_H_ */
