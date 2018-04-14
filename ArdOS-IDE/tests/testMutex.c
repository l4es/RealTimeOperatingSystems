/*
 * testMutex.c
 *
 * Created: 4/18/2013 7:22:55 PM
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

#include "ArdOS.h"
#include <stdlib.h>

unsigned int count=1;
OSCond c1, c2;
OSMutex mutex;

void task1(void *p)
{
	while(1)
	{
		OSTakeMutex(&mutex);
		// Flash 5 times
		
		for(unsigned int i=0; i<count; i++)
		{
			digitalWrite(6, LOW);
			OSSleep(125);
			digitalWrite(6, HIGH);
			OSSleep(125);			
		}
		
		digitalWrite(6, LOW);
		count++;
		OSSignal(&c2);
		OSWait(&c1, &mutex);
		OSGiveMutex(&mutex);
		OSSleep(1);
	}
}

void task2(void *p)
{
	while(1)
	{
		OSTakeMutex(&mutex);
		OSWait(&c2, &mutex);
		
		for(unsigned char i=0; i<count; i++)
		{
			digitalWrite(9, LOW);
			OSSleep(250);
			digitalWrite(9, HIGH);
			OSSleep(250);
		}

		digitalWrite(9, LOW);
		count+=2;
		OSSignal(&c1);
		OSGiveMutex(&mutex);
//		OSSleep(50);
	}
}

void idle(void *p)
{
	volatile unsigned long ctr=0;
	
	while(1)
	{
		ctr++;
	}
}
unsigned long t1Stack[30], t2Stack[30];

int main()
{
	OSInit();
	
	// Create the queue
	OSCreateMutex(&mutex);
	OSCreateConditional(&c1);
	OSCreateConditional(&c2);
		
	OSCreateTask(0, &t1Stack[29], task1, (void *) 3);
	OSCreateTask(1, &t2Stack[29], task2, (void *) 4);
	OSSetIdleTask(idle);
	pinMode(6, OUTPUT);
	pinMode(9, OUTPUT);
	OSRun();
}
