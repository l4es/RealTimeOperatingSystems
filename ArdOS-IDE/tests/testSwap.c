/*
 * testSwap.c
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
#include "sema.h"
#include <stdlib.h>

#define F_CPU	16000000
#include <util/delay.h>

TOSSema sem1, sem2;

void task1(void *p)
{
	/*unsigned int i=0;
	unsigned int j = (unsigned int) p;*/
	while(1)
	{
		// Flash 5 times
		
		for(unsigned char i=0; i<5; i++)
		{
			digitalWrite(6, LOW);
			OSSleep(5);
			digitalWrite(6, HIGH);
			OSSleep(5);			
		}
		
		digitalWrite(6, LOW);
		// Release semaphore
		//OSGiveSema(&sem1);
		
		// Wait for sem2 to be released
		//OSTakeSema(&sem2);
	}
}

void task2(void *p)
{
//	unsigned int i=0, j = (unsigned int) p;


	while(1)
	{
		// Take semaphore
		//OSTakeSema(&sem1);
		
		// Flash 5 times
		
		for(unsigned char i=0; i<5; i++)
		{
			digitalWrite(9, LOW);
			OSSleep(250);
			digitalWrite(9, HIGH);
			OSSleep(250);
		}

		digitalWrite(9, LOW);		
		// Give sema2
		//OSGiveSema(&sem2);
	}
}

unsigned long t1Stack[30], t2Stack[30];

int main()
{
	OSInit();
	
	// Initialize two binary semaphores to 0.
	OSInitSema(&sem1, 0, 1);
	OSInitSema(&sem2, 0, 1);
	
	OSCreateTask(0, &t1Stack[29], task1, (void *) 3);
	OSCreateTask(1, &t2Stack[29], task2, (void *) 4);
	pinMode(6, OUTPUT);
	pinMode(9, OUTPUT);
	OSRun();
}