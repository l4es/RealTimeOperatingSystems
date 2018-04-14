/*
 * testISRSema.c
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

TOSSema sema;
unsigned char flag=1;

void task1(void *p)
{
	
	while(1)
	{		
		//OSTakeSema(&sema);
		digitalWrite(6, HIGH);
		OSSleep(125);
		digitalWrite(6, LOW);	
		OSSleep(125);		
	}
}


ISR(INT0_vect)
{
	OSSuspendScheduler();
	if(flag)
		digitalWrite(9, HIGH);
	else
		digitalWrite(9, LOW);
	
	flag=!flag;
	OSGiveSema(&sema);
	OSResumeScheduler();
}	

void setupInt0()
{
	// Trigger INT0 on rising edge
	EICRA=0b11;
	EIMSK=0b1;
}

unsigned long t1Stack[30], t2Stack[30];
int main()
{
	OSInit();

	setupInt0();
	pinMode(9, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(2, INPUT);
	
	// Create a binary semaphore with initial value 0
	OSInitSema(&sema, 0, 1);		
	OSCreateTask(0, &t1Stack[29], task1, NULL);
	OSRun();
}
