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

#include <stdlib.h>

TOSSema sema;
unsigned char flag=1;

/* 
	This program demonstrates the use of semaphores from within ISRs, in particular the use of the OSSuspendScheduler and OSResumeScheduler calls.
	
	It assumes that LEDs are connected to pins 6 and 9, and that a pushbutton has been connected to pull pin 2 high when pressed 
	The LED at pin 9 will flash at a rate of 4 times per second, while the LED at pin 6 will be toggled whenever you press the push button.
*/

void task1(void *p)
{
	
	while(1)
	{		
		OSTakeSema(&sema);
		if(flag)
			digitalWrite(6, HIGH);
		else
			digitalWrite(6, LOW);
	}
}

void task2(void *p)
{
	while(1)
	{
		digitalWrite(9, HIGH);
		OSSleep(125);
		digitalWrite(9, LOW);
		OSSleep(125);		
	}
}

ISR(INT0_vect)
{
	OSSuspendScheduler();
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
	OSCreateTask(1, &t2Stack[29], task2, NULL);
	OSRun();
}
