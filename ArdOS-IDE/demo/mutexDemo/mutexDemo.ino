#include <kernel.h>
#include <mutex.h>
#include <queue.h>
#include <sema.h>


/*
 * mutexDemo
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

unsigned int count=1;
OSCond c1, c2;
OSMutex mutex;

void printSerial(unsigned int taskNum, unsigned int count)
{
  char buffer[32];
  
  sprintf(buffer, "Task %u flashing %u times.", taskNum, count);
  Serial.println(buffer);
  OSSleep(50);
}

/* This program demonstrates the use of mutex locks and conditional variables. It assumes that LEDs are connected to pins 6 and 9.

  task1 grabs a mutex, flashes LED6 while task2 sleeps on conditional variable c2. when task1 is ready it signals task 2 and goes
  to sleep on conditional variable c1. Task 2 flashes its LED and signals task 1, etc.
  
  Set the serial monitor at 115200 baud to monitor how many times each task is flashing its LED. */
  

void task1(void *p)
{
	while(1)
	{
                printSerial(1, count);		
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
	}
}

void task2(void *p)
{
	while(1)
	{
		OSTakeMutex(&mutex);
		OSWait(&c2, &mutex);
                printSerial(2, count);
		
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
	}
}


#define NUM_PROCESSES    2

void setup()
{
	OSInit(NUM_PROCESSES);
	
        Serial.begin(115200);
        
	// Create the mutexes and conditional variables.
	OSCreateMutex(&mutex);
	OSCreateConditional(&c1);
	OSCreateConditional(&c2);
	
        OSSetStackSize(60);	
	OSCreateTask(0, task1, NULL);
	OSCreateTask(1, task2, NULL);
	pinMode(6, OUTPUT);
	pinMode(9, OUTPUT);
	OSRun();
}

void loop()
{
  // Empty
}
