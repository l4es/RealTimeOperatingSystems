/*
 * testosswap.c
 *
 * Created: 5/21/2013 8:56:57 PM
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

void task1(void *p)
{
	while(1)
	{
		digitalWrite(9, HIGH);
		OSSwap();
		digitalWrite(9, LOW);
		OSSwap();
	}
}

void task2(void *p)
{
	while(1)
	{
		digitalWrite(6, HIGH);
		OSSwap();
		digitalWrite(6, LOW);
		OSSwap();
	}
}

unsigned long t1s[20], t2s[20];

int main()
{
	OSInit();
	OSCreateTask(0, &t1s[19], task1, NULL);
	OSCreateTask(1, &t2s[19], task2, NULL);
	OSRun();
}