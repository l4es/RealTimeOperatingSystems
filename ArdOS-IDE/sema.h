/*
 * sema.h
 *
 * Created: 23/4/2013 3:58:04 PM
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


#ifndef SEMA_H_
#define SEMA_H_

#include "kernel.h"

// Counting and binary semaphore services

#if OSUSE_SEMA==1
typedef struct bs
{
	// Semaphore flags
	unsigned int semaval, isBinary;
	unsigned char tasklist[OSMAX_TASKS];
	tQueue taskQ;
} TOSSema;

typedef TOSSema OSSema;

void OSCreateSema(TOSSema *sema, unsigned int initval, unsigned char isBinary);
void OSTakeSema(TOSSema *sema);
void OSGiveSema(TOSSema *sema);
#endif

#endif /* SEMA_H_ */
