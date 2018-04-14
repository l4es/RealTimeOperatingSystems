/*
 * profiler.h
 *
 * Created: 25/01/2016 04:47:52 PM
 *  Author: idamai
 *
 * profiler.c - Profiler main routines
 *
   	Copyright (C) 2016 Ignatius Damai
   	
   	
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
 
 
#ifndef PROFILER_H_
#define PROFILER_H_

#include "kernel.h"


#define PROFILERUSE_EEPROM	1

#define PROFILER_START_ADDRESS 0

/**
 *	This option should be left 0 as Serial Print is asynchronous and not thread safe. It might cause output corruption 
 **/
#define PROFILERUSE_PRINT	0

void setCurrentProfiled(int PID);

void tickTask();

void resetAllProfileBins();

#if PROFILERUSE_PRINT
void printTaskProfile(int PID);
#endif

void reportAndResetTaskProfile(int PID);

void resetTaskProfile(int PID);

uint16_t getTaskProfile(int PID);

#if PROFILERUSE_EEPROM
void updateWorstTimeAnalysis(int PID);

void loadWorstTimeAnalysis();

uint16_t getTaskProfileFromROM(int PID);

void clearEEPROMProfiler();

void reportWorstTimeAnalysis();

#endif

#endif /* PROFILER_H_ */