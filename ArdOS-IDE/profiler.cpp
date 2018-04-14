/*
 * profler.cpp
 *
 * Created: 24/01/2016 10:13:40 PM
 *  Author: idamai
 *
 * Contains main task profiler methods
 *
 *
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
 
#include "profiler.h"
#include "kernel.h"
// Use EEPROM

#if PROFILERUSE_EEPROM
#include <EEPROM.h>
#endif

volatile int _currentProfiled;

volatile uint16_t _profileBin[OSMAX_TASKS];

/**
 * setCurrentProfiled and tickTask needs to be inside an atomic chunk.
 *
 **/
void setCurrentProfiled(int PID)
{
	_currentProfiled = PID;
}

void tickTask()
{
	_profileBin[_currentProfiled]++;
};
 
 void resetAllProfileBins()
 {
	int i; 
	for(i = 0 ; i < _numTasks ; i++) {
		_profileBin[i] = 0;
	}
 }
 
 #if PROFILERUSE_PRINT
 void printTaskProfile(int PID)
 {
	char buffer[16];
	sprintf(buffer, "Task %d: %d", PID, _profileBin[PID] );
	Serial.println(buffer);
 }
 #endif
 
 void reportAndResetTaskProfile(int PID)
 {
	#if PROFILERUSE_PRINT
	printTaskProfile(PID);
	#endif
	 
	#if PROFILERUSE_EEPROM
	updateWorstTimeAnalysis(PID);
	#endif
	 
	resetTaskProfile(PID);
	 
 }
 
 void resetTaskProfile(int PID)
 {
	 
	OSMakeAtomic(&_csreg);
	_profileBin[PID] = 0;
	OSExitAtomic(_csreg);
 }
 
 uint16_t getTaskProfile(int PID)
 {
	 return _profileBin[PID];
 }
 
 // Warning: the usage of this eeprom writer and reader might cause a severe impact on the overall system performance 
 #if PROFILERUSE_EEPROM
 /**
  *	This update function consumes a lot of time. Use EEP profiler with care
  **/
 void updateWorstTimeAnalysis(int PID)
 {
	uint16_t value = getTaskProfileFromROM(PID);
	
	OSMakeAtomic(&_csreg);
	if (value < _profileBin[PID]) {
	
		int startingAddress = PROFILER_START_ADDRESS + ( PID * 2);
		
		byte lowByte = ((_profileBin[PID] >> 0) & 0xFF);
		byte highByte = ((_profileBin[PID] >> 8) & 0xFF);
		EEPROM.write(startingAddress, lowByte);
		EEPROM.write(startingAddress + 1, highByte);
		
	}
	OSExitAtomic(_csreg);
	
 }
 /**
  * Retrieves previously saved task profiles
  *
  **/
uint16_t getTaskProfileFromROM(int PID)
 {
	OSMakeAtomic(&_csreg);
	int startingAddress = PROFILER_START_ADDRESS + ( PID * 2);
		
	byte lowByte = EEPROM.read(startingAddress);
    byte highByte = EEPROM.read(startingAddress + 1);

	uint16_t value = ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
	OSExitAtomic(_csreg);
	
    return value;
 }
 
 /**
  * This function is to be run after the OS run stably.
  * This function wastes a lot of CPU calculation. Call it with caution.
  **/
 void reportWorstTimeAnalysis()
 {
	 
	OSMakeAtomic(&_csreg);
	int i;
	char buffer[16];
	for (i = 0; i < _numTasks; i++)
	{
		

		uint16_t value = getTaskProfileFromROM(i);
		
		sprintf(buffer, "Task %d: %u", i, (unsigned) value );
		Serial.println(buffer);
		
	}
	
	OSExitAtomic(_csreg);
 }
 
 void loadWorstTimeAnalysis()
 {
	OSMakeAtomic(&_csreg);
	int i;
	char buffer[16];
	for (i = 0; i < _numTasks; i++)
	{
		uint16_t value = getTaskProfileFromROM(i);
		_profileBin[i] = value;
	}
	
	OSExitAtomic(_csreg);
 }
 
 void clearEEPROMProfiler()
 {
	OSMakeAtomic(&_csreg);
	int i;
	for (i = PROFILER_START_ADDRESS; i < PROFILER_START_ADDRESS + (_numTasks * 2) ; i++)
	{
		EEPROM.write(i, 0);
	}
	OSExitAtomic(_csreg);
 }
 #endif
 
 