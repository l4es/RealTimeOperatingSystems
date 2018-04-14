/*
   Copyright (C) 2007-2012 by krasnop@bellsouth.net (Alexei Krasnopolski)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef _COMMONDEF_H
#define _COMMONDEF_H

#include "os_cpu.hpp"
#include <application.hpp>

/* Data structure configuration constants */
const DWORD AO_RINGBUFFER_LENGTH       = 128;  //* set ring buffer maximum length (in Messages) */
const DWORD AO_STACK_LENGTH            = 256;  //* set stack length of Active Object (in DWORDs)*/
const DWORD AO_LISTENERS_LIST_LENGTH   = 16;    //* set length of listeners list */
const DWORD AO_SCHEDULED_LIST_LENGTH   = 16;    //* set length of AO list for scheduler */
const DWORD AO_INTERRUPT_TABLE_LENGTH  = 17;    //* set length of interrupt service AO table */
const DWORD SCHEDULER_INTERRUPT_NUMBER = 16;    //* set number of scheduler programming interrupt */
const DWORD HEAP_MEMORY_SIZE           = 128000;    //* set size of heap memory for memory manager */
const BYTE LOGGING_LEVEL = 2;

/** Enumeration of all event id are using in RTOS applications */
enum MessageID
{
  ret = -1,  /** event is returned back to the ring buffer of AO, after processing was failed */
  no = 0,    /** unknown event */
  tick,      /** system clock event */
  sec,       /** system clock event with second period */
  logging,   /** tag for logging service */
  APP_MESSAGE_IDS
};

/** forward definition of a AObject class */
class AObject;

#endif
