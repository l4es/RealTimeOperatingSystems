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

#include "ISAObject.hpp"
#include "pc.hpp" // for debug only

ISAObject * ISAObject::interruptTable[AO_INTERRUPT_TABLE_LENGTH];
WORD ISAObject::nestedLevel;

ISAObject::ISAObject( DWORD prio, DWORD intNumber ) : AObject( prio ) {
  registerInterrupt( intNumber );
}

void
ISAObject::registerInterrupt( DWORD intNumber ) {
  interruptTable[intNumber] = this;
}

AO_STACK *
cdecl
processInterrupt( DWORD iN, AO_STACK * stp ) {
  ISAObject::nestedLevel++;    // count nested entering to the interrupt processing
  EXIT_CRITICAL()              // unmask interrupts

  ISAObject * obj;
  if( iN != SCHEDULER_INTERRUPT_NUMBER ) { // this is not scheduler.
    obj = ISAObject::interruptTable[iN];   // get IS Active Object corresponded iN
    if( obj != 0 )
    {
      obj->serviceInterrupt( stp );      // run interrupt processing
    }
  }

  if( ISAObject::nestedLevel == 1 )      // need epilog code ?
  {
    obj = ISAObject::interruptTable[SCHEDULER_INTERRUPT_NUMBER];  // if it is not nested interrupt
    stp = obj->serviceInterrupt( stp );                           // then run scheduler
  }

  ENTER_CRITICAL()                     // mask interrupts
  ISAObject::nestedLevel--;
  return stp;                          // cause context switch
}

void
ISAObject::run() {
  Message msg;
  while( stop == 0 ) {    // this is infinite loop while stop = 0;
    ready = (BYTE) getIncomingMessage( &msg );
    if (ready == 0) {
      AO_CONTEXT_SW();     // pass CPU control to others AO by invoking of scheduler
    } else {
      processMessage( &msg );
    }
  }
}
