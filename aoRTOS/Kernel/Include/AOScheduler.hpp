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

#ifndef _AOSCHEDULER_H
#define _AOSCHEDULER_H

#include "commonDef.hpp"
#include "ISAObject.hpp"

/**
 * static external functions invoked directly after interrupt.
 * Pointers to those function are kept in IDT ( see porting information )
 */
extern "C" void timerISR( void );
extern "C" void schedulerISR( void );

/**
 * Class AOScheduler is responsible for looking for Active Object in RTOS
 * that is ready to run and has highest priority. Then scheduler switch CPU context
 * into context of this Active Object.
 */
class AOScheduler : public ISAObject {
 private:
/**
 * scheduledAOTable keeps a references to Active Objects those take participation
 * in schedule process
 */
   AObject **scheduledAOTable; //[AO_SCHEDULED_LIST_LENGTH + 1];
/** N - limit of the scheduledAOTable */
   DWORD N;
/** Priority of current running Active Object*/
   DWORD currentPrio;
 protected:
   virtual void run();
/**
 *  This method overrides the processMessage() method from AObject superclass.
 */
   virtual DWORD processMessage( Message * );
/**
 *  This method overrides the serviceInterrupt() method from ISAObject superclass.
 */
   virtual AO_STACK * serviceInterrupt( AO_STACK * stp );
 public:
       AOScheduler();
      ~AOScheduler();
/**
 * Add Active Object to scheduler's table
 */
   DWORD add( AObject * );
/**
 * Remove Active Object from scheduler's table
 */
   DWORD remove( AObject * );
/**
 * Start up RTOS
 */
   void startOS();
};

#endif
