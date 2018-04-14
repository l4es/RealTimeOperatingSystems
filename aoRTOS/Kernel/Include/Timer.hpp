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

#ifndef _TIMER_H
#define _TIMER_H

#include "ISAObject.hpp"

/**
 * Class Timer is a wrapper for system board clock.
 * Timer activates RTOS scheduler each time when interrupt from system clock rise.
 * Furthermore Timer can send events to the Active Objects those need time service.
 * These Active objects have to register to the Timer as listeners.
 */
class Timer : public ISAObject
{
/**
 * Helper variables:
 *   second - helps to select a seconds from tick's flow.
 *   period - helps to determ a moment to change display symbol.
 */
 private:
   WORD_S second, period;
   Message tickMsg, secMsg;
   static DWORD timeStamp;
 protected:
/**
 *  This method overrides the processMessage() method from AObject superclass.
 */
   virtual DWORD processMessage( Message * );
/**
 *  This method overrides the serviceInterrupt() method from ISAObject superclass.
 */
   virtual AO_STACK * serviceInterrupt( AO_STACK * stp );
 public:
/** Constructor creates Timer object with priority p */
   Timer( DWORD p );
   inline static DWORD getTimeStamp() {return timeStamp;};
};

#endif
