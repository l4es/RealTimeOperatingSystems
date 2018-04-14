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

#ifndef _PROCESS_HPP
#define _PROCESS_HPP

#include "commonDef.hpp"

/**
 * Class is responsible for management of the stack, priority and ready states
 * of the active object. This is a superclass for all RTOS components.
 */
class Process {
 protected:
/** Flag ready is set when active object is ready to get processor resource.*/
   BYTE ready;
   BYTE stop;
/** Priority of the active object is used by scheduler for choosing active object to awake.*/
   DWORD priority;
/** Pointer to working stack element.*/
   AO_STACK *sp;
/** Stack itself.*/
   AO_STACK *stack;
/** Constructor of the class.
 *  @param prio - priority of the active object.
 */
   Process(DWORD prio);
   ~Process();
/** Method init() prepares the stack for starting of the active object ( subClass )
 *  and for working along with RTOS.
 *  @param subClassThis - this pointer to subclass of the Process
 *  @param fp - pointer to the run() function of subclass.
 */
   void  init( AObject * subClassThis, void cdecl (*fp)( AObject * ) );
 public:
/** returns priority of the active object.*/
   inline DWORD getPriority(){ return priority; };
/** sets and gets current stack pointer.
 *  @param stp - stack pointer
 */
   inline void setSP( AO_STACK * stp ){ sp = stp; };
/** @return current content of stack pointer */
   inline AO_STACK * getSP(){ return sp; };
/** @return ready state of the active object.*/
   inline BYTE isReady(){ return ready; };
};

#endif /* _PROCESS_HPP */
