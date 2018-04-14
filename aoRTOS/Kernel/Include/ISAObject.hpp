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

#ifndef _ISAOBJECT_H
#define _ISAOBJECT_H

#include "AObject.hpp"

/**
 * Static function processInterrupt() is invoked from porting assembler code.
 * This is a connector between low-level layer and c++ kernel layer. The function
 * forward execution to serverInterrupt method of ISAObject that responsible for processing
 * given interrupt.
 */
extern "C" AO_STACK * cdecl processInterrupt( DWORD, AO_STACK * );
/**
 * Class ISAObject (Interrupt Service Active Object) is responsible for handling
 * of an interrupt event.
 * The class is superclass for all AO that implements interrupt processing.
 */
class ISAObject : public AObject {
 public:
/** interruptTable keeps references to ISAO. Index of the table is a low-level number
 * of interrupt. ( see PMStarter.asm for Ix386 port )  */
   static ISAObject * interruptTable[AO_INTERRUPT_TABLE_LENGTH];
   static WORD nestedLevel;
   virtual AO_STACK * serviceInterrupt( AO_STACK * stp ){ return stp; };
 protected:
/**
 * Method run() executes infinite loop, retrieves messages from incoming buffer and processes its.
 */
   virtual void run();
/** Register this ISAO as a handler for intNumber interrupt. */
  inline void registerInterrupt( DWORD intNumber );

 public:
/** Constructors for ISAO
 *  @param prio - priority of the active object.
 *  @param intNumber - interrupt number the object is processed.
 */
       ISAObject( DWORD prio, DWORD intNumber );
};
#endif
