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

#include "Process.hpp"

Process::Process(DWORD prio) : priority(prio), ready(0), stop(0) {
  stack = new AO_STACK[AO_STACK_LENGTH];
}

Process::~Process() {
  delete [] stack;
}

void
Process::init( AObject * subClassThis, void cdecl (*fp)( AObject * ) ) {
  sp = &stack[AO_STACK_LENGTH];    // Load stack pointer
  *(--sp) = (DWORD)subClassThis;   // Simulate call to function with argument <this>
  *(--sp) = 0;                     // return address (fictive)
  *(--sp) = 0x00000202;            // SW = Interrupts enabled
  *(--sp) = 0x00000008;            // selector of code segment
  *(--sp) = (DWORD)fp;             // Put pointer to function staticRun() as point of interrupt
  *(--sp) = 0xAAAA;                // EAX = 0xAAAA
  *(--sp) = 0xCCCC;                // ECX = 0xCCCC
  *(--sp) = 0xDDDD;                // EDX = 0xDDDD
  *(--sp) = 0xBBBB;                // EBX = 0xBBBB
  *(--sp) = 0x0000;                // ESP = 0x0000
  *(--sp) = 0x1111;                // EBP = 0x1111
  *(--sp) = 0x2222;                // ESI = 0x2222
  *(--sp) = 0x3333;                // EDI = 0x3333
}
