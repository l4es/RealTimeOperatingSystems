/*
   Copyright (C) 2008 by krasnop@bellsouth.net (Alexei Krasnopolski)

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

#ifndef TABLESTATEMACHINE_HPP_
#define TABLESTATEMACHINE_HPP_
#define PhilN 7

#include "Hsm.h"

class TableStateMachine : public Hsm
{
 private:
   WORD isForkFree[PhilN];
   WORD isPhilosopherHungry[PhilN];
   MessageID eventType;
/* State implementation */
   virtual State* fireInit();
   virtual void enter();
   virtual State* fireEvent( Message *e );
   virtual void exit();
 public:
   TableStateMachine();
   WORD * getForks(){ return isForkFree; };
};

#endif /*TABLESTATEMACHINE_HPP_*/
