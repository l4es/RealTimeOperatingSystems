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

#ifndef MYAOSTATEMACHINE_HPP
#define MYAOSTATEMACHINE_HPP

#include "Hsm.h"

class MyAOStateMachine;

class State_WaitResp : public State
{
   MyAOStateMachine *sm;
 public:
   State_WaitResp( State *parent ) : State( parent ){ sm = (MyAOStateMachine*)root; };
/* State implementation */
   virtual State* fireInit();
   virtual void enter();
   virtual State* fireEvent( Message *e );
   virtual void exit();
};

class MyAOStateMachine : public Hsm
{
   friend class State_WaitResp;
 private:
   State_WaitResp s1;
   char outputString[80];
   int counter, second, lost, max;
   MessageID eventType;
/* State implementation */
   virtual State* fireInit();
   virtual void enter();
   virtual State* fireEvent( Message *e );
   virtual void exit();
 public:
   AObject * source;
   MyAOStateMachine( DWORD c );
   char* getString(){ return outputString; };
};

#endif /*MYAOSTATEMACHINE_HPP*/
