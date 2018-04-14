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

#ifndef DISPLAYAOSTATEMACHINE_HPP_
#define DISPLAYAOSTATEMACHINE_HPP_

#include "Hsm.h"
#include "MyAO.hpp"
#include "pc.hpp"

class DisplayAOStateMachine;

class State_Show : public State
{
   DisplayAOStateMachine *sm;
 public:
   State_Show( State *parent ) : State( parent ){ sm = (DisplayAOStateMachine*)root; };
/* State implementation */
   virtual State* fireInit();
   virtual void enter();
   virtual State* fireEvent( Message *e );
   virtual void exit();
};

class DisplayAOStateMachine : public Hsm
{
  friend class State_Show;
 private:
   State_Show s1;
   Display display;
   int ticktack, screenPosition;
   char* text;
   MyAO* task;
/* State implementation */
   virtual State* fireInit();
   virtual void enter();
   virtual State* fireEvent( Message *e );
   virtual void exit();
 public:
   DisplayAOStateMachine();
   Display * getDisplay(){ return &display; };
};

#endif /*DISPLAYAOSTATEMACHINE_HPP_*/
