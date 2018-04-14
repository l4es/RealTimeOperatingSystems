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

#ifndef PHILOSOPHER_HPP_
#define PHILOSOPHER_HPP_

#include "Hsm.h"

class PhilosopherStateMachine;

class State_Thinking : public State
{
  PhilosopherStateMachine *sm;
  DWORD timer;
 public:
   State_Thinking( State *parent ) : State( parent ){ sm = (PhilosopherStateMachine*)root; };
/* State implementation */
   virtual State* fireInit();
   virtual void enter();   
   virtual State* fireEvent( Message *e );
   virtual void exit();
   virtual char* getStateAsString() { return "THINKING"; }
};

class State_Hungry : public State
{
  PhilosopherStateMachine *sm;
 public:
   State_Hungry( State *parent ) : State( parent ){ sm = (PhilosopherStateMachine*)root; };
/* State implementation */
   virtual State* fireInit();
   virtual void enter();   
   virtual State* fireEvent( Message *e );
   virtual void exit();
   virtual char* getStateAsString() { return "HUNGRY  "; }
};

class State_Eating : public State
{
  PhilosopherStateMachine *sm;
  DWORD timer;
 public:
   State_Eating( State *parent ) : State( parent ){ sm = (PhilosopherStateMachine*)root; };
/* State implementation */
   virtual State* fireInit();
   virtual void enter();   
   virtual State* fireEvent( Message *e );
   virtual void exit();
   virtual char* getStateAsString() { return "EATING  "; }
};

class PhilosopherStateMachine : public Hsm
{
  friend class State_Thinking;
  friend class State_Hungry;
  friend class State_Eating;
 private:
   State_Thinking thinkingState;
   State_Hungry hungryState;
   State_Eating eatingState;
   DWORD timer;
/* State implementation */
 public:
   virtual State* fireInit();
   virtual void enter();   
   virtual State* fireEvent( Message *e );
   virtual void exit();
   virtual char* getStateAsString() { return "ROOT    "; }

   DWORD time;

   PhilosopherStateMachine();
};

#endif /*PHILOSOPHER_HPP_*/
