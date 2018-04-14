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

#include "PhilosopherAO.hpp"
#include "PhilosopherStateMachine.hpp"

int staticNumber = 0;
/*---------------------- PhilosopherAO:: active object -------------------------*/

PhilosopherAO::PhilosopherAO( DWORD prio, Hsm* sm ) : AObject(prio), stateMachine(sm) {
  stateMachine->setActiveObject(this);
  n = staticNumber++;
}

DWORD
PhilosopherAO::processMessage(Message * msg) {
  return stateMachine->dispatchEvent(msg);
}

/*---------------------- Root state of PhilosopherStateMachine:: -------------------------*/
PhilosopherStateMachine::PhilosopherStateMachine() :
  thinkingState( this ), hungryState( this ), eatingState( this )
{
  timer = ((PhilosopherAO*)getActiveObject())->getNumber() * 2937;
}

State*
PhilosopherStateMachine::fireInit()
{
  return &hungryState;
}

void
PhilosopherStateMachine::enter() {
}

State*
PhilosopherStateMachine::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      time++;
      break;
    default:
      break;
  }
  return 0;
}

void
PhilosopherStateMachine::exit() {
}

/*---------------------- State_Thinking -------------------------*/

State*
State_Thinking::fireInit() {
  return 0;
}

void
State_Thinking::enter() {
  Message pe( sm->getActiveObject(), show );
  ((PhilosopherAO *)sm->getActiveObject())->outputMessage( &pe );
  sm->timer = (sm->time & 0x000001FF) * 2 + 270; // pseudo random number
}

State*
State_Thinking::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      if( --(sm->timer) == 0 ) {
        transition( &(sm->hungryState) );
      }
      break;
    default:
      break;
  }
  return getParent();
}

void
State_Thinking::exit() {
}

/*---------------------- State_Hungry -------------------------*/

State*
State_Hungry::fireInit() {
  return 0;
}

void
State_Hungry::enter() {
  Message pe( sm->getActiveObject(), show );
  ((PhilosopherAO *)sm->getActiveObject())->outputMessage( &pe );

  pe.messageId = hungry;
  ((PhilosopherAO *)sm->getActiveObject())->outputMessage( &pe );
}

State*
State_Hungry::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      break;
    default:
      if( e->messageId == (eating + ((PhilosopherAO*)sm->getActiveObject())->getNumber()) ) {
        transition( &(sm->eatingState) );
      }
      break;
  }
  return getParent();
}

void
State_Hungry::exit() {
}
/*---------------------- State_Eating -------------------------*/

State*
State_Eating::fireInit() {
  return 0;
}

void
State_Eating::enter() {
  Message pe( sm->getActiveObject(), show );
  ((PhilosopherAO *)sm->getActiveObject())->outputMessage( &pe );
  sm->timer = (sm->time & 0x000001FF) + 270; // pseudo random number
}

State*
State_Eating::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      if( --(sm->timer) == 0 ) {
        transition( &(sm->thinkingState) );
      }
      break;
    default:
      break;
  }
  return getParent();
}

void
State_Eating::exit() {
  Message pe( sm->getActiveObject(), done );
  ((PhilosopherAO *)sm->getActiveObject())->outputMessage( &pe );
}
