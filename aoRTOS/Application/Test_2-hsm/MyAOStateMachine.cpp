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

#include "MyAO.hpp"
#include "MyAOStateMachine.hpp"

/*---------------------- MyAO:: active object -------------------------*/

MyAO::MyAO( DWORD prio, Hsm* sm ) : AObject( prio ), stateMachine(sm) {
  stateMachine->setActiveObject(this);
}

DWORD
MyAO::processMessage(Message * msg) {
  return stateMachine->dispatchEvent(msg);
}

/*---------------------- Root state of MyAOStateMachine:: -------------------------*/
MyAOStateMachine::MyAOStateMachine( DWORD c ) : s1( this ) {
  second = 100;
  counter = 0;
  lost = 0;
  max = c;
}

State*
MyAOStateMachine::fireInit() {
  return 0;
}

void
MyAOStateMachine::enter() {
}

State*
MyAOStateMachine::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      if( --second < 0 ) {
        second = 100;
        if( counter++ > max ) {
          counter = 0;
          lost = 0;
          Message pe( getActiveObject(), task );
          ((MyAO *)getActiveObject())->outputMessage( &pe );
        } else {
          Display::sprintf( outputString,
            "Active object #%d count=%7d[%7d] buffer=%3d      ",
          getActiveObject()->getPriority(), counter, lost, getActiveObject()->incomingBufferLoad() );
          source = getActiveObject();
          eventType = show;
          transition( &s1 );
        }
      }
      break;
    case done :
      break;
    case task :
      {
        source = e->ctrl.pAObject;
        Display::sprintf( outputString,
          "Task(active object) # %d receives message from task # %d buffer=%3d     ",
          getActiveObject()->getPriority(), source->getPriority(), getActiveObject()->incomingBufferLoad() );
        eventType = show1;
        transition( &s1 );
      }
      break;
    default:
      break;
  }
  return 0;
}

void
MyAOStateMachine::exit() {
}

/*---------------------- State_WaitResp -------------------------*/

State*
State_WaitResp::fireInit() {
  return 0;
}

void
State_WaitResp::enter() {
  Message pe(sm->getActiveObject(), sm->eventType);
  ((MyAO *)sm->getActiveObject())->outputMessage(&pe);
}

State*
State_WaitResp::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      if( sm->second == 0 ) {
        sm->lost++;
        return 0;
      }
      break;
    case done :
      transition( root );
      return 0;
    case task :
      e->messageId = ret;    // return the event back into queue
      return 0;
    default:
      break;
  }
  return getParent();
}

void
State_WaitResp::exit() {
}
