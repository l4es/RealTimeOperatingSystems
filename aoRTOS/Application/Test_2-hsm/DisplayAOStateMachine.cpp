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

#include "DisplayAO.hpp"
#include "DisplayAOStateMachine.hpp"
#include "MyAOStateMachine.hpp"

/*---------------------- Active Object DisplayAO:: -------------------------*/

DisplayAO::DisplayAO( DWORD prio, Hsm* sm ) : AObject( prio ), stateMachine(sm) {
  stateMachine->setActiveObject( this );
}

/**
 * If we do not use state machine template ( stateMachine = null ) we have to override this method
 * in superclass
 */
DWORD
DisplayAO::processMessage(Message * msg) {
  return stateMachine->dispatchEvent(msg);
}

/*---------------------- Root state of DisplayAO:: -------------------------*/

DisplayAOStateMachine::DisplayAOStateMachine() : display(0, 0xB8000), s1(this) {
}

State*
DisplayAOStateMachine::fireInit() {
  return 0;
}

void
DisplayAOStateMachine::enter() {
}

State*
DisplayAOStateMachine::fireEvent(Message *e) {
  switch( e->messageId ) {
    case tick :
      if (ticktack++ == 500) {
        display.clearRow( ' ', 11 );
        display.clearRow( ' ', 12 );
      }
      break;
    case show :
      task = (MyAO *)e->ctrl.pAObject;
      screenPosition = 5 + task->getPriority();
      text = ((MyAOStateMachine*)task->getStateMachine())->getString();
      transition( &s1 );
      break;
    case show1 :
      task = (MyAO *)e->ctrl.pAObject;
      screenPosition = 10 + ((MyAOStateMachine*)task->getStateMachine())->source->getPriority();
      text = ((MyAOStateMachine*)task->getStateMachine())->getString();
      transition( &s1 );
      break;
    case done :
      break;
    default:
      break;
  }
  return 0;
}

void
DisplayAOStateMachine::exit() {
}

/*---------------------- State_Show -------------------------*/

State*
State_Show::fireInit() {
  return 0;
}

void
State_Show::enter() {
  Message pe(sm->getActiveObject(), done);
  sm->display.setPosition(1, sm->screenPosition);
  Display::bgColor bg;
  Display::fgColor fg;
  switch( sm->screenPosition ) {
    case 11:
      bg = Display::BGND_RED;
      fg = Display::FGND_YELLOW;
      sm->ticktack = 0;
      break;
    case 12:
      bg = Display::BGND_BLUE;
      fg = Display::FGND_CYAN;
      sm->ticktack = 0;
      break;
    default:
      bg = Display::BGND_BLACK;
      fg = Display::FGND_WHITE;
      break;
  }
  sm->display.setColor( bg, fg );
  sm->display.print( sm->text );
  pe.dest = sm->task;
  ((DisplayAO *)sm->getActiveObject())->outputMessage( &pe );
  pe.dest = sm->getActiveObject();
  ((DisplayAO *)sm->getActiveObject())->outputMessage( &pe );
}

State*
State_Show::fireEvent(Message *e) {
  switch (e->messageId) {
    case tick :
      transition(root);
      return 0;
    case show :
      e->messageId = ret;    // return the event back into queue
      return 0;
    case show1 :
      e->messageId = ret;    // return the event back into queue
      return 0;
    case done :
      transition(root);
      return 0;
    default:
      break;
  }
  return getParent();
}

void
State_Show::exit() {
}
