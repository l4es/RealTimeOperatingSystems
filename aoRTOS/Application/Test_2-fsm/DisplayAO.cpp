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
#include "MyAO.hpp"

/*---------------------- Active Object DisplayAO:: -------------------------*/

DisplayAO::DisplayAO(DWORD prio) : AObject(prio) {
  stateMachine = new DisplayAOStateMachine(this);
}

/**
 */
DWORD
DisplayAO::processMessage(Message * msg) {
  return stateMachine->dispatch(msg);
}

/*---------------------- Finite state machine of DisplayAO:: -------------------------*/

DisplayAOStateMachine::DisplayAOStateMachine(DisplayAO *parent)
  : Fsm<Message>( (State)&DisplayAOStateMachine::initial ),
    display(0, 0xB8000), parent(parent) {
}

int
DisplayAOStateMachine::initial(Message *) {
  display.clearScreen(' ');
  display.print(" -- Hello world! (test 2 fsm) --");
  TRANSITION(&DisplayAOStateMachine::wait);
  return 1;
}

int
DisplayAOStateMachine::wait(Message *e) {
  switch (e->getMessageID()) {
    case tick :
      if (ticktack++ == 500) {
        display.clearRow(' ', 8);
        display.clearRow(' ', 9);
      }
      break;
    case show :
      task = (MyAO *) e->getSource();
      screenPosition = 4 + task->getPriority();
      text = e->getString();
      enter();
      TRANSITION(&DisplayAOStateMachine::draw);
      break;
    case show1 :
      task = (MyAO *) e->getSource();
      screenPosition = 7 + task->getPriority();
      text = e->getString();
      enter();
      TRANSITION(&DisplayAOStateMachine::draw);
      break;
    case done :
      break;
    default:
      break;
  }
  return 1;
}

/*---------------------- State Show -------------------------*/

void
DisplayAOStateMachine::enter() {
  display.setPosition(1, screenPosition);
  Display::bgColor bg;
  Display::fgColor fg;
  switch (screenPosition) {
    case 8:
      bg = Display::BGND_RED;
      fg = Display::FGND_YELLOW;
      ticktack = 0;
      break;
    case 9:
      bg = Display::BGND_BLUE;
      fg = Display::FGND_CYAN;
      ticktack = 0;
      break;
    default:
      bg = Display::BGND_BLACK;
      fg = Display::FGND_WHITE;
      break;
  }
  display.setColor(bg, fg);
  display.print((char*)text);
  delete [] text;
  Message pe(parent, task, 1, done);
  parent->outputMessage(&pe);
  Message pee(parent, parent, 1, done);
  parent->outputMessage(&pee);
}

int
DisplayAOStateMachine::draw(Message *e) {
  switch (e->getMessageID()) {
    case tick :
      TRANSITION(&DisplayAOStateMachine::wait);
      break;
    case show :
      e->setMessageID(ret);    // return the event back into queue
      return 0;
    case show1 :
      e->setMessageID(ret);    // return the event back into queue
      return 0;
    case done :
      TRANSITION(&DisplayAOStateMachine::wait);
      break;
    default:
      break;
  }
  return 1;
}
