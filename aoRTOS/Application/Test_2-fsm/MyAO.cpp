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

/*---------------------- MyAO:: active object -------------------------*/

MyAO::MyAO( DWORD prio) : AObject( prio ) {
  stateMachine = new MyAOStateMachine(this, 11 * getPriority());
}

DWORD
MyAO::processMessage(Message * msg) {
  return stateMachine->dispatch(msg);
}

/*---------------------- Root state of MyAOStateMachine:: -------------------------*/
MyAOStateMachine::MyAOStateMachine(MyAO *parent, int c)
  : Fsm<Message>( (State)&MyAOStateMachine::initial ), parent(parent){
  second = 100;
  counter = 0;
  lost = 0;
  max = c;
}

int
MyAOStateMachine::initial(Message *) {
  TRANSITION(&MyAOStateMachine::exec);
  return 1;
}

int
MyAOStateMachine::exec(Message *msg) {
  switch (msg->getMessageID()) {
    case tick :
      if (--second < 0) {
        second = 100;
        if (counter++ > max) {
          counter = 0;
          lost = 0;
          Message pe(parent, 0, 1, task);
          parent->outputMessage(&pe);
        } else {
          outputString = new char[80];
          Display::sprintf(outputString,
            "Active object #%d count=%7d[%7d] buffer=%3d      ",
            parent->getPriority(), counter, lost, parent->incomingBufferLoad());
          Message pe(parent, 0, (BYTE*) outputString, show);
          parent->outputMessage(&pe);
          TRANSITION(&MyAOStateMachine::wait);
        }
      }
      break;
    case done :
      break;
    case task :
      {
        AObject *source = msg->getSource();
        outputString = new char[80];
        Display::sprintf(outputString,
          "Task(active object) # %d receives message from task # %d buffer=%3d     ",
          parent->getPriority(), source->getPriority(), parent->incomingBufferLoad());
        Message pe(parent, 0, (BYTE*) outputString, show1);
        parent->outputMessage(&pe);
        TRANSITION(&MyAOStateMachine::wait);

        char logText[64];
        Display::sprintf(logText, ">>> [%d0ms] AO #%d receives msg from AO #%d.\n\r", Timer::getTimeStamp(), parent->getPriority(), source->getPriority());
        parent->log(3, logText);
      }
      break;
    default:
      break;
  }
  return 1;
}

int
MyAOStateMachine::wait(Message *msg) {
  switch(msg->getMessageID()) {
    case tick :
      if (second == 0) {
        lost++;
      }
      break;
    case done :
      TRANSITION(&MyAOStateMachine::exec);
      break;
    case task :
      msg->setMessageID(ret);    // return the event back into queue
      return 0;
    default:
      break;
  }
  return 1;
}
