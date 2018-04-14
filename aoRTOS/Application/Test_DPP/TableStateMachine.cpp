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

#include "TableAO.hpp"
#include "TableStateMachine.hpp"

/*---------------------- TableAO:: active object -------------------------*/

TableAO::TableAO( DWORD prio, Hsm* sm ) : AObject(prio), stateMachine(sm) {
  stateMachine->setActiveObject(this);
}

DWORD
TableAO::processMessage(Message * msg) {
  return stateMachine->dispatchEvent(msg);
}

/*---------------------- Root state of TableStateMachine:: -------------------------*/
TableStateMachine::TableStateMachine() {
}

State*
TableStateMachine::fireInit() {
  for( int i = 0; i < PhilN; i++ ) {
    isForkFree[i] = 1;
    isPhilosopherHungry[i] = 0;
  }
  Message pe( getActiveObject(), show1 );
  ((TableAO *)getActiveObject())->outputMessage( &pe );
  return 0;
}

void
TableStateMachine::enter() {
}

State*
TableStateMachine::fireEvent( Message *e ) {
  int n, r, l, rr;
  PhilosopherAO * philosopher = (PhilosopherAO*)e->ctrl.pAObject;
  n = philosopher->getNumber();
  r = (n + 1) % PhilN;
  l = (n + PhilN - 1) % PhilN;
  switch( e->messageId ) {
    case hungry :
      if( isForkFree[n] && isForkFree[r] ) {
        isForkFree[n] = 0;
        isForkFree[r] = 0;
        Message pe( getActiveObject(), eating + n );
        ((TableAO *)getActiveObject())->outputMessage( &pe );
        pe.messageId = show1;
//        Message pe( getActiveObject(), show1 );
        ((TableAO *)getActiveObject())->outputMessage( &pe );
      } else {
        isPhilosopherHungry[n] = 1;
      }
      break;
    case done :
    {
      isForkFree[r] = 1;
      isForkFree[n] = 1;
      if( isPhilosopherHungry[l] && isForkFree[l] ) {
        isForkFree[n] = 0;
        isForkFree[l] = 0;
        isPhilosopherHungry[l] = 0;
        Message pe( getActiveObject(), eating + l );
        ((TableAO *)getActiveObject())->outputMessage( &pe );
      }
      rr = (r + 1) % PhilN;
      if( isPhilosopherHungry[r] && isForkFree[rr] ) {
        isForkFree[r] = 0;
        isForkFree[rr] = 0;
        isPhilosopherHungry[r] = 0;
        Message pe( getActiveObject(), eating + r );
        ((TableAO *)getActiveObject())->outputMessage( &pe );
      }
      Message pe( getActiveObject(), show1 );
      ((TableAO *)getActiveObject())->outputMessage( &pe );
    }
      break;
    default:
      break;
  }
  return 0;
}

void
TableStateMachine::exit() {
}
