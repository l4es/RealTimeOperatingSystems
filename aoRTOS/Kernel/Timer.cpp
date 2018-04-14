/**
   Copyright (C) 2010 by krasnop@bellsouth.net (Alexei Krasnopolski)

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

#include "Timer.hpp"
#include "pc.hpp" // for debug only

DWORD Timer::timeStamp;

/** creates IS Active object with priority p and interrupt vector 0 */
Timer::Timer( DWORD p ) : ISAObject( p, 0 ) {
  second = 0;
  period = 0;
  Message e(this, 0, (DWORD) 0, tick);
  tickMsg = e;
  e.setMessageID(sec);
  secMsg = e;
  timeStamp = 0;
}

AO_STACK *
Timer::serviceInterrupt( AO_STACK *stk ) {
  /*
   * We are inside Interrupt Service Routine.
   * In this point Timer can do the all job that need immediate processing.
   * But if event processing can be delayed then Timer sends the message to itself and
   * its own thread will process this message by processMessage() method that is running with own priority.
   */
  tickMsg.setBinaryData(++timeStamp);
  putOutgoingMessage( &tickMsg );
  return stk;
}

DWORD
Timer::processMessage(Message*) {
  char p;
  if (--second < 0) {
    putOutgoingMessage(&secMsg);
    second = 100;
    switch (period++) {
      case 0:
        p = '-';
        break;
      case 1:
        p = '\\';
        break;
      case 2:
        p = '|';
        break;
      case 3:
        p = '/';
        break;
    }
    debugPrint( 72, "TIMER\0" );
    debugPrint( 78, p );
    if (period > 3)
      period = 0;
  }
  return 1;
}
