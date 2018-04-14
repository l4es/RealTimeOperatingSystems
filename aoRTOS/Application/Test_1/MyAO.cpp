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

MyAO::MyAO(DWORD prio, DWORD c ) : AObject(prio) {
  counter = 0;
  second = 100;
  lost = 0;
  max = c;
  acknowledge = 1;
}

DWORD
MyAO::processMessage(Message * e) {
  switch (e->messageId) {
    case tick :              // Message from Timer
      if (--second < 0) {     // filter ticks to seconds
        second = 100;
        if (++counter > max) { // each max second send message to other AO
          counter = 0;
          lost = 0;
          Message pe(this, task);
          putOutgoingMessage(&pe);
        }
        if (acknowledge == 1) { // if display finished previous 'show' task then send event to DisplayAO
          Message pe(this, show);
          Display::sprintf( outputString, "Active object #%d count=%7d[lost=%7d] buffer=%3d ",
            getPriority(), counter, lost, incomingBufferLoad() );
          putOutgoingMessage(&pe);
          acknowledge = 0;
        } else {
          lost++;  // if display is busy then AO have to keep outputString object unchanged
                   // and event 'tick' (100-nd) is lost
        }
      }
      return 1;
    case done :    // display is complete the job
      acknowledge = 1;
      return 1;
    case done1 :    // display is complete the job
      acknowledge = 1;
      return 1;
    case task :    // Message comes from another AO
      if( acknowledge == 1 ) { // if display finished previous 'show' task send event to DisplayAO
        Message pe(this, show1);
        AObject * obj = e->ctrl.pAObject;
        Display::sprintf( outputString, "Active Object # %d receives message from AO # %d buffer=%3d     ",
             getPriority(), obj->getPriority(), incomingBufferLoad() );
        putOutgoingMessage(&pe);
        acknowledge = 0;
      } else
        return 0;
      return 1;
    default:
      return 1;
  }
}
