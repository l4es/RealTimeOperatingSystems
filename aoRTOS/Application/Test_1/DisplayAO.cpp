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

DisplayAO::DisplayAO( DWORD prio ) : AObject( prio ), display( 0, 0xB8000 ) {
}

DWORD
DisplayAO::processMessage( Message * e ) {
  switch( e->messageId ) {
    case show : // show info about AO
      {
// Retrive event source
        MyAO *m = (MyAO *) e->ctrl.pAObject;
// set message position
        display.setPosition(1, 1 + m->getPriority() * 4);
// set color attributes
        display.setColor( Display::BGND_BLACK, Display::FGND_WHITE );
// print message
        display.print( m->getString() );
// send event back to the source AO
        Message pe(this, m, done);
        putOutgoingMessage( &pe );
      }
      return 1;
    case show1 : // show info about received message
      {
        MyAO *m = (MyAO *)e->ctrl.pAObject;
        DWORD prio = m->getPriority();
        display.setPosition(1, 2 + prio * 4);
        if (prio == 1)
          display.setColor(Display::BGND_RED, Display::FGND_YELLOW);
        else
          display.setColor(Display::BGND_BLUE, Display::FGND_CYAN);
        display.print(m->getString());
        display.setColor(Display::BGND_BLACK, Display::FGND_WHITE);

        Message pe(this, m, done1);
        ticktack = 0;
        putOutgoingMessage( &pe );
      }
      return 1;
    case tick:
      if (ticktack++ == 500) {
// clean lines with messages after delaY
        display.clearRow( ' ', 6 );
        display.clearRow( ' ', 10 );
      }
      return 1;
    default :
      return 1;
  }
}
