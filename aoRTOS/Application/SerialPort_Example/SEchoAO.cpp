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

#include "SEchoAO.hpp"

SEchoAO::SEchoAO( DWORD prio, Display* dsply ) : AObject( prio ), display(dsply) {
  commPtr = commandBuffer;
}

DWORD
SEchoAO::processMessage( Message * msg ) {
  switch (msg->getMessageID()) {
    case newData : // new byte arrives
      {
         char s = (BYTE) msg->getBinaryData();
         *(commPtr++) = s;
         if (commPtr - commandBuffer >= 16) { // 16 bytes have received
           *commPtr = 0;
           display->setPosition(0, 21);
           display->printf(" Receive = %s     !", commandBuffer);
    // send 16 messages to the serial port AO
           Message pe(0, 0, (DWORD)0, sp_byte_out);
           char* t = commandBuffer;
           while (t - commandBuffer < 16) {
             pe.setBinaryData(*(t++));
             putOutgoingMessage(&pe);
           }
           commPtr = commandBuffer;
         }
      }
      return 1;
    default :
      return 1;
  }
}

