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

#include "./Include/LoggingAO.hpp"

LoggingAO::LoggingAO(DWORD prio) : AObject(prio) {
  state = 0;
}

DWORD
LoggingAO::processMessage(Message * msg) {
  switch (msg->getMessageID()) {
    case tick :
      return 1;
    case logging : // new logging message arrives
      if (state == 0) {
        Message pe(0, 0, msg->getString(), sp_arr_out);
        putOutgoingMessage(&pe);
        state = 1;
        return 1;
      } else {
        return 0;
      }
    case done :
      state = 0;
      return 1;
    default :
      return 1;
  }
}
