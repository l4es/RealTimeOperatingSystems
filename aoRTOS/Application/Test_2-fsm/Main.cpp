/*
   Copyright (C) 2010-2012 by krasnop@bellsouth.net (Alexei Krasnopolski)

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
#include "DisplayAO.hpp"
#include "../SerialPort_Example/Include/BufferedSerialPortAO.hpp"
#include "../LoggingWithSerPort_Example/Include/LoggingAO.hpp"
#include "memory.hpp"

extern MemoryManager* mm;

int smain() {
  MemoryManager m;
  mm = &m;
  ISAObject::nestedLevel = 0;
  // Objects allocation
  MyAO *ao_1 = new MyAO(1);
  MyAO *ao_2 = new MyAO(2);
  DisplayAO *display_ao = new DisplayAO(5);
// For logging:
  BufferedSerialPortAO *spao = new BufferedSerialPortAO(3);
  LoggingAO *logao = new LoggingAO(4);

  Timer *timer = new Timer(0);
  AOScheduler *scheduler = new AOScheduler;

  timer->addListener(timer);
  timer->addListener(ao_1);
  timer->addListener(ao_2);
  timer->addListener(display_ao);
  timer->addListener(scheduler);

  ao_1->addListener(display_ao);
  ao_1->addListener(ao_2);
  ao_1->addListener(logao);

  ao_2->addListener(display_ao);
  ao_2->addListener(ao_1);
  ao_2->addListener(logao);

  logao->addListener(spao);
  logao->addListener(ao_1);
  logao->addListener(ao_2);

  spao->addListener(logao);

  display_ao->addListener(ao_1);
  display_ao->addListener(ao_2);

  scheduler->add(timer);
  scheduler->add(ao_1);
  scheduler->add(ao_2);
  scheduler->add(logao);
  scheduler->add(spao);
  scheduler->add(display_ao);

  scheduler->startOS();
  return 0;
}

int main () {
  return smain();
}
