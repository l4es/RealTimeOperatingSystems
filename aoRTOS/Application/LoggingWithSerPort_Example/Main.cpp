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

#include "LoggingAO.hpp"
#include "../SerialPort_Example/Include/SerialPortAO.hpp"
#include "../SerialPort_Example/Include/BufferedSerialPortAO.hpp"
#include "memory.hpp"

extern MemoryManager* mm;

int smain( void ) {
  MemoryManager m;
  mm = &m;

  ISAObject::nestedLevel = 0;
  Display *display = new Display( 0, 0xB8000 );
  display->clearScreen( ' ' );
  display->print( "Hello world! (Logging with Serial Port Example)" );

 // Active Objects allocation :
  AOScheduler *scheduler = new AOScheduler;
  Timer *timer = new Timer(0);
//  SerialPortAO *spao = new SerialPortAO(1, display);
  BufferedSerialPortAO *spao = new BufferedSerialPortAO(1);
  LoggingAO *logao = new LoggingAO(2, display);
  TestAO *testao = new TestAO(3);


  timer->addListener(timer);
  timer->addListener(scheduler);
  timer->addListener(logao);
  timer->addListener(testao);

  logao->addListener(spao);
  logao->addListener(testao);

  spao->addListener(logao);

  testao->addListener(logao);

  scheduler->add(timer);
  scheduler->add(spao);
  scheduler->add(logao);
  scheduler->add(testao);

  scheduler->startOS();
//  we never come here
  return 0;
}

int main () {
  return smain();
}
