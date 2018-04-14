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

#include "SerialPortAO.hpp"
#include "BufferedSerialPortAO.hpp"
#include "memory.hpp"
#include "SEchoAO.hpp"

extern MemoryManager* mm;

int smain( void ) {
  MemoryManager m;
  mm = &m;
  ISAObject::nestedLevel = 0;
  Display display( 0, 0xB8000 );
  display.clearScreen( ' ' );
  display.print( "   Hello world! (Buffered Serial Port Example)" );

 // Active Objects allocation :
  AOScheduler scheduler;
  Timer timer( 0 );
//  SerialPortAO spao( 1, &display );
  BufferedSerialPortAO spao(1);
  SEchoAO echoao( 2, &display );


  timer.addListener( &timer );
  timer.addListener( &scheduler );

  echoao.addListener( &spao );
  spao.addListener( &echoao );
  spao.addListener( &spao );

  scheduler.add( &timer );
  scheduler.add( &spao );
  scheduler.add( &echoao );

  scheduler.startOS();
//  we never come here
  return 0;
}

int main () {
  return smain();
}
