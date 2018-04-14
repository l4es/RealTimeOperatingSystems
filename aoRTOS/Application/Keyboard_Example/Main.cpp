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
#include "KeyboardISAO.hpp"

int smain()
{
  ISAObject::nestedLevel = 0;
  Timer timer( 0 );
  AOScheduler scheduler;
  Display display( 0, 0xB8000 );
  KeyboardISAO kbao( 1, &display );
  DisplayAO displayAO (2,  &display );

  display.clearScreen( ' ' );
  display.print( "Hello world! (Keyboard Example)" );

  timer.addListener( &timer );
  timer.addListener( &kbao );
  timer.addListener( &scheduler );

  kbao.addListener( &kbao );
  kbao.addListener( &displayAO );

  scheduler.add( &timer );
  scheduler.add( &kbao );
  scheduler.add( &displayAO );

  scheduler.startOS();

  return 0;
}

int main () {
  return smain();
}
