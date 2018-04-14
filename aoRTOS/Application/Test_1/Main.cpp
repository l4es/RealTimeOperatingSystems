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
#include "DisplayAO.hpp"

int smain( void )
{
  ISAObject::nestedLevel = 0;
  // Objects allocation :
  Timer timer( 0 );
  MyAO ao_1( 1, 25 ), ao_2( 2, 55 );
  DisplayAO display_ao( 3 );
  AOScheduler scheduler;

  display_ao.getDisplay()->clearScreen( ' ' );
  display_ao.getDisplay()->print( "Hello world! (test 1)" );

  timer.addListener( &timer );
  timer.addListener( &ao_1 );
  timer.addListener( &ao_2 );
  timer.addListener( &display_ao );
  timer.addListener( &scheduler );

  ao_1.addListener( &display_ao );
  ao_1.addListener( &ao_2 );

  ao_2.addListener( &display_ao );
  ao_2.addListener( &ao_1 );

  display_ao.addListener( &ao_1 );
  display_ao.addListener( &ao_2 );

  scheduler.add( &timer );
  scheduler.add( &ao_1 );
  scheduler.add( &ao_2 );
  scheduler.add( &display_ao );

  scheduler.startOS();
//  we never come here
  return 0;
}

int main () {
  return smain();
}
