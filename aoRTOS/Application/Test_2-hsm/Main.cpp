/*
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
#include "AOScheduler.hpp"
#include "MyAO.hpp"
#include "MyAOStateMachine.hpp"
#include "DisplayAO.hpp"
#include "DisplayAOStateMachine.hpp"

int smain() {
  ISAObject::nestedLevel = 0;
  // Objects allocation
  MyAOStateMachine hsm1( 10 ), hsm2( 27 );
  MyAO ao_1( 1, &hsm1 ), ao_2( 2, &hsm2 );

  DisplayAOStateMachine displayHsm;
  DisplayAO display_ao( 3, &displayHsm );

  Timer timer( 0 );
  AOScheduler scheduler;

  displayHsm.getDisplay()->clearScreen( ' ' );
  displayHsm.getDisplay()->print( "Hello world! (test 2 hsm)" );

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

  hsm1.init();
  hsm2.init();
  displayHsm.setActiveObject( &display_ao );
  displayHsm.init();

  scheduler.startOS();
  return 0;
}

int main () {
  return smain();
}
