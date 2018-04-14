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

#include "AOScheduler.hpp"
#include "Timer.hpp"
#include "PhilosopherAO.hpp"
#include "PhilosopherStateMachine.hpp"
#include "TableAO.hpp"
#include "TableStateMachine.hpp"
#include "DisplayAO.hpp"
#include "DisplayAOStateMachine.hpp"

#pragma initialize before program

int smain(void)
{
  ISAObject::nestedLevel = 0;
  staticNumber = 0;
  // Objects allocation
  PhilosopherStateMachine ph0, ph1, ph2, ph3, ph4, ph5, ph6;
  PhilosopherAO ao_0( 1, &ph0 ), ao_1( 2, &ph1 ), ao_2( 3, &ph2 ),
    ao_3( 4, &ph3 ), ao_4( 5, &ph4 ), ao_5( 6, &ph5 ), ao_6( 7, &ph6 );
  TableStateMachine table;
  TableAO tableAo( 8, &table );

  DisplayAOStateMachine displayHsm;
  DisplayAO display_ao( 9, &displayHsm );

  Timer timer( 0 );
  AOScheduler scheduler;

  displayHsm.getDisplay()->clearScreen( ' ' );
  displayHsm.getDisplay()->print( "Dining Philosopher Problem: (v 1.0.0)" );

  timer.addListener( &timer );
  timer.addListener( &ao_0 );
  timer.addListener( &ao_1 );
  timer.addListener( &ao_2 );
  timer.addListener( &ao_3 );
  timer.addListener( &ao_4 );
  timer.addListener( &ao_5 );
  timer.addListener( &ao_6 );
  timer.addListener( &display_ao );
  timer.addListener( &scheduler );

  ao_0.addListener( &display_ao );
  ao_0.addListener( &tableAo );

  ao_1.addListener( &display_ao );
  ao_1.addListener( &tableAo );

  ao_2.addListener( &display_ao );
  ao_2.addListener( &tableAo );

  ao_3.addListener( &display_ao );
  ao_3.addListener( &tableAo );

  ao_4.addListener( &display_ao );
  ao_4.addListener( &tableAo );

  ao_5.addListener( &display_ao );
  ao_5.addListener( &tableAo );

  ao_6.addListener( &display_ao );
  ao_6.addListener( &tableAo );

  tableAo.addListener( &ao_0 );
  tableAo.addListener( &ao_1 );
  tableAo.addListener( &ao_2 );
  tableAo.addListener( &ao_3 );
  tableAo.addListener( &ao_4 );
  tableAo.addListener( &ao_5 );
  tableAo.addListener( &ao_6 );
  tableAo.addListener( &display_ao );

  scheduler.add( &timer );
  scheduler.add( &ao_0 );
  scheduler.add( &ao_1 );
  scheduler.add( &ao_2 );
  scheduler.add( &ao_3 );
  scheduler.add( &ao_4 );
  scheduler.add( &ao_5 );
  scheduler.add( &ao_6 );
  scheduler.add( &tableAo );
  scheduler.add( &display_ao );

//  ph0.setActiveObject( &ao_0 );
  ph0.init();
//  ph1.setActiveObject( &ao_1 );
  ph1.init();
//  ph2.setActiveObject( &ao_2 );
  ph2.init();
//  ph3.setActiveObject( &ao_3 );
  ph3.init();
//  ph4.setActiveObject( &ao_4 );
  ph4.init();
//  ph5.setActiveObject( &ao_5 );
  ph5.init();
//  ph6.setActiveObject( &ao_6 );
  ph6.init();

  table.setActiveObject( &tableAo );
  table.init();

  displayHsm.setActiveObject( &display_ao );
  displayHsm.init();

//  debugPrint( 407, "Before start OS\0" );
  scheduler.startOS();
  return 0;
}

int main () {
  return smain();
}

