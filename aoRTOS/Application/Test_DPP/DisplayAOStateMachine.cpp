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
#include "DisplayAOStateMachine.hpp"
#include "PhilosopherStateMachine.hpp"
#include "TableStateMachine.hpp"

/*---------------------- Active Object DisplayAO:: -------------------------*/

DisplayAO::DisplayAO( DWORD prio, Hsm* sm ) : AObject(prio), stateMachine(sm) {
  stateMachine->setActiveObject( this );
}

/**
 * If we do not use state machine template ( stateMachine = null ) we have to override this method
 * in superclass
 */
DWORD
DisplayAO::processMessage( Message * msg ) {
  return stateMachine->dispatchEvent( msg );
}

/*---------------------- Root state of DisplayAO:: -------------------------*/

DisplayAOStateMachine::DisplayAOStateMachine() : display( 0, 0xB8000 ),
  ss( this ), ss1( this )
{
}

State*
DisplayAOStateMachine::fireInit() {
  return 0;
}

void
DisplayAOStateMachine::enter() {
}

State*
DisplayAOStateMachine::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      break;
    case show :
      philosopher = (PhilosopherAO *)e->ctrl.pAObject;
      transition( &ss );
      break;
    case show1 :
      table = (TableAO *)e->ctrl.pAObject;
      transition( &ss1 );
      break;
    case done :
      break;
    default:
      break;
  }
  return 0;
}

void
DisplayAOStateMachine::exit()
{
}

/*---------------------- State_Show (Philosopher view)-------------------------*/

State*
State_Show::fireInit()
{
  return 0;
}

void
State_Show::enter()
{
  Message pe( sm->getActiveObject(), done );
  int numb = sm->philosopher->getNumber();
  char output[80];
  char* text = sm->philosopher->getStateMachine()->getState()->getStateAsString();
  Display::sprintf( output, "  Philosopher # %2d is ", numb );
  sm->display.setPosition( 1, 7 + numb * 2 );
  sm->display.print( output );

  if(*text == 'T')
  {
    sm->display.setColor(Display::BGND_BLACK,Display::FGND_WHITE);
  }
  else if(*text == 'H')
  {
    sm->display.setColor(Display::BGND_GREEN,Display::FGND_WHITE);
  }
  else if(*text == 'E')
  {
    sm->display.setColor(Display::BGND_PURPLE,Display::FGND_WHITE);
  }
  sm->display.print( text );
  sm->display.setColor(Display::BGND_BLACK,Display::FGND_WHITE);

  ((DisplayAO *)sm->getActiveObject())->outputMessage( &pe );
}

State*
State_Show::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      transition( root );
      return 0;
    case show :
      e->messageId = ret;    // return the event back into queue
      return 0;
    case show1 :
      e->messageId = ret;    // return the event back into queue
      return 0;
    case done :
      transition( root );
      return 0;
    default:
      break;
  }
  return getParent();
}

void
State_Show::exit() {
}
/*---------------------- State_Show1 (Table view)-------------------------*/

State*
State_Show1::fireInit() {
  return 0;
}

void
State_Show1::enter() {
  Message pe( sm->getActiveObject(), done );
  WORD * forks = ((TableStateMachine*)sm->table->getStateMachine())->getForks();
  sm->display.setPosition( 32, 3 );
  sm->display.print( "F O R K S:" );
  sm->display.setPosition( 32, 4 );
  sm->display.print( "STATE  : #" );
  for (int i = 0; i < PhilN + 1; i++) {
    WORD fork;
    char output[80];
    fork = forks[i % PhilN];
    Display::sprintf(output, "%s", (fork)?" FREE ":" BUSY ");
    if(fork) {
      sm->display.setColor(Display::BGND_GREEN,Display::FGND_BLACK);
    } else {
      sm->display.setColor(Display::BGND_PURPLE,Display::FGND_YELLOW);
    }
    sm->display.setPosition( 32, 6 + 2*i );
    sm->display.print( output );
    sm->display.setColor(Display::BGND_BLACK,Display::FGND_WHITE);
    Display::sprintf(output, " : %d", (i % PhilN));
    sm->display.setPosition( 38, 6 + 2*i );
    sm->display.print( output );
  }
  ((DisplayAO *)sm->getActiveObject())->outputMessage( &pe );
}

State*
State_Show1::fireEvent( Message *e ) {
  switch( e->messageId ) {
    case tick :
      transition( root );
      return 0;
    case show :
      e->messageId = ret;    // return the event back into queue
      return 0;
    case show1 :
      e->messageId = ret;    // return the event back into queue
      return 0;
    case done :
      transition( root );
      return 0;
    default:
      break;
  }
  return getParent();
}

void
State_Show1::exit() {
}
