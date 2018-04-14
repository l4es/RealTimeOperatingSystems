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

#include "Include/Hsm.h"

void
Hsm::init()
{
//  activeObject = ao;
  enter();
  setState( this );
}

State*
Hsm::getState()
{
  return currState;
}

  /**
   * Set-method for current state. After setting currState field, we have to check
   * does have this state init transition? If it has HSM moves to up along branch of state tree.
   */
void
Hsm::setState( State *dest )
{
  currState = dest;
  State *daughter;

  while( (daughter = currState->fireInit()) != 0 ) // if fireInit() returns a state
  {
    (currState = daughter)->enter(); // enter to the state and try fireInit() again ^
  }
}

  /**
   * Dispatch method for external events. Pass the event to the states. Current state
   * is always first. After processing event HSM checks return value and in case null
   * the process is over.
   */
int
Hsm::dispatchEvent( Message *e )
{
  State *parentState = currState;   // start with current state
  while( (parentState = parentState->fireEvent( e )) != 0 ); // fire event to all states
                        // below current state and stop on root or state that return 0.
  if( e->getMessageID() < 0 )  // if event can not be processed, fireEvent() marks event as 'ret'=-1.
    return 0;           // return 'failed' flag
  return 1;             // return 'success'
}
