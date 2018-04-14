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

#include "Include/State.h"
#include "Include/Hsm.h"

void 
State::transition( State* destination )
{
// Note: this is reference to state instance of transition source
  int sourceLevel = this->level; // level of source node of the transition
// testedNode - reference that run on all instances of State on the way from current state to destination
  State* testedNode = root->getState();
// Adjust destination node: if it far away from root then current state, move it to current state level
  State* destinationNode = destination->pullDownToLevel( testedNode->level );
// Moving along path of tree from current state to root of the tree.
// For each leaving state execute exit() method.
  while( testedNode != 0 )
  {
//    if( testedNode->level <= sourceLevel )  // while do not reach of source node level skip a testing
//    {
      if( testedNode == destination && this == destination ) // self transition
      {  
        testedNode->exit();
        testedNode->enter();
        destinationNode = destination;  // needed?
        break;  
      }
      if( testedNode == destinationNode ) 
        break;  // reach a common node for a current state and a destination state branches
//    }
      testedNode->exit(); // execute exit procedure
      destinationNode = destinationNode->pullDownToLevel( testedNode->level - 1 );
      testedNode = testedNode->parent;
  }
// Now move from common node to destination node in opposite direction.
// Other direction is a reason to execute enter() methods for each state in the path
  while( destinationNode != destination )
  {
    (destinationNode = destinationNode->daughter)->enter();
  }
// Set SM in the destination state. Check and execute initial transition(s) for given state.
  root->setState( destination );
}
