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

#include "Include/Path.h"
#include "Include/State.h"

/** Constructor for root state */
Path::Path()
{
  parent = 0;
  daughter = 0;
  level = 0;
}

/** Constructor for state with parent */
Path::Path( State* prnt )
{
  parent = prnt;
  daughter = 0;
  level = parent->level + 1;
}

State*
Path::getParent()
{
  return parent;
};

State*
Path::pullDownToLevel( int lvl )
{
  State* tmp = (State *)this;      // starting position of tmp is this state
  State* prnt;
  while(  tmp->level > lvl )       // if leval of tmp > lvl (the end poin of moving)
  {                                // let's move
    if( (prnt = tmp->getParent()) == 0 )  // does tmp is root?
      return 0;
    prnt->daughter = tmp;   // mark a way we are going: daughter will help us when we 
                            // go opposit direction
    tmp = prnt;             // next step
  }
  return tmp;
};

int 
Path::getLevel()
{
  return level;  
};
  
