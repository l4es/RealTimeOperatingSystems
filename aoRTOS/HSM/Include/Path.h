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

#ifndef _Path_H
#define _Path_H

/** forward definition */
class State;

/**
 * This class supports operations on a graph of HSM states tree. 
 */ 

class Path
{
  protected:
/** Reference to the parent node for this node in tree hierarchy */
    State *parent;
  /**
   * Reference to the daughter node for this node in tree hierarchy. This helper field 
   * trails a moving from leaf nodes to root. This allows make moving back on the same way.
   */
    State *daughter;
  /**
   * Holds the index of a node's level in a graph of state tree. 
   * Level equals 0 corresponds with root node of state machine
   */
    int level;

  /**
   * Constructor without parameter uses for creates instance of root node.
   */
    Path();
  /**
   * Constructor creates instance of Path for given instance of State (Path class is superclass of
   * State). It takes one parameter that is a reference to the parent node of the current
   * state in the state tree.
   */
    Path( State *parent );
  /**
   * Get-method returns reference to the parent node of the instance.
   */
    State* getParent();
  /**
   * Method return parent node of state with lvl level from Path of this State: 
   * 1) if parameter lvl <= level of this state - method returns this state;
   * 2) if parameter lvl > level of this state - then method navigates throw the branch
   *  of state tree in direction to root and stores the path of the navigation in 'daughter' fields
   * of passed state tree nodes.
   * @param lvl is level of node in the current path (branch) of SM tree 
   * @return parent state of node lvl level
   */
    State* pullDownToLevel( int lvl );
  /**
   * Get-method returns the level of the State instance.
   */
    int getLevel();
};
#endif
