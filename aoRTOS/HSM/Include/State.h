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

#ifndef _State_H
#define _State_H

#include "Path.h"

/** forward definitions */
class Hsm;
struct Message;

/**
 * This class represents a state of state machine
 */
class State : public Path
{
  protected:
  /**
   * reference to the root node of HSM state tree. It is common for all states of HSM
   */
    Hsm *root;
  /**
   * Constructor creates instance of root state (represents Hierarchical State Machine - HSM)
   */
	  State() : Path(){ root = (Hsm *)this; };
  /**
   * The method moves HSM from current state to destination state. The given instance is a state
   * stimulated this transition.
   */
    void transition( State* destination );

  public:
  /**
   * Constructor creates instance of State.
   * @param parent - parent of this state.
   */
    State( State* parent ) : Path( parent ){ root = parent->root; };
  /**
   * Get-method returns reference to the root noode of the path which is common node for all
   * branches of state node tree of the HSM.
   */
    Hsm* getRoot(){ return root; };
/**
 *  These virtual methods have to be implemented in subclasses. They are represents
 * behavior of the state.
 */
    virtual State* fireInit(){ return 0; };// = 0;
    virtual void enter(){};// = 0;
    virtual State* fireEvent( Message* ){ return 0; };// e=e - shut down compiler.
    virtual void exit(){};// = 0;
  /**
   * Useful method to obtain human readable name of state
   */
    virtual char* getStateAsString(){ return ""; };
};

#endif
