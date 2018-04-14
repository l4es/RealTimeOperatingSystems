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

#ifndef _Hsm_H
#define _Hsm_H

#include "State.h"
#include "RingBuffer.hpp"  
//#include "Message.h"

/**
 * This class represents a Hierarchical State Machine (HSM)
 */  
class Hsm : public State
{
 protected:
  /**
   * Holds reference to Active Object associated with this HSM 
   */
   AObject * activeObject;
  /**
   * Holds reference to current state of HSM 
   */
   State* currState;
   
 public:
   Hsm() : State(){};
  /**
   * Initializing of HSM. Moving from pseudostate (null) to initial state (this), 
   * root of HSM's state tree. 
   */  
   void init();
  /** Set- Get- methods for activeObject field */
   void setActiveObject( AObject * ao ){ activeObject = ao; };
   AObject* getActiveObject(){ return activeObject; };
  /**
   * Get-method for current state 
   */  
   State* getState();
  /**
   * Set-method for current state. After setting currState field, we have to check
   * does have this state init transition? If it has HSM moves to up along branch of state tree. 
   */  
   void setState( State *dest );
  /**
   * Dispatch method for external events. Pass the event to the states. Current state
   * is always first. After processing event HSM checks return value and in case null 
   * the process is over. 
   */  
   int dispatchEvent( Message *e );
};
#endif
