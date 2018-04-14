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
#ifndef MYAO_HPP_
#define MYAO_HPP_

#include "AObject.hpp"
#include "Hsm.h"

class MyAO : public AObject {
 private:
/** Reference to state machine object associated with this active object. */
   Hsm * stateMachine;
 protected:
   virtual DWORD processMessage( Message * );

 public:
   MyAO( DWORD, Hsm* );
/** gets reference to associated state machine.
 *  @return Hsm * - reference to state machine instance.
 */
   inline Hsm * getStateMachine(){ return stateMachine; };

   inline DWORD outputMessage(Message * msg) {return putOutgoingMessage(msg);};
};

#endif /*MYAO_HPP_*/
