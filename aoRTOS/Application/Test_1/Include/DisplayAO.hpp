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

#ifndef _DisplayAO_H
#define _DisplayAO_H

#include "AObject.hpp"
#include "pc.hpp"

class DisplayAO : public AObject
{
 private:
/* Object represents PC display */
  Display display;
/* helper variable to count visualisation period */
  int ticktack;
 protected:
  virtual DWORD processMessage( Message * );

 public:
  DisplayAO( DWORD );
  Display * getDisplay(){ return &display; };
};

#endif
