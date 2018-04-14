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

#include "Timer.hpp"
#include "AOScheduler.hpp"
#include "pc.hpp"

class MyAO : public AObject
{
 private:
  char outputString[80];
  int second, counter, lost, max, acknowledge;
 protected:
  virtual DWORD processMessage( Message * );

 public:
         MyAO( DWORD, DWORD );
  char * getString(){ return outputString; };
};

#endif /*MYAO_HPP_*/
