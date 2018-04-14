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

#include "ListenerList.hpp"

ListenerList::ListenerList(int n) : N(n), size(0) {
  list = new AObject*[n];
}

ListenerList::ListenerList() : N(AO_LISTENERS_LIST_LENGTH), size(0) {
  list = new AObject*[AO_LISTENERS_LIST_LENGTH];
}

ListenerList::~ListenerList() {
  delete[] list;
}

int
ListenerList::add( AObject * obj )
{
  if( size >= N )                  // list is full
    return 0;
  for( int i = 0; i < size; i++ )
  {
    if( obj == list[i] )
    {
      return 1;                    // obj is already in the list
    }
  }
  list[size++] = obj;              // put in the list and increase size
  return 1;
}

int
ListenerList::remove( AObject * obj )
{
  for( int i = 0; i < size; i++ )
  {
    if( obj == list[i] )              // find obj in the list
    {
      size--;
      for( int j = i; j < size; j++ ) // shift the rest of elements to eliminate a 'hole'
      {
        list[j] = list[j+1];
      }
      return 1;
    }
  }
  return 0;
}
