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

#ifndef _LISTENERLIST_H
#define _LISTENERLIST_H

#include "commonDef.hpp"

/**
 * Class ListenerList keeps references to an Active Object that want to receive messages
 * from this Active Object.
 */
class ListenerList {
  private:
/** Array of listeners */
   AObject **list;
/** Limit of amount of listeners */
   int N;
/** current size of list */
   int size;

  public:
/** Constructors for ListenerList
 *  @param int - limit of number of listeners
 */
        ListenerList( int );
        ListenerList();
       ~ListenerList();
/**
 * Add Active object to listeners list
 * @return 0 - adding failed; 1 - successful adding.
 * @param ao - Active Object is to add to list.
 */
   int add( AObject * ao );
/**
 * Remove Active object from listeners list
 * @return 0 - removing failed; 1 - successful removing.
 * @param ao - Active Object is to remove from list.
 */
   int remove( AObject * ao );
/**
 * @return size of list
 */
   inline int length() { return size; };
/**
 * @return Active Object from list at position i
 * @param i - element position in list.
 */
   inline AObject * elementAt( int i ) { return list[i]; };
};

#endif
