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

#include "DisplayAO.hpp"

/*---------------------- Active Object DisplayAO:: -------------------------*/

DisplayAO::DisplayAO( DWORD prio, Display* dspl ) : AObject( prio ), display(dspl), count(0) {
}

DWORD
DisplayAO::processMessage( Message *e )
{
	char * chr = " \0";
	switch( e->messageId )
	{
		case show :              // Event from Keyboard
			chr[0] = (char) (e->ctrl.data & 0x000000FF);
			display->setPosition( 0, 7 + (count % 4) );
			display->printf("  %4h  =  '%s' [%d]", e->ctrl.data, chr, count++);
			break;
	}
	return 1;
}
