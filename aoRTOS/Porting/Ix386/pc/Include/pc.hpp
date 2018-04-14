/*
   Copyright (C) 2010-2012 by krasnop@bellsouth.net (Alexei Krasnopolski)

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

#ifndef _PC_HPP
#define _PC_HPP
#include "os_cpu.hpp"
#include "display.hpp"
#include <fsm.hpp>
#include "formatter.hpp"

#define  idtStart                  0x0       // Start Interrupt Descriptor Table in memory

inline BYTE inport( WORD port ) { BYTE value; inp(port, value) return value; };
inline void outport( WORD port, BYTE value ) { outp(port, value) };

class Register
{
	private:
		WORD address;

	protected:
    Register( WORD adrs ) : address(adrs) { byte = 0; };

	public:
		BYTE byte;
    inline void save() { outport( address, byte ); };
    inline void load() { byte = inport( address ); };
};

void *
PC_VectGet( BYTE vect );
WORD *
PC_VectSet( BYTE vect, void (*isr)(void) );
void
debugPrint( int pos, char s );
void
debugPrint( int pos, char* s );
#endif /* _PC_HPP */
