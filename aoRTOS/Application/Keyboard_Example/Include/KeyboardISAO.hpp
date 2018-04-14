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
#ifndef KEYBOARDISAO_HPP_
#define KEYBOARDISAO_HPP_

#include "Timer.hpp"
#include "AOScheduler.hpp"
#include "ISAObject.hpp"
#include "pc.hpp"

/**
 * static external functions invoked directly after interrupt.
 * Pointers to those function are kept in IDT ( see porting information )
 */
extern "C" void keyboardISR( void );

class KBD_FLAGS_1 : public Register
{
		public:
			struct R {
				unsigned INT_PENDING : 1;
				unsigned INT_ID   : 2;
				unsigned : 5;
			} *r;
			KBD_FLAGS_1() : Register(0), r((R*)&byte ) {};
};

class KeyboardISAO : public ISAObject
{
	private:
		Display *display;

		union {
			BYTE byte;
			struct {
				unsigned RShift : 1;
				unsigned LShift : 1;
				unsigned Ctrl : 1;
				unsigned Alt : 1;
				unsigned Scroll : 1;
				unsigned NumLock : 1;
				unsigned CapsLock : 1;
				unsigned Insert : 1;
			} bit;
		} kbdFlags1;

		union {
			BYTE byte;
			struct {
				unsigned LCtrl : 1;
				unsigned LAlt : 1;
				unsigned SysReq : 1;
				unsigned HoldPause : 1;
				unsigned Scroll : 1;
				unsigned NumLock : 1;
				unsigned CapsLock : 1;
				unsigned Insert : 1;
			} bit;
		} kbdFlags2;

	union
	{
		BYTE byte;
		struct
		{
			unsigned E1 :1;
			unsigned E0 :1;
			unsigned :2;
			unsigned AT :1;
			unsigned :3;
		} bit;
	} kbdFlags3;

	union {
		BYTE byte;
		struct {
			unsigned ScrollLED : 1;
			unsigned NumLockLED : 1;
			unsigned CapsLockLED : 1;
			unsigned  : 1;
			unsigned Acknowledge : 1;
			unsigned Resend : 1;
			unsigned  : 1;
			unsigned Error : 1;
		} bit;
	} kbdFlags4;

   static WORD ScanXlat[];

   void command( BYTE );
   void sendCommand( BYTE );
   void setLEDs();
   WORD convert( BYTE );
 protected:
   virtual DWORD processMessage( Message * );
   virtual AO_STACK * serviceInterrupt( AO_STACK * stp );
 public:
   KeyboardISAO( DWORD, Display * );
};

#endif /*KEYBOARDISAO_HPP_*/
