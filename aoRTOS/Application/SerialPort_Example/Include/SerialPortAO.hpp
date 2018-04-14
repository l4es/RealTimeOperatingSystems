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
#ifndef SERIALPORTAO_HPP_
#define SERIALPORTAO_HPP_

#include "Timer.hpp"
#include "AOScheduler.hpp"
#include "os_cpu.hpp"
#include "pc.hpp"
#include "Registers.hpp"

/**
 * static external functions invoked directly after interrupt.
 * Pointers to those function are kept in IDT ( see porting information )
 */
extern "C" void serialISR( void );

class BaseSerialPortAO : public ISAObject {
 protected:
	TRANSMIT_RECEIVE_BUFFER buffer;
	INTERRUPT_ENABLE_REG interruptEnableRg;
	INTERRUPT_ID_REG interruptIdRg;
  LINE_STATUS_REG lsr;
  MODEM_STATUS_REG msr;
  void interruptEnable(BYTE);
  void init();

 public:
       BaseSerialPortAO(DWORD);
};

class SerialPortAO : public BaseSerialPortAO {
 protected:
  virtual void run();
  virtual DWORD processMessage(Message*);
  virtual AO_STACK * serviceInterrupt(AO_STACK *stp);

 public:
         SerialPortAO(DWORD);
};

#endif /*SERIALPORTAO_HPP_*/
