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
#ifndef REGISTERS_HPP_
#define REGISTERS_HPP_

#include "os_cpu.hpp"
#include "pc.hpp"

#define BASE_ADDRESS 0x03F8 /** COM1 */

class TRANSMIT_RECEIVE_BUFFER : public Register
{
	public:
    TRANSMIT_RECEIVE_BUFFER() : Register(BASE_ADDRESS + 0) {};
};

class INTERRUPT_ENABLE_REG : public Register
{
	public:
		struct R {
			unsigned ENBL_DATA : 1;
			unsigned ENBL_THRE : 1;
			unsigned ENBL_LINE_ST : 1;
			unsigned ENBL_MDM_ST : 1;
			unsigned : 4;
		} *bits;
    INTERRUPT_ENABLE_REG() : Register(BASE_ADDRESS + 1), bits((R*)&byte ) {};
};

class INTERRUPT_ID_REG : public Register
{
	public:
		struct R {
			unsigned INT_PENDING : 1;
			unsigned INT_ID   : 2;
			unsigned : 5;
		} *bits;
		INTERRUPT_ID_REG() : Register(BASE_ADDRESS + 2), bits((R*)&byte ) {};
};

class LINE_CONTROL_REG : public Register
{
	public:
		struct R {
			unsigned WORD_LENGTH : 2;
			unsigned STOP_BITS   : 1;
			unsigned PARITY: 3;
			unsigned BREAK: 1;
			unsigned ENBL_BAUND_RATE: 1;
		} *bits;
		LINE_CONTROL_REG() : Register(BASE_ADDRESS + 3), bits((R*)&byte ) {};
};

class MODEM_CONTROL_REG : public Register
{
	public:
		struct R {
			unsigned DTR : 1;
			unsigned RTS   : 1;
			unsigned OUT1: 1;
			unsigned OUT2: 1;
			unsigned ENBL_LOOP_TEST: 1;
			unsigned : 3;
		} *bits;
		MODEM_CONTROL_REG() : Register(BASE_ADDRESS + 4), bits((R*)&byte )  {};
};

class LINE_STATUS_REG : public Register
{
	public:
		struct R {
			unsigned READY : 1;
			unsigned ERROR : 4;
			unsigned THR_EMPTY: 1;
			unsigned TSRE_EMPTY: 1;
			unsigned : 1;
		} *bits;
		LINE_STATUS_REG() : Register(BASE_ADDRESS + 5), bits((R*)&byte ) {};
};

class MODEM_STATUS_REG : public Register
{
	public:
		struct R {
			unsigned CTS_CHGD : 1;
			unsigned DSR_CHGD : 1;
			unsigned RI_CHGD : 1;
			unsigned CARRIER_CHGD : 1;
			unsigned CTS : 1;
			unsigned DSR: 1;
			unsigned RI : 1;
			unsigned CARRIER: 1;
		} *bits;
		MODEM_STATUS_REG() : Register(BASE_ADDRESS + 6), bits((R*)&byte ) {};
};

#endif /*REGISTERS_HPP_*/
