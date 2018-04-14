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

#include "./Include/SerialPortAO.hpp"

BaseSerialPortAO::BaseSerialPortAO(DWORD prio) : ISAObject( prio, 4 ) {
  init();
}

void
BaseSerialPortAO::init() {
  PC_VectSet( 0x24, serialISR );

  LINE_CONTROL_REG control;
  MODEM_CONTROL_REG mdm;
// Disabling FIFO:
  interruptIdRg.byte = 0;
  interruptIdRg.save();
// set up serial port : BaundRate, 8, N, 1
  control.bits->WORD_LENGTH = 3;      // 8 bits in word
  control.bits->PARITY = 0;         // No parity
  control.bits->STOP_BITS = 0;        // 1 stop bit
  control.bits->BREAK = 0;            // turn off break
// set baud rate = 0x0006 (19200); 0x000C (9600)
  control.bits->ENBL_BAUND_RATE = 1;
  control.save();
//  buffer.byte = 0x06; // LSB = 06 (BaundRate = 19200)
  buffer.byte = 0x0C; // LSB = 0C (BaundRate = 9600)
  buffer.save();
  interruptEnableRg.byte = 0;       // MSB = 00
  interruptEnableRg.save();
  control.bits->ENBL_BAUND_RATE = 0;
  control.save();

  mdm.byte = 0x0F;
  mdm.save();
// enable interupts
  interruptEnableRg.bits->ENBL_DATA = 1;
  interruptEnableRg.bits->ENBL_THRE = 1;
  interruptEnableRg.bits->ENBL_LINE_ST = 1;
//  interruptEnableRg.bits->ENBL_MDM_ST = 1;
  interruptEnableRg.save();
// set up interrupt controller 8259
  BYTE a;
  inp(0x21, a)
  a &= ~0x10;
  outp(0x21, a)
// init serial port UART
  buffer.load();
  interruptIdRg.load();
  lsr.load();
  msr.load();
}

void
BaseSerialPortAO::interruptEnable(BYTE enbl) {
  interruptEnableRg.load();
  interruptEnableRg.bits->ENBL_THRE = enbl;
  interruptEnableRg.save();
}

SerialPortAO::SerialPortAO(DWORD prio) : BaseSerialPortAO(prio) {
  init();
}

AO_STACK *
SerialPortAO::serviceInterrupt(AO_STACK * stkp) {
  Message msg_inp(0, 0, (DWORD)0, newData);
  Message msg_out;
  Message msg_output_done(0, 0, (DWORD)0, done);

	while (1) {
		interruptIdRg.load();
		if (interruptIdRg.bits->INT_PENDING == 1) {
			break;
		}

		switch (interruptIdRg.bits->INT_ID) {
      case 3:     // Line status is changed
        lsr.load();     // reset interrupt
        break;
      case 2:     // Data available
        buffer.load();
        msg_inp.setBinaryData(buffer.byte);
        putOutgoingMessage(&msg_inp);
        break;
			case 1:			// Transmit register is empty
				if (getIncomingMessage(&msg_out) != 0) {
					buffer.byte = (BYTE) msg_out.getBinaryData();
					buffer.save();
				} else {
	        putOutgoingMessage(&msg_output_done);
          interruptEnable(0);
				}
				break;
      case 0:     // Modem status is changed
        msr.load();     // reset interrupt
        break;
		}
	}
	return stkp;
}

void
SerialPortAO::run() {
  while (stop == 0) {    // this is infinite loop while stop = 0;
    processMessage((Message *)0);
    ready = 0;          // have no an events to process
    AO_CONTEXT_SW();    // pass CPU control to others AO by invoking of scheduler
  }
}

DWORD
SerialPortAO::processMessage(Message *) {
  interruptEnable(1);
  return 1;
}
