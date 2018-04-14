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

#include "./Include/BufferedSerialPortAO.hpp"
#include "./Include/SerialPortAO.hpp"

BufferedSerialPortAO::BufferedSerialPortAO(DWORD prio) : BaseSerialPortAO(prio) {
  inputBuffer = new RingBuffer<BYTE>;
  outputBuffer = new RingBuffer<BYTE>;
  buffIsReady = 0;
}

AO_STACK *
BufferedSerialPortAO::serviceInterrupt(AO_STACK * stkp) {
  Message msg_input_ready(this, this, (DWORD) 0, sp_arr_in);
  Message msg_output_done(this, 0, (DWORD) 0, done);

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
      { buffer.load();
        inputBuffer->write(&buffer.byte);
        if (buffIsReady == 0 && (inputBuffer->bufferLoad() >= 32 || buffer.byte == 0)) {
          putOutgoingMessage(&msg_input_ready);
          buffIsReady = 1;
        }
      }
        break;
			case 1:			// Transmit register is empty
				if (outputBuffer->get(&buffer.byte) != 0) {
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

DWORD
BufferedSerialPortAO::processMessage(Message *msg) {
  switch (msg->getMessageID()) {
    case sp_byte_out : // byte to output
      {
        BYTE b = (BYTE) msg->getBinaryData();
        outputBuffer->write(&b);
        interruptEnable(1);
      }
      return 1;
    case sp_arr_out :  // byte array to output
      {
        BYTE *arr = msg->getString();
        while (*arr != 0) {
          outputBuffer->write(arr++);
        }
        interruptEnable(1);
        delete [] msg->getString();
      }
      return 1;
    case sp_arr_in :   // input buffer is ready to send
      {
        BYTE byte;
        Message msg_inp(0, 0, 1, newData);
        for (int i = 0; i < 32; i++) {
          if (inputBuffer->get(&byte) == 0) break;
          msg_inp.setBinaryData(byte);
          putOutgoingMessage(&msg_inp);
        }
        buffIsReady = 0;
      }
      return 1;
    default :
      return 1;
  }
}
