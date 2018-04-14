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

#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include "Message.hpp"

/**
 * Class RingBuffer keeps incoming messages. Pointer wrPo points to first empty element of buffer,
 * ready to accept an incoming message. rdPo points to first buffer element ready to be read out.
 * Pointers are moving thru buffer cyclically when pointer reach the bottom of buffer it go to
 * top.
 */
template <class MessageType>
class RingBuffer {
/***************** Fields ***************/
 private:
/** wrPo keeps index of element of queue that empty and ready to accept new message.*/
	DWORD wrPo;
/** rdPo keeps index of element of queue that ready to be read.*/
	DWORD rdPo;
  DWORD load;
/** Size of queue.*/
  DWORD N;
/** Array of Messages */
  MessageType *queue; //[AO_RINGBUFFER_LENGTH];
/***************** Methods ***************/

 public:
/** Constructor that creates RingBuffer with size of queue.
 *  @param int size - length of queue.
 */
    RingBuffer( DWORD size );
/** Default constructor.*/
    RingBuffer();
    ~RingBuffer();
/** The method writes message msg to buffer.
 *  @param MessageType * msg - reference to incoming message
 *  @return int - 1 if success, 0 if buffer is full.
 */
    DWORD write( MessageType * msg );
/** The method reads first available element of buffer, saves it in (*msg) and moves a read pointer.
 *  @param MessageType * msg - reference to destination Message object
 *  @return int - 1 if success, 0 if buffer is empty.
 */
    DWORD get( MessageType * msg );
/** The method returns amount of elements that are available for reading.
 *  ( for debugging use )
 */
    inline DWORD bufferLoad(){return load;};
};

template <class MessageType>
RingBuffer<MessageType>::RingBuffer(DWORD n) : wrPo(0), rdPo(0), load(0) {
  N = ( n > AO_RINGBUFFER_LENGTH ) ? AO_RINGBUFFER_LENGTH : n;
  queue = new MessageType[N];
}

template <class MessageType>
RingBuffer<MessageType>::RingBuffer() : N(AO_RINGBUFFER_LENGTH), wrPo(0), rdPo(0), load(0) {
  queue = new MessageType[N];
}

template <class MessageType>
RingBuffer<MessageType>::~RingBuffer() {
  delete [] queue;
}

template <class MessageType>
DWORD
RingBuffer<MessageType>::write( MessageType * message ) {
  if( load < N ) {            // is buffer full ?
    queue[wrPo++] = *message;
    if (wrPo >= N) wrPo = 0;  // set pointer to next element and revert to 0 if wrPo >= N (implements a ring)
    load++;
    return 1;
  }
  return 0;
}

template <class MessageType>
DWORD
RingBuffer<MessageType>::get( MessageType * message ) {
  if( load > 0 ) {             // is a buffer empty ?
    *message = queue[rdPo++];
    if (rdPo >= N) rdPo = 0;   // go to next element and revert to 0 if rdPo >= N (implements a ring)
    load--;
    return 1;
  }
  return 0;
}

#endif
