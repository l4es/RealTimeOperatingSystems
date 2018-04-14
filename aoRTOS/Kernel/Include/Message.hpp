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

#ifndef _MESSAGE_HPP
#define _MESSAGE_HPP

#include "commonDef.hpp"

enum MessageType {
  binary,
  string
};

/**
 *  class Message encapsulates fields:
 *  src -
 *  dest -
 *  data -
 *  type -
 *  messageId -
 */
class Message {
 private:
  AObject *src, *dest;
  BYTE* data;
  WORD_S type;
  WORD_S messageId;

 public:
  Message() : src((AObject*)0), dest((AObject*)0), data(0), type(binary), messageId(no) {}
  Message(AObject *src, AObject *dest, BYTE *data, MessageType type, MessageID mid) :
    src(src), dest(dest), data(data), type(type), messageId(mid) {}
  Message(AObject *src, AObject *dest, DWORD data, MessageID mid) :
    src(src), dest(dest), data((BYTE*)data), type(binary), messageId(mid) {}
  Message(AObject *src, AObject *dest, BYTE *data, MessageID mid) :
    src(src), dest(dest), data(data), type(string), messageId(mid) {}

  Message(const Message &msg) {src = msg.src; dest = msg.dest; data = msg.data; type = msg.type; messageId = msg.messageId;}
  void operator = (const Message &msg){src = msg.src; dest = msg.dest; data = msg.data; type = msg.type; messageId = msg.messageId;}

  inline MessageID getMessageID() {return (MessageID) messageId;}
  inline void setMessageID(MessageID mid) {messageId = (MessageID) mid;}
  inline MessageType getType() {return (MessageType) type;}
  DWORD getBinaryData() {
    switch (type) {
      case binary :
        return (DWORD) data;
      case string :
        return (DWORD) -1;
    }
    return (DWORD) -1;
  }
  inline void setBinaryData(DWORD d) {data = (BYTE*) d;}
  BYTE* getString() {
    switch (type) {
      case binary :
        return (BYTE*) -1;
      case string :
        return data;
    }
    return (BYTE*) -1;
  }
  inline void setString(BYTE *d) {data = d;}
  inline AObject* getSource() {return src;}
  inline AObject* getDestination() {return dest;}
};

#endif /* _MESSAGE_HPP */
