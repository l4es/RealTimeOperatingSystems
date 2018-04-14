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

#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

/*
newData - a message with new byte from serial port arrives.
done - the buffer for byte sending is empty, switch off interrupt 'transmit buffer empty'
*/

#define APP_MESSAGE_IDS newData,\
  done,\
  sp_byte_out,\
  sp_arr_out,\
  sp_arr_in

#endif /* APPLICATION_HPP_ */
