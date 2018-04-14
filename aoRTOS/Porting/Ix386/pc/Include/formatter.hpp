/*
   Copyright (C) 2010 - 2012 by krasnop@bellsouth.net (Alexei Krasnopolski)

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

#ifndef _FORMATTER_HPP
#define _FORMATTER_HPP
#include    <stdarg.h>
#include "os_cpu.hpp"

class FormatParser : public Fsm<char> {
  private:
    char *output, *s;
    int align, width;
    va_list valist;
  public:
             FormatParser( char * out, va_list ap ) : Fsm<char>( (State)&FormatParser::initial )
             { output = out; align = 0; width = 0;
               va_copy(valist, ap);
//               valist= ap;
               s = output; };
//    char     getOutput( int i ){ return (((output + i) <= s)? output[i] : 0); };
    int      isError(){ return (myState == (State)&FormatParser::error); };
    int      isFinish(){ return (myState == (State)&FormatParser::finish); };
  private:
    int  initial( char* );
    int  text( char* );
    int  control( char* );
    int  controlDig( char*);
    int  controlCmd( char* );
    int  finish( char* );
    int  error( char* );
    int  processParam( char* );
};

char *
intToString( char *s, int arg, int width, char type );

char *
stringToString( char *s, char * arg, int width );

#endif /* _FORMATTER_HPP */
