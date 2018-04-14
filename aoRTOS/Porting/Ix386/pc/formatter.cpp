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

#include "formatter.hpp"

int
FormatParser::initial( char* ) {
  output[0] = 0;
  s = output;
  align = 0;
  width = 0;
  TRANSITION( &FormatParser::text );
  return 1;
}

int
FormatParser::text(char *sig) {
  switch (*sig) {
    case '%' :
      width = 0;
      align = 0;
      TRANSITION( &FormatParser::control );
      break;
    case '\0' :
      *s = *sig;
      TRANSITION( &FormatParser::finish );
      break;
    default :
      *s++ = *sig;
  }
  return 1;
}

int
FormatParser::control(char *sig) {
  switch(*sig) {
    case '%' :
      *s++ = *sig;
      TRANSITION( &FormatParser::text );
      break;
    case '-' :
      align = 1;
      break;
    case '0' : case '1' : case '2' : case '3' : case '4' :
    case '5' : case '6' : case '7' : case '8' : case '9' :
      width = (int) (*sig - '0');
      TRANSITION( &FormatParser::controlDig );
      break;
    case 's' : case 'd' : case 'h' : case 'u' :
      processParam( sig );
      break;
    default :
      *s = 0;
      TRANSITION( &FormatParser::error );
  }
  return 1;
}

int
FormatParser::controlDig(char *sig) {
  switch(*sig) {
    case '0' : case '1' : case '2' : case '3' : case '4' :
    case '5' : case '6' : case '7' : case '8' : case '9' :
      width = width * 10 + (int)(*sig - '0');
      break;
    case 's' : case 'd' : case 'h' : case 'u' :
      processParam( sig );
      break;
    default :
      *s = 0;
      TRANSITION( &FormatParser::error );
  }
  return 1;
}

int
FormatParser::controlCmd( char *sig ) {
  switch(*sig) {
    case '%' :
      width = 0;
      align = 0;
      TRANSITION( &FormatParser::control );
      break;
    case '\0' :
      *s = 0;
      TRANSITION( &FormatParser::error );
      break;
    default :
      *s++ = *sig;
      TRANSITION( &FormatParser::text );
  }
  return 1;
}

int
FormatParser::processParam(char *sig) {
  if( align == 1 ) width = -width;

  if(*sig != 's') {
    s = intToString( s, va_arg( valist, int ), width, *sig );
  } else if( *sig == 's' ) {
      s = stringToString( s, va_arg( valist, char*), width );
    } else {
      *s = 0;
      TRANSITION( &FormatParser::error );
      return 0;
    }
  TRANSITION( &FormatParser::controlCmd );
  return 1;
}

int
FormatParser::finish( char* ) {
  return 1;
}

int
FormatParser::error( char* ) {
  return 1;
}

char *
stringToString( char *s, char * arg, int width ) {
  int i = 0, align = 0, length, j;
  char *e = arg;

  if( width < 0 ) {
    width = -width;
    align = 1;
  }

  while( *e++ != 0 ) {
    i++;
  };
  length = i;
  if( width == 0 ) {
    width = length;
  } else {
    if( length > width ) {
      i = 0;
      while( width > i++ )
        *(s++) = '*';
      return s;
    }
  }
  e = arg;
  i = 0;
  if( align == 0 ) {
    for( ; i < width - length; i++ )
      *(s + i) = ' ';
  }
  for( j = 0; j < length; j++,i++ )
    *(s + i) = *(e++);
  for( ; i < width; i++ )
    *(s + i) = ' ';
  return (s + width);
}

char *
intToString( char *s, int arg, int width, char type ) {
  static char digits[] = {'0','1','2','3','4','5','6','7','8','9',
                          'A','B','C','D','E','F'};
  unsigned int a;
  char buf[80];
  int i, j, length, align = 0, minus = 0, base = ( type == 'h')? 16 : 10;
  if( width < 0 ) {
    width = -width;
    align = 1;
  }

  if( type == 'u' || type == 'h' ) {
    a = arg;
  } else {
    if( arg < 0 ) {
      a = -arg;
      minus = 1;
    } else {
      a = arg;
    }
  }

  i = 0;
  if( a == 0 ) {
    *(buf + i++) = digits[0];
  } else {
    while( a > 0 ) {
      *(buf + i++) = digits[a % base];
      a = a / base;
    }
  }

  if( minus == 1 ) {
    *(buf + i++) = '-';
  }
  length = i;
  if( width == 0 ) {
    width = length;
  } else {
    if( length > width ) {
      i = 0;
      while( width > i )
        *(buf + i++) = '*';
      length = width;
    }
  }

  i = 0;
  if( align == 0 ) {
    for( ; i < width - length; i++ )
      *(s + i) = ' ';
  }
  for( j = 0; j < length; i++, j++ )
    *(s + i) = *(buf + length - j - 1);
  for( ; i < width; i++ )
    *(s + i) = ' ';
  return (s + width);
};
