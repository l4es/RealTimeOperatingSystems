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

#ifndef _DISPLAY_HPP
#define _DISPLAY_HPP
#include "os_cpu.hpp"

class Display
{
  public:
    typedef enum fgColor
    { FGND_BLACK = 0x00,
      FGND_BLUE  = 0x01,
      FGND_GREEN = 0x02,
      FGND_CYAN  = 0x03,
      FGND_RED   = 0x04,
      FGND_PURPLE = 0x05,
      FGND_BROWN = 0x06,
      FGND_LIGHT_GRAY = 0x07,
      FGND_DARK_GRAY = 0x08,
      FGND_LIGHT_BLUE = 0x09,
      FGND_LIGHT_GREEN = 0x0A,
      FGND_LIGHT_CYAN = 0x0B,
      FGND_LIGHT_RED = 0x0C,
      FGND_LIGHT_PURPLE = 0x0D,
      FGND_YELLOW = 0x0E,
      FGND_WHITE = 0x0F
    } fgColor;
    typedef enum bgColor
    { BGND_BLACK = 0x00,
      BGND_BLUE = 0x10,
      BGND_GREEN = 0x20,
      BGND_CYAN = 0x30,
      BGND_RED = 0x40,
      BGND_PURPLE = 0x50,
      BGND_BROWN = 0x60,
      BGND_LIGHT_GRAY = 0x70
    } bgColor;
    typedef enum blink
    { BLINK = 0x80,
      NOBLINK = 0
    } blink;

  private:
    int port;
    int x, y;
    int maxX, maxY;
    unsigned long baseAddress;
    int color;

  public:
    Display( int prt, unsigned long bAdr) : port( prt ), baseAddress( bAdr )
     { setPosition( 0, 0 ); maxX = 80; maxY = 25; setColor( BGND_BLACK, FGND_LIGHT_GRAY, NOBLINK ); };
    void  clearScreen( int ch );
    void  clearColomn( int ch, int x );
    void  clearRow( int ch, int y );
    void  setColor( bgColor c1, fgColor c2, blink blk = NOBLINK ){ color = c1 | c2 | blk; };
    void  setPosition( int coln, int row ){ y = row; x = coln; };
    void  print( char s );
    void  print( char * );
    void  print( int x, int y, char s ){ setPosition( x, y ); print( s ); };
    void  print( int x, int y, char *s ){ setPosition( x, y ); print( s ); };
    void  printf( char * f, ... );
    static void  sprintf( char *out, char * f, ... );
};

#endif /* _DISPLAY_HPP */
