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

#include "display.hpp"

void
Display::clearScreen( int ch )
{ BYTE  *pscr = (BYTE *)(baseAddress);

  for( int i = 0; i < (maxX * maxY); i++)
  {                                                     // PC display has 80 columns and 25 lines
    *pscr++ = (BYTE)ch;                                // Put character in video RAM
    *pscr++ = (BYTE)color;                             // Put video attribute in video RAM
  }
}

void
Display::clearColomn( int ch, int x )
{ BYTE *pscr = (BYTE *)(baseAddress + x * 2);
  for( int i = 0; i < maxY; i++)
  {
    *pscr++ = (BYTE)ch;             // Put character in video RAM
    *pscr-- = (BYTE)color;          // Put video attribute in video RAM
     pscr = pscr + maxX * 2;         // Position on next row
  }
}

void
Display::clearRow( int ch, int y )
{ BYTE *pscr= (BYTE *)(baseAddress + y * maxX * 2);

  for( int i = 0; i < maxX; i++)
  {
    *pscr++ = (BYTE)ch;               // Put character in video RAM
    *pscr++ = (BYTE)color;            // Put video attribute in video RAM
  }
}

void
Display::print( char s )
{ BYTE  *pscr = (BYTE *)(baseAddress + y * maxX * 2 + x * 2);

  *pscr++ = s;                         // Put character in video RAM
  *pscr   = (BYTE)color;              // Put video attribute in video RAM
  if( ++x >= maxX )
  {
    x = 0;
    y++;
  };
}

void
Display::print( char * s ) // TO DO do not change parameter s ???
{
  while( (*s) != 0 )
  {
    print( *s++ );
  }
}

void
Display::printf( char * format, ... )
{
  char out[250];
  va_list ap;
  va_start( ap, format );
  FormatParser fp( out, ap );
  fp.init();

  while( !fp.isFinish() && !fp.isError() )
  {
    fp.dispatch( format++ );
  }
  va_end( ap );
  print( out );
}

void
Display::sprintf( char * out, char * format, ... )
{
  va_list ap;
  va_start( ap, format );
  FormatParser fp( out, ap );
  fp.init();

  while( !fp.isFinish() && !fp.isError() )
  {
    fp.dispatch( format++ );
  }
  va_end( ap );
}
