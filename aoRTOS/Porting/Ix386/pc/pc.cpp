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

#include "pc.hpp"

#include "display.cpp"
#include "formatter.cpp"

/*
*********************************************************************************************************
*                                        OBTAIN INTERRUPT VECTOR
* Description: This function reads the pointer stored at the specified vector.
* Arguments  : vect  is the desired interrupt vector number, a number between 0 and 255.
* Returns    : The address of the Interrupt handler stored at the desired vector location.
*********************************************************************************************************
*/
void * PC_VectGet( BYTE vect )
{
    WORD    *pvect;
    DWORD     hi_isr;
    DWORD     lo_isr;

    pvect = (WORD *)(idtStart + vect * 8);              /* Point into IDT at desired vector location     */
//    OS_ENTER_CRITICAL();
    hi_isr   = *(pvect + 3);                            /* Obtain the vector's OFFSET                    */
    lo_isr   = *pvect;
//    OS_EXIT_CRITICAL();
    return (void *)((hi_isr << 16) | (lo_isr & 0xFFFF));
}

/*
*********************************************************************************************************
*                                        INSTALL INTERRUPT VECTOR
* Description: This function sets an interrupt vector in the interrupt vector table.
* Arguments  : vect  is the desired interrupt vector number, a number between 0 and 255.
*              isr   is a pointer to a function to execute when the interrupt or exception occurs.
* Returns    : none
*********************************************************************************************************
*/
WORD * PC_VectSet( BYTE vect, void (*isr)(void) )
{
    WORD    *pvect;

    pvect    = (WORD *)(idtStart + vect * 8);     /* Point into IDT at desired vector location     */
//    OS_ENTER_CRITICAL();
    *(pvect + 3) = (WORD)((DWORD)isr >> 16);                  /* Store ISR offset*/
    *pvect   = (WORD)((DWORD)isr & 0x0000FFFF);
//    OS_EXIT_CRITICAL();
    return pvect;
}

void
debugPrint( int pos, char s )
{ BYTE  *pscr = (BYTE *)(0xb8000 + pos * 2);

  *pscr++ = s;                         // Put character in video RAM
  *pscr = 0x07;
}

void
debugPrint( int pos, char* s )
{
  while( *s != 0 )
  {
    debugPrint( pos++, *s );
    s++;
  }
}
