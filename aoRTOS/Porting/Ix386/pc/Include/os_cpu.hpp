/*
   Copyright (C) 2010 by krasnop@bellsouth.net (Alexei Krasnopolski)

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
#ifndef _OS_CPU_HPP
#define _OS_CPU_HPP

#ifdef _GCC_
#define cdecl
#endif /* _GCC_ */
/*
***************************************************************************
*                          DATA TYPES
*                     (Compiler Specific)
***************************************************************************
*/

typedef unsigned char    BOOLEAN;
typedef unsigned char    BYTE;                     /* Unsigned  8 bit type */
typedef signed   char    BYTE_S;                   /* Signed    8 bit type */
typedef unsigned short   WORD;                     /* Unsigned 16 bit type */
typedef signed   short   WORD_S;                   /* Signed   16 bit type */
typedef unsigned long    DWORD;                    /* Unsigned 32 bit type */
typedef signed   long    DWORD_S;                  /* Signed   32 bit type */

typedef unsigned long    AO_STACK;                 /* Each stack entry is 32-bit wide */

/*
***************************************************************************
*                   Intel 80x86 (Protected-Mode, Flat Model)
*  Macroses for port access:
*
***************************************************************************
*/
#ifdef _GCC_
#define inp( _register_, _value_ )   \
{                                               \
    asm volatile ( "xor %%eax,%%eax ;"          \
                   "inb %%dx, %%al"             \
                   : "=a" (_value_)             \
                   :  "d"(_register_)           \
        );                                      \
}

#define outp( _register_, _value_ )          \
{                                                       \
    asm volatile ( "outb %%al,%%dx"                     \
                   :                                    \
                   : "a" (_value_), "d"(_register_) \
        );                                              \
}

#endif /* _GCC_ */

#ifdef _WATCOM_
#define inp( port, value )       \
{                                \
  _asm { mov dx, port   };      \
  _asm { in al, dx   };      \
  _asm { mov value, al };      \
}

#define outp( port, value )      \
{                                \
  _asm { mov dx, port   };      \
  _asm { mov al, value };      \
  _asm { out dx, al  };      \
}
#endif /* _WATCOM_ */

/*
***************************************************************************
*                   Intel 80x86 (Protected-Mode, Flat Model)
*
* CPU interrupt enable/disable macros:
* Disable/Enable interrupts using simple instructions.
*
***************************************************************************
*/
#ifdef _GCC_
/* Disable interrupts                        */
#define  ENTER_CRITICAL()                       \
{                                               \
    asm("cli");                                 \
}
/* Enable  interrupts                        */
#define  EXIT_CRITICAL()                        \
{                                               \
    asm("sti");                                 \
}
#endif /* _GCC_ */

#ifdef _WATCOM_
/* Disable interrupts                        */
#define  ENTER_CRITICAL()                       \
{                                               \
    _asm { cli };                                 \
}
/* Enable  interrupts                        */
#define  EXIT_CRITICAL()                        \
{                                               \
    _asm { sti };                                 \
}
#endif /* _WATCOM_ */
/*
*********************************************************************************************************
*                           Intel 80x386 (Protected-Mode, Flat Model)
* Context switching by using program interrupt 0x30
*********************************************************************************************************
*/
#ifdef _GCC_
#define  AO_CONTEXT_SW()               \
{                                      \
    asm("int $0x30");                  \
}
#endif /* _GCC_ */

#ifdef _WATCOM_
#define  AO_CONTEXT_SW()               \
{                                      \
    _asm { int 0x30 };                  \
}
#endif /* _WATCOM_ */

#endif /* _OS_CPU_HPP */
