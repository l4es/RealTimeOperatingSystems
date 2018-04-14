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

#include "os_cpu.hpp"
#include "startup.hpp"
#include "pc.hpp" // for debug only

extern "C" void cdecl exception_noerr(void);
extern "C" void cdecl exception_err(void);
extern "C" void cdecl iTrap(void);
extern "C" void cdecl pc_IRQ_handler_0(void);
extern "C" void cdecl pc_IRQ_handler_1(void);

struct IDTSelectorEntry
{
  WORD offset_15_00;
  WORD selector;
  WORD P_DPL_igate;
  WORD offset_31_16;
/*
  IDTSelectorEntry( const IDTSelectorEntry &se )
  {
    offset_15_00 = se.offset_15_00;
    selector = se.selector;
    P_DPL_igate = se.P_DPL_igate;
    offset_31_16 = se.offset_31_16;
  }
*/
  void operator = ( const IDTSelectorEntry &se )
  {
    offset_15_00 = se.offset_15_00;
    selector = se.selector;
    P_DPL_igate = se.P_DPL_igate;
    offset_31_16 = se.offset_31_16;
  }
};

typedef void cdecl (*ISR)(void);

ISR idt[] = {
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,

  &exception_err,
  &exception_noerr,
  &exception_err,
  &exception_err,
  &exception_err,
  &exception_err,
  &exception_err,
  &exception_noerr,

  &exception_noerr,
  &exception_err,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,

  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,
  &exception_noerr,

  &pc_IRQ_handler_0,		// INTERRUPT_TIMER            IRQ0 (20)
  &pc_IRQ_handler_0,		// INTERRUPT_KEYBOARD         IRQ1
  &pc_IRQ_handler_0,		// INTERRUPT_SLAVE8259        IRQ2
  &pc_IRQ_handler_0,		// INTERRUPT_COM2             IRQ3
  &pc_IRQ_handler_0,		// INTERRUPT_COM1             IRQ4 (24)
  &pc_IRQ_handler_0,		// INTERRUPT_LPT2             IRQ5
  &pc_IRQ_handler_0,		// INTERRUPT_FDD              IRQ6
  &pc_IRQ_handler_0,		// INTERRUPT_LPT1             IRQ7

  &pc_IRQ_handler_1,		// INTERRUPT_WALLCLOCK        IRQ8
  &pc_IRQ_handler_1,		// INTERRUPT_SLAVE8259REDIR   IRQ9
  &pc_IRQ_handler_1,		//                            IRQ10
  &pc_IRQ_handler_1,		//                            IRQ11
  &pc_IRQ_handler_1,		//                            IRQ12
  &pc_IRQ_handler_1,		// INTERRUPT_COPRO            IRQ13
  &pc_IRQ_handler_1,		// INTERRUPT_HDD              IRQ14
  &pc_IRQ_handler_1,		//                            IRQ15

  &iTrap,								// PROGRAMM INTERRUPT              (30)
};

void pmStart(void)
{
  IDTSelectorEntry * memory = 0;
  DWORD f = (DWORD)&iTrap;
  IDTSelectorEntry idtEntry = {(WORD)f, 0x8, 0x8E00, (WORD)(f>>16)};
  for (int i = 0; i < 256; i++)
  {
    *memory = idtEntry;
    memory += 1;
  }

  memory = 0;
  for (int i = 0; i < 49; i++)
  {
    f = (DWORD)idt[i];
    IDTSelectorEntry idtEntry = {(WORD)f, 0x8, 0x8E00, (WORD)(f>>16)};
    *memory = idtEntry;
    memory += 1;
  }
//  The interrupt controller is configured so that IRQ levels 0-7 trigger
//  interrupt vector 32-39; levels 8-15 trigger 40-47.
//                 mov     al, 0x11       ; 1 -  ICW4 is needed
//                                        ; 0 -  cascading I8259
//                                        ; 0 -  8 byte interrupt vector
//                                        ; 0 -  edge triggered mode
//                                        ; 1 -  must be 1 for ICW1
//                                        ; 0 -
//                                        ; 0 -
//                                        ; 0 -
    outp(0x20,0x11)
//                 out     0x20, al       ; ICW 1
//                 mov     al, 0x20       ; A7-A3 =  20(32.)
    outp(0x21,0x20)
//                 out     0x21, al       ; ICW 2
//                 mov     al, 0x04       ; irq 2 (34.) has slave
    outp(0x21,0x04)
//                 out     0x21, al       ; ICW 3
//                 mov     al, 0x01       ; 1 - 80x86 mode
//                                        ; 0 - normal EOI
//                                        ; 00 - not buffered
//                                        ; 0 - sequential
    outp(0x21,0x01)
//                 out     0x21, al       ; ICW 4
//   ;   #  Mask off all interrupts except 2.
//                 mov     al, 0xFB       ;
//                 out     0x21, al       ; OCW 1
    outp(0x21,0xFB)
//                 mov     al, 0x11       ;
//                 out     0xA0, al       ; ICW 1
    outp(0xA0,0x11)
//                 mov     al, 0x28       ;
//                 out     0xA1, al       ; ICW 2
    outp(0xA1,0x28)
//                 mov     al, 0x02       ;
//                 out     0xA1, al       ; ICW 3
    outp(0xA1,0x02)
//                 mov     al, 0x01       ;
//                 out     0xA1, al       ; ICW 4
    outp(0xA1,0x01)
//  ;    #   Mask off all interrupts.
//                 mov     al, 0xFF       ;
//                 out     0xA1, al       ; OCW 1
    outp(0xA1,0xFF)
//  ;    #   set clock
//  PC_PIT_CONTROL    EQU   (0x43)
//  PC_PIT_CLOCK_0    EQU   (0x40)
//  PC_PIT_CLOCK_1    EQU   (0x41)
//  PC_PIT_CLOCK_2    EQU   (0x42)
//                 mov     al, 0x34
    outp(0x43,0x34)
//                 out     PC_PIT_CONTROL, al
//                 mov     ax, 11932
    BYTE a = (BYTE)(11932 & 0xFF);
    outp(0x40, a)
//                 out     PC_PIT_CLOCK_0, al
//                 shr     ax, 8
    a = (BYTE)((11932 >> 8) & 0xFF);
    outp(0x40, a)
//                 out     PC_PIT_CLOCK_0, al
//
//  ;    #   unmask int 32 - timer interrupt gate
//                 intc_unmask_int 32
//    %macro intc_unmask_int 1
//    %if (%1 >= 0x20) && (%1 < 0x28)
//    %assign port 0x21
//    %else
//    %assign port 0xA1
//    %endif
//                   mov     ah, ~(1 << ( %1 & 0x07 ))
//                   in      al, port
//                   and     al, ah
//                   out     port, al
//    %endmacro
    inp(0x21, a)
    a &= ~0x01;
    outp(0x21, a)

}
