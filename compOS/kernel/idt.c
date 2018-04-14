/*
 idt.c - code to set up idt
  
 Author:        Paul Barker
 Part of:       COS
 Created:       15/04/04
 Last Modified: 06/10/04

 Copyright (C) 2004 Paul Barker
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

                     (See file "Copying")

	Some code has been included from GeekOS, see "COPYING-GEEKOS"
*/

#include <cosbase.h>

#include <cos/int.h>
#include <cos/symbols.h>
#include <cos/mem.h>
#include <x86-asm.h>
#include <cos/debug.h>
#include <cos/sysinfo.h>

// the default IDT is 2k in size

// imports from int.s
extern char g_entryPointTableStart, g_entryPointTableEnd;
extern int g_handlerSizeNoErr, g_handlerSizeErr;

// interrupts must be disabled when calling this
void low_install_handler(idt_entry_t* desc, u32_t addr)
{
	desc->ig.offsetLow = addr & 0xffff;
	desc->ig.segmentSelector = KERNEL_CS;
	desc->ig.reserved = 0;
	desc->ig.signature = 0x70; // == 01110000b
	desc->ig.dpl = 0;
	desc->ig.present = 1;
	desc->ig.offsetHigh = addr >> 16;
}

// initialise the idt
void idt_init()
{
	TRACE(("idt_init()\n"));
	word_t limitAndBase[3];
	count_t i = 0;
	idt_entry_t* the_idt = (idt_entry_t*) (sysinfo->interrupts->idt);

	// Blank out the IDT page before we start
	memset((ptr_t)the_idt, 0, 4096);

	u32_t addr = (ulong_t) &g_entryPointTableStart;
	// load dummy interrupt handlers
	while (i < IDT_NENTRIES)
	{
		low_install_handler(&(the_idt[i++]), addr);
		addr += g_handlerSizeNoErr;
	}

	addr = (u32_t)the_idt;
	
	// Cruft together a 16 bit limit and 32 bit base address
	// to load into the IDTR.
	limitAndBase[0] = IDT_NENTRIES << 3;
	limitAndBase[1] = addr & 0xffff;
	limitAndBase[2] = addr >> 16;

	// Install the new table in the IDTR.
	lidtr(limitAndBase);
	
	TRACE(("idt_init() done\n"));
}

// public function to install an interrupt handler,
//  interrupts must be disabled?
void install_handler(u8_t i, interrupt_handler_t hand)
{
	TRACE(("Installing handler for int %d,\n", i));
	TRACE(("\tat 0x%x\n", hand));
	sysinfo->interrupts->handlers[i] = hand;
}
