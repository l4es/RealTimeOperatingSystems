/*
 interrupt.c - code to manage interrupts
  
 Author:        Paul Barker
 Part of:       COS
 Created:       02/09/04
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
*/

/*
	This has been split from idt.c
*/

#include <cosbase.h>

#include <cos/string.h>
#include <cos/debug.h>
#include <x86-asm.h>

#include <cos/sysinfo.h>
#include <cos/init.h>

// Table of error names, from the Intel manual
static string_t error_table[20] = {
	"Divide error",
	"Reserved #DB",
	"NMI",
	"Breakpoint",
	"Overflow",
	"BOUND range exceeded",
	"Invalid opcode",
	"Device not available / No math coprocessor",
	"Double Fault",
	"Coprocessor segment overrun / reserved",
	"Invalid TSS",
	"Segment not present",
	"Stack segment fault",
	"General protection fault",
	"Page fault",
	"Reserved",
	"x87 Floating-Point Error / Math Fault",
	"Alignment check",
	"Machine check",
	"SIMD Floating-Point exception"
};

// this should be kept up to date with what we load when booting
static word_t irqMask = 0xFFFB;

// Get the master and slave parts of an IRQ mask, straight from GeekOS
#define MASTER(mask) ( (mask) & 0xff )
#define SLAVE(mask) ( ((mask)>>8) & 0xff )

//////////////////////////////////////////////////////////////////////////
// Private functions

// Translate an interrupt error code to the error name
string_t translate_error_code(u8_t i)
{
	if (i > 47)
		return "Other";
	
	if (i > 31)
		return "IRQ";
	
	if (i > 19)
		return "Reserved";
	
	return error_table[i];
}

// taken from GeekOS
static void print_selector( const char* regName, unsigned int value )
{
    TRACE(( "%s: index=%d, ti=%d, rpl=%d\n",
	regName, value >> 3, (value >> 2) & 1, value & 3 ));
}

// dummy interrupt handler
void dummy_interrupt(interrupt_state_t* state)
{
	TRACE((" *** Bad Interrupt ***\n"));
	// dump the whole state, taken from GeekOS, with modifications
	unsigned int errorCode = state->errorCode;
	TRACE(("eax=%x ebx=%x ecx=%x edx=%x\n",
		state->eax, state->ebx, state->ecx, state->edx));
	TRACE(("esi=%x edi=%x ebp=%x\n",
		state->esi, state->edi, state->ebp));
	TRACE(("eip=%x cs=%x eflags=%x\n",
		state->eip, state->cs, state->eflags));
	TRACE(("interrupt number=%d (%s), error code=%d\n",
		state->intNum, translate_error_code(state->intNum),
		errorCode));
	TRACE(("index=%d, TI=%d, IDT=%d, EXT=%d\n",
		errorCode >> 3, (errorCode >> 2) & 1,
		(errorCode >> 1) & 1, errorCode & 1));
		
	print_selector( "cs", state->cs );
	print_selector( "ds", state->ds );
	print_selector( "es", state->es );
	print_selector( "fs", state->fs );
	print_selector( "gs", state->gs );
	
	panic("Cannot recover");
}

//////////////////////////////////////////////////////////////////////////
// Public functions

void int_init()
{
	idt_init();
	
	count_t i;
	for (i = 0; i < IDT_NENTRIES; i++)
		install_handler(i, dummy_interrupt);
}

// enable or disable an irq, mostly taken from GeekOS
void enable_irq(u8_t irq, bool_t enable)
{
	TRACE(("setting irq %d %s\n", irq, (enable ? "on" : "off")));
	u8_t oldMask, newMask, mask;

	if (enable)
		mask = irqMask & ~(1 << irq);
	else
		mask = irqMask | (1 << irq);

	oldMask = MASTER(irqMask);
	newMask = MASTER(mask);
	if ( newMask != oldMask )
	{
		outb(newMask, 0x21);
	}

	oldMask = SLAVE(irqMask);
	newMask = SLAVE(mask);
	if ( newMask != oldMask )
	{
		outb(newMask, 0xA1);
	}

	irqMask = mask;
}
