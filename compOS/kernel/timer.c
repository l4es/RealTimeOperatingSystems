/*
 timer.c - setup timers
  
 Author:        Paul Barker
 Part of:       COS
 Created:       31/08/04
 Last Modified: 29/10/04

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
	For now we just set up the hardware clock,
	in future we will probably do more.
	
	The clock is set to 100Hz.
*/

#include <cosbase.h>

#include <x86-asm.h>
#include <cos/int.h>
#include <cos/debug.h>

#define TIMER_EOI_COMMAND 0x60

// implementation at end of file
void init_clock();

void clock_int(interrupt_state_t* state)
{
	// TODO: setup pre-emptive multitasking

	outb_p(TIMER_EOI_COMMAND, MASTER_PIC_PORT); // EOI to master PIC
}

void init_timer()
{
	install_handler(IDT_FIRST_EXTERNAL, &clock_int);
	init_clock();
	enable_irq(0, 1);
}

/*
    this stuff is taken from
	http://my.execpc.com/~geezer/osd/misc/index.htm#8253
    and is public domain code.
*/

#define HZ      100

#define foo (1193182L / HZ)

void init_clock()
{
	TRACE(("Starting Clock at %dHz\n", HZ));
	outb_p(0x36, 0x43);	/* channel 0, LSB/MSB, mode 3, binary */
	outb_p(foo & 0xFF, 0x40);	/* LSB */
	outb_p(foo >> 8, 0x40);       /* MSB */
}
