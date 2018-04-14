/*
 debug.c - debugging routines
  
 Author:        Paul Barker
 Part of:       COS
 Created:       07/09/04
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

	Expect some more debugging functions in a later release, this stuff
is pretty simple and better debugging will be needed before we can develop
really good code.

	This stuff uses printk() wherever possible except in panic(), where
the output must be guarenteed safe. If the console is broke then printk could
fail.

*/

#include <cosbase.h>
#include <cos/debug.h>
#include <cos/string.h>
#include <x86-asm.h>
#include <cos/thread.h>

// including <cos/string.h> gives us varargs support

void panic(cstring_t msg)
{
	cli();
	TRACE(("Kernel Panic: %s\n", msg));
//	printk("Kernel Panic: %s\n", msg);
	
	// try a halt to preserve processor power
	while (1)
		halt();
}

void FailAssert(cstring_t cond, cstring_t file, int_t line)
{
	TRACE(("Failed assertion (%s) at %s: %d\n", cond, file, line));
//	printk("Failed assertion (%s) at %s: %d\n", cond, file, line);
	panic("Cannot recover!");
}

//////////////////////////////////////////////////////////////////////////
// Debug messages

static char_t trace_buffer[1024];
extern bool_t sout(char_t ch, char_t port);
static mutex_t trace_m;

void Trace(cstring_t fmt, ...)
{
	static u8_t first_call = 1;
	va_list args;
	string_t s;
	if (first_call)
	{
		minit(&trace_m);
	}
	first_call = 0;
	
	// this needs to be protected by a Mutex (damn)
	mlock(&trace_m);
	
	va_start(args, fmt);
	
	vsnprintk(trace_buffer, 1024, fmt, args);
	
	// output the string
	s = trace_buffer;
	while (*s)
		sout(*s++, 0);
	
	va_end(args);
	
	munlock(&trace_m);
}
