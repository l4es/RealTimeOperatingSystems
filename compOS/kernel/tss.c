/*
 tss.c - TSS management
  
 Author:	Paul Barker
 Part of:	COS
 Created:	01/09/04
 Last Modified:	06/10/04

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

#include <cosbase.h>

#include <cos/mem.h>
#include <x86-asm.h>
#include <cos/debug.h>
#include <cos/sysinfo.h>

// initialise the kernel TSS descriptor
void tss_init()
{
	TRACE(("Initialising TSS...\n"));
	memset(sysinfo->tss_seg, 0, 4096);
	
	// values taken from GeekOS
	u8_t entry = gdt_add(0x09, // 1001b: 32 bit, !busy
			(u32_t)sysinfo->tss_seg,
			sizeof(tss_t),
			0,
			0,
			0,
			0);
	
	u16_t selector = SELECTOR(0, 0, entry);
	TRACE(("Loading selector 0x%x\n", selector));
	ltr(selector);
	TRACE(("TSS Ready\n"));
}
