/*
 gdt.c - GDT management
  
 Author:        Paul Barker
 Part of:       COS
 Created:       17/04/04
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

#include <cosbase.h>

#include <cos/sysinfo.h>
#include <cos/init.h>
#include <cos/debug.h>
#include <cos/mem.h>
#include <x86-asm.h>

//////////////////////////////////////////////////////////////////////////
// private functions

// loop through the gdt looking for a free entry
u8_t gdt_get_free()
{
	gdt_entry_t* desc;
	count_t i = 1;	// start with 1st entry, since 0 is left blank
	
	while (i < GDT_NUM_ENTRIES)
	{
		desc = gdt_get(i);
		if (desc->avail == 1)
		{
			return i;
		}
		++i;
	}
	
	// there doesn't seem to be any free GDT entries!
	return 0xFF;
}

//////////////////////////////////////////////////////////////////////////
// public functions

// initialise the kernel GDT
void gdt_init()
{
	TRACE(("Initialising GDT...\n"));
	word_t limitAndBase[3];
	count_t i;

	// firstly, wipe our GDT for safety
	memset((ptr_t)sysinfo->gdt, 0, 4096);

	// initialise the kernel code segment
	gdt_set_simple(1, 0x0A);
		// 1010b: code, !conforming, readable, !accessed
		// (This line taken from GeekOS)

	// initialise the kernel data segment by hand
	gdt_set_simple(2, 0x02);
		// 0010b: data, expand-up, writable, !accessed
		// (This line taken from GeekOS)

	// now set every other entry as available
	i = 3;	// start with 3rd entry
	while (i < GDT_NUM_ENTRIES)
	{
		sysinfo->gdt->entries[i++].avail = 1;
	}

	u32_t addr = (u32_t)sysinfo->gdt;
	u16_t sz = GDT_NUM_ENTRIES * sizeof(gdt_entry_t);
	
	// cruft together a limit and base, then load the GDT.
	//   This bit is taken from GeekOS, with modifications
	limitAndBase[0] = sz;
	limitAndBase[1] = addr & 0xffff;
	limitAndBase[2] = addr >> 16;

	TRACE(("Loading GDT at 0x%x, size %d bytes\n", addr, sz));
	lgdtr(limitAndBase);
	
	TRACE(("GDT ready, %d free entries of %d total entries\n",
	       GDT_NUM_ENTRIES - 3, GDT_NUM_ENTRIES));
}

// add an entry, returns the entry number
// assume interrupts are off otherwise we might have problems
u8_t gdt_add(u8_t type,
	     u32_t base /* = 0 */,
	     u32_t size /* = 0xFFFFFFFF */,
	     u8_t system /* = 1 */,
	     u8_t granularity /* = 1 */,
	     u8_t dbBit /* = 1 */,
	     u8_t dpl /* = 0 */)
{
	TRACE(("gdt_add: Adding GDT entry\n"));
	TRACE(("type=%x, base=%x, size=%x\n", type, base, size));
	TRACE(("system=%d, granularity=%d, dbBit=%d, dpl=%d\n",
		system, granularity, dbBit, dpl));
	
	u32_t i = gdt_get_free();
	gdt_entry_t* desc = &(sysinfo->gdt->entries[i]);
	
	if (i == 0xFF)
		return 0xFF;
	
	TRACE(("found free entry %d\n", i));
	
	gdt_set_size(desc, size);
	gdt_set_base(desc, base);
	desc->system = system;
	desc->present = 1;
	desc->granularity = granularity;
	desc->type = type;
	desc->dbBit = dbBit;
	desc->dpl = dpl;
	desc->avail = 0;
	
	TRACE(("gdt_add() done\n"));
	
	return i;
}

// get an existing entry
gdt_entry_t* gdt_get(u8_t entry)
{
	return &(sysinfo->gdt->entries[entry]);
}

// copy an entry into the table
// I expect this will rarely be used, but could reduce the amount of time
//    interrupts will be disabled for once memcpy() is optimised.
void gdt_put(u8_t entry, gdt_entry_t* desc)
{
	gdt_entry_t* ent = gdt_get(entry);
	TRACE(("gdt_put: copying from %x to descriptor %d at %x\n",
	       desc, entry, ent));
	memcpy(ent, desc, sizeof(gdt_entry_t));
}

// remove an entry
// again interrupts should be disabled, we always assume this will work
void gdt_remove(u8_t entry)
{
	gdt_entry_t* desc = gdt_get(entry);
	TRACE(("gdt_remove: removing entry %d at %x\n", entry, desc));
	memset((ptr_t)desc, 0, sizeof(gdt_entry_t));
	desc->avail = 1;
}

// modify an entry, or add an entry at a certain position
// assume interrupts are disabled yet again
void gdt_set(u8_t entry,
	    u8_t type,
	    u32_t base /* = 0 */,
	    u32_t size /* = 0xFFFFFFFF */,
	    u8_t system /* = 1 */,
	    u8_t granularity /* = 1 */,
	    u8_t dbBit /* = 1 */,
	    u8_t dpl /* = 0 */)
{
	gdt_entry_t* desc = gdt_get(entry);
	
	TRACE(("gdt_set: Setting GDT entry %d at %x\n", entry, desc));
	TRACE(("type=%x, base=%x, size=%x\n", type, base, size));
	TRACE(("system=%d, granularity=%d, dbBit=%d, dpl=%d\n",
		system, granularity, dbBit, dpl));
		
	memset((ptr_t)desc, 0, sizeof(gdt_entry_t));
	gdt_set_size(desc, size);
	gdt_set_base(desc, base);
	desc->system = system;
	desc->present = 1;
	desc->granularity = granularity;
	desc->type = type;
	desc->dbBit = dbBit;
	desc->dpl = dpl;
	desc->avail = 0;
	
	TRACE(("gdt_set() done\n"));
}
