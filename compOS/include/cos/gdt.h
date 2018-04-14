/*
 gdt.h - definitions for gdt access
  
 Author:        Paul Barker
 Part of:       COS
 Created:       15/04/04
 Last Modified: 02/09/04

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

#ifndef _COS_GDT_H_
#define _COS_GDT_H_

// The general format of a segment descriptor, taken from GeekOS
typedef struct gdt_entry {
	word_t sizeLow				 __attribute__((packed)) ;
	dword_t baseLow			: 24 __attribute__((packed)) ;
	dword_t type			: 4  __attribute__((packed)) ;
	dword_t system			: 1  __attribute__((packed)) ;
	dword_t dpl				: 2  __attribute__((packed)) ;
	dword_t present			: 1  __attribute__((packed)) ;
	dword_t sizeHigh		: 4  __attribute__((packed)) ;
	dword_t avail			: 1  __attribute__((packed)) ;
	dword_t reserved		: 1  __attribute__((packed)) ; // set to zero
	dword_t dbBit			: 1  __attribute__((packed)) ;
	dword_t granularity		: 1  __attribute__((packed)) ;
	half_t baseHigh				 __attribute__((packed)) ;
}
gdt_entry_t;

#define GDT_NUM_ENTRIES 64

typedef struct _gdt
{
	gdt_entry_t	entries[GDT_NUM_ENTRIES];
}
gdt_t;

#define gdt_set_size(g, sz) (g)->sizeLow = (sz & 0xFFFF); (g)->sizeHigh = ((sz >> 16) & 0x0F)
#define gdt_set_base(g, b) (g)->baseLow = (b & 0xFFFFFF); (g)->baseHigh = (b >> 24)

// public functions from gdt.c
gdt_entry_t* gdt_get(u8_t entry);
void gdt_put(u8_t entry, gdt_entry_t* desc);
void gdt_remove(u8_t entry);

u8_t gdt_add(u8_t type,
	     u32_t base,
	     u32_t size,
	     u8_t system,
	     u8_t granularity,
	     u8_t dbBit,
	     u8_t dpl);

void gdt_set(u8_t entry,
	     u8_t type,
	     u32_t base,
	     u32_t size,
	     u8_t system,
	     u8_t granularity,
	     u8_t dbBit,
	     u8_t dpl);

#define gdt_set_simple(a, b) gdt_set(a, b, 0, 0xFFFFFFFF, 1, 1, 1, 0)
#define gdt_add_simple(a) gdt_add(a, 0, 0xFFFFFFFF, 1, 1, 1, 0)

// Construct a segment selector.
//	From GeekOS, modified into a macro
#define SELECTOR(rpl, notGDT, index) ((rpl & 0x3) | (notGDT<< 2) | ((index & 0x1FFF) << 3))

#endif // !_COS_GDT_H_

