/*
 sysinfo.h - kernel system info
  
 Author:        Paul Barker
 Part of:       COS
 Created:       05/10/04
 Last Modified: 05/10/04

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
  This way of doing things means there must be a lot of definitions here.
  I think that the alternative (all pointers as void* or u32_t) would be
  pointlessly complex to implement as every step down the tree would involve
  a type cast. This way we can do
  
  root_info->interrupts->idt[40].ig.segmentSelector = 0;
  
  If we really want to, instead of
  
  interrupt_data_t* i = (interrupt_data_t*) (root_info->interrupts);
  idtEntry* entry = (idtEntry*) (i->idt[40]);
  entry->ig.segmentSelector = 0;
  
  I know which i prefer, but you may differ.
*/

#ifndef _COS_SYSINFO_H_
#define _COS_SYSINFO_H_

#include <multiboot.h>
#include <cos/int.h>
#include <cos/gdt.h>
#include <cos/tss.h>
#include <cos/page.h>
#include <cos/heap.h>

// interrupt management
typedef struct interrupt_data
{
	idt_entry_t		idt[256];	// 2k
	interrupt_handler_t	handlers[256];	// 1k
	u8_t			reserved[1024];	// 1k
}
interrupt_data_t;

// root system info structure
typedef struct root_sysinfo
{
	interrupt_data_t*	interrupts;
	multiboot_info_t*	multiboot;
	gdt_t*			gdt;
	tss_segment_t*		tss_seg;
	page_manager_t*		page_manager;
	heap_t*			kernel_heap;
	u32_t			kernel_state;
}
root_sysinfo_t;

// some random initialisation stuff
typedef struct init_data
{
	iptr_t	multiboot_end;
	u32_t	sysinfo_size;
}
init_data_t;

extern root_sysinfo_t* sysinfo;

// kernel states
#define THREAD_ENABLED	(0x02)

#endif // !_COS_SYSINFO_H_
