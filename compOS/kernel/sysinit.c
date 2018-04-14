/*
 sysinit.c - Initialise system info
  
 Author:        Paul Barker
 Part of:       COS
 Created:       05/10/04
 Last Modified: 06/11/04

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
#include <cos/string.h>

u32_t mmap_640[5];	// map of memory from 0 to 640k
root_sysinfo_t* sysinfo;

#define setbit(addr, bit) (addr |= (1 << (bit)))

u32_t mark_pg(u32_t start, u32_t len)
{
	u32_t ret = len;
	u32_t page = (start & PAGE_MASK) / PAGE_SIZE;
	len = (len & PAGE_MASK) + ((len & ~PAGE_MASK) ? PAGE_SIZE : 0);
	len /= PAGE_SIZE;
	
	while (len--)
	{
		if (page > 160)
			return ret;

		setbit(mmap_640[page / 32], page % 32);
		TRACE(("marking page %d, at 0x%x\n", page, page * PAGE_SIZE));
		page++;
	}
	
	return ret;
}

// does not update mmap_640 so this can only be called once
void* get_free(u32_t c)
{
	count_t n = 0;
	count_t x;
	c /= PAGE_SIZE;
	ptr_t a_start = 0;
	u32_t i = c;
	
	while (n < 5) // loop through each byte of the page table
	{
		x = 0;
		while (x < 32)
		{
			if (!(mmap_640[n] & (1 << x)))
						// check allocated flag
			{
				if (!a_start)
					a_start = (ptr_t)
						  (4096 * (x + n * 32));
				if (--i == 0)
					return a_start;
			}
			else
			{
				i = c;
				a_start = 0;
			}
			++x;
		}
		++n;
	}
	return NULL;
}

inline u32_t do_copy(u32_t* src, ptr_t dest, count_t sz)
{
	if (*src)
		*src = (u32_t)memcpy(dest, (ptr_t)*src, sz);
	return sz;
}

void sys_init(multiboot_info_t* info)
{
	count_t i;
	module_t* mptr;
	u32_t sum = 0;
	
	TRACE(("sysinit()\n"));
	
	/*
	   First things first we need some memory in which to place our
	   system objects, avoiding the multiboot structure and bios memory.
	   We know that no kernel info is below 1M.
	   
	   For now we ignore the elf or aout symbol/section tables.
	*/
	memzero(mmap_640, 20);
	setbit(mmap_640[0], 0);		// mark first page in use (BIOS)
	setbit(mmap_640[4], 31);	// last page incase we have 639k
					//   instead of 640k
	
	// sum the amount of memory used by the multiboot info,
	// marking pages as we go
	sum += mark_pg((u32_t)info, sizeof(multiboot_info_t));
	sum += mark_pg(info->cmdline, strlen((string_t)info->cmdline));
	sum += mark_pg(info->mods_addr, info->mods_count * 16);
	
	// keep the modules table but ignore the modules
	for (i = 0; i < info->mods_count; i++)
	{
		mptr = (module_t*)(info->mods_addr + (16 * i));
		sum += mark_pg(mptr->string, strlen((string_t)mptr->string));
	}
	
	sum += mark_pg(info->mmap_addr, info->mmap_length);
	
	// find a free block large enuff to fit this memory,
	// and copy the lot to there, adjusting pointers as necessary.
	// we also reserve a page for some other stuff
	sum = (sum & PAGE_MASK) + ((sum & ~PAGE_MASK) ? PAGE_SIZE : 0);
	u8_t* p_start = get_free(sum + PAGE_SIZE);
	u8_t* p = p_start + PAGE_SIZE;
	
	p += do_copy((u32_t*)&(info), p, sizeof(multiboot_info_t));
	p += do_copy(&(info->cmdline), p, strlen((string_t)info->cmdline));
	p += do_copy(&(info->mods_addr), p, info->mods_count * 16);
	
	for (i = 0; i < info->mods_count; i++)
	{
		mptr = (module_t*)(info->mods_addr + (16 * i));
		p += do_copy(&(mptr->string), p,
			     strlen((string_t)mptr->string));
	}
	
	p += do_copy(&(info->mmap_addr), p, info->mmap_length);
	
	// store some stuff in the reserved page
	init_data_t* idata = (init_data_t*)p_start;
	idata->multiboot_end = (iptr_t)p;
	idata->multiboot_end = (idata->multiboot_end & PAGE_MASK)
			       + ((idata->multiboot_end & ~PAGE_MASK)
			          ? PAGE_SIZE : 0);
	
	// We now want to set up the IDT so we can handle errors and the
	// physical page manager so we can manage memory
	// IDT requires GDT and TSS.
	// So we need 5 pages for sysinfo, IDT, GDT, TSS and page_manager
#define NEEDED_SIZE (5 * PAGE_SIZE)
	
	// check below multiboot info
	if (((iptr_t)p_start) - PAGE_SIZE >= NEEDED_SIZE)
		sysinfo = (root_sysinfo_t*)PAGE_SIZE;
	
	// check above
	else if ((640 * 1024) - PAGE_SIZE - idata->multiboot_end
		 >= NEEDED_SIZE)
		sysinfo = (root_sysinfo_t*)
			  (idata->multiboot_end); // will be page aligned
	
	else
		panic("No memory for sysinfo!");
	
	idata->sysinfo_size = NEEDED_SIZE;
	memzero(sysinfo, NEEDED_SIZE);
	
#undef NEEDED_SIZE

	// now set the lot up
	iptr_t ptr = (iptr_t)sysinfo;
	sysinfo->gdt = (gdt_t*)(ptr + PAGE_SIZE);
	sysinfo->tss_seg = (tss_segment_t*)(ptr + 2 * PAGE_SIZE);
	sysinfo->interrupts = (interrupt_data_t*)(ptr + 3 * PAGE_SIZE);
	sysinfo->page_manager = (page_manager_t*)(ptr + 4 * PAGE_SIZE);
	
	sysinfo->multiboot = info;
}

/*
  TODO:	Protect the sysinfo struct with a lock which can allow reading but
		deny writing while it is being read and deny both reading and
		writing while it is being written to.
*/
