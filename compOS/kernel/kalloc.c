/*
 kalloc.c - kernel memory allocation
  
 Author:	Paul Barker
 Part of:	COS
 Created:	01/09/04
 Last Modified:	11/10/04

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

#ifdef USE_BGET
#include <bget.h>
#else
#include <cos/heap.h>
#endif

#include <cos/sysinfo.h>

#define KERNEL_HEAP_SIZE (512 * 1024)
	// 512k

ptr_t kalloc(size_t sz)
{
#ifdef USE_BGET
	return bget(sz);
#else
	return heap_alloc(sysinfo->kernel_heap, sz);
#endif
}

void kfree(ptr_t p)
{
#ifdef USE_BGET
	brel(p);
#else
	heap_free(sysinfo->kernel_heap, p);
#endif
}

void kalloc_init()
{
#ifdef USE_BGET
	void* p = phys_alloc(KERNEL_HEAP_SIZE / PAGE_SIZE, PG_CLUSTER_HEAP,
			PG_CLUSTER_FREE);
	bpool(p, KERNEL_HEAP_SIZE);
#else
	sysinfo->kernel_heap = heap_create(KERNEL_HEAP_SIZE, 0);
#endif
}

/*
  TODO:	Put locks around access to sysinfo.
	Update this when heap.c has been finished.
*/
