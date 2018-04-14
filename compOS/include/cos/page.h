/*
 page.h - low level page management
  
 Author:        Paul Barker
 Part of:       COS
 Created:       06/10/04
 Last Modified: 07/10/04

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

#ifndef _COS_PAGE_H_
#define _COS_PAGE_H_

struct page_manager;
typedef struct page_manager page_manager_t;

struct page_cluster;
typedef struct page_cluster page_cluster_t;

// 32 bytes
struct page_cluster
{
	u32_t			start;
	u32_t			end;		// *non-inclusive*
	u8_t			type;
	u8_t			reserved[3];	// alignment filler
	page_cluster_t*		next;		// next in group of clusters
						// eg next free
	u32_t			reserved_2[4];	// for future use
};

// 1 page size
struct page_manager
{
	page_manager_t*		next;		// next page manager
	u32_t			reserved[7];	// for future use, again
	page_cluster_t		clusters[127];
};

/*
  start = end = 0 in a page cluster defines a cluster that is not valid
  (free for over-writing with a valid cluster).
  
  next = 0 defines the last page manager, a new one can be allocated if
  needed.
  
  To gest the page manager from a cluster, do this
	ptr_manager = (ptr_cluster & PAGE_MASK);

  So, no parent pointer is needed in a page cluster (as long as all page
  clusters are contained in a page manager.
*/

#define PAGE_MASK	(0xFFFFF000)

// page cluster types
#define PG_CLUSTER_EMPTY	0x00		// error: should never be used
						// in a valid cluster
#define PG_CLUSTER_FREE		0x01		// above 1M
#define PG_CLUSTER_BIOS		0x02
#define PG_CLUSTER_ISA_HOLE	0x03
#define PG_CLUSTER_KERNEL	0x04
#define PG_CLUSTER_FREE_LOWER	0x05		// < 640k

#define PG_CLUSTER_IN_USE	0x10		// general, no specific use
#define PG_CLUSTER_HEAP		0x11		// managed by kalloc()
#define PG_CLUSTER_SYSTEM	0x12		// system stuff like GDT, IDT
#define PG_CLUSTER_MANAGER	0x13

#define PG_CLUSTER_RESERVED	0x20		// no specific use
#define PG_CLUSTER_DMA		0x21		// managed by DMA manager

#define PG_CLUSTER_NOT_PRESENT	0xFE		// not present in memory
						// (maybe above max memory)
#define PG_CLUSTER_UNKNOWN	0xFF		// unknown

// functions in page.c
page_manager_t* phys_alloc_manager(page_manager_t* last_man);
page_cluster_t* phys_add_cluster(iptr_t p_start, iptr_t p_end, u8_t p_type,
				 page_cluster_t* p_next);
void phys_remove_cluster(page_cluster_t* c);
void phys_mark_by_ptr(u32_t p, u32_t len, u8_t type_p);
void phys_init();
ptr_t phys_alloc(count_t n, u8_t use_type, u8_t from_type);
void phys_free(ptr_t p, u8_t to_type);

#endif // !_COS_PAGE_H_
