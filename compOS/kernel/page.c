/*
 page.c - low level page management
  
 Author:	Paul Barker
 Part of:	COS
 Created:	01/09/04
 Last Modified:	06/11/04

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
#include <cos/debug.h>
#include <cos/symbols.h>
#include <cos/page.h>
#include <cos/mem.h>

// add a new manager
page_manager_t* phys_alloc_manager(page_manager_t* last_man)
{
	page_manager_t* man = sysinfo->page_manager;
	u32_t i = 0;
	page_cluster_t* c = &(man->clusters[i]);
	count_t n = 0;
	
	TRACE(("Allocating a new manager\n"));
	TRACE(("First manager=0x%x, Last manager=0x%x\n", man, last_man));

	// loop to find a cluster representing free memory
	while (c->type != PG_CLUSTER_FREE)
	{
		TRACE(("Loop (%d): cluster=0x%x, type=0x%x\n", n, c,
			c->type));
		TRACE(("\tstart=0x%x, end=0x%x\n", c->start, c->end));
		// handle a bad cluster
		if (c->start >= c->end)
		{
			TRACE(("\tBad cluster, Removing!\n"));
			phys_remove_cluster(c);
			TRACE(("\treturning current manager\n"));
			return man;		// return this manager as it
						// now has a free cluster
		}
		
		if (++i >= 127)
		{
			TRACE(("End of this manager!\n"));
			man = man->next;
			assert(man);
			TRACE(("Going to manager at 0x%x\n", man));
			i = 0;
		}
		c = &(man->clusters[i]);
		n++;
	}
	TRACE(("Found some free memory! cluster=0x%x", c));
	TRACE(("start=0x%x, end=0x%x\n", c->start, c->end));
	
	// one last check for a bad cluster
	if (c->start >= c->end)
	{
		TRACE(("\tBad cluster, Removing!\n"));
		phys_remove_cluster(c);
		TRACE(("\treturning current manager\n"));
		return man;
	}
	
	// nudge this cluster along a page
	TRACE(("Nudging this cluster along a page and"
		" setting up new manager\n"));
	man = (page_manager_t*)c->start;
	c->start += PAGE_SIZE;
	
	// setup new manager
	memzero(man, PAGE_SIZE);
	iptr_t m = (iptr_t)man;
	last_man->next = man;
	
	TRACE(("Adding cluster to describe the new manager\n"));
	phys_add_cluster(m, m + PAGE_SIZE, PG_CLUSTER_MANAGER, 0);
	
	TRACE(("Returning manager at 0x%x\n", man));
	return man;
}

page_cluster_t* phys_add_cluster(iptr_t p_start, iptr_t p_end, u8_t p_type,
				 page_cluster_t* p_next)
{
	page_manager_t* man = sysinfo->page_manager;
	u32_t i = 0;
	page_cluster_t* c = &(man->clusters[i]);
	count_t n = 0;
	
	TRACE(("Adding a new cluster from 0x%x to 0x%x\n", p_start, p_end));
	TRACE(("type=0x%x, next=0x%x\n", p_type, p_next));
	TRACE(("First manager=0x%x\n", man));
	
	assert(!(p_start & ~PAGE_MASK));
	assert(!(p_end & ~PAGE_MASK));	// must already be page aligned

	// loop to find a free cluster
	while (c->start || c->end)
	{
		TRACE(("Loop (%d): cluster=0x%x, type=0x%x\n", n, c,
			c->type));
		TRACE(("\tstart=0x%x, end=0x%x\n", c->start, c->end));
		if (++i >= 127)
		{
			TRACE(("End of this manager!\n"));
			if (!man->next)
			{
				TRACE(("Need to get a new manager\n"));
				man = phys_alloc_manager(man);
				assert(man);
			}
			else
				man = man->next;
			i = 0;
			TRACE(("Going to manager at 0x%x\n", man));
		}
		c = &(man->clusters[i]);
		n++;
	}
	
	TRACE(("Got a free cluster at 0x%x\n", c));
	TRACE(("start=0x%x, end=0x%x, type=0x%x, next=0x%x\n", c->start,
		c->end, c->type, c->next));
	
	c->start = p_start;
	c->end = p_end;
	c->type = p_type;
	c->next = p_next;
	
	return c;
}

void phys_remove_cluster(page_cluster_t* c)
{
	TRACE(("Removing cluster at 0x%x\n", c));
	memzero(c, sizeof(page_cluster_t));
}

// this is ONLY to be used in initialisation,
// it is far too slow for normal use
void phys_mark_by_ptr(u32_t p, u32_t len, u8_t type_p)
{
	TRACE(("Marking by pointer at 0x%x, length %d (0x%x)\n", p, len, len));
	TRACE(("Will set type to 0x%x\n", type_p));
	
	if (len == 0)
		return;

	u32_t p_end = ((p + len) & PAGE_MASK) +
		      (((p + len) & ~PAGE_MASK) ? 4096 : 0);
	page_manager_t* man;
	page_cluster_t* c;
	u32_t i;
	count_t n = 0;
	
	p &= PAGE_MASK;
	
	u32_t true_start = p;
	u32_t xx;
	
	TRACE(("Ready to search, true start is 0x%x, end 0x%x\n", p, p_end));

start_search:
	man = sysinfo->page_manager;
	i = 0;
	++n;
	xx = 0;
	
	TRACE(("Starting search %d, p=0x%x\n", n, p));
	
	while (p < p_end)
	{
		// check if end of this manager
		if (i >= 127)
		{
			TRACE(("End of this manager!\n"));
			man = man->next;
			if (!man)
				panic("Could not mark page!");
			i = 0;
			TRACE(("Going to manager at 0x%x\n", man));
		}
		
		c = &(man->clusters[i]);
		
		TRACE(("Loop (%d): cluster at 0x%x\n", xx, c));
		TRACE(("\tstart=0x%x, end=0x%x\n", c->start, c->end));
		TRACE(("\ttype=0x%x, next=0x%x\n", c->type, c->next));
		
		// first check the obvious, p starts below c
		if (p < c->start)
			TRACE(("\tp is below c\n"));
		else if (p > c->end)
			TRACE(("\tp is above c\n"));
		
		// next check if p is at start of c
		else if (p == c->start)
		{
			// now check if c contains p...
			if (p_end < c->end)
			{
				TRACE(("\tc contains p,"
					" start of p is start of c\n"));
				c->start = p_end;
				goto finish;
			}
			
			// or if p contains c (including p is exactly c)
			else
			{
				TRACE(("\tp contains c\n"));
				p = c->end;
				TRACE(("\tRemoving c\n"));
				phys_remove_cluster(c);
				goto start_search;
			}
		}
		
		// finally check if p starts within c
		else if (p < c->end)
		{
			// maybe p is completely within c
			if (p_end <= c->end)
			{
				TRACE(("\tc contains p\n"));
				TRACE(("\tAdding new cluster from end of p"
					" to end of c\n"));
				page_cluster_t* nxt_c = phys_add_cluster(
					p_end, c->end, c->type, c->next);
				c->end = p;
				c->next = nxt_c;
				goto finish;
			}
			
			// else p extends beyond c
			else
			{
				TRACE(("\tp extends beyond c\n"));
				u32_t tmp = p;
				p = c->end;
				c->end = tmp;
				goto start_search;
			}
		}
		
		// otherwise, ignore the bugger
		++i;
		++xx;
	}

finish:
	TRACE(("Finished, adding cluster for p\n"));

	phys_add_cluster(true_start, p_end, type_p, 0);
}

// init physical page manager
void phys_init()
{
	TRACE(("Initialising physical page manager...\n"));

	multiboot_info_t* info = sysinfo->multiboot;
	page_manager_t* pgman = sysinfo->page_manager;
	page_cluster_t* c = 0;
	u32_t n;	// for calculations
	
	TRACE(("sysinfo at 0x%x, first manager at 0x%x\n", info, pgman));
	
	memzero (pgman, 4096);
	
	// setup BIOS data area (page 0)
	c = &(pgman->clusters[0]);
	c->start = 0;
	c->end = PAGE_SIZE;
	c->type = PG_CLUSTER_BIOS;
	
	TRACE(("BIOS data area: start=0x%x, end=0x%x, type=0x%x\n",
		c->start, c->end, c->type));
	
	n = (info->mem_lower << 10) & PAGE_MASK;
	
	// setup remaining low memory
	++c;
	c->start = PAGE_SIZE;
	c->end = n;
	c->type = PG_CLUSTER_FREE_LOWER;
	
	TRACE(("Low memory: start=0x%x, end=0x%x, type=0x%x\n",
		c->start, c->end, c->type));
		
	// setup ISA hole
	++c;
	c->start = n;
	c->end = 0x100000;
	c->type = PG_CLUSTER_ISA_HOLE;
	
	TRACE(("ISA Hole: start=0x%x, end=0x%x, type=0x%x\n",
		c->start, c->end, c->type));
	
	n = ((iptr_t)&end_kernel & PAGE_MASK) + (((iptr_t)&end_kernel & ~PAGE_MASK)
					   ? 4096 : 0);
	
	// setup kernel memory
	++c;
	c->start = (iptr_t)&kernel & PAGE_MASK;
	c->end = n;
	c->type = PG_CLUSTER_KERNEL;
	
	TRACE(("Kernel: start=0x%x, end=0x%x, type=0x%x\n",
		c->start, c->end, c->type));
	
	// setup remaining high memory
	++c;
	c->start = n;
	
	n = ((info->mem_upper + 1024) << 10) & PAGE_MASK;
	
	c->end = n;
	c->type = PG_CLUSTER_FREE;
	
	TRACE(("High memory: start=0x%x, end=0x%x, type=0x%x\n",
		c->start, c->end, c->type));
	
	// setup non present memory
	++c;
	c->start = n;
	c->end = ~0;	// as unsigned gives MAXINT
	c->type = PG_CLUSTER_NOT_PRESENT;
	
	TRACE(("Non-present memory: start=0x%x, end=0x%x, type=0x%x\n",
		c->start, c->end, c->type));
	
/*
  Full 4GB address space is now managed, so we never have to worry
  about a page not being a member of a cluster.
  
  Space used for this method:
	Best case (now) = (6 clusters * 32 bytes + 32 bytes for manager)
			= 224 bytes(4k allocated)
	
	Worst case (1 pg / cluster) = 2^20 clusters in 8257 managers
				    = 32.2519... MB (32.2539... MB allocated)
				    = 0.787... % of a 4GB system
		(This worst case can only happen on a 4GB system)
		(To be honest, it will never happen)
*/
	
	// grab the idata structure
	init_data_t* idata = (init_data_t*)(((iptr_t)sysinfo->multiboot)
					    - PAGE_SIZE);
					    
	TRACE(("idata found at 0x%x\n", idata));
	
	// factor in what has already been alloc'd
	phys_mark_by_ptr((iptr_t)sysinfo->multiboot,
			 idata->multiboot_end - (iptr_t)sysinfo->multiboot,
			 PG_CLUSTER_SYSTEM);
	
	phys_mark_by_ptr((iptr_t)sysinfo, idata->sysinfo_size,
			 PG_CLUSTER_SYSTEM);
	
	TRACE(("Physical page manager ready!\n"));
}

ptr_t phys_alloc(count_t n, u8_t use_type, u8_t from_type)
{
	TRACE(("Allocating physical memory, %d pages long\n", n));
	TRACE(("From type 0x%x to type 0x%x\n", from_type, use_type));
	
	if (n == 0)
		return NULL;

	page_manager_t* man = sysinfo->page_manager;
	u32_t i = 0;
	page_cluster_t* c;
	u32_t sz = n * PAGE_SIZE;
	count_t xx = 0;
	
	TRACE(("First manager at 0x%x, sz=0x%x\n", man, sz));
	
	// find cluster of at least n pages, of type from_type
	while (1)
	{
		// check if end of this manager
		if (i >= 127)
		{
			TRACE(("End of this manager!\n"));
			man = man->next;
			if (!man)
				panic("Out of memory");
			i = 0;
			TRACE(("Going to manager at 0x%x\n", man));
		}
		
		c = &(man->clusters[i]);
		
		TRACE(("Loop (%d): cluster at 0x%x\n", xx, c));
		TRACE(("\tstart=0x%x, end=0x%x\n", c->start, c->end));
		TRACE(("\ttype=0x%x, next=0x%x\n", c->type, c->next));
		
		// check type
		if (c->type == from_type)
		{
			TRACE(("Correct type\n"));
			// check size
			if ((c->end - c->start) >= sz)
			{
				TRACE(("It's large enough!\n"));
				// add a cluster to describe remainder
				if ((c->end - c->start) > sz)
				{
					TRACE(("Adding a cluster to describe"
						" remainder\n"));
					phys_add_cluster(c->start + sz,
							 c->end, c->type,
							 c->next);
				}
				
				// modify this cluster to describe allocated
				//  region
				c->end = c->start + sz;
				c->type = use_type;
				c->next = 0;
				
				TRACE(("Returning found pages\n"));
				TRACE(("\tstart=0x%x, end=0x%x\n",
					c->start, c->end));
				TRACE(("\ttype=0x%x, next=0x%x\n",
					c->type, c->next));
				
				return (ptr_t)c->start;
			}
		}
		++xx;
		++i;
	}
}

void phys_free(ptr_t p, u8_t to_type)
{
	TRACE(("Freeing physical memory at 0x%x, to type 0x%x\n",
		p, to_type));
	page_manager_t* man = sysinfo->page_manager;
	u32_t i = 0;
	page_cluster_t* c;
	
	TRACE(("First manager at 0x%x\n", man));
	count_t xx = 0;
	
	// find cluster starting at p
	while (1)
	{
		// check if end of this manager
		if (i >= 127)
		{
			TRACE(("End of this manager!\n"));
			man = man->next;
			if (!man)
				panic("Could not find pointer to free");
			i = 0;
			TRACE(("Going to manager at 0x%x\n", man));
		}
		
		c = &(man->clusters[i]);
		
		TRACE(("Loop (%d): cluster at 0x%x\n", xx, c));
		TRACE(("\tstart=0x%x, end=0x%x\n", c->start, c->end));
		TRACE(("\ttype=0x%x, next=0x%x\n", c->type, c->next));
		
		if (c->start == (iptr_t)p)
		{
			TRACE(("Found correct pointer, changing type and"
				" returning\n"));
			c->type = to_type;
			return;
		}
		++xx;
		++i;
	}
}

/*
  TODO: Validation / Compaction functions.
	Manage a free list during alloc / free.
	Preserve next pointer in clusters when removing a cluster.
	Protect the lot with mutexes.
	Test, test and test again!
	Preserve modules passed by bootloader.
*/
