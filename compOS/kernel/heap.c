/*
 heap.c - allocation thru a heap
  
 Author:	Paul Barker
 Part of:	COS
 Created:	01/09/04
 Last Modified:	16/10/04

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

#include <cos/debug.h>
#include <cos/page.h>
#include <cos/mem.h>
#include <cos/heap.h>

#define SIZE_OF_NODE(n) ((n->end - (iptr_t)n) - sizeof(heap_node_t))

// minimum allocation size
#define QUANTUM		8
#define QUANTUM_MASK	(~(QUANTUM - 1))

// given ptr must already be alloc'd, of at least size init_size
// I suggest that the type PG_CLUSTER_HEAP is *always* used for heaps.
heap_t* heap_create(size_t init_size, ptr_t start)
{
	TRACE(("Creating Heap\n"));
	// handle no given pointer
	if (start == 0)
	{
		TRACE(("Was not given a pointer,"));
		// handle 0 initial size
		if (init_size == 0)
			init_size = PAGE_SIZE;
		
		init_size = (init_size & PAGE_MASK) +
			     ((init_size & ~PAGE_MASK) ? 4096 : 0);
		
		TRACE((" will allocate %d (0x%x) bytes\n",
			init_size, init_size));
		
		start = phys_alloc(init_size / PAGE_SIZE, PG_CLUSTER_HEAP,
				   PG_CLUSTER_FREE);
	}
	
	if (init_size == 0)
		return 0;	// invalid if start given
	
	TRACE(("Using pointer 0x%x\n", start));
	
	memzero(start, init_size);
	
	heap_t* h = (heap_t*)start;
	iptr_t p = (iptr_t)start;		// need to do addition below
	
	h->end = p + init_size;
	h->first_free = (heap_node_t*)(p + sizeof(heap_t));
	
	heap_node_t* node = h->first_free;
	node->flags = HEAP_FREE;
	node->end = h->end;
	
	TRACE(("Done\n"));
	
	return h;
}

// add free node
void heap_add_free(heap_t* heap, heap_node_t* p)
{
	TRACE(("Adding free node at 0x%x to heap at 0x%x\n", p, heap));

	p->flags |= HEAP_FREE;
	
	// first find end of free list
	heap_node_t* free = heap->first_free;
	
	TRACE(("First free node at 0x%x\n", free));
	
	if (free == NULL)
	{
		heap->first_free = p;
		return;
	}
	
	while (free->next)
	{
		free = free->next;
		TRACE(("Moving to next free node at 0x%x\n", free));
	}
	
	// now set the next free to the given node
	free->next = p;
	
	TRACE(("Done\n"));
}

// allocate from a heap
ptr_t heap_alloc(heap_t* heap, size_t sz)
{
	heap_node_t* node = heap->first_free;
	u32_t s = 0;
	count_t n = 0;
	
	sz &= QUANTUM_MASK;
	
	TRACE(("Allocating memory of size %d (0x%x) bytes\n", sz, sz));
	TRACE(("from heap at 0x%x, first free is at 0x%x\n", heap, node));
	
	while (node)
	{
		s = SIZE_OF_NODE(node);
		TRACE(("Loop (%d): Node of size %d (0x%x) bytes\n", n, s, s));
		TRACE(("\tstart=0x%x, end=0x%x\n",
			((iptr_t)node) + sizeof(heap_node_t), node->end));
		if ((s >= sz) && (node->flags & HEAP_FREE))
			goto finish;
		node = node->next;
		TRACE(("\tMoving to next node at 0x%x\n", node));
		++n;
	}
	
	TRACE(("Not enough free memory!\n"));
	
	// could not find a node big enough
	return NULL;
	
finish:
	TRACE(("Found a node large enough\n"));
	
	iptr_t i = (iptr_t)node;
	
	// cut out a new node if possible
	if (s >= (sz + sizeof(heap_node_t) + QUANTUM))
	{
		heap_node_t* new_node = (heap_node_t*)
					(i + sizeof(heap_node_t) + sz);
		TRACE(("Splitting a free node at 0x%x\n", new_node));
		
		new_node->end = node->end;
		node->end = (iptr_t)new_node;
		heap_add_free(heap, new_node);
	}
	
	node->flags &= ~HEAP_FREE;
	
	iptr_t ret = i + sizeof(heap_node_t);
	TRACE(("Returning pointer 0x%x\n", ret));
	return (ptr_t)ret;
}

// free previously allocated memory
void heap_free(heap_t* heap, ptr_t p)
{
	TRACE(("Freeing memory at 0x%x from heap at 0x%x\n", p, heap));
	// mark the node as free
	iptr_t i = (iptr_t)p;
	heap_node_t* node = (heap_node_t*)(i - sizeof(heap_node_t));
	
	node->flags |= HEAP_FREE;
	
	TRACE(("\tnode at 0x%x, end at 0x%x\n", node, node->end));
	
	count_t sz = SIZE_OF_NODE(node);
	TRACE(("\tmeaning size is %d (0x%x) bytes\n", sz, sz));
}

/*
  TODO:	Support best fit allocation.
	Use phys_alloc() to fulfil requests for more memory than is free in
		the requested heap.
	Support requesting alignment.
	Compaction of a heap (meaning remove unused pages).
	Validation function.
	In heap_free(), we must condense adjacent free nodes into a single
		node.
	Add mutexes.
	heap_destroy().
*/
