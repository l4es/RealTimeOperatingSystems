/*
 heap.h - memory allocation using heaps
  
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

#ifndef _COS_HEAP_H_
#define _COS_HEAP_H_

struct _heap_node;
typedef struct _heap_node heap_node_t;

struct _heap_node
{
	iptr_t		end;
	u32_t		flags;
	heap_node_t*	next;
};

typedef struct _heap
{
	iptr_t		end;
	heap_node_t*	first_free;
}
heap_t;

// heap functions
heap_t* heap_create(size_t init_size, ptr_t start);
void heap_add_free(heap_t* heap, heap_node_t* p);
ptr_t heap_alloc(heap_t* heap, size_t sz);
void heap_free(heap_t* heap, ptr_t p);

// flag that marks a node as free
#define HEAP_FREE 0x01

#endif // !_COS_HEAP_H_
