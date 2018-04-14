/*
 mem.h - memory manipulation functions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       14/04/04
 Last Modified: 16/09/04

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
02/09/04:	moved InitPG to init.h
		merged with kalloc.h
*/

#ifndef _COS_MEM_H_
#define _COS_MEM_H_

// a couple of definitions to stop compiler warnings
// about built-in functions
#define memcpy MemCpy
#define memset MemSet
#define memcmp MemCmp

// from mem.c
ptr_t memzero(ptr_t dest, count_t count);
ptr_t memcpy(ptr_t dest, cptr_t src, count_t count);
ptr_t memset(ptr_t dest, u8_t c, count_t count);
int_t memcmp(cptr_t p1, cptr_t p2, count_t count);

// kalloc and kfree
ptr_t kalloc(size_t sz);
void kfree(ptr_t p);
void kalloc_init();

// other definitions
#define PAGE_SIZE          (0x1000)

#endif // !_COS_MEM_H_
