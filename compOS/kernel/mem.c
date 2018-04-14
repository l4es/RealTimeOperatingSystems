/*
 mem.c - memory management routines
  
 Author:        Paul Barker
 Part of:       COS
 Created:       14/04/04
 Last Modified: 04/10/04

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
  Stripped down for the re-write of cos
*/

#include <cosbase.h>

#include <cos/mem.h>
#include <cos/debug.h>

//////////////////////////////////////////////////////////////////////////
// mem* functions
ptr_t memcpy(ptr_t dest, cptr_t src, count_t count)
{
	u8_t* pSrc;
	u8_t* pDest;
	
	// save us from bad pointers
	assert_ptr(dest);
	assert_ptr(src);
	
	pSrc = (u8_t*)src;
	pDest = (u8_t*)dest;

	while (count--)
		*pDest++ = *pSrc++;

	return dest;
}

ptr_t memset(ptr_t dest, u8_t c, count_t count)
{
	u8_t* p;
	
	// save us from bad pointers
	assert_ptr(dest);

	p = (u8_t*)dest;

	while (count--)
		*p++ = c;

	return dest;
}

ptr_t memzero(ptr_t dest, count_t count)
{
	return memset(dest, 0, count);
}

int_t memcmp(cptr_t p1, cptr_t p2, count_t count)
{
	u8_t* pa1;
	u8_t* pa2;
	// save us from bad pointers
	assert_ptr(p1);
	assert_ptr(p2);
	
	pa1 = (u8_t*)p1;
	pa2 = (u8_t*)p2;
	while (count--)
	{
		if (!(*pa1 == *pa2))
			return ((int_t)*pa2) - ((int_t)*pa1);
		pa1++;
		pa2++;
	}
	return 0;
}
