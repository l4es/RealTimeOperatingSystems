/*
 debug.h - debugging functions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       30/08/04
 Last Modified: 11/09/04

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
02/09/04:	Changed name from assert.h
*/

#ifndef _COS_DEBUG_H_
#define _COS_DEBUG_H_

// currently we have no need to address stuff above 4MB
#define POINTER_MAX ((void*)(4096*1024))

// always include debugging stuff for now
#define assert(x) if (!(x)) FailAssert(#x, __FILE__, __LINE__)
#define assert_ptr(p) assert ((p) && (((void*)(p)) <= POINTER_MAX))

// in util.c
void FailAssert(cstring_t cond, cstring_t file, int_t line);
void Trace(cstring_t fmt, ...);

void panic(cstring_t msg);

#define TRACE(x) Trace x
// expected to be called as TRACE((x));

#endif // !_COS_DEBUG_H_
