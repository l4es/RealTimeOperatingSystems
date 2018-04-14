/*
 string.h - string handling functions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       30/08/04
 Last Modified: 02/09/04

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
02/09/04:	merged with print.h
*/

#ifndef _COS_STRING_H_
#define _COS_STRING_H_

// varargs support, from compiler
#include <stdarg.h>

// stop us iterating forever!
#define DEFAULT_STRINGN 1024

// string handling
count_t nstrcpy(string_t dest, cstring_t src, count_t n);
count_t nstrlen(cstring_t str, count_t n);
int_t nstrcmp(cstring_t str1, cstring_t str2, count_t n);
count_t nstrset(string_t dest, char_t ch, count_t n);

// print strings
count_t vsnprintk(string_t dest, count_t n, cstring_t fmt, va_list args);
count_t snprintk(string_t dest, count_t n, cstring_t fmt, ...);
count_t sprintk(string_t dest, cstring_t fmt, ...);
count_t printk(cstring_t fmt, ...);

// other versions of the above
#define strcpy(a, b) nstrcpy(a, b, DEFAULT_STRINGN)
#define strlen(a) nstrlen(a, DEFAULT_STRINGN)
#define strcmp(a, b) nstrcmp(a, b, DEFAULT_STRINGN)
#define strset(a, b) nstrset(a, b, DEFAULT_STRINGN)
#define vsprintk(a, b, c) vsnprintk(a, DEFAULT_STRINGN, b, c)

// a VERY simple implementation of isprint()
//	WARNING: evaluates c multiple times!
#define isprint(c) ((((c) > 0x1F) && ((c) < 0x80)) ? (c) : (0))

#endif // !_COS_STRING_H_
