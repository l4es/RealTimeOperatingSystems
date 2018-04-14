/*
 string.c - string handling functions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       30/08/04
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

#include <cosbase.h>

#include <cos/string.h>

count_t nstrcpy(string_t dest, cstring_t src, count_t n)
{
	count_t sz = 0;
	string_t s = (string_t)src;
	while ((sz < n) && *s)
	{
		*dest++ = *s++;
		++sz;
	}
	return sz;
}

count_t nstrlen(cstring_t str, count_t n)
{
	count_t sz = 0;
	string_t s = (string_t)str;
	while ((sz < n) && *s++)
		++sz;

	return sz;
}

// +ve if str1 > str2, -ve if str1 < str2, 0 if eq
int_t nstrcmp(cstring_t str1, cstring_t str2, count_t n)
{
	string_t s1 = (string_t)str1;
	string_t s2 = (string_t)str2;
	
	while ((*s1 == *s2) && *s1 && n--)
	{
		++s1;
		++s2;
	}
	
	return (*s1 - *s2);
}

count_t nstrset(string_t s, char_t ch, count_t n)
{
	count_t sz = 0;
	while ((sz < n) && *s)
	{
		*s++ = ch;
		sz++;
	}
	return sz;
}
