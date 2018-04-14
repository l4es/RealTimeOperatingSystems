/*
 printk.c - kernel printing functions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       30/08/04
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
16/09/04:	Now uses psnprintf as backend.
*/

#include <cosbase.h>

#include <cos/string.h>

/*
// only used by printk()
#include <cos/file.h>
#include <cos/mem.h>
#include <cos/thread.h>
#include <cos/video.h>
#include <cos/console.h>
*/

// we use psnprintf as a backend
#include <psnprintf.h>

count_t vsnprintk(string_t dest, count_t n, cstring_t fmt, va_list args)
{
	return (count_t)pvsnprintf(dest, n, fmt, args);
}

// other version of vsnprintk
count_t snprintk(string_t dest, count_t n, cstring_t fmt, ...)
{
	va_list args;
	count_t ret;
	va_start(args, fmt);
	
	ret = vsnprintk(dest, n, fmt, args);
	
	va_end(args);
	return ret;
}

count_t sprintk(string_t dest, cstring_t fmt, ...)
{
	va_list args;
	count_t ret;
	va_start(args, fmt);
	
	ret = vsprintk(dest, fmt, args);
	
	va_end(args);
	return ret;
}

/*
// The most important one of the lot

// Now uses the console once it is available

static string_t printk_buffer = 0;
static Mutex printk_m;

count_t printk(cstring_t fmt, ...)
{
	// only need to allocate buffer on first call
	if (printk_buffer == 0)
	{
		printk_buffer = kalloc(4096);
		minit(&printk_m);
	}

	mlock(&printk_m);
	va_list args;
	va_start(args, fmt);
	
	count_t ret = vsnprintk(printk_buffer, 4096, fmt, args);
	if (gKernelState & CONSOLE_ENABLED)
		write(stdout, printk_buffer, strlen(printk_buffer));
	else
		VidWrite(printk_buffer);
	
	va_end(args);
	munlock(&printk_m);
	return ret;
}
*/
