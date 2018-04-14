/*
 vtext.h - Text mode vga definitions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       29/10/04
 Last Modified: 29/10/04

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

#ifndef _COS_VTEXT_H_
#define _COS_VTEXT_H_

typedef struct vtext_cursor
{
	u8_t x, y;
}
vtext_cursor_t;

// port definitions
#define VGA_AREG		0x3D4
#define VGA_DREG		0x3D5
#define VGA_CURSOR_LOC_HIGH_REG	0x0E
#define VGA_CURSOR_LOC_LOW_REG	0x0F

#define VGA_START		((char_t*)0xB8000)
#define VGA_NCOLS		80
#define VGA_NROWS		25

#endif // !_COS_VTEXT_H_
