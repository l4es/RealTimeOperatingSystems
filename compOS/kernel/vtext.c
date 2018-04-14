/*
 vtext.c - Text mode vga stuff
  
 Author:        Paul Barker
 Part of:       COS
 Created:       28/10/04
 Last Modified: 05/11/04

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

#include <x86-asm.h>
#include <cos/vtext.h>
#include <ascii.h>
#include <cos/mem.h>

static vtext_cursor_t	position;
static vtext_cursor_t	extent;
static u8_t		attrib;

/* current cursor position */
#define CURRENT_POSN() (VGA_START + 2 * (position.x + (extent.x * position.y)))

#define TAB_SPACES 8

void vtext_set_hw_cursor()
{
    u16_t pos = (position.y * extent.x) + position.x;
    u8_t orig;

    // Save original contents of CRT address register.
    orig = inb_p(VGA_AREG);

    // Set the high cursor location byte
    outb_p(VGA_CURSOR_LOC_HIGH_REG, VGA_AREG);
    outb_p((pos>>8) & 0xff, VGA_DREG);

    // Set the low cursor location byte
    outb_p(VGA_CURSOR_LOC_LOW_REG, VGA_AREG);
    outb_p(pos & 0xff, VGA_DREG);

    // Restore contents of the CRT address register
    outb(orig, VGA_AREG);
}

void vtext_get_hw_cursor()
{
    u8_t orig;
    u16_t pos;

    // Save original contents of CRT address register.
    orig = inb_p(VGA_AREG);

    // Get the high cursor location byte
    outb_p(VGA_CURSOR_LOC_HIGH_REG, VGA_AREG);
    pos = (inb_p(VGA_DREG) << 8);

    // Get the low cursor location byte
    outb_p(VGA_CURSOR_LOC_LOW_REG, VGA_AREG);
    pos += inb_p(VGA_DREG);

    // Restore contents of the CRT address register
    outb(orig, VGA_AREG);
    
    position.x = pos % extent.x;
    position.y = pos / extent.x;
}

void vtext_set_cursor(vtext_cursor_t cur)
{
	position = cur;
	vtext_set_hw_cursor();
}

vtext_cursor_t vtext_get_cursor()
{
	return position;
}

vtext_cursor_t vtext_get_extent()
{
	return extent;
}

void vtext_set_attrib(u8_t at)
{
	attrib = at;
}

u8_t vtext_get_attrib()
{
	return attrib;
}

void vtext_init()
{
	position.x = position.y = 0;
	extent.x = VGA_NCOLS;
	extent.y = VGA_NROWS;
}



/* clear video memory */
count_t vtext_clear()
{
	char_t* p = VGA_START;
	count_t x, y;
	count_t i = 0;
	
	for (y = 0; y < extent.y; y++)
	{
		for (x = 0; x < extent.x; x++)
		{
			*p++ = ' ';		/* clear character */
			*p++ = attrib;		/* set current attribute */
			++i;			/* increment counter */
		}
	}
	position.x = position.y = 0;
	vtext_set_hw_cursor();
	return i;
}

/* scroll video memory */
void vtext_scroll()
{
	count_t x;
	char_t* p = (char_t*)(2 * extent.x * (extent.y - 1));

	memcpy(VGA_START, VGA_START + 2 * extent.x,
		(count_t)p);
	
	p += (iptr_t)VGA_START;

	for (x = 0; x < extent.x; x++)
	{
		*p++ = ' ';		/* clear character */
		*p++ = attrib;		/* set current attribute */
	}
}

/* moves the cursor to the next position */
void vtext_next()
{
	position.x++;
	
	if (position.x >= extent.x)
	{
		position.x = 0;
		position.y++;
		
		if (position.y >= extent.y)
		{
			vtext_scroll();
			position.y--;
		}
	}
	
	vtext_set_hw_cursor();
}

/* raw output - no ctrl character parsing */
void vtext_output(char_t ch)
{
	char_t* p = CURRENT_POSN();
	*p++ = ch;
	*p = attrib;
	vtext_next();
}

/*------------------------------------------------------------------------
 control characters
------------------------------------------------------------------------*/

void vtext_ht()
{
	count_t tomove = TAB_SPACES - (position.x % TAB_SPACES);
	char_t* p = CURRENT_POSN();
	
	if ((position.x + tomove) < extent.x)
	{
		while (tomove--)
		{
			*p++ = ' ';
			*p++ = attrib;
		}
		
		position.x += tomove;
	}
}

void vtext_bs()
{
	char_t* p = CURRENT_POSN() - 2;
	
	if (position.x)
	{
		*p++ = ' ';
		*p = attrib;
		--(position.x);
	}
}

/* vertical tab starts a new line at the current position */
void vtext_vt()
{
	char_t* p = CURRENT_POSN();
	count_t n;
	
	position.y++;
	
	/* handle scrolling */
	if (position.y >= extent.y)
	{
		vtext_scroll();
		position.y -= 2;
		p = CURRENT_POSN();
		position.y++;
	}
	
	/* fill a blank line upto the same place on the next line */
	for (n = 0; n < extent.y; n++)
	{
		*p++ = ' ';
		*p++ = attrib;
	}
}

void vtext_lf()
{
	char_t* p = CURRENT_POSN();
	
	/* pad to end of line */
	while (position.x < extent.x)
	{
		*p++ = ' ';
		*p++ = attrib;
		
		position.x++;
	}
	
	/* start a new line */
	position.x = 0;
	position.y++;
	
	if (position.y >= extent.y)
	{
		vtext_scroll();
		position.y--;
	}
}

void vtext_write(char_t ch)
{
	switch (ch)
	{
		case ASCII_NUL:		/* padding character */
		
		case ASCII_SOH:		/* all these aren't handled yet */
		case ASCII_STX:
		case ASCII_ETX:
		case ASCII_EOT:
		case ASCII_ENQ:
		case ASCII_ACK:
		case ASCII_BEL:
		case ASCII_CR:
		case ASCII_SO:
		case ASCII_SI:
		case ASCII_DLE:
		case ASCII_DC1:
		case ASCII_DC2:
		case ASCII_DC3:
		case ASCII_DC4:
		case ASCII_NAK:
		case ASCII_SYN:
		case ASCII_ETB:
		case ASCII_CAN:
		case ASCII_EM:
		case ASCII_FS:
		case ASCII_GS:
		case ASCII_RS:
		case ASCII_US:
		case ASCII_FF:
		case ASCII_SUB:
		
		case ASCII_ESC:		/* should be handled by caller */
		
			break;
			
		case ASCII_LF:		/* line feed (new line) */
			vtext_lf();
			break;
		case ASCII_BS:		/* backspace */
			vtext_bs();
			break;
		case ASCII_HT:		/* horizontal tab */
			vtext_ht();
			break;
		case ASCII_VT:		/* vertical tab */
			vtext_vt();
			break;
		
		default:		/* all other characters */
			vtext_output(ch);
	}
}

/* pos.x = 0xFF means read current position */
int_t vtext_read(vtext_cursor_t pos)
{
	if (pos.x == 0xFF)
		pos = position;
	
	if ((pos.x >= extent.x) || (pos.y >= extent.y))
		return -1;
	
	return *( (char_t*)(VGA_START + 2 * (pos.x + (extent.x * pos.y))) );
}

/*
 TODO:	Header file.
	When this is finished, write printk()
*/
