/*
 sout.c - simple polled (no interrupts) Serial OUTput, for debugging
  
 Author:	Paul Barker
 Part of:	COS
 Created:	10/09/04
 Last Modified:	10/09/04

 This is based on code from:
	http://my.execpc.com/~geezer/osd/serial/pollout.c
	(see http://my.execpc.com/~geezer/osd)

 This code is public domain.

 This is intended to be self contained except for the headers,
 (ie. no external function calls).
*/

#include <cosbase.h>

#include <x86-asm.h>

static word_t port_table[4] = {
	0x03F8, 0x02F8, 0x03E8, 0x02E8
	};

static word_t my_flags = 0;

// change this if io is too slow/fast
#define SLOW_COUNT 5

// slow down io so that old serial controllers can cope
void outb_slow(u8_t value, u16_t addr)
{
	int a;
	outb(value, addr);
	a = SLOW_COUNT;
	while (a--);
}

u8_t inb_slow(u16_t addr)
{
	int a;
	u8_t b = inb(addr);
	a = SLOW_COUNT;
	while (a--);
	
	return b;
}

bool_t sout(u8_t ch, u8_t port)	// port is 0-based index
{
	u16_t io_addr;
	count_t c;
	// translate base port
	if (port & 0xFC)
		return False;	// port out of range
	io_addr = port_table[port];
	
	// check if this port is initialised
	if (!(my_flags & (1 << port)))
	{
		// the port has not been initialised so do this now
		outb_slow(0x80, io_addr + 3);
		// 115200 baud
		outb_slow(1, io_addr);
		outb_slow(0, io_addr + 1);
		// 8 data bits + 1 stop bit (methinks)
		outb_slow(0x03, io_addr + 3);
		// all interrupts disabled
		outb_slow(0, io_addr + 1);
		// turn off FIFO, if any
		outb_slow(0, io_addr + 2);
		// loopback off, interrupts (Out2) off, Out1/RTS/DTR off
		outb_slow(0, io_addr + 4);
		
		my_flags |= 1 << port;
	}
	
	// wait for the transmitter to be ready, with sensible time out
	for (c = 0; c < 0x10000; c++)
	{
		if (inb_slow(io_addr + 5) & 0x40)
			goto ready;
	}
	
	return False;

ready:
	outb_slow(ch, io_addr);
	return True;
}

