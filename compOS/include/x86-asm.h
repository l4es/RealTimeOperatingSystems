/*
 x86-asm.h - assembly language operations for kernel
  
 Author:        Paul Barker
 Part of:       COS
 Created:       14/04/04
 Last Modified: 02/09/04

 Shamelessly stolen from linux v0.01, with comments added.

This should be useful outside cos, it also *should* be compatible with the
original linux <asm.h> it is based on.
*/

/*
02/09/04:	Added STOP()
*/

#ifndef _X86_ASM_H_
#define _X86_ASM_H_

// enable and disable interrupts
#define sti() __asm__ ("sti"::)
#define cli() __asm__ ("cli"::)

// no operation
#define nop() __asm__ ("nop"::)

// return from interrupt
#define iret() __asm__ ("iret"::)

// input and output
#define outb(value,port) \
__asm__ ("outb %%al,%%dx"::"a" (value),"d" (port))

#define inb(port) ({ \
unsigned char _v; \
__asm__ volatile ("inb %%dx,%%al":"=a" (_v):"d" (port)); \
_v; \
})

// input and output with delay
#define outb_p(value,port) \
__asm__ ("outb %%al,%%dx\n" \
		"\tjmp 1f\n" \
		"1:\tjmp 1f\n" \
		"1:"::"a" (value),"d" (port))

#define inb_p(port) ({ \
unsigned char _v; \
__asm__ volatile ("inb %%dx,%%al\n" \
	"\tjmp 1f\n" \
	"1:\tjmp 1f\n" \
	"1:":"=a" (_v):"d" (port)); \
_v; \
})

// load tss
#define ltr(xx) __asm__ volatile ("ltr %0" :: "a" (xx))

// lgdtr will be defined in real assembly because it requires a far jump.
// lidtr is also going to be in real assembly because i cant understand
//    the inline stuff or gas syntax!
void lgdtr(word_t* limitAndBase);
void lidtr(word_t* limitAndBase);
// if this is used outside cos you will have to implement it yourself ;-)

// issue a hlt instruction to the processor
#define halt() __asm__ ("hlt"::)

#endif // !_X86_ASM_H_
