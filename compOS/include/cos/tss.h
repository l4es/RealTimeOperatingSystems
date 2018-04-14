/*
 tss.h - TSS definitions
  
 Created:       31/08/04
 Last Modified: 02/09/04

	This stuff is taken from GeekOS,
		see "COPYING-GEEKOS"

	Modified for COS by Paul Barker.
*/

#ifndef _COS_TSS_H_
#define _COS_TSS_H_

/*
 The TSS struct is taken directly from GeekOS, my only addition
	is the typedef.

 Source for GeekOS: "Protected Mode Software Architecture" by Tom Shanley,
	ISBN 020155447X.
*/

// NOTE: all reserved fields must be set to zero.

// I should change the types but there's no real point

typedef struct _tss {
    // Link to nested task.  For example, if an interrupt is handled
    // by a task gate, the link field will contain the selector for
    // the TSS of the interrupted task.
    unsigned short link;
    unsigned short reserved1;

    // Stacks for privilege levels.  esp0/ss0 specifies the kernel stack.
    unsigned long esp0;
    unsigned short ss0;
    unsigned short reserved2;
    unsigned long esp1;
    unsigned short ss1;
    unsigned short reserved3;
    unsigned long esp2;
    unsigned short ss2;
    unsigned short reserved4;

    // Page directory register.
    unsigned long cr3;

    // General processor registers.
    unsigned long eip;
    unsigned long eflags;
    unsigned long eax;
    unsigned long ecx;
    unsigned long edx;
    unsigned long ebx;
    unsigned long esp;
    unsigned long ebp;
    unsigned long esi;
    unsigned long edi;

    // Segment registers and padding.
    unsigned short es;
    unsigned short reserved5;
    unsigned short cs;
    unsigned short reserved6;
    unsigned short ss;
    unsigned short reserved7;
    unsigned short ds;
    unsigned short reserved8;
    unsigned short fs;
    unsigned short reserved9;
    unsigned short gs;
    unsigned short reserved10;

    // GDT selector for the LDT descriptor.
    unsigned short ldt;
    unsigned short reserved11;

    // The debug trap bit causes a debug exception upon a switch
    // to the task specified by this TSS.
    unsigned int debugTrap : 1;
    unsigned int reserved12 : 15;

    // Offset in the TSS specifying where the io map is located.
    unsigned short ioMapBase;
}
tss_t;

typedef struct tss_segment
{
	tss_t	main_tss;
}
tss_segment_t;

#endif // !_COS_TSS_H_
