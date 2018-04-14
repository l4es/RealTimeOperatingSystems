/*
 Symbols defined in kernel.ld
 */

#ifndef _COS_SYMBOLS_H_
#define _COS_SYMBOLS_H_

typedef char* linker_symbol;

extern linker_symbol kernel;
extern linker_symbol end_kernel;
extern linker_symbol cosinit;
extern linker_symbol end_cosinit;
extern linker_symbol code;
extern linker_symbol end_code;
extern linker_symbol data;
extern linker_symbol end_data;
extern linker_symbol bss;
extern linker_symbol end_bss;
extern linker_symbol rodata;
extern linker_symbol end_rodata;

// Misc. kernel definitions is taken from GeekOS
//   Paul Barker (PaulB0100@aol.com).

// Original file Copyright (c) 2001, David H. Hovemeyer <daveho@cs.umd.edu>
// $Revision: 1.1.1.1 $

// This is free software.  You are permitted to use,
// redistribute, and modify it as specified in the file "COPYING-GEEKOS".

// Kernel code and data segment selectors.
// Keep these up to date with defs.asm.
#define KERNEL_CS  (1<<3)
#define KERNEL_DS  (2<<3)

// Address where kernel is loaded (1MB)
#define KERNEL_START_ADDR (0x100000)

#define KERNEL_PRIVILEGE 0
#define USER_PRIVILEGE 3

// Software interrupt for syscalls
#define SYSCALL_INT 0x90

#endif // !_COS_SYMBOLS_H
