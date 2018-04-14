; boot.s - bootstrap the kernel image from a multiboot-compatible loader
;  
; Author:        Paul Barker
; Part of:       COS
; Created:       14/09/04
; Last Modified: 04/10/04
;
; Copyright (C) 2004 Paul Barker
;    
;    This program is free software; you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation; either version 2 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program; if not, write to the Free Software
;    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
;
;                     (See file "Copying")
;
;	Some of this code is taken from GeekOS, copyright below:
;
; Copyright (c) 2001, David H. Hovemeyer <daveho@cs.umd.edu>
; $Revision: 1.1.1.1 $
;
; This is free software.  You are permitted to use,
; redistribute, and modify it as specified in the file "COPYING-GEEKOS".
;
; A lot of this code is adapted from Kernel Toolkit 0.2
; and Linux version 2.2.x, so the following copyrights apply:
;
; Copyright (C) 1991, 1992 Linus Torvalds
; modified by Drew Eckhardt
; modified by Bruce Evans (bde)
; adapted for Kernel Toolkit by Luigi Sgro
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	This is the main entry point for our kernel, and changes here will
; almost certainly mean changes are needed elsewhere. See:
;	- include/asm/defs.h
;	- include/asm/defs.s
;
;	This is designed to be loaded from a multiboot compatible bootloader
; so the machine state will be as follows:
;	- CS = 32-bit r-x code segment, base 0, limit 0xFFFFFFFF
;	- DS, ES, FS, GS, SS = 32-bit rw- data segment, base 0, limit 0xFFFFFFFF
;	- A20 enabled
;	- Protected mode, paging and interrupts disabled
;	- magic value in eax
;	- address of multiboot info struct in ebx
;
;	Note that there is no valid stack and all other state is undefined.
;
;;;;;;;;;;;;;;;;;;;;

[BITS 32]

; ----------------------------------------------------------------------
; Imports / Exports
; ----------------------------------------------------------------------

EXTERN kmain
EXTERN g_stack

EXTERN bss
EXTERN end_bss

GLOBAL kstart

; ----------------------------------------------------------------------
; Definitions
; ----------------------------------------------------------------------

%include "defs.inc"

; ----------------------------------------------------------------------
; Code
; ----------------------------------------------------------------------

[SECTION .text]

; multiboot header, we assume elf
align 8
mboot_header:
	dd MBOOT_HEADER_MAGIC
	dd MBOOT_HEADER_FLAGS
	dd MBOOT_HEADER_CHKSUM

; start address
align 8
kstart:
	; assert correct magic number in eax
	cmp eax, MBOOT_LOADER_MAGIC
	jne NEAR .loop
	
	; zero bss, this MUST be done before we use the stack
	;	(stack is in bss)
	mov eax, bss
.lbss:
	cmp eax, end_bss
	je .done
	mov [eax], dword 0
	add eax, 4
	jmp .lbss
.done:

	; setup a stack
	mov esp, g_stack
	add esp, 16384
	
	; reset EFLAGS
	push	dword 0
	popf
	
	; push multiboot information structure onto stack
	push ebx
	
	; kill the FDD motor so the kernel is in a known state
	;	assuming FDD boot ;-)
	;	This is taken from Linux
	mov	dx, 0x3f2
	xor	al, al
	out	dx, al
	
	; Initialize master and slave PIC!
	; From GeekOS / Linux again
	mov	al, ICW1
	out	0x20, al		; ICW1 to master
	call	Delay
	out	0xA0, al		; ICW1 to slave
	call	Delay
	mov	al, ICW2_MASTER
	out	0x21, al		; ICW2 to master
	call	Delay
	mov	al, ICW2_SLAVE
	out	0xA1, al		; ICW2 to slave
	call	Delay
	mov	al, ICW3_MASTER
	out	0x21, al		; ICW3 to master
	call	Delay
	mov	al, ICW3_SLAVE
	out	0xA1, al		; ICW3 to slave
	call	Delay
	mov	al, ICW4
	out	0x21, al		; ICW4 to master
	call	Delay
	out	0xA1, al		; ICW4 to slave
	call	Delay
	mov	al, 0xff		; mask all ints in slave
	out	0xA1, al		; OCW1 to slave
	call	Delay
	mov	al, 0xfb		; mask all ints but 2 in master
	out	0x21, al		; OCW1 to master
	call	Delay
	
	; call main c function
	call kmain

.loop:	hlt
	jmp .loop

; Linux uses this code.
; The idea is that some systems issue port I/O instructions
; faster than the device hardware can deal with them.
Delay:
	jmp	.done
.done:	ret
