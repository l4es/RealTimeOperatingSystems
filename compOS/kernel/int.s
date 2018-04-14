; int.s - low-level interrupt routines
;  
; Author:        Paul Barker
; Part of:       COS
; Created:       15/04/04
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

; See "COPYING-GEEKOS" about the code here that is taken from GeekOS

[BITS 32]

; include this for KERNEL_CS
%include "defs.inc"

GLOBAL lgdtr
GLOBAL lidtr

; Sizes of interrupt handler entry points for interrupts with
; and without error codes.  The code in idt.c uses this
; information to infer the layout of the table of interrupt
; handler entry points, without needing a separate linker
; symbol for each one (which is quite tedious to type :-)
GLOBAL g_handlerSizeNoErr
GLOBAL g_handlerSizeErr

; Beginning and end of the table of interrupt entry points.
GLOBAL g_entryPointTableStart
GLOBAL g_entryPointTableEnd

; get interrupts tbl from sysinfo->interrupts
EXTERN sysinfo

; fake return address to setup a thread
GLOBAL FakeReturnAddr

; ----------------------------------------------------------------------
; Interrupt Definitions
;	- We are using the GeekOS way of handling interrupts,
;	most of this code is taken from GeekOS
; ----------------------------------------------------------------------

; This is the size of the Interrupt_State struct in int.h
INTERRUPT_STATE_SIZE equ 64

; Save registers prior to calling a handler function.
; This must be kept up to date with:
;   - Interrupt_State struct in int.h
%macro Save_Registers 0
	push	eax
	push	ebx
	push	ecx
	push	edx
	push	esi
	push	edi
	push	ebp
	push	ds
	push	es
	push	fs
	push	gs
%endmacro

; Restore registers and clean up the stack after calling a handler function
; (i.e., just before we return from the interrupt via an iret instruction).
%macro Restore_Registers 0
	pop	gs
	pop	fs
	pop	es
	pop	ds
	pop	ebp
	pop	edi
	pop	esi
	pop	edx
	pop	ecx
	pop	ebx
	pop	eax
	add	esp, 8	; skip int num and error code
%endmacro

; Number of bytes between the top of the stack and
; the interrupt number after the general-purpose and segment
; registers have been saved.
REG_SKIP equ (11*4)

; Template for entry point code for interrupts that have
; an explicit processor-generated error code.
; The argument is the interrupt number.
%macro Int_With_Err 1
align 8
	push	dword %1	; push interrupt number
	jmp	Handle_Interrupt ; jump to common handler
%endmacro

; Template for entry point code for interrupts that do not
; generate an explicit error code.  We push a dummy error
; code on the stack, so the stack layout is the same
; for all interrupts.
%macro Int_No_Err 1
align 8
	push	dword 0		; fake error code
	push	dword %1	; push interrupt number
	jmp	Handle_Interrupt ; jump to common handler
%endmacro

; ----------------------------------------------------------------------
; Code
; ----------------------------------------------------------------------

[SECTION .text]

; Load IDTR with 6-byte pointer whose address is passed as
; the parameter. Taken from GeekOS, like so much else...
align 8
lidtr:
	mov	eax, [esp+4]
	lidt [eax]
	ret

; Load GDTR with 6-byte pointer whose address is passed as
; the parameter. This really belongs somewhere else, but for
; now it is here.
align 8
lgdtr:
	mov eax, [esp+4]
	lgdt [eax]

	mov ax, KERNEL_DS
	mov ds, ax
	mov es, ax
	mov fs, ax
	mov gs, ax

	; now do a far jump to reload everything
	jmp KERNEL_CS:.here

.here:
	ret

; Common interrupt handling code.
; Save registers, call C handler function,
; possibly choose a new thread to run, restore
; registers, return from the interrupt.
align 8
Handle_Interrupt:
	; Save registers (general purpose and segment)
	Save_Registers

	; Ensure that we're using the kernel data segment
	mov	ax, KERNEL_DS
	mov	ds, ax
	mov	es, ax

	; Get the address of the C handler function from the
	; table of handler functions.
	mov	eax, [sysinfo]		; sysinfo
	mov	eax, [eax]		; sysinfo->interrupts
	add	eax, 2048
		; interrupts is first entry of sysinfo,
		; and handler table is 2k into interrupts

	mov	esi, [esp+REG_SKIP]	; get interrupt number
	mov	ebx, [eax+esi*4]	; get address of handler function

	; Call the handler.
	; The argument passed is a pointer to an Interrupt_State struct,
	; which describes the stack layout for all interrupts.
	push	esp
	call	ebx
FakeReturnAddr:
	add	esp, 4			; clear 1 argument

	; Restore registers
	Restore_Registers

	; Return from the interrupt.
	iret

; ----------------------------------------------------------------------
; Generate interrupt-specific entry points for all interrupts.
; We also define symbols to indicate the extend of the table
; of entry points, and the size of individual entry points.
; ----------------------------------------------------------------------
align 8
g_entryPointTableStart:

; Handlers for processor-generated exceptions, as defined by
; Intel 486 manual.
Int_No_Err 0
align 8
Before_No_Err:
Int_No_Err 1
align 8
After_No_Err:
Int_No_Err 2	; Described in my Intel manual as NMI
		; an int 2 instruction does not do a 'real' NMI
Int_No_Err 3
Int_No_Err 4
Int_No_Err 5
Int_No_Err 6
Int_No_Err 7
align 8
Before_Err:
Int_With_Err 8
align 8
After_Err:
Int_No_Err 9	; Described as Coprocessor segment overrun. Apparantly
		; recent IA-32 processors should NOT generate this exception
Int_With_Err 10
Int_With_Err 11
Int_With_Err 12
Int_With_Err 13
Int_With_Err 14
Int_No_Err 15	; RESERVED
Int_No_Err 16
Int_With_Err 17

; The remaining interrupts (18 - 255) do not have error codes.
; We can generate them all in one go with nasm's %rep construct.
%assign intNum 18
%rep (256 - 18)
Int_No_Err intNum
%assign intNum intNum+1
%endrep

align 8
g_entryPointTableEnd:

[SECTION .rodata]

; Exported symbols defining the size of handler entry points
; (both with and without error codes).
align 4
g_handlerSizeNoErr: dd (After_No_Err - Before_No_Err)
align 4
g_handlerSizeErr: dd (After_Err - Before_Err)
