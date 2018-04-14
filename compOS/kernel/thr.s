; thr.s - low-level thread management
;  
; Author:        Paul Barker
; Part of:       COS
; Created:       31/08/04
; Last Modified: 05/11/04
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This has been stripped down a bit for the new multiboot cos,
;	plus some new stuff added.
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; First revision was incorrectly put into CVS as thread.s
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


[BITS 32]


; include this for KERNEL_CS
%include "defs.inc"

GLOBAL Get_EFLAGS
GLOBAL Get_ESP
GLOBAL test_and_set
GLOBAL test_and_set_r
GLOBAL Schedule

EXTERN get_current_thread
EXTERN do_sched

; ----------------------------------------------------------------------
; Definitions
; ----------------------------------------------------------------------



; ----------------------------------------------------------------------
; Code
; ----------------------------------------------------------------------

[SECTION .text]

; return my guess at the current esp
align 16
Get_ESP:
	mov eax, esp
	add eax, 4		; skip the return address
	ret

; Return current contents of eflags register.
;	Taken from GeekOS.
align 16
Get_EFLAGS:
	pushfd			; push eflags
	pop	eax		; pop contents into eax
	ret

; test and set a 32-bit value, this first bit is callable from C as
;	int test_and_set(int value, int* dest);
;
; this is really only for setting to 1 or 0 for mutexes
align 16
test_and_set:
	mov eax, [esp + 4]
	mov ebx, [esp + 8]

; the real work, value in eax, dest in ebx, returns result in eax
;	callable from asm
test_and_set_r:
	mov ecx, eax
	lock			; next memory access is locked for an SMP machine
	xchg eax, [edx]
	xor eax, ecx
	ret

align 16
Schedule:
	; This is an interrupt handler
	call get_current_thread	; returns current thread in eax
	mov [eax], esp		; store esp in current thread
	call do_sched		; returns thread to switch to in eax
	mov esp, [eax]		; load new esp
	ret