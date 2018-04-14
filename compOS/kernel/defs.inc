; Definitions taken from GeekOS code, adapted for COS by
;   Paul Barker (PaulB0100@aol.com).


; Original file Copyright (c) 2001, David H. Hovemeyer <daveho@cs.umd.edu>
; $Revision: 1.1.1.1 $

; This is free software.  You are permitted to use,
; redistribute, and modify it as specified in the file "COPYING-GEEKOS".

; A lot of this code is adapted from Kernel Toolkit 0.2
; and Linux version 2.2.x, so the following copyrights apply:

; Copyright (C) 1991, 1992 Linus Torvalds
; modified by Drew Eckhardt
; modified by Bruce Evans (bde)
; adapted for Kernel Toolkit by Luigi Sgro

%ifndef DEFS_ASM
%define DEFS_ASM

; 8259A PIC initialization codes.
; Source: Linux bootsect.S, and Intel 8259A datasheet

; The most important reason why we reprogram the PICs is to
; route the hardware interrupts through vectors *above*
; those reserved by Intel.  The BIOS (for historical reasons :-)
; routes them such that they conflict with internal processor-generated
; interrupts.

ICW1 equ 0x11		; ICW1 - ICW4 needed, cascade mode, interval=8,
			;   edge triggered. (I think interval is irrelevant
			;   for x86.)
ICW2_MASTER equ 0x20	; put IRQs 0-7 at 0x20 (above Intel reserved ints)
ICW2_SLAVE equ 0x28	; put IRQs 8-15 at 0x28
ICW3_MASTER equ 0x04	; IR2 connected to slave
ICW3_SLAVE equ 0x02	; slave has id 2
ICW4 equ 0x01		; 8086 mode, no auto-EOI, non-buffered mode,
			;   not special fully nested mode

; Kernel code and data segment selectors.
; Keep these up to date with defs.h.
KERNEL_CS equ 1<<3	; kernel code segment is GDT entry 1
KERNEL_DS equ 2<<3	; kernel data segment is GDT entry 2

; Multiboot stuff
MBOOT_HEADER_MAGIC      equ 0x1BADB002
MBOOT_HEADER_FLAGS      equ 0x00000003
MBOOT_HEADER_CHKSUM     equ -(MBOOT_HEADER_MAGIC + MBOOT_HEADER_FLAGS)

MBOOT_LOADER_MAGIC      equ 0x2BADB002

%endif
