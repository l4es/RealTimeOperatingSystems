;--------------------------------------------------------------------------
;   Copyright (C) 2006 by krasnop@bellsouth.net (Alexei Krasnopolski)
;
;   Licensed under the Apache License, Version 2.0 (the "License");
;   you may not use this file except in compliance with the License.
;   You may obtain a copy of the License at
;
;       http://www.apache.org/licenses/LICENSE-2.0
;
;   Unless required by applicable law or agreed to in writing, software
;   distributed under the License is distributed on an "AS IS" BASIS,
;   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
;   See the License for the specific language governing permissions and
;   limitations under the License.
;--------------------------------------------------------------------------

; ------------------ M A C R O S -------------------------------------
; macro for interrupt service routine
pc_irq macro p1, p2
pc_irq&p1:
               push    eax
               mov      al, 020h       ; non-specific EOI
IF ( p2 GE 0 ) AND ( p2 LT 8 )
               out      020h, al       ;
ELSE
               out      0A0h, al       ;
ENDIF
               pop     eax
               iret
ENDM
; ------------------ M A C R O  #1 -------------------------------------
; macro for mask of interrupt controller 8259
intc_mask_int MACRO p1
IF (p1 GE 020h) AND (p1 LT 028h)
port = 021h
ELSE
port = 0A1h
ENDIF
               mov     ah, (1 SHL ( p1 AND 07h ))
               in      al, port
               or      al, ah
               out     port, al
ENDM
; ------------------ M A C R O  #2 -------------------------------------
; macro for unmask of interrupt controller 8259
intc_unmask_int MACRO p1
IF (p1 GE 020h) AND (p1 LT 028h)
port = 021h
ELSE
port = 0A1h
ENDIF
               mov     ah, NOT (1 SHL ( p1 AND 07h ))
               in      al, port
               and     al, ah
               out     port, al
ENDM
; ------------------ M A C R O  #3 -------------------------------------
; my macro for acknoledge of interrupt controller
intc_ack MACRO p1
               mov      al, 020h       ; non-specific EOI
IF (p1 GE 020h) AND (p1 LT 028h)
               out      020h, al       ;
ELSE
               out      0A0h, al       ;
ENDIF
ENDM
; ------------------ E N D   O F   M A C R O S -------------------------------------

.CODE
; ------------------ M E M O R Y   M A P -------------------------------------
stackStart           EQU     09FFF0h
gdtStart             EQU     0800h
idtStart             EQU     0
A_7C00               EQU     07C00h   ; This is a value of code segment register (CS) when this module is running in real mode
Display_Buffer       equ     0B8000h

; ------------------ E L F   O F F S E T S -------------------------------------
; offsets for variables in ELF header :
entry_address        EQU     018h     ; 4
prog_hdr_off         EQU     01Ch     ; 4
sect_hdr_off         EQU     020h     ; 4
header_size          EQU     028h     ; 2
prog_hdr_entry_size  EQU     02Ah     ; 2
prog_hdr_entry_count EQU     02Ch     ; 2
sect_hdr_entry_size  EQU     02Eh     ; 2
sect_hdr_entry_count EQU     030h     ; 2
sect_name_entr_index EQU     032h     ; 2

; offsets for variables in a program segment headers
segment_type         EQU     00h      ; 4
offset_in_file       EQU     04h      ; 4
phys_address         EQU     0Ch      ; 4
segm_size_in_file    EQU     010h     ; 4

; offsets for variables in a section headers
section_name         EQU     00h      ; 4
section_type         EQU     04h      ; 4
section_flags        EQU     08h      ; 4
section_address      EQU     0Ch      ; 4
section_file_offset  EQU     010h     ; 4
section_size         EQU     014h     ; 4

              ALIGN  4
gdt:               ; memory image of global descriptor table register
                   dw   gdtEnd - gdtStartIm - 1
                   dd   gdtStart                          ;

              ALIGN  4
idt:               ; memory image of interrupt descriptor table register
                   dw   07FFh                            ; space for 256 entries
                   dd   idtStart

idtStartIm:
                   dd pc_exception_noerr         ;VECTOR_DIV0                    0
                   dd pc_exception_noerr         ;VECTOR_DEBUG                   1
                   dd pc_exception_noerr         ;VECTOR_NMI                     2
                   dd pc_exception_noerr         ;VECTOR_BREAKPOINT              3
                   dd pc_exception_noerr         ;VECTOR_OVERFLOW                4
                   dd pc_exception_noerr         ;VECTOR_BOUND                   5
                   dd pc_exception_noerr         ;VECTOR_OPCODE                  6
                   dd pc_exception_noerr         ;VECTOR_NO_DEVICE               7
                   dd pc_exception_err           ;VECTOR_DOUBLE_FAULT            8
                   dd pc_exception_noerr
                   dd pc_exception_err           ;VECTOR_INVALID_TSS            10
                   dd pc_exception_err           ;VECTOR_SEGV                   11
                   dd pc_exception_err           ;VECTOR_STACK_FAULT            12
                   dd pc_exception_err           ;VECTOR_PROTECTION             13
                   dd pc_exception_err           ;VECTOR_PAGE                   14
                   dd pc_exception_noerr
                   dd pc_exception_noerr         ;VECTOR_FPE                    16
                   dd pc_exception_err           ;VECTOR_ALIGNMENT              17
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd pc_exception_noerr
                   dd timer_vsr                   ;INTERRUPT_TIMER            IRQ0 (20)
                   dd pc_irq1                     ;INTERRUPT_KEYBOARD         IRQ1
                   dd pc_irq2                     ;INTERRUPT_SLAVE8259        IRQ2
                   dd pc_irq3                     ;INTERRUPT_COM2             IRQ3
                   dd pc_irq4                     ;INTERRUPT_COM1             IRQ4
                   dd pc_irq5                     ;INTERRUPT_LPT2             IRQ5
                   dd pc_irq6                     ;INTERRUPT_FDD              IRQ6
                   dd pc_irq7                     ;INTERRUPT_LPT1             IRQ7
                   dd pc_irq8                     ;INTERRUPT_WALLCLOCK        IRQ8
                   dd pc_irq9                     ;INTERRUPT_SLAVE8259REDIR   IRQ9
                   dd pc_irqA                     ;                           IRQ10
                   dd pc_irqB                     ;                           IRQ11
                   dd pc_irqC                     ;                           IRQ12
                   dd pc_irqD                     ;INTERRUPT_COPRO            IRQ13
                   dd pc_irqE                     ;INTERRUPT_HDD              IRQ14
                   dd pc_irqF                     ;                           IRQ15
                   dd pr_int                      ; PROGRAMM INTERRUPT              (30)
idtEnd:

PM_set_start:
;   relocation of GDT and program code in proper place for 32bit flat memory model.
;   set ES == gdtStart
                   cli
                   mov     eax, gdtStart
                   shr     eax, 4                        ;
                   mov     es, ax                        ;
                   mov     di, 0                         ; destination es:di
                   mov     si, gdtStartIm                ; source      ds:si
                   mov     cx, program_end - gdtStartIm  ; number of bytes
               rep movsb
; fill out the whole IDT (256 entries)
                   mov     eax, idtStart
                   shr     eax, 4                        ;
                   mov     es, ax                        ;
                   mov     di, 0
                   mov     cx, 256
.loop:
                   xor     ax, ax
                   stosw                                 ; offset 15-0
                   mov     ax, 8
                   stosw                                 ; selector
                   mov     ax, 08E00h
                   stosw                                 ; P=1, DPL=0, interrupt gate
                   xor     ax, ax
                   stosw                                 ; offset 31-16
                   loop    .loop
; set up interrupt gates of the IDT for AO-RTOS
                   mov     di, 0                         ; destination es:di
                   mov     si, idtStartIm                ; source      ds:si
cycle:
                   lodsd
                   add     eax, A_7C00                  ; value of labels in compile time have to be increased in run time: in real mode cs = 7c0, in pm cs = 0
                   stosw                                ; offset 15-0
                   mov     ax, 8
                   stosw                                ; selector
                   mov     ax, 08E00h
                   stosw                                ; P=1, DPL=0, interrupt gate
                   shr     eax, 16
                   stosw                                ; offset 31-16
                   cmp     si, idtEnd
                   jl      cycle
;   # load GDTR and IDTR
                   cli
                   lgdt    fword ptr [gdt]
                   lidt    fword ptr [idt]
;   # Switch to protected mode.
                   mov     eax, cr0                       ;
                   or      al,  1                         ;
                   mov     cr0, eax                       ;
               jmp     dword ptr 08h:[(next + A_7C00)]
;    # We arrive here in protected mode
next:
;               BITS 32
;    # Load data selectors
               mov     ax, 010h                     ;
               mov     ds, ax                       ;
               mov     es, ax                       ;
               mov     fs, ax                       ;
               mov     gs, ax                       ;
               mov     ss, ax
;    # Set up SP
               mov     esp, stackStart              ;
;    # Reset the flags register.
               push    0                            ;
               popf                                 ;

;    #  The interrupt controller is configured so that IRQ levels 0-7 trigger
;    #  interrupt vector 32-39; levels 8-15 trigger 40-47.
               mov     al, 011h       ; 1 -  ICW4 is needed
                                      ; 0 -  cascading I8259
                                      ; 0 -  8 byte interrupt vector
                                      ; 0 -  edge triggered mode
                                      ; 1 -  must be 1 for ICW1
                                      ; 0 -
                                      ; 0 -
                                      ; 0 -
               out     020h, al       ; ICW 1
               mov     al, 020h       ; A7-A3 =  20(32.)
               out     021h, al       ; ICW 2
               mov     al, 04h       ; irq 2 (34.) has slave
               out     021h, al       ; ICW 3
               mov     al, 01h       ; 1 - 80x86 mode
                                      ; 0 - normal EOI
                                      ; 00 - not buffered
                                      ; 0 - sequential
               out     021h, al       ; ICW 4
 ;   #  Mask off all interrupts except 2.
               mov     al, 0FBh       ;
               out     021h, al       ; OCW 1

               mov     al, 011h       ;
               out     0A0h, al       ; ICW 1
               mov     al, 028h       ;
               out     0A1h, al       ; ICW 2
               mov     al, 02h       ;
               out     0A1h, al       ; ICW 3
               mov     al, 01h       ;
               out     0A1h, al       ; ICW 4
;    #   Mask off all interrupts.
               mov     al, 0FFh       ;
               out     0A1h, al       ; OCW 1
;    #   set clock
PC_PIT_CONTROL    EQU   (043h)
PC_PIT_CLOCK_0    EQU   (040h)
PC_PIT_CLOCK_1    EQU   (041h)
PC_PIT_CLOCK_2    EQU   (042h)
               mov     al, 034h
               out     PC_PIT_CONTROL, al
               mov     ax, 11932
               out     PC_PIT_CLOCK_0, al
               shr     ax, 8
               out     PC_PIT_CLOCK_0, al

;    #   unmask int 32
               intc_unmask_int 32
               sti

; go to application
; RELOCATION file.elf from temp buffer to original (physical) address space
; Keep in mind that temp buffer and physical addresses are both absolute addresses in flat memory (32bit PM)
               mov     ebx, ds:[temp_buffer + prog_hdr_off]            ; program segment header pointer (offset value)
               add     ebx, temp_buffer                             ; absolute value
               movzx   ecx, ds:[temp_buffer + prog_hdr_entry_count] ; number of entries in program segment header
lookup_in_segm:
               cmp     dword ptr [ebx], 1                           ; is type of program segment "load" ?
               jne     next_1                                    ; no - go to .next
; relocating of program segment from temp buffer to executable address space
               push    ecx
               cld
               mov     esi, [ebx + offset_in_file]       ; offset of code (program) segment in file
               add     esi, temp_buffer                  ; absolute value
               mov     edi, [ebx + phys_address]         ; physical address of the code segment in memory
               mov     ecx, [ebx + segm_size_in_file]    ; size of code segment
          rep  movsb
               pop     ecx
next_1:
               movzx   eax, word ptr ds:[temp_buffer + prog_hdr_entry_size]  ; size of current entry in program header
               add     ebx, eax                                       ; point to the next entry
               loop    lookup_in_segm                                ; go to process next entry

sections:
; point ebx to a section header
               mov     ebx, ds:[temp_buffer + sect_hdr_off]
               add     ebx, temp_buffer
; point edx to the header entry of the name section
               mov     cx,  word ptr ds:[temp_buffer + sect_name_entr_index]
               movzx   eax, word ptr ds:[temp_buffer + sect_hdr_entry_size]
               imul    ax,  cx
               add     eax, ebx                         ; points to a name section header
; point edx to the name section body
               mov     edx, [eax + section_file_offset]
               add     edx, temp_buffer                 ; points to a body of name section

               movzx   ecx, word ptr ds:[temp_buffer + sect_hdr_entry_count] ; number of entries in section header
lookup_in_sect:
               cmp     dword ptr [ebx + section_type], 1                  ; is a type of the section "PROGBITS" ?
               jne     next_2
; find a section with name = ".init"
               mov     esi, [ebx + section_name]											; index in name section body
               add     esi, edx
               mov     edi, initSect + A_7C00
               push    ecx

               mov     ecx, 5
         repe  cmpsb
               pop     ecx
               je      find
next_2:
               movzx   eax, word ptr ds:[temp_buffer + sect_hdr_entry_size]  ; size of current entry in section header
               add     ebx, eax                                       ; point to the next entry
               loop    lookup_in_sect                                ; go to process next entry
               jmp     $																							; error: can not find .init section
find:
; prepares to call .init (embedds a 'ret' instruction after end of the section)
               mov     edi, [ebx + section_address]
               add     edi, [ebx + section_size]             ; points to the section's end
               mov     eax, [edi]                            ; save a first dword of next section
               mov     ds:[save + A_7C00], eax
               mov     eax, ds:[retInst + A_7C00]
               mov     [edi], eax

               push    edi
               mov     esi, [ebx + section_address]
               call    esi									 ; call .init section ELF (on host it does underlying OS)									; call .init module
               pop     edi

               mov     eax,   ds:[save + A_7C00]                 ; restore a first dword of next section
               mov     [edi], eax

;               mov     esi, [temp_buffer + entry_address]
;               mov     cx, 8
;               call    displayHex

               mov     esi, ds:[temp_buffer + entry_address]     ; entry (start point of the application) address in elf header
               jmp     esi                                    ; GO TO AO-RTOS application !!!

initSect:      db      ".init"
save:          dd      0
retInst:       ret
;-------------------------------------------------------------------------------
;               TIMES   0800h - ($ - $$)   db 0
gdtStartIm:
;    /* Selector 0x00 == invalid. */
                   dw   00000h
                   dw   00000h
                   db   000h
                   db   000h
                   db   000h
                   db   000h
;    /* Selector 0x08 == code. */
                   dw   0FFFFh                      ; limit 15-0
                   dw   00000h                      ; base  15-0
                   db   000h                        ; base  23-16
                   db   09Bh                        ; AR: P=1, DPL=0, S=1, type=101(code, ER), A=1
                   db   0CFh                        ; G=1(page granularity), D=1(32bit mode), limit 19-16
                   db   000h                        ; base 31-24
                                                    ; base=0x00000000; limit=0xFFFFF
;    /* Selector 0x10 == data. */
                   dw   0FFFFh
                   dw   00000h
                   db   000h
                   db   093h                        ; AR: P=1, DPL=0, S=1, type=001(data, WR), A=1
                   db   0CFh
                   db   000h
                                                    ; base=0x00000000; limit=0xFFFFF
;    /* Selector 0x18 == shorter code: faults any code access 0xF0000000-0xFFFFFFFF. */
                   dw   0FFFFh
                   dw   00000h
                   db   000h
                   db   09Bh                        ; AR: P=1, DPL=0, S=1, type=101(code, ER), A=1
                   db   0C7h
                   db   000h
                                                     ; base=0x00000000; limit=0x7FFFF
;    /* Selector 0x20 == data; faults any access 0xF0000000-0xFFFFFFFF. */
                   dw   0FFFFh
                   dw   00000h
                   db   000h
                   db   093h                        ; AR: P=1, DPL=0, S=1, type=001(data, WR), A=1
                   db   0C7h
                   db   000h
gdtEnd:                                             ; base=0x00000000; limit=0x7FFFF

symbol:        db      '1'
second:        db      100

timer_vsr:
               push    eax
               push    ebx

               intc_ack 32
               sti

               dec     byte ptr [second]
               jne     .exit
               mov     byte ptr [second], 100
               mov     ah, 007h
               mov     al, [symbol]
               xor     al, 001h
               mov     [symbol], al
               mov     ebx, Display_Buffer + 79*2         ;
               mov     [ebx], ax
.exit:
               pop     ebx
               pop     eax
               iret

;timer_os:
;               push   eax
;               intc_ack 0x20
;               pop     eax
;               iret

trap_error_code:
               dd      0
; exception handler (no error code)
pc_exception_noerr:
               mov     dword ptr [trap_error_code], 0
               iret

; exception handler (with error code)
pc_exception_err:
               pop     dword ptr [trap_error_code]
               iret

; irq handler (no error code)
pc_irq 1,1
pc_irq 2,2
pc_irq 3,3
pc_irq 4,4
pc_irq 5,5
pc_irq 6,6
pc_irq 7,7
pc_irq 8,8
pc_irq 9,9
pc_irq {A},10
pc_irq {B},11
pc_irq {C},12
pc_irq {D},13
pc_irq {E},14
pc_irq {F},15
pr_int:        iret
