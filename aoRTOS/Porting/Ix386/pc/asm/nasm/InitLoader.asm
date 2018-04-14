;--------------------------------------------------------------------------
;   Copyright (C) 2010 by krasnop@bellsouth.net (Alexei Krasnopolski)
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
; ------------------ M A C R O  #2 -------------------------------------
; macro for unmask of interrupt controller 8259
%macro intc_unmask_int 1
%if (%1 >= 0x20) && (%1 < 0x28)
%assign port 0x21
%else
%assign port 0xA1
%endif
               mov     ah, ~(1 << ( %1 & 0x07 ))
               in      al, port
               and     al, ah
               out     port, al
%endmacro

segment         code
                BITS 16
begin:          jmp     main
msg1            db      "- BIN FILE LOADER. (c) 2009 AK -",0Ah,0Dh,0
msg2            db      "Boot Failure",0Ah,0Dh,0
msg3            db      "- END LOAD -",0Ah,0Dh,0
secPerTrack     EQU      18
gdtStart        EQU     0x800
relocSegm       EQU     0x00009D00
relocSP         EQU     0x2FFE
stackStart      EQU     0x9FFF0

; for loading an BIN file
size_file_in_sect    EQU  100      ; 100 for simpleness, but
                                   ; actually we have to calculate the size of
                                   ; the application file
temp_buffer          EQU  0x1000  ; load application file in start address

drive           db      0

main:
; Disable interrupt handling.
                cli
                cld
                call    assign_ip
assign_ip:
                pop     si
                sub     si, assign_ip    ; difference between run-time IP and compile-time IP
                shr     si, 4            ; in paragraphs
                xor     eax, eax
                mov     ax, cs
                add     ax, si           ; assign to segment DS compile-time value
                mov     ds, ax
; Relocate init loader to relocSegm:begin address
                mov     ax, relocSegm
                mov     es, ax
                mov     di, begin
                mov     si, di
                mov     cx, 0x200		; in this point we have loaded only 1 sector from floppy
            rep movsb
; Set Stack Segment
                mov     ss, ax
                mov     sp, relocSP
; Set Data Segment
                mov     ds, ax
; go to relocated code
                push    ax          ; will be cs
                push    newLocation ; will be ip
                retf
newLocation:
                mov     bp, msg1
                call    putString
; reading next n=1 sectors from floppy. The sectors contain the rest of code of the Init loader.
                mov     bx, 0x200
                mov     ax, 10                   ; number sectors for reading = 5 ( for sure )
                mov     cx, 0                    ; track = 0
                mov     di, 2                    ; So = 2
                mov     [drive], dl              ; drive
                mov     dh, 0                    ; head

                call    readSectors

                or      ah, ah
                jnz     short reboot                       ; error !!!
; Loading of .bin file with AO-RTOS application
                mov     ax, size_file_in_sect   ; number sectors for reading = 150 (while)
                mov     cx, 1                   ; track = 1
                mov     di, 1                   ; So = 1
                mov     dh, 0                   ; head
                mov     dl, [drive]
                mov     bx, (temp_buffer >> 4)  ; start address of temp buffer for my application = 0x70000 (while)
                mov     es, bx                  ; bx = segment file.bin address
                xor     bx, bx                  ; bx = 0 offset file.bin address

                call    readArrSect

                or      ah, ah
                jz      OK
reboot:                                         ; error !!!
                mov     bp, msg2
                call    putString
                int     19h                     ; DISK BOOT causes reboot of disk system
OK:
                mov     bp, msg3
                call    putString

; stop floppy motor
	              mov	    dx, 0x3f2
	              xor	    al, al
	              out	    dx, al
                jmp     switchToPM
;-------------------------------------------------------------------------------
;  Read nSect sectors from disk into memory ( nSect < sectPerTrack )
;  input:
;           AX,      CX,       DI,      DH,   DL,       BX,           ES
;         [nSect], [track], [sector], [head,drive], [buffer.OFF], [buffer.SEG]
;  output: AL - real read sectors, AH - error code
;--------------------------------------------------------------------------------
readSectors:
                push    cx
                push    si
                push    di
; set counter of trying
                mov     si, 5
; prepare track
                xchg    ch, cl
                and     cl, 0C0h
; prepare sector
                push    ax
                mov     ax, di
                and     al, 3Fh         ; for sure
                or      cl, al
; prepare head, drive - already done
; prepare pointer to memory - already done
; test nSect
                mov     ax, secPerTrack
                sub     ax, di
                inc     ax              ; possible number of sectors to read: sPT - sector + 1
                mov     di, ax
                pop     ax
                cmp     di, ax
                jge     again           ; if di >= nSect
                mov     ax, di
again:
; prepare nSectors
                push    ax
                mov     ah, 2

                int     13h             ; DISK - READ SECTORS INTO MEMORY
                                        ; AL = number of sectors to read, CH = track, CL = sector
                pop di                  ; DH = head, DL = drive, ES:BX -> buffer to fill
                                        ; Return: CF set on error, AH = status (see AH=01h), AL = number of sectors read
                jnb     exit
                cmp     ah, 9
                jnz     errorReset
                dec     di               ; DMA access across 64K boundary decrease number of sectors and try again
                jz      errorDMA
                mov     ax, di
                jmp     short again
errorReset:
                dec     si
                jz      exit
                mov     ah, 0

                int     13h             ; DISK - RESET DISK SYSTEM

                mov     ax, di
                jmp     short again
errorDMA:
                mov     ah, 09h
exit:
                pop     di
                pop     si
                pop     cx
                retn
; ******************** read array of sectors ( > secPerTrack )  ------------------
;  input: BP -pointer to array of parameters:
;           AX,      CX,       DI,      DH,   DL,       BX,           ES
;         [nSect], [track], [sector], [head,drive], [buffer.OFF], [buffer.SEG]
; *********************************************************************************
readArrSect:
                push    bp
                push    si

                mov     bp, secPerTrack
r1:
                push    ax
                call    readSectors
                pop     si

                or      ah, ah
                jz      continue
                cmp     ah, 09h
                jnz     r_exit          ; error !!!
                mov     ax, bx          ; DMA error!!!
                shr     ax, 4
                jmp     r3
continue:
; change and test number of sectors to read
                sub     si, ax          ; minus number of readed sectors
                jz      ok_exit          ; OK exit !
; change and test sector number
                add     di, ax          ; new start sector So
                cmp     di, bp          ; di <= sPTr
                jle      r2
; change and test head, track
                mov     di, 1           ; So = 1
                or      dh, dh
                jz      h1
                inc     cx              ; track++
h1:
                xor     dh, 1           ; head ^= 1
r2:
; test buffer address
                shl     ax, 9           ; ax = nSect * 200
                add     ax, bx          ; ax = nSect * 200 + buffer.OFF
                jnc     b1
                rcr     ax, 1
                shr     ax, 3           ; align for segment
r3:
                mov     bx, es
                add     ax, bx          ; add to buffer.SEG
                mov     es, ax
                xor     ax, ax          ; ax = 0
b1:
                mov     bx, ax
                mov     ax, si
                jmp     r1
ok_exit:
                mov     ax, si
r_exit:
                pop     si
                pop     bp
                retn
;-------------------------------------------------------------------------------
;
;  print char
;  input: al - character:
;
;-------------------------------------------------------------------------------
putChar:
                mov     ah, 0Eh
                push    bx
                push    dx
                xor     bx, bx
                mov     bl, 0FFh
                int     10h             ; - VIDEO - WRITE CHARACTER AND ADVANCE CURSOR (TTY WRITE)
                                        ; AL = character, BH = display page (alpha modes)
                pop     dx              ; BL = foreground color (graphics modes)
                pop     bx
                retn
;-------------------------------------------------------------------------------
;
;  print string
;  input: BP -pointer to array of symbols:
;
;-------------------------------------------------------------------------------
putString:
                mov     al, [bp]
                or      al, al
                jz      finish
                call    putChar
                inc     bp
                jmp     short putString
finish:
                retn

;----------------------------------------------------------------------------------------------
; Data to adjust protected mode :
              ALIGN  4, db 0
gdtRegister:  ; memory image of global descriptor table register
                   dw   (8 * 5 - 1)
                   dd   gdtStart                          ;
              ALIGN  4, db 0
idtRegister:  ; memory image of interrupt descriptor table register
                   dw   0x07FF                            ; space for 256 entries =(256*8-1)
                   dd   0x0
gdtStartIm:
;    /* Selector 0x00 == invalid. */
                   dw   0x0000
                   dw   0x0000
                   db   0x00
                   db   0x00
                   db   0x00
                   db   0x00
;    /* Selector 0x08 == code. */
                   dw   0xFFFF                      ; limit 15-0
                   dw   0x0000                      ; base  15-0
                   db   0x00                        ; base  23-16
                   db   0x9B                        ; AR: P=1, DPL=0, S=1, type=101(code, ER), A=1
                   db   0xCF                        ; G=1(page granularity), D=1(32bit mode), limit 19-16
                   db   0x00                        ; base 31-24
                                                    ; base=0x00000000; limit=0xFFFFF
;    /* Selector 0x10 == data. */
                   dw   0xFFFF
                   dw   0x0000
                   db   0x00
                   db   0x93                        ; AR: P=1, DPL=0, S=1, type=001(data, WR), A=1
                   db   0xCF
                   db   0x00
                                                    ; base=0x00000000; limit=0xFFFFF
;    /* Selector 0x18 == shorter code: faults any code access 0xF0000000-0xFFFFFFFF. */
                   dw   0xFFFF
                   dw   0x0000
                   db   0x00
                   db   0x9B                        ; AR: P=1, DPL=0, S=1, type=101(code, ER), A=1
                   db   0xC7
                   db   0x00
                                                     ; base=0x00000000; limit=0x7FFFF
;    /* Selector 0x20 == data; faults any access 0xF0000000-0xFFFFFFFF. */
                   dw   0xFFFF
                   dw   0x0000
                   db   0x00
                   db   0x93                        ; AR: P=1, DPL=0, S=1, type=001(data, WR), A=1
                   db   0xC7
                   db   0x00
gdtEnd:                                             ; base=0x00000000; limit=0x7FFFF
switchToPM:
;   relocation of GDT in proper place for protected mode and 32bit flat memory model.
;   set ES == gdtStart
                   cli
                   mov     eax, gdtStart                 ; = 0x800
                   shr     eax, 4                        ;
                   mov     es, ax                        ;
                   mov     di, 0                         ; destination es:di = 0080:0000
                   mov     si, gdtStartIm                ; source      ds:si
                   mov     cx, 8 * 5                     ; number of bytes   = 40
               rep movsb
;*********************************************************************************************************
;                               Switch to Protected Mode
;*********************************************************************************************************
;    load GDTR and IDTR
                   cli
                   lgdt    [gdtRegister]
                   lidt    [idtRegister]
;    Switch to protected mode.
               mov      eax, cr0
               or       al, 1
               mov      cr0, eax

               jmp     dword 0x8:(next + (relocSegm << 4))
;     We arrive here in protected mode
next:
                BITS 32
;    # Load data selectors
               mov     ax, 0x10                     ;
               mov     ds, ax                       ;
               mov     es, ax                       ;
               mov     fs, ax                       ;
               mov     gs, ax                       ;
               mov     ss, ax
;    # Set up SP
               mov     esp, stackStart
;    # Reset the flags register.
               push    0                            ;
               popf

; go to start point of AO-RTOS application
;               jmp     dword 0x8:(temp_buffer)
                push    0x08
                push    temp_buffer
                retf

;*******************************************************************************
;
;                                 Debug helper
;
;*******************************************************************************

                BITS 16

;-------------------------------------------------------------------------------
HexDigitList:   db      "0123456789ABCDEF"
msg4:
                db      "eDI=        ",0Ah,0Dh
                db      "eSI=        ",0Ah,0Dh
                db      "eBP=        ",0Ah,0Dh
                db      "eSP=        ",0Ah,0Dh
                db      "eBX=        ",0Ah,0Dh
                db      "eDX=        ",0Ah,0Dh
                db      "eCX=        ",0Ah,0Dh
                db      "eAX=        ",0Ah,0Dh
                db      " IP=        ",0Ah,0Dh
                db      0
;-------------------------------------------------------------------------------
displayRegisters:
                push    eax
                push    ecx
                push    edx
                push    ebx
                push    esp
                push    ebp
                push    esi
                push    edi

                mov     bp,  sp
                mov     cx, 9
                mov     bx, msg4 + 4
.1:
                mov     edx, dword [bp]
                call    near hex8mem
                add     bp,  4
                add     bx,   14
                loop    .1

                mov     bp, msg4
                call    putString

                pop     edi
                pop     esi
                pop     ebp
                pop     esp
                pop     ebx
                pop     edx
                pop     ecx
                pop     eax
                retn
;-------------------------------------------------------------------------------
; edx - input; bx - address
hex8mem:
                push    cx
                push    ax
                push    bx
                push    bp

                mov     cx, 8
.1:
                rol     edx, 4
                mov     bp, dx
                and     bp, 0x000F
                mov     al, [bp + HexDigitList]
                mov     [bx], al
                inc     bx
                loop    .1

                pop     bp
                pop     bx
                pop     ax
                pop     cx
                retn

program_end:   db      'The End'
