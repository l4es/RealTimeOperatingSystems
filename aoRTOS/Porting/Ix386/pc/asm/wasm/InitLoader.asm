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
.MODEL FLAT
.CODE

begin:          jmp     main
msg1            db      "- ELF FILE LOADER. (c) 2003 AK -",0Ah,0Dh,0
msg2            db      "Boot Failure",0Ah,0Dh,0
msg3            db      "END LOAD",0Ah,0Dh,0
secPerTrack     EQU      18

; for loading an ELF file
size_file_in_sect    EQU  50       ; 50 for simpleness, but
                                   ; actually we have to calculate the size by using ELF header of
                                   ; the application file
temp_buffer          EQU  070000h  ; keep application file in ELF befor we go to protected mode

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
                add     ax, si           ; return segments DS, ES, SS to compile-time value
                mov     ds, ax
                mov     es, ax
; Set Stack Segment
                mov     ss, ax
                mov     sp, 07ffeh
; go to new CS ( return CS:IP to compile-time value )
                push    ax
                push    newCS
                retf
newCS:
                mov     bp, offset msg1
                call    putString
; reading next n sectors from floppy. The sectors contain a 'PM Starter' code
                mov     bx, 0200h
                mov     ax, 10                   ; number sectors for reading = 10 ( for sure )
                mov     cx, 0                    ; track = 0
                mov     di, 2                    ; So = 2
                mov     [drive], dl              ; drive
                mov     dh, 0                    ; head

                call    readSectors

                or      ah, ah
                jnz     short reboot                       ; error !!!
; Loading of .elf file with AO-RTOS application
                mov     ax, size_file_in_sect   ; number sectors for reading = 50 (while)
                mov     cx, 1                   ; track = 1
                mov     di, 1                   ; So = 1
                mov     dh, 0                   ; head
                mov     dl, [drive]
                mov     bx, (temp_buffer SHR 4)  ; start address of temp buffer for my application = 0x70000 (while)
                mov     es, bx                  ; bx = segment file.bin address
                xor     bx, bx                  ; bx = 0 offset file.bin address

                call    readArrSect

                or      ah, ah
                jnz     reboot                  ; error !!!

                mov     bp, offset msg3
                call    putString

; stop motor
;                xor     ax, ax
;                xor     dl, dl
;                int     13h

	            mov	    dx, 03f2h
	            xor	    al, al
	            out	    dx, al

                jmp     PM_set_start

reboot:
                mov     bp, offset msg2
                call    putString
                int     19h                     ; DISK BOOT causes reboot of disk system

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

include PMStarter.asm
include ScreenLib.asm

program_end:   db      'The End'
end
