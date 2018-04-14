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

.CODE
PUBLIC          screenInit
PUBLIC          screenPuts
;===========================================================================================================
;-----------------------------------------  S C R E E N    L I B R A R Y  ----------------------------------
;===========================================================================================================
;                BITS 32
DisplayBuffer  equ     0xB8000
DisplayPort    equ     0x03d4
ScreenWidth    equ     80
ScreenLength   equ     25

DisplayAttr    dw      0x0700
XPos           dw      0
YPos           dw      0

clearScreen:
               push    eax
               push    ecx
               push    edi
               mov     edi, DisplayBuffer
               mov     eax, ScreenWidth
               imul    eax, ScreenLength
               mov     ecx, eax                       ; ScreenWidth * ScreenLength
               mov     ax, [DisplayAttr]
               or      al, ' '
               cld
          rep  stosw
               pop     edi
               pop     ecx
               pop     eax
               retn
moveCursor:
               push    eax
               push    edx
               mov     ax, [YPos]
               imul    ax, ScreenWidth
               add     ax, [XPos]
               push    eax                           ; YPos * ScreenWidth + XPos
               push    eax
               mov     al, 0x0e
               mov     dx, DisplayPort
               out     dx, al
               pop     eax
               shr     ax, 8
               inc     dx
               out     dx, al
               mov     al, 0x0f
               dec     dx
               out     dx, al
               pop     eax
               inc     dx
               out     dx, al
               pop     edx
               pop     eax
               retn
scrollUp:
; parameter::           ecx - lines
               push    eax
               push    edi
               push    esi
               mov     eax, ScreenWidth
               mul     ecx
               movzx   esi, ax
               push    esi                   ; lines * ScreenWidth
               add     esi, esi              ; lines * ScreenWidth * 2
               add     esi, DisplayBuffer    ; DisplayBuffer + (lines * ScreenWidth) * 2
               mov     edi, DisplayBuffer
               mov     eax, ScreenLength
               sub     eax, ecx
               imul    eax, ScreenWidth
               mov     ecx, eax              ; (ScreenLength - lines) * ScreenWidth
               cld
          rep  movsw
               pop     ecx                   ; lines * ScreenWidth
               mov     ax, [DisplayAttr]
               or      al, ' '
          rep  stosw
               pop     esi
               pop     edi
               pop     eax
               retn
scrollDown:
; par::                 ecx - lines
               push    eax
               push    edi
               push    esi
               mov     eax, (ScreenWidth * 2)
               imul    eax, ScreenLength
               dec     eax                   ; (ScreenLength * ScreenWidth * 2) - 1
               add     eax, DisplayBuffer    ; DisplayBuffer + (ScreenLength * ScreenWidth) * 2 - 1
               mov     edi, eax
               mov     esi, eax
               mov     eax, ScreenWidth
               mul     ecx
               push    eax                 ; lines * ScreenWidth
               add     eax, eax            ; lines * ScreenWidth * 2
               sub     esi, eax            ; DisplayBuffer + (ScreenLength - lines) * ScreenWidth * 2 - 1
               mov     eax, ScreenLength
               sub     eax, ecx
               imul    eax, ScreenWidth
               mov     ecx, eax               ; (ScreenLength - lines)*ScreenWidth
               std
          rep  movsw
               pop     ecx                    ; lines * ScreenWidth
               mov     ax, [DisplayAttr]
               or      al, ' '
          rep  stosw
               pop     esi
               pop     edi
               pop     eax
               retn
newLine:
               push    ecx
               mov     word ptr [XPos], 0
               inc     word ptr [YPos]
               cmp     word ptr [YPos], ScreenLength         ;  YPos - ScreenLength >= 0
               jl      L1                           ; go if < 0
               dec     word ptr [YPos]
               mov     ecx, 1
               call    scrollUp
L1:
               call    moveCursor
               pop     ecx
               retn
displayChar:
; par::   al - character
               push    edx
               xor     ah, ah
               or      ax, [DisplayAttr]
               push    eax
               movzx   edx, word ptr [XPos]
               movzx   eax, word ptr [YPos]
               imul    eax, ScreenWidth
               add     eax, edx
;               add     eax, DisplayBuffer            ; (ScreenWidth * YPos + XPos) + DisplayBuffer
               pop     edx
               mov     [eax*2+DisplayBuffer], dx
               inc     word ptr [XPos]
               cmp     word ptr [XPos], ScreenWidth
               jl      L2
               call    newLine
L2:
               call    moveCursor
               pop     edx
               retn
screenInit:
               mov     word ptr [XPos], 0
               mov     word ptr [YPos], 0
               call    clearScreen
               call    moveCursor
               retn
screenPuts:
; parameter::    si - string address
               push    ebp
               mov     ebp, esp
               push    esi
               push    ecx
               mov     esi, [ebp+8]
continue_:
               lodsb
               cmp     al, 0
               je      exit_
               cmp     al, 0x0a                     ; \n
               je      sL1
               cmp     al, 0x0d                     ; \r
               je      sL2
               cmp     al, 0x08                     ; \b
               je      sL3
               cmp     al, 0x09                     ; \t
               je      sL4
               cmp     al, 0x0C                     ;
               je      sL5
               cmp     al, 1
               je      sL6
               cmp     al, 2
               je      sL7

sL0:           call    displayChar
               jmp     continue_
sL1:
               call    newLine
               jmp     continue_
sL2:
               mov     word ptr [XPos], 0
               call    moveCursor
               jmp     continue_
sL3:
               cmp     word ptr [XPos], 0
               je      continue_
               dec     word ptr [XPos]
               call    moveCursor
               jmp     continue_
sL4:
               mov     al, ' '
               call    displayChar
               test    word ptr [XPos], 0x0007
               jne     sL4
               jmp     continue_
sL5:
               call    screenInit
               jmp     continue_
sL6:
               mov     ecx, 1
               call    scrollUp
               mov     word ptr [XPos], 0
               mov     word ptr [YPos], ScreenLength - 1
               call    moveCursor
               jmp     continue_
sL7:
               mov     ecx, 1
               call    scrollDown
               mov     word ptr [XPos], 0
               mov     word ptr [YPos], 0
               call    moveCursor
               jmp     continue_
exit_:
               pop     ecx
               pop     esi
               leave
               retn
;-------------------------------------------------------------------------------
;
; print 4 hex digits ( binary word ) dx - input word
;
;-------------------------------------------------------------------------------
HexDigits:      db      "0123456789ABCDEF"
hex4:
                push    ecx
                mov     cx, 4
m1:
                rol     dx, 4
                mov     bx, dx
                and     ebx, 0x0000000F
                mov     al, [HexDigits + ebx]
                call    displayChar
                loop    m1
                mov     al, " "
                call    displayChar
                pop     ecx
                retn
;-------------------------------------------------------------------------------
;
; print 8 hex digits ( binary double word ) edx - input word
;
;-------------------------------------------------------------------------------
hex8:
                push    cx
                mov     cx, 8
m2:
                rol     edx, 4
                mov     bx, dx
                and     ebx, 0x0000000F
                mov     al, [HexDigits + ebx]
                call    displayChar
                loop    m2
                mov     al, " "
;                call    displayChar
                pop     cx
                retn
;-------------------------------------------------------------------------------
;
; print array of binary words in hex. Input: esi - array begin, cx - number of words
;
;-------------------------------------------------------------------------------
displayHex:
                pusha
                mov     di, 0
d1:
                or      di, di
                jne     d2
                call    newLine
                mov     edx, esi
                call    hex8
                mov     al, ":"
                call    displayChar
                mov     al, " "
                call    displayChar
                mov     di, 8
d2:
                mov     dx, [esi]
                add     esi, 2
                call    hex4
                dec     di
                loop    d1
                popa
                retn

displayRegisters:
                pusha
                mov     ebp,  esp
                mov     ecx, 8
                call    newLine
dr:
                mov     al,   " "
                call    displayChar
                mov     edx,  [ebp]
                call    hex8
                add     ebp,  4
                loop    dr

                popa
                retn
