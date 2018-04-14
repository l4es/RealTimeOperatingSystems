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

;********************************************************************************************************
;                   Intel 80x86 (Protected-Mode, Flat Model)
;                         INTERRUPT SERVICE ROUTINES:
;********************************************************************************************************


;********************************************************************************************************
;                                    PUBLIC and EXTERNAL REFERENCES
;********************************************************************************************************
.MODEL FLAT
.CODE
            PUBLIC timerISR_
            PUBLIC schedulerISR_
            EXTERN processInterrupt_:proc        ; AO_STACK * ISAObject::processInterrupt( DWORD, AO_STACK * )

;*********************************************************************************************************
;                                            HANDLE TICK ISR
;
;*********************************************************************************************************
timerISR:
               pushad                         ; Save interrupted thread's context

               push    esp                     ; the parameter of processInterrupt( TH_STACK * stkp )
               push    dword ptr 0                 ; the parameter of processInterrupt( int )
               call    processInterrupt
               mov     esp, eax     ; gets a return from the routine and puts it to esp -> switch context of CPU
; acknowledge of interrupt controller (8259 PIC)
               mov      al, 020h       ; non-specific EOI
               out      020h, al
               popad
               iret                            ; Return to interrupted thread

;*********************************************************************************************************
;                                       SCHEDULER'S PROGRAM INTERRUPT SR
;
;*********************************************************************************************************
schedulerISR:
               pushad
               push    esp                     ; the parameter of processInterrupt( TH_STACK * stkp )
               push    dword ptr 16                ; the parameter of processInterrupt( int )
               call    processInterrupt
               mov     esp, eax     ; gets a return from the routine and puts it to esp -> switch context of CPU
               popad
               iret                            ; Return to interrupted thread

;*********************************************************************************************************
;                                            KEYBORD ISR
;
;*********************************************************************************************************
keyboardISR:
               pushad                         ; Save interrupted thread's context

               push    esp                     ; the parameter of processInterrupt( TH_STACK * stkp )
               push    dword ptr 1                 ; the parameter of processInterrupt( int )
               call    processInterrupt
               mov     esp, eax     ; gets a return from the routine and puts it to esp -> switch context of CPU
; acknowledge of interrupt controller (8259 PIC)
               mov      al, 020h       ; non-specific EOI
               out      020h, al
               popad
               iret                            ; Return to interrupted thread

END

