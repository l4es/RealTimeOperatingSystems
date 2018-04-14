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

; my macro for acknoledge of interrupt controller
%macro intc_ack 1
               mov      al, 0x20       ; non-specific EOI
%if (%1 >= 0x20) && (%1 <= 0x28)
               out      0x20, al       ;
%else
               out      0xA0, al       ;
%endif
%endmacro

;********************************************************************************************************
;                                    PUBLIC and EXTERNAL REFERENCES
;********************************************************************************************************

            global timerISR
            global keyboardISR
            global serialISR
            global schedulerISR
            global exception_noerr
            global exception_err
            global iTrap
            global pc_IRQ_handler_0
            global pc_IRQ_handler_1

;            global __gxx_personality_v0
;            global free

            extern  processInterrupt        ; AO_STACK * ISAObject::processInterrupt( DWORD, AO_STACK * )t

segment     .text
;__gxx_personality_v0:   dd     0
;free:                   dd     0
;*********************************************************************************************************
;                                            HANDLE TICK ISR
;
;*********************************************************************************************************
timerISR:
               pushad                         ; Save interrupted thread's context

               push    esp                     ; the parameter of processInterrupt( TH_STACK * stkp )
               push    dword 0                 ; the parameter of processInterrupt( int )
               call    processInterrupt
               mov     esp, eax     ; gets a return from the routine and puts it to esp -> switch context of CPU
               intc_ack 0x20                  ; serves 8259 PIC
               popad
               iret                            ; Return to interrupted thread

;*********************************************************************************************************
;                                       SCHEDULER'S PROGRAM INTERRUPT SR
;
;*********************************************************************************************************
schedulerISR:
               pushad
               push    esp                     ; the parameter of processInterrupt( TH_STACK * stkp )
               push    dword 16                ; the parameter of processInterrupt( int )
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
               push    dword 1                 ; the parameter of processInterrupt( int )
               call    processInterrupt
               mov     esp, eax     ; gets a return from the routine and puts it to esp -> switch context of CPU
               intc_ack 0x21                  ; serves 8259 PIC
               popad
               iret                            ; Return to interrupted thread
;*********************************************************************************************************
;                                            SERIAL PORT ISR
;
;*********************************************************************************************************
serialISR:
               pushad                         ; Save interrupted process's context

               push    esp                     ; the parameter of processInterrupt( TH_STACK * stkp )
               push    dword 4                 ; the parameter of processInterrupt( int )
               call    processInterrupt
               mov     esp, eax     ; gets a return from the routine and puts it to esp -> switch context of CPU
               intc_ack 0x24                  ; serves 8259 PIC
               popad
               iret                            ; Return to interrupted thread
;*********************************************************************************************************
;                                            Exception Handler (ISR)
;                                            no error code
;*********************************************************************************************************
trap_error_code: dd 0
exception_noerr:
               mov dword [trap_error_code], 0
               iret
;*********************************************************************************************************
;                                            Exception Handler (ISR)
;                                            with error code
;*********************************************************************************************************
exception_err:
               pop dword [trap_error_code]
               iret
;*********************************************************************************************************
;                                            Interrupt trap (ISR)
;
;*********************************************************************************************************
iTrap:
               iret
pc_IRQ_handler_0:
               push    eax
               intc_ack 0x20
               pop     eax
               iret
pc_IRQ_handler_1:
               push    eax
               intc_ack 0x28
               pop     eax
               iret

