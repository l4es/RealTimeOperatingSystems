;*********************************************************************
; File: Start.s
;
; created: 8/1/95, PBB
;
; desc:
; 
;  System initialisation routine - entry point for the application.
;  The stack pointer is initialised, then the hardware initialisation
;  routine called.  The static data areas are then initialised, before
;  the main function is executed.  A simple exit funtion is also
;  supplied
;
;*********************************************************************

    .h8300h
    .section .text
    .global _start
    .global _exit

    .extern _main

    .extern _data
    .extern _mdata
    .extern _edata
    .extern _bss
    .extern _ebss
    .extern _stack

_start:
    ; initialise the SP for non-vectored code
    mov.l   #_stack,er7
    ; get the boundaries for the .data section initialisation
    mov.l   #_data,er0
    mov.l   #_edata,er1
    mov.l   #_mdata,er2
    cmp.l   er0,er1
    beq     start_1
start_l:
    mov.b   @er2,r3l  ;get from src
    mov.b   r3l,@er0  ;place in dest
    inc.l   #1,er2    ;inc src
    inc.l   #1,er0    ;inc dest
    cmp.l   er0,er1   ;dest == edata?
    bne     start_l
start_1:
    ; zero out bss
    mov.l   #_bss,er0
    mov.l   #_ebss,er1
    cmp.l   er0,er1         
    beq     start_3
    sub.b   r2l,r2l
start_2:
    mov.b   r2l,@er0
    inc.l   #1,er0
    cmp.l   er0,er1
    bne     start_2
start_3:
    ; call the mainline     
    jsr     @_main

    ; jump to exit
    mov.l   er0,er4
    jmp     @_exit

_exit:  nop
    bra _exit
