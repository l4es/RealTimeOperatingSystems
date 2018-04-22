;
; Copyright (C) 2002-2003 by egnite Software GmbH. All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the distribution.
; 3. All advertising materials mentioning features or use of this
;    software must display the following acknowledgement:
;
;    This product includes software developed by egnite Software GmbH
;    and its contributors.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
; FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
; BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
; OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
; AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
; THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
; SUCH DAMAGE.
;
; For additional information see http://www.ethernut.de/
;
; $Log$
; Revision 1.1  2003/11/03 15:51:31  haraldkipp
; First check in
;
;

.nolist
#include "avr/io.h"
.list

; This program implements a minimalist STK500 compatible programming
; adapter for the serial port on an AT90S2313 MCU. It had been
; originally written for AVR Studio, but then ported to AVR-GCC and
; will no longer build with AVR Studio. However, the resulting
; programmer had been successfully tested with AVR Studio 4.07 and
; uisp version 20030827cvs on ATmega128 and ATmega13 targets.
;
; *********************************************************************
; Protocol definitions

; Initially the STK500 protocol had been evaluated by re-engineering.
; Some months later Atmel decided to publish it. See
; http://www.atmel.com/atmel/acrobat/doc2525.pdf

#define STK_OK          0x10
#define STK_INSYNC      0x14
#define STK_NOSYNC      0x15

#define CRC_EOP         0x20

; *********************************************************************
; General configurations

; Baudrate factor. AVR Studio uses a fixed rate of 115,200 Baud, which
; results in a baudrate register factor of 1 on a 3.6864 MHz MCU. See
; the AT90S2313 datasheet for further details.

#define BRFACTOR    1

; *********************************************************************
; Port Usage

; PB0: Unused
; PB1: Unused
; PB2: ISP-LED (out)
; PB3: LED (out)
; PB4: ISP-RESET (out)
; PB5: ISP-MOSI (out)
; PB6: ISP-MISO (in)
; PB7: ISP-SCK (out)
;
; PD0: RXD (in)
; PD1: TXD (out)
; PD2: HSIN/INT0 (in)
; PD3: HSOUT (out)
; PD4: unused
; PD5: unused
; PD6: unused
;

#define ISPLED	2
#define LEDIND	3
#define ISPRST	4
#define ISPMOSI	5
#define ISPMISO	6
#define ISPSCK	7

#define HSIN    2
#define HSOUT   3

; *********************************************************************
; Register usage

#define    r_temp   r16
#define    r_data   r17
#define    r_isp    r18
#define    r_shift  r19
#define    r_delay  r20
#define    r_retry  r21
#define    r_size   r22
#define    r_parm   r23

; *********************************************************************
; RAM variables

                .section .bss

paddrl:         .space   1
paddrh:         .space   1

unicmd1:        .space   1
unicmd2:        .space   1
unicmd3:        .space   1
unicmd4:        .space   1

pm_devicecode:  .space   1   ; device code
pm_revision:    .space   1   ; device revision
pm_progtype:    .space   1   ; "0" both, "1" par only
pm_parmode:     .space   1   ; "0" pseudo, "1" full
pm_polling:     .space   1   ; "0" no, "1" yes
pm_selftimed:   .space   1   ; "0" no, "1" yes
pm_lockbytes:   .space   1   ; # of lock bytes
pm_fusebytes:   .space   1   ; # of fuse bytes
pm_flpollval1:  .space   1   ; Flash polling value
pm_flpollval2:  .space   1   ; Flash polling value
pm_eepollval1:  .space   1   ; EEPROM polling value
pm_eepollval2:  .space   1   ; EEPROM polling value
pm_pagesizeh:   .space   1   ; Page size high byte
pm_pagesizel:   .space   1   ; Page size low byte
pm_eesizeh:     .space   1   ; EEPROM size high byte
pm_eesizel:     .space   1   ; EEPROM size low byte
pm_flsize4:     .space   1   ; Flash size MSB
pm_flsize3:     .space   1   ; Flash size
pm_flsize2:     .space   1   ; Flash size
pm_flsize1:     .space   1   ; Flash size LSB


; *********************************************************************
; Vector table

        .section .text

        .global sisp
sisp:
        rjmp    main

; *********************************************************************
; Constant data

        .type   signon_msg, @object
        .global signon_msg
signon_msg:
        .string     "\x14\x41VR STK\x10"

        .type   version_msg, @object
        .global version_msg
version_msg:
        .string     "SISP 1.1.1  Copyright 2002-2003 by egnite Software GmbH\r\n"

; *********************************************************************
; Main program entry

.global main
main:
        ;
        ; Set stack pointer
        ldi     r_temp, lo8(RAMEND)
        out     _SFR_IO_ADDR(SPL), r_temp

        ;
        ; Initialize Port B
        ; PB2: ISP-LED z-state
        ; PB3: LED output low
        ; PB4: ISP-RESET z-state
        ; PB5: ISP-MOSI z-state
        ; PB6: ISP-MISO input
        ; PB7: ISP-SCK z-state
        clr     r_temp
        out     _SFR_IO_ADDR(PORTB), r_temp
        out     _SFR_IO_ADDR(DDRB), r_temp
        sbi     _SFR_IO_ADDR(DDRB), LEDIND

        ldi     r_retry, 3
blink:
        rcall   xdelay
        rcall   xdelay
        cbi     _SFR_IO_ADDR(PORTB), LEDIND   ; LED on
        ldi     r_delay, 64
        rcall   xdelay
        sbi     _SFR_IO_ADDR(PORTB), LEDIND   ; LED off
        dec     r_retry
        brne    blink

        ;
        ; Init UART
        ldi     r_temp, BRFACTOR
        out     _SFR_IO_ADDR(UBRR), r_temp
        ldi     r_temp, (1<<RXEN) | (1<<TXEN)
        out     _SFR_IO_ADDR(UCR), r_temp

        ;
        ; Set handshake line
        sbi     _SFR_IO_ADDR(PORTD), HSOUT
        sbi     _SFR_IO_ADDR(DDRD), HSOUT

; *********************************************************************
; Command handler
cmdnxt:
        sbi     _SFR_IO_ADDR(PORTB), LEDIND   ; LED off
        rcall   getc
        cbi     _SFR_IO_ADDR(PORTB), LEDIND   ; LED on

; *********************************************************************
; Version output

        cpi     r_data, 0x18    ; Ctrl-X
        brne    c30

        ldi     r30, lo8(version_msg)
        ldi     r31, hi8(version_msg)

        rjmp    c31a

; *********************************************************************
; PC command: Get synchronization.
;
; The PC uses this command to regain synchronization with the
; programmer. The command may be send several times until the
; programmer responds with STK_INSYNC.

c30:
        cpi     r_data, 0x30    ; '0'
        brne    c31

; Receive last character of a command. If it's an EOP, then
; respond with INSYNC. Otherwise send back NOSYNC.
cmdend:
        rcall   getc
        cpi     r_data, CRC_EOP
        breq    cmdok

        ldi     r_data, STK_NOSYNC
        rcall   putc
        rjmp    cmdnxt          ; Wait for next command

; Send back positive response.
cmdok:
        ldi     r_data, STK_INSYNC
        rcall   putc
cmdfirm:
        ldi     r_data, STK_OK
        rcall   putc

        rjmp    cmdnxt          ; Wait for next command

; *********************************************************************
; PC command: Check if starterkit present.
;
; The programmer responds with a sign-on message.
;
c31:
        cpi     r_data, 0x31    ; '1'
        brne    c41

        rcall   getc            ; Ignore CRC_EOP

        ldi     r30, lo8(signon_msg)
        ldi     r31, hi8(signon_msg)
c31a:
        lpm
        tst     r0
        breq    cmdnxt          ; Done, wait for next command
        mov     r_data, r0
        rcall   putc
        adiw    r30, 1
        rjmp    c31a

; *********************************************************************
; PC command: Get parameter value
c41:
        cpi     r_data, 0x41    ; 'A'
        brne    c42

        rcall   getc            ; Receive parameter id
        mov     r_parm, r_data

        rcall   getc            ; Ignore CRC_EOP

        ldi     r_data, STK_INSYNC
        rcall   putc

        rcall   getparm
        rcall   putc

        rjmp    cmdfirm

; *********************************************************************
; PC command: Set device programming parameters
;
; We ignore these parameters.

c42:
        cpi     r_data, 0x42    ; 'B'
        brne    c45

        ldi     r_size, 20
_c42nxt:
        rcall   getc
        dec     r_size
        brne    _c42nxt

        rjmp    cmdend

; *********************************************************************
; PC command: Set extended device programming parameters
;
; We ignore these parameters.

c45:
        cpi     r_data, 0x45    ; 'E'
        brne    c50

        ldi     r_size, 5
_c45nxt:
        rcall   getc
        dec     r_size
        brne    _c45nxt

        rjmp    cmdend

; *********************************************************************
; PC command: Enter Program Mode

c50:
        cpi     r_data, 0x50    ; 'P'
        brne    c51
        rcall   getc            ; Ignore CRC_EOP

        ;
        ; Enable ISP interface.
        ; PB2: ISP-LED output low
        ; PB3: LED output low
        ; PB4: ISP-RESET output low
        ; PB5: ISP-MOSI output high
        ; PB6: ISP-MISO input
        ; PB7: ISP-SCK output low
        ldi     r_temp, (1<<ISPMISO)
        out     _SFR_IO_ADDR(PORTB), r_temp
        ldi     r_temp, (1<<ISPLED)|(1<<LEDIND)|(1<<ISPRST)|(1<<ISPMOSI)|(1<<ISPSCK)
        out     _SFR_IO_ADDR(DDRB), r_temp

        rcall   xdelay

        ldi     r_retry, 32

        ;
        ; Send programming enable
c50a:
        ldi     r_isp, 0xAC
        rcall   ispxcg
        ldi     r_isp, 0x53
        rcall   ispxcg

        rcall   ispget
        cpi     r_isp, 0x53     ; Are we in sync
        breq    c50z            ; Yes, jump

        ;
        ; Handle sync failure

        rcall   ispget          ; Read last SPI byte

        sbi     _SFR_IO_ADDR(PORTB), ISPRST   ; Toggle target's reset line
        rcall   xdelay
        cbi     _SFR_IO_ADDR(PORTB), ISPRST
        rcall   xdelay

        dec     r_retry         ; Loop for retries
        brne    c50a
        rjmp    cmdok           ; Send positive resopnse

c50z:
        rcall   ispget
        rjmp    cmdok           ; Send positive resopnse

; *********************************************************************
; PC command: Leave program mode
c51:
        cpi     r_data, 0x51    ; 'Q'
        brne    c52
        rcall   getc            ; Ignore CRC_EOP

        ;
        ; Disable ISP interface.
        ; PB2: ISP-LED z-state
        ; PB3: LED output low
        ; PB4: ISP-RESET z-state
        ; PB5: ISP-MOSI z-state
        ; PB6: ISP-MISO input
        ; PB7: ISP-SCK z-state
        clr     r_temp
        out     _SFR_IO_ADDR(PORTB), r_temp
        out     _SFR_IO_ADDR(DDRB), r_temp
        sbi     _SFR_IO_ADDR(DDRB), LEDIND

        rjmp    cmdok           ; Send positive resopnse

; *********************************************************************
; PC command: Chip erase
c52:
        cpi     r_data, 0x52    ; 'R'
        brne    c55
        rcall   getc

        ldi     r_isp, 0xAC
        rcall   ispxcg
        ldi     r_isp, 0x80
        rcall   ispxcg
        ldi     r_isp, 0x04
        rcall   ispxcg
        rcall   ispget

        rcall   xdelay          ; Very long delay. May be reduced
        rcall   xdelay          ; but who really cares?

        rjmp    cmdok           ; Send positive resopnse

; *********************************************************************
; PC command: Load address

c55:
        cpi     r_data, 0x55    ; 'U'
        brne    c56

        rcall   getc
        sts     paddrl, r_data
        rcall   getc
        sts     paddrh, r_data

        rjmp    cmdend

; *********************************************************************
; PC command: Universal
c56:
        cpi     r_data, 0x56    ; 'V'
        brne    c64

        rcall   getc
        sts     unicmd1, r_data
        rcall   getc
        sts     unicmd2, r_data
        rcall   getc
        sts     unicmd3, r_data
        rcall   getc
        sts     unicmd4, r_data

        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC
        rcall   putc

        lds     r_isp, unicmd1
        rcall   ispxcg
        lds     r_isp, unicmd2
        rcall   ispxcg
        lds     r_isp, unicmd3
        rcall   ispxcg
        lds     r_isp, unicmd4
        rcall   ispxcg

        ldi     r_delay, 64
        rcall   delay

        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm

; *********************************************************************
; PC command: Program page

; This is a very tricky routine. The STK500 insists on sending
; data with 115,200 Baud. We hope, that the target MCU will be
; fast enough.

c64:
        cpi     r_data, 0x64    ; 'd'
        brne    c72

        rcall   getc            ; Size of this block to Y-register
        mov     r29, r_data
        rcall   getc
        mov     r28, r_data

        rcall   getc            ; Memory type (F)lash or (E)eprom
        cpi     r_data, 'F'
        brne    c64ee

        clr     r30

c64a:
        ldi     r_isp, 0x40     ; Send low byte
        rcall   ispput
        clr     r_isp
        rcall   ispput
        mov     r_isp, r30
        rcall   ispput
c64b:
        sbis    _SFR_IO_ADDR(USR), RXC
        rjmp    c64b
        in      r_isp, _SFR_IO_ADDR(UDR)
        rcall   ispput

        ldi     r_isp, 0x48     ; Send high byte
        rcall   ispput

        clr     r_isp
        rcall   ispput

        mov     r_isp, r30
        rcall   ispput
c64c:
        sbis    _SFR_IO_ADDR(USR), RXC
        rjmp    c64c
        in      r_isp, _SFR_IO_ADDR(UDR)
        rcall   ispput

        inc     r30             ; Increment address
        sbiw    r28, 2          ; Decrement length
        brne    c64a

        rcall   getc

        ldi     r_isp, 0x4C     ; Write memory page
        rcall   ispxcg
        lds     r_isp, paddrh
        rcall   ispxcg
        lds     r_isp, paddrl
        rcall   ispxcg
        rcall   ispget

        ldi     r_delay, 168    ; Target requires delay after each write
        rcall   delay           ; We use 35 milliseconds, OK for most targets

        rjmp    cmdok           ; Send positive resopnse


c64ee:

c64ea:
        sbis    _SFR_IO_ADDR(USR), RXC
        rjmp    c64ea
        in      r_isp, _SFR_IO_ADDR(UDR)

c64ec:
        sbis    _SFR_IO_ADDR(USR), RXC
        rjmp    c64ec
        in      r_isp, _SFR_IO_ADDR(UDR)

        sbiw    r28, 2
        brne    c64a

        rjmp    cmdend

; *********************************************************************
; PC command: Read Fuse Bits
c72:
        cpi     r_data, 0x72        ; 'r'
        brne    c73

        rcall   getc                ; EOP

        ldi     r_data, STK_INSYNC  ; INSYNC
        rcall   putc

        ldi     r_isp, 0x50         ; Read fuse bits command
        rcall   ispxcg
        ldi     r_isp, 0x00         ; Must be zero, don't care on mega103
        rcall   ispxcg
        rcall   ispget              ; Don't care
        rcall   ispget              ; These are the bits
        mov     r_data, r_isp
        rcall   putc

        ; ATmega128 only
        ldi     r_isp, 0x58         ; Read high fuses bits command
        rcall   ispxcg
        ldi     r_isp, 0x08         ; Must be 8
        rcall   ispxcg
        rcall   ispget              ; Don't care
        rcall   ispget              ; Here we get them
        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm

; *********************************************************************
; PC command: Read Lock Bits
c73:
        cpi     r_data, 0x73    ; 's'
        brne    c74

        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC    ; INSYNC
        rcall   putc


        ldi     r_isp, 0x58
        rcall   ispxcg
        ldi     r_isp, 0x00
        rcall   ispxcg
        rcall   ispget
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm


; *********************************************************************
; PC command: Read Page
c74:
        cpi     r_data, 0x74    ; 't'
        brne    c75

        rcall   getc
        mov     r29, r_data
        rcall   getc
        mov     r28, r_data

        rcall   getc            ; 'F' or 'E'
        cpi     r_data, 'F'
        brne    c74ee

        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC    ; INSYNC
        rcall   putc

        lds     r30, paddrl
        lds     r31, paddrh

c74a:
        ldi     r_isp, 0x20
        rcall   ispxcg
        mov     r_isp, r31
        rcall   ispxcg
        mov     r_isp, r30
        rcall   ispxcg
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        ldi     r_isp, 0x28
        rcall   ispxcg
        mov     r_isp, r31
        rcall   ispxcg
        mov     r_isp, r30
        rcall   ispxcg
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        adiw    r30, 1
        sbiw    r28, 2
        brne    c74a

        rjmp    cmdfirm

c74ee:
        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC    ; INSYNC
        rcall   putc

        lds     r30, paddrl
        lds     r31, paddrh

c74ea:
        ldi     r_isp, 0xA0
        rcall   ispxcg
        mov     r_isp, r31
        rcall   ispxcg
        mov     r_isp, r30
        rcall   ispxcg
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        adiw    r30, 1
        sbiw    r28, 1
        brne    c74ea

        rjmp    cmdfirm


; *********************************************************************
; PC command: Read Signature
c75:
        cpi     r_data, 0x75    ; 'u'
        brne    c76
        rcall   getc

        ldi     r_data, STK_INSYNC
        rcall   putc

        ldi     r_isp, 0x30
        rcall   ispxcg
        rcall   ispget
        rcall   ispget
        rcall   ispget

        mov     r_data, r_isp
        rcall   putc

        ldi     r_isp, 0x30
        rcall   ispxcg
        rcall   ispget
        ldi     r_isp, 0x01
        rcall   ispxcg
        rcall   ispget

        mov     r_data, r_isp
        rcall   putc

        ldi     r_isp, 0x30
        rcall   ispxcg
        rcall   ispget
        ldi     r_isp, 0x02
        rcall   ispxcg
        rcall   ispget

        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm

; *********************************************************************
; PC command: Read Oscillator Calibration Byte
c76:
        cpi     r_data, 0x76    ; 'v'
        brne    c77

        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC    ; INSYNC
        rcall   putc

        ldi     r_isp, 0x38
        rcall   ispxcg
        rcall   ispget
        rcall   ispget
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm


; *********************************************************************
; PC command: Read Fuse Bits Extended
c77:
        cpi     r_data, 0x77    ; 'w'
        brne    c78

        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC    ; INSYNC
        rcall   putc


        ldi     r_isp, 0x50
        rcall   ispxcg
        ldi     r_isp, 0x00
        rcall   ispxcg
        rcall   ispget
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        ldi     r_isp, 0x58
        rcall   ispxcg
        ldi     r_isp, 0x08
        rcall   ispxcg
        rcall   ispget
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        ldi     r_isp, 0x50
        rcall   ispxcg
        ldi     r_isp, 0x08
        rcall   ispxcg
        rcall   ispget
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm

; *********************************************************************
; PC command: Read Oscillator Calibration Byte Extended
c78:
        cpi     r_data, 0x78    ; 'x'
        brne    cbad

        rcall   getc
        mov     r30, r_data

        rcall   getc            ; EOP

        ldi     r_data, STK_INSYNC    ; INSYNC
        rcall   putc

        ldi     r_isp, 0x38
        rcall   ispxcg
        rcall   ispget
        mov     r_isp, r30
        rcall   ispxcg
        rcall   ispget
        mov     r_data, r_isp
        rcall   putc

        rjmp    cmdfirm

; *********************************************************************
; PC command: Unknown
cbad:
        rjmp    cmdend

; *********************************************************************
; *** GetParm
getparm:
        ;
        ; Hardware version
        cpi     r_parm, 0x80
        brne    _gp81
        ldi     r_data, 2
        ret

        ;
        ; Software version major
_gp81:
        cpi     r_parm, 0x81
        brne    _gp82
        ldi     r_data, 1
        ret

        ;
        ; Software version minor
_gp82:
        cpi     r_parm, 0x82
        brne    _gp83
        ldi     r_data, 14
        ret

        ;
        ; LEDs
_gp83:
        cpi     r_parm, 0x83
        brne    _gp84
        ldi     r_data, 1
        ret

        ;
        ; Target voltage
_gp84:
        cpi     r_parm, 0x84
        brne    _gp85
        ldi     r_data, 50
        ret

        ;
        ; Adjustable Voltage
_gp85:
        cpi     r_parm, 0x85
        brne    _gp86
        ldi     r_data, 50
        ret

        ;
        ; Oscillator timer prescaler value
_gp86:
        cpi     r_parm, 0x86
        brne    _gp87
        ldi     r_data, 0
        ret

        ;
        ; Oscillator timer compare match value
_gp87:
        cpi     r_parm, 0x87
        brne    _gp88
        ldi     r_data, 0
        ret

        ;
        ; Reset duration
_gp88:
        cpi     r_parm, 0x88
        brne    _gp89
        ldi     r_data, 0x30
        ret

        ;
        ; SCK duration
_gp89:
        cpi     r_parm, 0x89
        brne    _gp90
        ldi     r_data, 0x30
        ret

        ;
        ; Bufsiz low
_gp90:
        cpi     r_parm, 0x90
        brne    _gp91
        ldi     r_data, 64
        ret

        ;
        ; Bufsiz high
_gp91:
        cpi     r_parm, 0x91
        brne    _gp92
        ldi     r_data, 0
        ret

        ;
        ; Device
_gp92:
        cpi     r_parm, 0x92
        brne    _gp93
        lds     r_data, pm_devicecode
        ret

        ;
        ; Progmode
_gp93:
        cpi     r_parm, 0x93
        brne    _gp94
        lds     r_data, pm_progtype
        ret

        ;
        ; Para mode
_gp94:
        cpi     r_parm, 0x94
        brne    _gp95
        lds     r_data, pm_parmode
        ret

        ;
        ; Polling
_gp95:
        cpi     r_parm, 0x95
        brne    _gp96
        lds     r_data, pm_polling
        ret

        ;
        ; Self timed
_gp96:
        cpi     r_parm, 0x96
        brne    _gpuk
        lds     r_data, pm_selftimed
        ret

        ;
        ; Unknown
_gpuk:
        clr     r_data
        ret

;
; *********************************************************************
; UART routines
; The UART is used in polling mode.

; Receive a single byte into register r_data.
;
; This routine will not return until a byte is available.
;
getc:
        sbis    _SFR_IO_ADDR(USR), RXC
        rjmp    getc
        in      r_data, _SFR_IO_ADDR(UDR)
        ret

; Send a single byte passed in register r_data.
;
putc:
        sbis    _SFR_IO_ADDR(USR), UDRE
        rjmp    putc
        out     _SFR_IO_ADDR(UDR), r_data
        ret

; *********************************************************************
; Exchange ISP Byte

ispget:
        clr     r_isp
ispxcg:
        ldi     r_temp, 8
        clr     r_shift

        ;
        ; Start of 8-bit loop
_ispa:
        rol     r_isp                           ; (1)
        brcc    _ispb                           ; (2/1)
        sbi     _SFR_IO_ADDR(PORTB), ISPMOSI    ; (2)
        rjmp    _ispc                           ; (2)
_ispb:
        cbi     _SFR_IO_ADDR(PORTB), ISPMOSI    ; (2)
_ispc:
        nop                                     ; (1)
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK     ; (2)
        lsl     r_shift                         ; (1)
        sbic    _SFR_IO_ADDR(PINB), ISPMISO     ; (2/1)
        ori     r_shift, 1                      ; (1)
        cbi     _SFR_IO_ADDR(PORTB), ISPSCK     ; (2)

        ;
        ; End of 8-bit loop
        dec     r_temp                          ; (1)
        brne    _ispa                           ; (2/1)

        mov     r_isp, r_shift                  ; (1)

        ret                                     ; (4)

; *********************************************************************
; Send out a byte to ISP as fast as possible.
;
; Runtime calculation:
; + 7 cycles for rcall/return
; + 1 cycle to initially read the port status
; + 4 cycle to set the final port status
; + 48 cycles for output, 6 cycles per bit
; The total of 60 cycles takes 16.276 microseconds on a 3.6864 MHz MCU.
; At 115 kBaud a new character is received every 86.8 microsecond.
; The SCK low time is 814 nanoseconds and the high time is 543 nanoseconds.
; Following the datasheet, the target requires a minimum SCK low and
; high time of 2 clock cycles. So we may not be able to program targets
; running with same or higher crystals than our own.
; Tests with an ATmega128 showed, that it was possible to program the
; chip while running on its internal oscillator down to 2 MHz.
;

ispput:                                     ; (3) Costly call
        in      r_temp, _SFR_IO_ADDR(PORTB) ; (1) Load current port status

        bst     r_isp, 7                    ; (1) Bit7 -> T
        bld     r_temp, ISPMOSI             ; (1) T -> MOSI
        out     _SFR_IO_ADDR(PORTB), r_temp ; (1) Output, SCK still low
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK ; (2) SCK high

        bst     r_isp, 6                    ; (1)
        bld     r_temp, ISPMOSI             ; (1)
        out     _SFR_IO_ADDR(PORTB), r_temp ; SCK back to low after about 3 cycles
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK ; SCK high again after about 2 cycles

        bst     r_isp, 5
        bld     r_temp, ISPMOSI
        out     _SFR_IO_ADDR(PORTB), r_temp
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK

        bst     r_isp, 4
        bld     r_temp, ISPMOSI
        out     _SFR_IO_ADDR(PORTB), r_temp
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK

        bst     r_isp, 3
        bld     r_temp, ISPMOSI
        out     _SFR_IO_ADDR(PORTB), r_temp
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK

        bst     r_isp, 2
        bld     r_temp, ISPMOSI
        out     _SFR_IO_ADDR(PORTB), r_temp
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK

        bst     r_isp, 1
        bld     r_temp, ISPMOSI
        out     _SFR_IO_ADDR(PORTB), r_temp
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK

        bst     r_isp, 0
        bld     r_temp, ISPMOSI
        out     _SFR_IO_ADDR(PORTB), r_temp
        nop
        sbi     _SFR_IO_ADDR(PORTB), ISPSCK

        nop
        nop
        nop
        out     _SFR_IO_ADDR(PORTB), r_temp ; (1) SCK low

        ret                                 ; (4) Costly return


; *********************************************************************
; Delay
;
; xdelay takes about 53.6 ms
xdelay:                         ; (3)
        clr     r_delay         ; (1)
delay:
        clr     r_temp          ; (1) Outer loop with r_delay * 772 cycles
dl:
        dec     r_temp          ; (1) Inner loop with 3 cycles
        brne    dl              ; (2/1)

        dec     r_delay         ; (1)
        brne    delay           ; (2/1)
        ret                     ; (4)

#ifdef SISPDEBUG
; *********************************************************************
; Debug
;
puthexb:
        push    r_data
        swap    r_data
        rcall   puthexc
        pop     r_data

puthexc:
        push    r_temp
        mov     r_temp, r_data
        andi    r_temp, 0x0f
        subi    r_temp, -'0'
        cpi     r_temp, 0x3a
        brlo    _putx1
        subi    r_temp, -7
_putx1:
        push    r_data
        mov     r_data, r_temp
        rcall   putc
        pop     r_data
        pop     r_temp
        ret
#endif
