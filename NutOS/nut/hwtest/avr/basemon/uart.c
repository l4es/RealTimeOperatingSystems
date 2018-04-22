/*
 * Copyright (C) 2001-2006 by egnite Software GmbH
 * Copyright (C) 2009 by egnite GmbH
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*
 * $Id: uart.c 3597 2011-10-21 16:17:18Z haraldkipp $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

#include <stdio.h>
#include <string.h>

#include <sys/timer.h>
#include <arpa/inet.h>

#include "utils.h"
#include "uart.h"

prog_char presskey_P[] = "Press any key to stop\n";

char inbuff[32];

/*
 * Wait for a character from the serial port.
 */
uint8_t GetChar(void)
{
    uint8_t ch;

#ifdef HEARTBEAT_BIT
    HeartBeat();
#endif
#if defined (__AVR__)
    if ((inb(USR) & _BV(RXC)) == 0)
        return 0;
    ch = inb(UDR);
#else
    ch = 0;
#endif
    return ch;
}

/*
 * Get a line of input.
 */
int GetLine(char * line, int size)
{
    int cnt = 0;
    uint8_t ch;

    GetChar();
    for (;;) {
        if (cnt > size - 1)
            break;
        while ((ch = GetChar()) == 0);
        if (ch < 32)
            break;
        putchar(ch);
        *line++ = ch;
        cnt++;
    }
    *line = 0;
    putchar('\n');

    return cnt;
}

/*
 * Get a line of input.
 */
char *GetIP(char * prompt, char * value)
{
    static prog_char badip_P[] = "Bad IP address";

    for (;;) {
        printf("%s (%s): ", prompt, value);
        if (GetLine(inbuff, sizeof(inbuff)) == 0)
            break;
        if (inet_addr(inbuff) != (uint32_t) (-1L)) {
            strcpy(value, inbuff);
            break;
        }
        puts_P(badip_P);
    }
    return value;
}

/*
 * Automatic baudrate detection.
 *
 * Switch to different baud rates and wait
 * for a space character.
 */
int DetectSpeed(void)
{
#if defined (__AVR__)
    uint8_t bstab[] = {
        1,      /* 115200 @  3.6864 */
        7,      /* 115200 @ 14.7456 */
        23,     /*   9600 @  3.6864, 19200 @  7.3728, 38400 @ 14.7456 */
        11,     /*  19200 @  3.6864, 38400 @  7.3728, 76800 @ 14.7456 */
        12,     /*  19200 @  4.0000, 38400 @  8.0000, 57600 @ 12.0000, 76800 @ 16.0000 */
        25,     /*   9600 @  4.0000, 19200 @  8.0000, 38400 @ 16.0000 */
        5,      /*  38400 @  3.6864, 76800 @  7.3728 */
        47,     /*   9600 @  7.3728, 19200 @ 14.7456 */
        51,     /*   9600 @  8.0000, 19200 @ 16.0000 */
        3,      /* 115200 @  7.3728 */
        9,      /* 115200 @ 18.4320 */
        2,      /*  76800 @  3.6864 */
        14,     /*  76800 @ 18.4320 */
        29,     /*  38400 @ 18.4320 */
        64,     /*  19200 @ 20.0000 */
        59,     /*  19200 @ 18.4320 */
        95,     /*   9600 @ 14.7456 */
        103,    /*   9600 @ 16.0000 */
        119,    /*   9600 @ 18.4320 */
        129,    /*   9600 @ 20.0000 */
        77      /*   9600 @ 12.0000 */
    };
    uint8_t bsx;
    int bs;
    uint16_t i;
    uint8_t t;
    uint8_t rec;
    uint8_t ict;                 /* Imagecraft dummy */

    /*
     * Enable UART transmitter and receiver.
     */
    outb(UCR, _BV(RXEN) | _BV(TXEN));

    /*
     * Sixteen retries.
     */
    for (t = 1; t < 16; t++) {
        for (bsx = 0; bsx < sizeof(bstab); bsx++) {
            bs = bstab[bsx];

            /*
             * Set baudrate selector.
             */
            Delay(1000);
            if ((inb(USR) & _BV(RXC)) != 0)
                ict = inb(UDR);
            outb(UBRR, (uint8_t) bs);
            Delay(1000);
            if ((inb(USR) & _BV(RXC)) != 0)
                ict = inb(UDR);

            /*
             * Now wait for some time for a character received.
             */
            for (i = 0; i < 40; i++) {
                if ((inb(USR) & _BV(RXC)) != 0)
                    break;
                Delay(1000);
            }
            rec = 0;
            for (i = 1; i; i++) {
                if ((inb(USR) & _BV(RXC)) != 0) {
                    if (inb(UDR) == 32) {
                        if (rec++)
                            return bs;
                        i = 1;
                    } else {
                        rec = 0;
                        break;
                    }
                } else if (rec)
                    Delay(100);
            }
        }
    }
    outb(UBRR, 23);
#endif
    return -1;
}

