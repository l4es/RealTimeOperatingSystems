/*!
 * Copyright (C) 2001-2003 by egnite Software GmbH
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

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <dev/gpio.h>
#include <dev/usb_stm32/stm32_otg.h>


static char *banner = "\nNut/OS USB Sample " __DATE__ " " __TIME__ "\n";
static char *pgm_ptr = "\nHello stranger and our/my friends!\n";

static char inbuf[128];

extern NUTDEVICE devStm32Otg;

FILE *usb;
#if defined(LED2_PORT) && defined( LED2_PIN)
THREAD(LedBlink, arg)
{
    GpioPinConfigSet( LED2_PORT, LED2_PIN, GPIO_CFG_OUTPUT);
    for(;;)
    {
        NutSleep(100);
        GpioPinSetHigh(LED2_PORT, LED2_PIN);
        NutSleep(100);
        GpioPinSetLow(LED2_PORT, LED2_PIN);
    }
}
#endif

/*
 * USB sample.
 *
 */
int main(void)
{
    int res;
    char *cp;
    uint32_t baud = 9600;

    res = NutRegisterDevice(&DEV_UART, 0, 0);
    freopen(DEV_UART_NAME, "w", stdout);
    freopen(DEV_UART_NAME, "r", stdin);

    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    printf(banner);

#if defined(LED1_PORT) && defined( LED1_PIN)
    GpioPinConfigSet( LED1_PORT, LED1_PIN, GPIO_CFG_OUTPUT);
#endif

    if (NutThreadCreate("t2", LedBlink, 0, 512)== 0)
        printf("Can't create LED thread\n");
    else
        printf("LED thread started\n");
    printf("Trying register OTG");
    res = NutRegisterDevice(&devStm32Otg, 0,0);
    if(res)
        printf(" failure\n");
    else
        printf(" success\n");

    usb = fopen("usb_otg", "r+");
    if(usb == 0)
        printf("Can't open OTG\n");
    else
        printf("OTH open Success\n");

    NutSleep(1000); /* Sleep sufficent or next print will stall */
    fprintf(usb, banner);
    /*
     * Nut/OS never expects a thread to return. So we enter an
     * endless loop here.
     */
    for (;;) {
#if defined(LED1_PORT) && defined( LED1_PIN)
        GpioPinSet(LED1_PORT, LED1_PIN,
                   !(GpioPinGet(LED1_PORT, LED1_PIN)));
#endif
        /*
         * A bit more advanced input routine is able to read a string
         * up to and including the first newline character or until a
         * specified maximum number of characters, whichever comes first.
         */
        puts("\nEnter your name: ");
        fflush(stdout);
        fgets(inbuf, sizeof(inbuf), stdin);

        /*
         * Chop off trailing linefeed.
         */
        cp = strchr(inbuf, '\n');
        if (cp)
            *cp = 0;

        /*
         * Streams support formatted output as well as printing strings
         * from program space.
         */
        if (inbuf[0])
            printf("\nHello %s!\n", inbuf);
        else {
            puts(pgm_ptr);
        }
        fprintf(usb, "Bla\n");

    }
    return 0;
}
