/*!
 * Copyright (C) 2011-2012 Uwe Bonnes
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
#if defined(DEF_TWIBUS)
#include <dev/twif.h>
#endif
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <dev/gpio.h>


static const char *banner = "\nNut/OS TW Sample " __DATE__ " " __TIME__;

#if defined(DEF_TWIBUS)
static char inbuf[128];
#endif

#if defined(LED1_PORT) && defined( LED1_PIN)
#define LED1_INIT GpioPinConfigSet( LED1_PORT, LED1_PIN, GPIO_CFG_OUTPUT)
#define LED1_TOGGLE  GpioPinSet(LED1_PORT, LED1_PIN, \
                   !(GpioPinGet(LED1_PORT, LED1_PIN)))
#else
#define LED1_INIT
#define LED1_TOGGLE
#endif

#if defined(LED2_PORT) && defined( LED2_PIN)
#define LED2_START_THREAD                           \
    if (NutThreadCreate("t2", LedBlink, 0, 512)== 0)\
        printf("Can't create LED thread\n");        \
    else                                            \
        printf("LED thread started\n");

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
#else
#define LED2_START_THREAD
#endif

void Hardware_Init(void)
{
#if defined(F4_DISCOVERY)
/* Set all CS43L22 I2C relates pins to a safe state*/
    GpioPinConfigSet(NUTGPIO_PORTE, 3, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTE, 3);
    GpioPinConfigSet(NUTGPIO_PORTC, 7, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 7);
    GpioPinConfigSet(NUTGPIO_PORTC, 10, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 10);
    GpioPinConfigSet(NUTGPIO_PORTC, 12, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 12);
    GpioPinConfigSet(NUTGPIO_PORTC, 14, GPIO_CFG_OUTPUT);
    GpioPinSetHigh(NUTGPIO_PORTC, 14);
    GpioPinConfigSet(AUDIO_RST_PORT, AUDIO_RST_PIN, GPIO_CFG_OUTPUT);
    GpioPinSetLow(AUDIO_RST_PORT, AUDIO_RST_PIN);
    NutSleep(11);
    GpioPinSetHigh(AUDIO_RST_PORT, AUDIO_RST_PIN);
    NutSleep(10);
#endif
}

int ScanBus(uint32_t tmo)
{
    int res = 1;
#if defined(DEF_TWIBUS)
    uint8_t a;

    /* Don't do a general call (a == 0) */
    for(a=1; a <80; a++)
    {
        if ((NutTwiMasterTranceive(&DEF_TWIBUS, a, NULL, 0, NULL, 0, tmo))
            != -1)
        {
            printf("Found I2C device at 0x%02x\n", a);
            res = 0;
        }
    }
#endif
    return res;
}

/*
 * TWI sample: Scan the TWI Bus and look for connected devices
 *
 * Some functions do not work with ICCAVR.
 */
int main(void)
{
    int res;
    uint32_t baud = 115200;
#if defined(DEF_TWIBUS)
    uint32_t tmo;
#endif

    res = NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    if (res )
        goto error;

    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    puts(banner);

    LED1_INIT;
    LED2_START_THREAD;
    Hardware_Init();

#if !defined(DEF_TWIBUS)
    puts("Please indicate the TWI Bus to scan!");
    goto error;
#else
    res = NutRegisterTwiBus( &DEF_TWIBUS, 0);
    if (res !=0)
    {
        printf("NutRegisterTwiBus failed\n");
        goto error;
    }
    else
    {
        printf("NutRegisterTwiBus success\n");
    }

    baud = 100000;
    res = NutTwiIOCtl(&DEF_TWIBUS, TWI_SETSPEED, &baud);
    if (res !=0)
    {
        printf("Can't set speed TWI\n");
        tmo = 1;
    }
    else
    {
        NutTwiIOCtl(&DEF_TWIBUS, TWI_GETSPEED, &baud);
        printf("TWI speed is %ld. ", baud );
        /* Waiting for bus free often takes 1 ms */
        tmo = 20000/baud + 2;
        printf("Using %ld ms as Timeout\n", tmo);
    }

    for (;;) {
        ScanBus(tmo);
        LED1_TOGGLE;
        puts("Press \"Enter\" to scan I2C bus for devices ");
        fgets(inbuf, sizeof(inbuf), stdin);
    }
    return 0;
#endif

error:
    while(1)
    {
        LED1_TOGGLE;
        NutSleep(100);
    }
    return 0;
}
