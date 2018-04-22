/*!
 * Copyright (C) 2013, 2016 Uwe Bonnes
 *                           (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/*!
 * $Id: pioled.c 6534 2016-09-22 14:40:27Z u_bonnes $
 */

/*!
 * \file app/pioled/pioled.c
 * \brief Portable GPIO access
 *
 * This file demonstrates portable macros for PIO access by blinking
 * LED1 and LED2. Most boards should have defined at least LED1 in
 * dev/../board/<BOARD.h>
 */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <dev/gpio.h>
#include <dev/canbus.h>
#include <sys/timer.h>
#include <sys/event.h>
#include <sys/atom.h>

static const char *banner = "\nPIOLED on " BOARDNAME
    " " __DATE__ " " __TIME__"\n";

#if defined(LED1_PORT) && defined(LED1_PIN)
#undef GPIO_ID
#define GPIO_ID LED1_PORT
#include <cfg/arch/piotran.h>
static INLINE void MY_LED1_INIT(void)   { GPIO_INIT(LED1_PIN); }
static INLINE void MY_LED1_OUTPUT(void) { GPIO_OUTPUT(LED1_PIN); }
static INLINE void MY_LED1_SET(void)    { GPIO_SET_HI(LED1_PIN); }
static INLINE void MY_LED1_CLR(void)    { GPIO_SET_LO(LED1_PIN); }
static INLINE int  MY_LED1_GET(void)    { return GPIO_GET(LED1_PIN); }
#else
#define MY_LED1_INIT()
#define MY_LED1_OUTPUT()
#define MY_LED1_SET()
#define MY_LED1_CLR()
#define MY_LED1_GET() 0
#endif

#if defined(LED2_PORT) && defined(LED2_PIN)
#undef GPIO_ID
#define GPIO_ID LED2_PORT
#include <cfg/arch/piotran.h>
static INLINE void MY_LED2_INIT(void)   { GPIO_INIT(LED2_PIN); }
static INLINE void MY_LED2_OUTPUT(void) { GPIO_OUTPUT(LED2_PIN); }
static INLINE void MY_LED2_SET(void)    { GPIO_SET_HI(LED2_PIN); }
static INLINE void MY_LED2_CLR(void)    { GPIO_SET_LO(LED2_PIN); }
static INLINE int  MY_LED2_GET(void)    { return GPIO_GET(LED2_PIN); }
#else
#define MY_LED2_INIT()
#define MY_LED2_OUTPUT()
#define MY_LED2_SET()
#define MY_LED2_CLR()
#define MY_LED2_GET() 0
#endif


int main(void)
{
    uint32_t baud = 115200, read_timeout = 10;
    FILE *uart;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    uart = fopen(DEV_CONSOLE.dev_name, "r+");
    _ioctl(_fileno(uart), UART_SETSPEED, &baud);
    _ioctl(_fileno(uart), UART_SETREADTIMEOUT, &read_timeout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    printf(banner);
#if !defined(LED1_PORT) || !defined(LED1_PIN) ||\
    !defined(LED2_PORT) || !defined(LED2_PIN)
    printf("Some LEDs are not defined for yout board\n");
#endif
    MY_LED1_INIT();
    MY_LED1_OUTPUT();
    MY_LED2_INIT();
    MY_LED2_OUTPUT();
    while(1)
    {
        MY_LED1_SET();
        NutSleep(100);
        MY_LED1_CLR();
        if (MY_LED2_GET())
            MY_LED2_CLR();
        else
            MY_LED2_SET();
        NutSleep(100);
    }
}
