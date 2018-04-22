/*
 * Copyright (C) 2009 by Rittal GmbH & Co. KG,
 * Ulrich Prinz <prinz.u@rittal.de>
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
 * $Log$
 *
 * Revision 1.0  2009/04/13 ulrichprinz
 * First checkin, driver for PCA9555 I2C I/O-Expander (currently SAM7X256 is
 * tested only)
 *
 */

/*!
 * \example ioexpander/ioexpander.c
 */

#include <stdio.h>
#include <io.h>

#include <cfg/arch.h>
#include <dev/board.h>
#include <dev/gpio.h>

#include <sys/thread.h>
#include <sys/timer.h>
#include <dev/pca9555.h>
#include <dev/led.h>

#if !defined(MCU_AT91) || defined(MCU_AT91R40008) || defined(MCU_GBA)

int main(void)
{
    uint32_t baud = 115200;

    NutRegisterDevice(&DEV_DEBUG, 0, 0);
    freopen(DEV_DEBUG_NAME, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    puts("TwMasterReg functions are not available on your platform.");
    for (;;);
}

#else

#define KEY1 (1<<0)
#define KEY2 (1<<1)
#define KEY3 (1<<2)
#define KEY4 (1<<3)

/******************************************************************/
THREAD(Thread1, arg)
/******************************************************************/
{
    HANDLE ds1, ds2, ds3;
    int ledmask = 1;

    if( NutRegisterLed( &ds1, IOXP_PORT1, 0) == 0)
        printf( "register LED 1 OK\n");
    if( NutRegisterLed( &ds2, IOXP_PORT1, 1) == 0)
        printf( "register LED 2 OK\n");
    if( NutRegisterLed( &ds3, IOXP_PORT1, 2) == 0)
        printf( "register LED 3 OK\n");

    NutThreadSetPriority(128);
    for (;;) {
        NutSetLed( ds1, (ledmask>>0) & 1, 0, 0);
        NutSetLed( ds2, (ledmask>>1) & 1, 0, 0);
        NutSetLed( ds3, (ledmask>>2) & 1, 0, 0);

        ledmask <<= 1;
        if( ledmask & (1<<3)) ledmask = 1;

        NutSleep(250);
    }
}

/******************************************************************/
THREAD(Thread2, arg)
/******************************************************************/
{
    uint8_t key, oldkey;
    uint8_t flag = 1;
    int rc;
    HANDLE led3;

    printf( "Key and LED test for PCA9555\n" );

    if( NutRegisterLed( &led3, IOXP_PORT1, 3) == 0)
        printf( "register LED 4 OK\n");

    oldkey = ~key;

    NutThreadSetPriority(128);
    for (;;)
    {
        key = 0;
        rc = IOExpRawRead( 0, &key);
        if( key != oldkey) {
            if( key > oldkey) {
                /* flash led if key is pressed */
                NutSetLed( led3, LED_ON, 200, 0);
            }

            oldkey = key;
            printf( "IOER rc=%d key=0x%02x\n", rc, key);
            if( rc >= 0)
            {
                if( flag == 0 )
                {
                    flag = 1;

                    if( key & KEY1) printf( "Key 1 pressed\n" );
                    if( key & KEY2) printf( "Key 2 pressed\n" );
                    if( key & KEY3) printf( "Key 3 pressed\n" );
                    if( key & KEY4) printf( "Key 4 pressed\n" );
                }
            }
            else
            {
                flag = 0;
            }
        }
        NutSleep(125);
    }
}

/******************************************************************/
int main(void)
/******************************************************************/
{
    uint32_t baud = 115200;
    HANDLE led4;
    /*
     * Register the UART device, open it, assign stdout to it and set
     * the baudrate.
     */
    NutRegisterDevice(&DEV_DEBUG, 0, 0);
    freopen(DEV_DEBUG_NAME, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    printf("Init TWI... ");
    baud = 400000;
    if( TwInit( 0 ) == 0) /* par = slave address but we are master */
        printf( "OK\n");
    else
        printf( "FAIL\n");
    TwIOCtl( TWI_SETSPEED, &baud);

    printf("Init PCA9555... ");
    if( IOExpInit() == 0)
        printf( "OK\n");
    else
        printf( "FAIL\n");

    if( NutRegisterLed( &led4, IOXP_PORT1, 4) == 0)
        printf( "register LED B OK\n");
    NutSetLed( led4, LED_BLINK, 100, 100);

    /*
     * Start two additional threads. All threads are started with
     * priority 64.
     */
    NutThreadCreate("led", Thread1, 0, 512);
    NutThreadCreate("key", Thread2, 0, 512);

    /*
     * Endless loop in main thread.
     */
    for (;;)
    {
        NutSleep(5000);
    }
    return 0;
}

#endif /* MCU_AT91 */
