/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG GmbH. All rights reserved.
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
 * \verbatim
 * $Id: eeprom_test.c 3108 2010-09-15 21:11:15Z Astralix $
 * \endverbatim
 */

/*!
 * \example eeprom/eeprom.c
 *
 * Test application for new AT24C EEPROM driver.
 * (currently SAM7X256 and STM32F is tested only).
 *
 */

/* Test */
#include <stdio.h>
#include <string.h>
#include <io.h>
#include <dev/gpio.h>

#include <cfg/arch.h>
#include <dev/board.h>

#include <sys/thread.h>
#include <sys/timer.h>
#include <dev/twif.h>
#include <dev/eeprom.h>

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

void HexDump( char *rxb, uint16_t len )
{
    uint16_t i;
    uint16_t f = 1;
    char ascii[17];

    for( i = 0; i < len; i++ )
    {
        if( f == 1 )
        {
            f = 0;
            printf ( "%04x : ", i );
        }
        printf( "%02X ", rxb[i] );
        ascii[ i % 16] = ( rxb[i] > 31 && rxb[i] < 128 ) ? rxb[i] : '.';

        if((( i + 1) % 16 ) == 0 )
        {
            f = 1;
            ascii[16] = 0;
            printf( "| %s\n", ascii );
        }
    }
}

const char teststr[64] = { "This is an ultimatly long string that reaches 61 bytes length\0"};
const char failstr[] = "\033[1;37;41m\033[K*** FAIL rc=%d ***\033[0m\n";
const char infostr[] = "\n\033[30;46m\033[K%s\033[0m\n";
/*
 * Main application thread.
 */
int main(void)
{

    char txBuffer[128];
    char rxBuffer[128];
    uint32_t baud = 115200;
    int rc = 0;

    /*
     * Register the UART device, open it, assign stdout to it and set
     * the baudrate.
     */
    NutRegisterDevice(&DEV_DEBUG, 0, 0);
    freopen(DEV_DEBUG_NAME, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    /* Clear terminal screen and cursor home */
    printf( "\033[2J\033[H");
    printf( "\033[1;37;44m\033[K*** EEPROM Test ***\033[0m\n");

    baud = 100000;
    TwInit( 0 ); /* par = slave address but we are master */
    TwIOCtl( TWI_SETSPEED, &baud);

    EEInit();

    /*
     * Start two additional threads. All threads are started with
     * priority 64.
     */
    // NutThreadCreate("Led", Led, 0, 512);


    /****************************** 1st part *******************************/
    /*       Before starting the test, fill the EEPROM with 0xFF           */
    /***********************************************************************/

    /* Clear the Receive buffer */
    memset( rxBuffer, 0x00, sizeof( rxBuffer));
    memset( txBuffer, 0xff, sizeof( txBuffer));

    printf( infostr, "*** Read ***" );
    rc = EEReadData( 0x0100, rxBuffer, 64);
    HexDump( rxBuffer, 64 );
    if( rc) {
        printf(failstr, rc);
        printf("Please make sure that you have an EEPROM connected on your board.\n");
        while(1) NutSleep(1000);
    }

    if( !strncmp( rxBuffer, teststr, strlen( teststr)))
        printf( "Test successfull, data is equal!\n");
    else
        printf( "Test failed, data not equal!\n");

    printf( infostr,"Init: Fill 0xFF" );
    rc = EEWriteData( 0x0100, txBuffer, 64);
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    rc = EEReadData( 0x0100, rxBuffer, 64);
    HexDump( rxBuffer, 64 );
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    /*****************/

    printf( infostr, "Step 1" );
    strcpy( (char*)txBuffer, "First, " );
    rc = EEWriteData( 0x0100+24, txBuffer, strlen( (char*)txBuffer ));
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    rc = EEReadData( 0x0100, rxBuffer, 64);
    HexDump( rxBuffer, 64 );
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    /*****************/

    printf( infostr, "Step 2" );
    strcpy( txBuffer, (const char*)"Third!" );
    rc = EEWriteData( 0x0100+24+16, (const char*)txBuffer, strlen( txBuffer ));
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    rc = EEReadData( 0x0100, rxBuffer, 64);
    HexDump( rxBuffer, 64 );
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    /*****************/

    printf( infostr, "Step 3" );
    strcpy( txBuffer, "Second, " );
    rc = EEWriteData( 0x0100+24+8, txBuffer, strlen( txBuffer ));
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    rc = EEReadData( 0x0100, rxBuffer, 64);
    HexDump( rxBuffer, 64 );
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    /*****************/

    printf( infostr, "Step 4" );
    rc = EEWriteData( 0x0100, (void*)teststr, strlen( teststr));
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    rc = EEReadData( 0x0100, rxBuffer, 64);
    HexDump( rxBuffer, 64 );
    if( rc) {
        printf(failstr, rc);
        while(1) NutSleep(1000);
    }

    printf( "\n\033[30;34m\033[K*** END ***\033[0m\n" );

    /*
     * Endless loop in main thread.
     */
    for (;;)
    {
        NutSleep(1000);
    }
    return 0;
}

#endif

