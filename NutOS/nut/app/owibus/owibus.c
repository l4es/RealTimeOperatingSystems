/*
 * Copyright (C) 2012, 2013, 2016 Uwe Bonnes.
 *                               (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * OR TORT (INCLUDING NEGLIGENce OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*!
 * \file app/owibus/owibus.c
 * \brief Example access to a DS18B20.
 *
 * The Onewire device needs to be connected to a port pin with pull-up
 * or the uart with TX low driving RX low and TX high not touching RX
 *
 * STM32 UART can do that by itself, with RX connected internal to TX
 * and TX driven Tristate
 *
 * \verbatim
 */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <cfg/arch.h>
#include <stdint.h>
#include <sys/timer.h>
#include <dev/board.h>
#include <dev/gpio.h>

#if !defined(DEF_OWIBUS)

int main(void)
{
    uint32_t baud = 115200;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
#if !defined(__GNUC__)
    puts("This program requires a compiler that supports 64-bit integers.");
#else
    puts("This program requires a configured OWIBUS.");
#endif
    for (;;);
    return 0;
}

#else

#include <dev/owibus.h>

static char *banner = "\nNut/OS OWI Bus on " BOARDNAME
    " "__DATE__ " " __TIME__"\n";

/* Maximum number of devices we scan for. */
#define N_MAX 8
static uint8_t hid[N_MAX][8]  = { 0 };
static int num_devices;
/*
 * UART sample.
 *
 */
int main(void)
{

    uint32_t baud = 115200;
    FILE *uart;
    int res, i = 0;
    int run =0;
    uint8_t raw[9];
    uint8_t diff;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);

    uart = fopen(DEV_CONSOLE.dev_name, "r+");

    _ioctl(_fileno(uart), UART_SETSPEED, &baud);

    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    fprintf(stdout, banner);


    res = OwiInit(&DEF_OWIBUS);
    if(res)
    {
        printf("Owi Error %d\n", res);
        while(1)
            NutSleep(10);
    }
    diff = OWI_SEARCH_FIRST;
    i = 0;
    do {
        res = OwiRomSearch(&DEF_OWIBUS, &diff,
                           (i) ? hid[i - 1] : hid[0], hid[i]);
        if(res)
        {
            printf("OwiRomSearch failed\n");
            while(1)
                NutSleep(10);
        }
        fprintf(stdout,
                "Hardware ID of device %d: %02x%02x%02x%02x%02x%02x%02x%02x\n",
                i,
                hid[i][7], hid[i][6], hid[i][5], hid[i][4],
                hid[i][3], hid[i][2], hid[i][1], hid[i][0]);
        i++;
    } while ((i < N_MAX) && (diff != OWI_LAST_DEVICE));
    num_devices = i;
    while(1)
    {
        res = OwiCommand(&DEF_OWIBUS, OWI_CONVERT_T, NULL);
        if (res)
            printf("OwiCommand convert_t error %d\n", res);
        NutSleep(1000);
        printf("%4d: ", run++);
        for (i = 0; i < num_devices; i++) {
            int status = 0;
            int position = 0;
            int j = 0;
            int32_t xcelsius = 0;
            res = OwiCommand(&DEF_OWIBUS, OWI_READ, hid[i]);
            if (res) {
                printf("Device %d: OwiCommand read error %d\n", i, res);
            }
            do {
                res = OwiReadBlock(&DEF_OWIBUS, raw, 9 * 8);
                j++;
            } while ((res == OWI_CRC_ERROR) && (j < 10));
            if (res) {
                printf("Device %d: OwiReadBlock error %d\n", i, res);
            }
            switch (hid[i][0]) {
            case W1_THERM_DS18B20:
                xcelsius = (raw[1] & 0x80)? 0xffff0000: 0;
                xcelsius |= ((raw[1] << 8) | raw[0]);
                xcelsius *= 625;
                break;
            case W1_THERM_DS18S20:
                xcelsius = (raw[1] & 0x80)? 0xffff0000: 0;
                xcelsius |= ((raw[1] << 8) | raw[0]);
                xcelsius *= 5000;
                break;
            case W1_THERM_DS1825:
                xcelsius = (raw[3] & 0x80)? 0xfffff000: 0;
                xcelsius |= ((raw[3] << 4) | (raw[2] >> 4));
                xcelsius *= 625;
                status = raw[2] & 7;
                position = raw[4] & 0xf;
                break;
            }
            printf(" P%02d:", position);
            switch (status) {
            case 0:
                printf(" Okay");
                break;
            case 1:
                printf(" Open");
                break;
            case 2:
                printf(" SGND");
                break;
            case 3:
                printf(" SVDD");
                break;
            default:
                printf(" MULT");
            }
            printf("%4ld.%04ld ",
                   xcelsius / 10000,
                   ((xcelsius < 0) ? -xcelsius : xcelsius) % 10000);
        }
        putchar('\n');
    }
    return 0;
}

#endif
