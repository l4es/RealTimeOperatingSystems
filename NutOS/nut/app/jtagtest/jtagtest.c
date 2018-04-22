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

/*!
 * \brief Print IDs of devices connected to the JTAG cable
 *
  */
#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <dev/jtag_tap.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>

static char *banner = "\nNut/OS JTAG Sample " __DATE__ " " __TIME__ "\n";

/* Reimplementation of xc3sprog jtag.cpp: :getChain
 *
 * Shift in set of 32 '0' bits and if we get something back that
 * is not 0 or 0xffff it is a device
 */

int ScanJtagBus(JTAG_TAP *jtag)
{
    int numDevices=0;
    uint32_t idx;
    uint8_t zeros[4] = {0, 0, 0, 0};

    TapStateWalk(jtag, JTAG_SHIFT_DR);
    while(1)
    {
        TapData(jtag, (uint8_t*) &idx, zeros, 32, 0);
        if ((idx != 0) && (idx != 0xffffffff))
        {
            fprintf(stdout, "Found device 0x%08lx\n", idx);
            numDevices++;
        }
        else
            break;
    }
    TapData(jtag, (uint8_t*) &idx, zeros, 1, 1);
    return numDevices;
}

/*
 * JTAG  sample: Scan the Jtag Bus for connected devices
 *
 */
int main(void)
{
    int res;
    uint32_t baud = 115200;
    JTAG_TAP *jtag = 0;

    res = NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    if (res )
        goto error;

    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    fprintf(stdout, banner);

#if !defined(DEF_JTAG_CABLE)
    fprintf(stdout, "Please indicate the JTAG Cable to scan! "
            "Best place is <board>.h\n");
    (void) jtag;
    goto error;
#else
    jtag = TapOpen( &DEF_JTAG_CABLE);
    if (jtag == NULL)
    {
        printf("TapOpen failed\n");
        goto error;
    }
    else
    {
        printf("TapOpen success\n");
    }

    ScanJtagBus(jtag);

    for (;;) {
        char inbuf[128];
        puts("Press \"Enter\" to scan JTAG bus for devices ");
        fflush(stdout);
        fgets(inbuf, sizeof(inbuf), stdin);
        ScanJtagBus(jtag);
    }
    return 0;
#endif

error:
    while(1)
    {
        NutSleep(100);
    }
    return 0;
}
