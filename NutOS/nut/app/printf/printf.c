/*!
 * Copyright (C) 2013 Uwe Bonnes
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
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/version.h>
#include <sys/heap.h>


#include <math.h>

static char *banner = "\nNut/OS Printf Sample on " BOARDNAME 
    " " __DATE__ " " __TIME__ "\n";

int main(void)
{
    int res;
    uint32_t baud = 115200;
    char inbuf[16];

    res = NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    if (res )
        while(10)
              NutSleep(100);

    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    fprintf(stdout, banner);

    for (;;) {
        uint8_t pba = 0xaa, pb5 = 0x55;
        uint64_t pll = 0x0123456789abcdefLL;
        uint64_t plli = 1000000000000LL;
        double pd = 3.14159;
/* first the problematic cases as of SVN r5314
 *
 * - Printing of 64-bit variable and double not implemented
 * - va_list not cleared of full 64-bit argument
 * - on arm va_arg(ap, double) gets wrong data due to alignment issues
 */
        printf("\n\nNut/OS %s\n", NutVersionString());
        printf("%zd bytes free, debug dev: %s\n", NutHeapAvailable(),
               DEV_CONSOLE.dev_name);
        printf("\nExpext pll 0x0123456789abcdef, pba 0xaa, pb5 0x55\n");
        printf("pll %20llx pba 0x%02x pb5 0x%02x\n", pll, pba, pb5);
        printf("pba 0x%02x pll%20llx pb5 0x%02x\n", pba, pll, pb5);

        printf("\nExpext plli 1000000000000\n");
        printf("plli %32lld\n", plli);

        printf("\nExpext pd 3.14159, pba 0xaa, pb5 0x55\n");
        printf("pd %f, pba 0x%02x, pb5 0x%02x\n", pd, pba, pb5);
        printf("pba 0x%02x, pd %f,  pb5 0x%02x\n", pba, pd, pb5);
        puts("\nPress \"Enter\" to repeat output");
        fflush(stdout);
        fgets(inbuf, sizeof(inbuf), stdin);
    }
    return 0;
}
