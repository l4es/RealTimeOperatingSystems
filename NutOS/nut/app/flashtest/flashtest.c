/*!
 * Copyright (C) 2001-2003 by egnite Software GmbH
 * Copyright (C) 2013, 2016-17 Uwe Bonnes
 *                              (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: flashtest.c 6609 2017-02-15 15:36:45Z u_bonnes $
 */

/*!
 * \example uart/uart.c
 *
 * This sample demonstrates the usage of the parameter memory.
 */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <dev/iap_flash.h>
#include <dev/nvmem.h>
#include <sys/timer.h>

static char *banner = "\nNut/OS Flash Sample on " BOARDNAME " " __DATE__ " " __TIME__"\n";
static char *pattern =  "0123456789abcdef0123456789ABCDEF";
static char *pattern1 = "FEDCBA9876543210fedcba987654321";
static char *pattern2 = "0123456789abcdef0123456789ABCD";
static char *pattern3 = "abcdef0123456789abcdef0123456";
static const char  pattern4 = 0x55;
static const uint32_t cafebabe = 0xCAFEBABE;

static void Compare(char *wr, size_t len)
{
    char rd[8];
    NutNvMemLoad(0x0, rd, 5);
    if (strncmp(rd, wr, 5)) {
        printf("NutNvMemSave compare failed: %5s vs %s\n", rd, wr);
    }
}

/*
 * Test for the IAP flash and configuration storage API.
 *
 */
int main(void)
{
    int res;
    uint32_t baud = 115200, read_timeout = 10;
    FILE *uart;
    char buffer[7];
    size_t iap_flash_end = IapFlashEnd();
    void *dword_aligned_end;
    uint32_t rd;
    int i;
    uint32_t *rptr;
    size_t user_area;
    uint64_t deadbeef_ll = 0xdeadbeef01234567LL;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);

    uart = fopen(DEV_CONSOLE.dev_name, "r+");
    _ioctl(_fileno(uart), UART_SETSPEED, &baud);
    _ioctl(_fileno(uart), UART_SETREADTIMEOUT, &read_timeout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    printf(banner);
    NutSleep(10);
    strcpy(buffer, "Save_");
    res = NutNvMemSave(0x0, buffer, 5);
    if(res)
        printf("NutNvMemSave failed: %d\n", res);
    else {
        Compare(buffer, 5);
        strcpy(buffer, "Save1");
        res = NutNvMemSave(0x0, buffer, 5);
        if(res) {
            printf("NutNvMemSave failed: %d\n", res);
        }
        Compare(buffer, 5);
        strcpy(buffer, "Save2");
        res = NutNvMemSave(0x0, buffer, 5);
        if(res) {
            printf("NutNvMemSave failed: %d\n", res);
        }
        Compare(buffer, 5);
        res = NutNvMemSave(0x0, "u", 1);
        buffer[0] = 'u';
        if(res) {
            printf("NutNvMemSave failed: %d\n", res);
        }
        Compare(buffer, 5);
        res = NutNvMemSave(0x1, "x", 1);
        buffer[1] = 'x';
        if(res) {
            printf("NutNvMemSave failed: %d\n", res);
        }
        Compare(buffer, 5);
    }
    printf("NutNvMem test done\n");

    printf("Free Flash from 0x%08zx to 0x%08zx\n",
           IapProgramEnd(), iap_flash_end);
    printf("Last 128 bytes in user space");
    user_area = (iap_flash_end & ~0x7f);
    rptr = (uint32_t *)user_area;
    for (i = 0; i < 128 / 4; i++) {
        if (rptr[i] !=  FLASH_ERASED_PATTERN32) {
            printf(" Not");
            /* User area already programmed. Change patter so that
             * erase is needed.
             */
            if(*(uint64_t*)(user_area) == deadbeef_ll)
                deadbeef_ll |=  FLASH_ERASED_PATTERN32;
            break;
        }
    }
    printf(" Empty\n");
    printf("Write uint64_t to erased flash: ");
    res = IapFlashWrite((void*)user_area, &deadbeef_ll,
                        8, FLASH_ERASE_ALWAYS);
    if (res != FLASH_COMPLETE) {
        printf("Failed: res %d\n", res);
    } else {
        if (*(uint64_t*)(user_area) != deadbeef_ll) {
            puts("Compare failed");
        } else {
            puts("Succeeded");
        }
    }

    user_area += 8;
    printf("Write uint32_t to erased flash: ");
    res = IapFlashWrite((void*)user_area, &cafebabe,
                        4, FLASH_ERASE_ALWAYS);
    if (res != FLASH_COMPLETE) {
        if (res == FLASH_ERR_ALIGNMENT){
            puts("Device only supports full 8-byte writes. Aborting");
            goto end;
        }
        printf("Failed: res %d\n", res);
    } else {
        if (*(uint32_t*)(user_area) != cafebabe) {
            puts("Compare failed");
        } else {
            puts("Succeeded");
        }
    }

    memset(buffer, FLASH_ERASED_PATTERN32 & 0xff, sizeof(buffer));

    FlashUntouch();
    printf("Write to erased flash\n");
    res = IapFlashWrite((void*)(iap_flash_end ), &pattern4,
                        1, FLASH_ERASE_FIRST_TOUCH);
    printf("%40s %3d: ", "0x55 at (address & 3 == 3). Res", res);
    rd = *(uint32_t*)(iap_flash_end & ~3);
    printf("0x%08lx\n", rd);
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end - 5), &pattern4,
                        1, FLASH_ERASE_FIRST_TOUCH);
    printf("%40s %3d: ", "0x55 at (address & 3 == 2). Res", res);
    rd = *(uint32_t*)(iap_flash_end &  ~7);
    printf("0x%08lx\n", rd);
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end - 10), &pattern4,
                        1, FLASH_ERASE_FIRST_TOUCH);
    printf("%40s %3d: ", "0x55 at (address & 3 == 1). Res", res);
    rd = *(uint32_t*)((iap_flash_end - 8) & ~3);
    printf("0x%08lx\n", rd);
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end - 15), &pattern4,
                        1, FLASH_ERASE_FIRST_TOUCH);
    printf("%40s %3d: ", "0x55 at (address & 3 == 0). Res", res);
    rd = *(uint32_t*)(iap_flash_end &  ~15);
    printf("0x%08lx\n", rd);
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end -0x3f), pattern1,
                        strlen(pattern1) + 1, FLASH_ERASE_FIRST_TOUCH);
    (void) pattern1;
    printf("%40s %3d: ", "Up to (address & 3 == 3). Res", res);
    if (*(uint32_t*)(iap_flash_end -0x3f) !=  FLASH_ERASED_PATTERN32) {
        printf((char*)(iap_flash_end -0x3f));
    }
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end -0x7f), pattern2,
                        strlen(pattern2) + 1, FLASH_ERASE_FIRST_TOUCH);
    (void) pattern2;
    printf("%40s %3d: ", "Up to (address & 3 == 2). Res", res);
    if (*(uint32_t*)(iap_flash_end -0x7f) !=  FLASH_ERASED_PATTERN32) {
        printf((char*)(iap_flash_end -0x7f));
    }
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end -0xcf), pattern3,
                        strlen(pattern2) + 1, FLASH_ERASE_FIRST_TOUCH);
    (void) pattern3;
    printf("%40s %3d: ", "Up to (address & 3 == 1). Res", res);
    if (*(uint32_t*)(iap_flash_end -0xcf) !=  FLASH_ERASED_PATTERN32) {
        printf((char*)(iap_flash_end -0xcf));
    }
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite((void*)(iap_flash_end -0xff), pattern,
                        strlen(pattern) + 1, FLASH_ERASE_FIRST_TOUCH);
    printf("%40s %3d: ", "Up to (address & 3 == 0). Res", res);
    printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    dword_aligned_end = (void*)((iap_flash_end - 0xff + strlen(pattern))
                                & ~0xf);
    (void) dword_aligned_end;
    memset(buffer, 0, sizeof(buffer));
    printf("Write NULL byte/halfword/word to programmed flash\n");
    res = IapFlashWrite(dword_aligned_end - 1, buffer, 1,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00 at (address & 3 == 3). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    NutSleep(10);
    printf("\n");

    res = IapFlashWrite(dword_aligned_end - 2, buffer, 1,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00 at (address & 3 == 2). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    NutSleep(10);
    printf("\n");

    res = IapFlashWrite(dword_aligned_end - 3, buffer, 1,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00 at (address & 3 == 1). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 4, buffer, 1,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00 at (address & 3 == 0). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 6, buffer, 2,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x0000 at (address & 3 == 2). Res",
           res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 8, buffer, 2,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x0000 at (address & 3 == 0). Res",
           res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 11, buffer, 2,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x0000 at (address & 3 == 1). Res",
           res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 13, buffer, 2,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x0000 at (address & 3 == 3). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 17, buffer, 4,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00000000 at (address & 3 == 3). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 22, buffer, 4,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00000000 at (address & 3 == 2). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 27, buffer, 4,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00000000 at (address & 3 == 1). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

    res = IapFlashWrite(dword_aligned_end - 32, buffer, 4,
                        FLASH_ERASE_NEVER);
    printf("%40s %3d: ", "0x00000000 at (address & 3 == 0). Res", res);
    if ((res == 0) || (res == FLASH_COMPARE))
        printf((char*)(iap_flash_end -0xff));
    printf("\n");
    NutSleep(10);

end:
    for (;;) {
        NutSleep(100);
    }
    return 0;
}
