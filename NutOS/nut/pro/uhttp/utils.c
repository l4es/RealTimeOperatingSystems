/*
 * Copyright (C) 2012 by egnite GmbH
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
 * $Id$
 */

#include <pro/uhttp/utils.h>
#include <memdebug.h>

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

void HttpUrlUnescape(char *str)
{
    char *ptr1, *ptr2, ch;
    char hexstr[3] = { 0, 0, 0 };
    for (ptr1 = ptr2 = str; *ptr1; ptr1++) {
        if (*ptr1 == '+')
            *ptr2++ = ' ';
        else if (*ptr1 == '%') {
            hexstr[0] = ptr1[1];
            hexstr[1] = ptr1[2];
            ch = (char) strtol(hexstr, 0, 16);
            *ptr2++ = ch;
            ptr1 += 2;
        } else
            *ptr2++ = *ptr1;
    }
    *ptr2 = 0;
}

static int DecodeHex(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

char *UriUnescape(char *path)
{
    int x1;
    int x2;
    char *src = path;
    char *dst = path;
    char last = *path;

    if (last != '/')
        return 0;
    while (*++src) {
        if (*src == '%' &&
            (x1 = DecodeHex(*(src + 1))) >= 0 &&
            (x2 = DecodeHex(*(src + 2))) >= 0) {
            src += 2;
            if ((*src = x1 * 16 + x2) == 0)
                break;
        }
        if (*src == '\\')
            *src = '/';
        if ((last != '.' && last != '/') || (*src != '.' && *src != '/'))
            *dst++ = last = *src;

    }
    *dst = 0;

    return path;
}

static const char base64dtab[96] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63, /* 32..47 */
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1, /* 48..63 */
    -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, /* 64..79 */
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1, /* 80..95 */
    -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, /* 96..111 */
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1, /* 112..127 */
};

char *HttpDecodeBase64(char *str)
{
    char code;
    char *sp;
    char *tp;
    char last = -1;
    char step = 0;

    for (tp = sp = str; *sp; ++sp) {
        if (*sp < 32)
            continue;
        if ((code = base64dtab[(int) *sp - 32]) == (char)-1)
            continue;
        switch (step++) {
        case 1:
            *tp++ = ((last << 2) | ((code & 0x30) >> 4));
            break;
        case 2:
            *tp++ = (((last & 0xf) << 4) | ((code & 0x3c) >> 2));
            break;
        case 3:
            *tp++ = (((last & 0x03) << 6) | code);
            step = 0;
            break;
        }
        last = code;
    }
    *tp = 0;
    return str;
}

char *AllocConcatStrings(const char *str, ...)
{
    va_list ap;
    int len;
    char *cp;
    char *rp;

    va_start(ap, str);
    for (len = strlen(str); (cp = va_arg(ap, char *)) != NULL; len += strlen(cp));
    va_end(ap);
    rp = malloc(len + 1);
    if (rp) {
        va_start(ap, str);
        for (strcpy(rp, str); (cp = va_arg(ap, char *)) != NULL; strcat(rp, cp));
        va_end(ap);
    }
    return rp;
}

char *AllocConcatStringLen(const char *str1, const char *str2, int len2)
{
    int len1 = strlen(str1);
    char *rp = malloc(len1 + len2 + 1);

    if (rp) {
        strcpy(rp, str1);
        strncpy(rp + len1, str2, len2);
        *(rp + len1 + len2) = '\0';
    }
    return rp;
}
