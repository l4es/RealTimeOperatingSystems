/*
 * Copyright (C) 2001-2003 by egnite Software GmbH
 * Copyright (c) 1993 by Digital Equipment Corporation
 * Copyright (c) 1983, 1993 by The Regents of the University of California
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
 * \file net/inet.c
 * \brief Internet address helper functions.
 *
 * \verbatim
 * $Id: inet.c 4473 2012-08-20 15:12:45Z haraldkipp $
 * \endverbatim
 */

/*!
 * \addtogroup xgIP
 */
/*@{*/

#include <arpa/inet.h>

/*!
 * \brief Convert decimal dotted ASCII representation into numeric IP address.
 *
 * \param str String containing the ASCII representation.
 *
 * \return IP address in network byte order.
 */
uint32_t inet_addr(const char * str)
{
    uint_fast16_t num;
    uint32_t addr = 0;
    uint_fast8_t parts = 0;
    uint8_t *ap;

    ap = (uint8_t *) & addr;
    while (parts < 4) {
        if (*str < '0' || *str > '9')
            break;
        for (num = 0; num <= 255;) {
            num = (num * 10) + (*str - '0');
            if (*++str < '0' || *str > '9')
                break;
        }
        if (num > 255)
            break;
        parts++;
        *ap++ = (uint8_t) num;
        if (*str != '.') {
            if (parts == 4)
                return addr;
            break;
        }
        str++;
    }
    return -1;
}

/*!
 * \brief Convert numeric IP address into decimal dotted
 *        ASCII representation.
 *
 * \note This function is not thread safe. Each subsequent
 *       call will destroy the previous result. Applications
 *       should locally store the result before calling the
 *       function again or allowing other threads to call it.
 *
 * \param addr IP address in network byte order.
 *
 * \return Pointer to a static buffer containing the
 *         ASCII representation.
 */
char *inet_ntoa(uint32_t addr)
{
    static char str[16];
    char inv[3];
    char *rp;
    uint8_t *ap;
    uint8_t rem;
    uint_fast8_t n;
    uint_fast8_t i;

    rp = str;
    ap = (uint8_t *) & addr;
    for (n = 0; n < 4; n++) {
        i = 0;
        do {
            rem = *ap % (uint8_t) 10;
            *ap /= (uint8_t) 10;
            inv[i++] = '0' + rem;
        } while (*ap);
        while (i--)
            *rp++ = inv[i];
        *rp++ = '.';
        ap++;
    }
    *--rp = 0;
    return str;
}

/*!
 * \brief Convert numeric MAC address array into double dotted
 *        ASCII representation.
 *
 * \note This function is not thread safe. Each subsequent
 *       call will destroy the previous result. Applications
 *       should locally store the result before calling the
 *       function again or allowing other threads to call it.
 *
 * \param mac Pointer to MAC address array.
 *
 * \return Pointer to a static buffer containing the
 *         ASCII representation.
 */
char *inet_mtoa(uint8_t *mac)
{
    static char str[18];
    char hex[16]="0123456789ABCDEF";
    int i, p;

    p=0;
    for(i=0;i<6;i++)
    {
        str[p++]=hex[mac[i]>>4];
        str[p++]=hex[mac[i]&0xf];
        if(i<5) str[p++]=':'; else str[p++]='\0';
    }
    return str;
}

/*@}*/
