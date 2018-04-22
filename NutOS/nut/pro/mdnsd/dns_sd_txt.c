/*
 * Copyright (C) 2003 Jeremie Miller <jer@jabber.org>
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

/* This code is based on
 * Based on BSD licensed mdnsd implementation by Jer <jer@jabber.org>
 * http://dotlocal.org/mdnsd/
 *
 * Unfortunately this site is now longer alive. You can still find it at:
 * http://web.archive.org/web/20080705131510/http://dotlocal.org/mdnsd/
 *
 * mdnsd - embeddable Multicast DNS Daemon
 * =======================================
 *
 * "mdnsd" is a very lightweight, simple, portable, and easy to integrate
 * open source implementation of Multicast DNS (part of Zeroconf, also called
 * Rendezvous by Apple) for developers. It supports both acting as a Query and
 * a Responder, allowing any software to participate fully on the .localnetwork
 * just by including a few files and calling a few functions.  All of the
 * complexity of handling the Multicast DNS retransmit timing, duplicate
 * suppression, probing, conflict detection, and other facets of the DNS
 * protocol is hidden behind a very simple and very easy to use interface,
 * described in the header file. The single small c source file has almost no
 * dependencies, and is portable to almost any embedded platform.
 * Multiple example applications and usages are included in the download,
 * including a simple persistent query browser and a tool to advertise .local
 * web sites.
 *
 * The code is licensed under both the GPL and BSD licenses, for use in any
 * free software or commercial application. If there is a licensing need not
 * covered by either of those, alternative licensing is available upon request.
 *
 */


/*!
 * \file pro/dns_sd_txt.c
 * \brief Helper functions to hadle the DNS-SD (Multicast DNS) TXT record format
 *
 * \verbatim
 *
 * $Id$
 *
 * \endverbatim
 */
#include <stdlib.h>
#include <string.h>
#include <pro/mdnsd/dns_sd_txt.h>
#include <pro/mdnsd/mdnsd.h>
#include <gorp/shash.h>

/*!
 * \addtogroup xgMulticastDns
 */
/*@{*/


/*!
 * \brief Calculate the length of one key=value pair
 *
 * \param key   key name
 * \param val   hashed value string
 */
static int DnsSd2TxtLen(const char *key, char *val)
{
    int ret = strlen(key);

    if(*val == 0) {
        return ret;
    }

    ret += strlen(val);
    ret++;
    return ret;
}


/*!
 * \brief Count the length of hashedd key=value pairs
 *
 * Callback routine called by SHashForEach() in DnsSd2Txt()
 *
 * \param hash  The hash table
 * \param key   key name
 * \param val   hashed value string
 * \param arg   pointer to the count variable
 */
static void DnsSd2TxtCount_CB(SHASH UNUSED(hash), const char *key, void *val, void *arg)
{
    int *count = (int*)arg;
    *count += DnsSd2TxtLen(key, (char*)val) + 1;
}


/*!
 * \brief Append hashed key=value pairs to the text buffer
 *
 * Callback routine called by SHashForEach() in DnsSd2Txt()
 *
 * \param hash  The hash table
 * \param key   key name
 * \param val   hashed value string
 * \param arg   pointer to the text buffer, where we want to append the value of the hashed entry to
 */
static void DnsSd2TxtWrite_CB(SHASH UNUSED(hash), const char *key, void *val, void *arg)
{
    unsigned char **txtp = (unsigned char **)arg;
    char *cval = (char*)val;

    /* Copy the length... */
    **txtp = DnsSd2TxtLen(key, (char*)val);
    (*txtp)++;

    memcpy(*txtp, key, strlen(key));
    *txtp += strlen(key);

    if(*cval == 0) {
        return;
    }

    /* ...and then the strings. */
    **txtp = '=';
    (*txtp)++;

    memcpy(*txtp, cval, strlen(cval));
    *txtp += strlen(cval);
}


/*!
 * \brief Returns a raw block of data that can be send with a SD TXT record and also
          sets length.
 *
 * \param hash  The hash table from which the SD TXT record data shall be generated from
 * \param len   Call by reference: Returns the length of the raw data record
 *
 * \return      The newly allocated and filled hash table
 */
unsigned char *DnsSd2Txt(SHASH hash, int *len)
{
    unsigned char *buf, *raw;
    *len = 0;

    SHashForEach(hash, DnsSd2TxtCount_CB, (void*)len);
    if(*len == 0) {
        *len = 1;
        buf = (unsigned char *)malloc(1);
        *buf = 0;
        return buf;
    }
    buf = (unsigned char *)malloc(*len);
    raw = buf;

    SHashForEach(hash, DnsSd2TxtWrite_CB, &buf);
    return raw;
}


/*!
 * \brief Returns a hashtable of strings of the raw SD TXT record rdata
 *
 * \param txt   The SD TXT record data
 * \param len   Length of the data
 *
 * \return      The newly allocated and filled hash table
 */
SHASH DnsTxt2Sd(unsigned char *txt, int len)
{
    char key[256], *val;
    SHASH hash = NULL;

    if ((txt == 0) || (len == 0) || (*txt == 0)) {
        return 0;
    }

    hash = SHashInit(23);

    /* Loop through the data, break out each block and store it into the hash table */
    for(; (*txt <= len) && (len > 0); len -= *txt, txt += *txt + 1) {
        if(*txt == 0) {
            break;
        }
        memcpy(key, txt+1, *txt);
        key[*txt] = 0;

        val = strchr(key, '=');
        if (val != NULL) {
            *val = 0;
            val++;
        }

        SHashStore(hash, key, strlen(key), val, val ? strlen(val) : 0);
    }
    return hash;
}

/*@}*/
