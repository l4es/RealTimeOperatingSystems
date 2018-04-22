/*
 * Copyright (C) 2012-2013 by egnite GmbH
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

#include <sys/thread.h>
#include <pro/ssdp.h>

#include <string.h>

/* Receiver thread stack size. */
#ifndef SSDP_RECEIVER_STACK
#ifndef NUT_THREAD_SSDPSTACK
#define NUT_THREAD_SSDPSTACK    384
#endif
#define SSDP_RECEIVER_STACK (NUT_THREAD_SSDPSTACK * NUT_THREAD_STACK_MULT + NUT_THREAD_STACK_ADD)
#endif

const char ct_uuid_[] = "uuid:";
const char ct_upnp_rootdevice[] = "upnp:rootdevice";
const char ct_239_255_255_250[] = "239.255.255.250";

static HANDLE rx_thread;

static SSDP_LISTENER_FUNCTION notification_listener;
static SSDP_RESPONDER_FUNCTION search_listener;

static THREAD(SsdpReceiver, arg)
{
    HTTPU_SESSION *s;
    int timer = 0;

    s = HttpuSessionCreate(1900);
    for (;;) {
        if (HttpuReceive(s, 1000) > 0) {
            const char *req = HttpuGetHeader(&s->s_rcvhdr, NULL);

            if (strncasecmp(req, "M-SEARCH", 8) == 0) {
                if (search_listener) {
                    (*search_listener)(s);
                }
            }
            else if (strncasecmp(req, "NOTIFY", 6) == 0) {
                if (notification_listener) {
                    (*notification_listener)(&s->s_rcvhdr);
                }
            }
        }
        else if (timer-- <= 0) {
            timer = SsdpSendNotifications(s, "ssdp:alive");
        }
    }
}

int SsdpRegisterListener(SSDP_LISTENER_FUNCTION callback)
{
    notification_listener = callback;
    if (rx_thread == NULL) {
        rx_thread = NutThreadCreate("ssdpd", SsdpReceiver, NULL, SSDP_RECEIVER_STACK);
    }
    return rx_thread ? 0 : -1;
}

int SsdpRegisterResponder(SSDP_RESPONDER_FUNCTION callback)
{
    search_listener = callback;
    if (rx_thread == NULL) {
        rx_thread = NutThreadCreate("ssdpd", SsdpReceiver, NULL, SSDP_RECEIVER_STACK);
    }
    return rx_thread ? 0 : -1;
}

void SsdpUuidSetMac(char *uuid, const uint8_t *mac)
{
    static const char hexdigit[] = "0123456789abcdef";
    uint_fast8_t i;

    uuid = strrchr(uuid, '-');
    if (uuid) {
        for (i = 0; i < 6; i++) {
            *++uuid = hexdigit[(mac[i] >> 4) & 15];
            *++uuid = hexdigit[mac[i] & 15];
        }
    }
}

void SsdpSplitWords(char *str, char delim, char **words, int n)
{
    int cnt;
    char *cp;

    for (cnt = 0; cnt < n; cnt++) {
        words[cnt] = str;
        if (*str) {
            cp = strchr(str, delim);
            if (cp == NULL) {
                while (*++str);
            } else {
                str = cp;
                *str++ = '\0';
            }
        }
    }
}
