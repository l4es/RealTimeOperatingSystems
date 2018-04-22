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

#include <sys/nutdebug.h>

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <pro/httpu.h>


/* Use for local debugging only. Do NOT include into NUTDEBUG.
#define DEBUG_HTTPU
*/
#ifdef DEBUG_HTTPU
#include <stdio.h>
#endif

/*!
 * \addtogroup xgHTTPU HTTPU
 *
 * \verbatim File version $Id$ \endverbatim
 */

HTTPU_SESSION *HttpuSessionCreate(uint16_t port)
{
    HTTPU_SESSION *s;

    s = calloc(1, sizeof(*s));
    if (s) {
        s->s_sock = NutUdpCreateSocket(port);
        if (s->s_sock == NULL) {
            free(s);
            s = NULL;
        } else {
            /* Enable UDP receive buffers. Otherwise the IP stack will
               only store the most recent datagram. */
            uint16_t rxbs = HTTPU_MAX_DATAGRAM_SIZE * 2;
            NutUdpSetSockOpt(s->s_sock, SO_RCVBUF, &rxbs, sizeof(rxbs));
        }
    }
    return s;
}

void HttpuSessionDestroy(HTTPU_SESSION * s)
{
    NUTASSERT(s != NULL);
    NutUdpDestroySocket(s->s_sock);
    free(s);
}

int HttpuReceive(HTTPU_SESSION * s, uint32_t tmo)
{
    char *lp;

    NUTASSERT(s != NULL);

    /* Set buffer pointer and receive datagram. */
    lp = s->s_rcvbuf.msg_buff;
    s->s_rcvbuf.msg_len =
        NutUdpReceiveFrom(s->s_sock, &s->s_reqip, (uint16_t *) &s->s_reqport, lp,
                          sizeof(s->s_rcvbuf.msg_buff) - 1, tmo);
    if (s->s_rcvbuf.msg_len >= 0) {
        int idx;
        char *cp;
        char *nxt;

        *(lp + s->s_rcvbuf.msg_len) = '\0';

#ifdef DEBUG_HTTPU
        printf("HTTPU Recv(%d)\n%s", s->s_rcvbuf.msg_len,
               s->s_rcvbuf.msg_buff);
#endif
        /* Split the header into lines. */
        for (idx = 0; idx < HTTPU_MAX_HEADER_LINES && *lp; idx++, lp = nxt) {
            /* Search next carriage return. */
            nxt = strchr(lp, '\r');
            if (nxt == NULL) {
                /* No more lines available. */
                break;
            }
            /* Terminate line and skip line feed. */
            *nxt++ = '\0';
            if (*nxt == '\n') {
                nxt++;
            }
            if (idx == 0) {
                /* The first line is without name. */
                cp = lp;
            } else {
                /* Split name and value. */
                cp = strchr(lp, ':');
                if (cp == NULL) {
                    /* This is not a header line. */
                    break;
                }
                *cp++ = '\0';
                s->s_rcvhdr.hdr_name[idx] = lp;
            }
            /* Skip leading spaces in the value. */
            while (*cp == ' ') {
                cp++;
            }
            s->s_rcvhdr.hdr_value[idx] = cp;
        }
        s->s_rcvhdr.hdr_num = idx;
    }
    return s->s_rcvbuf.msg_len;
}

const char *HttpuGetHeader(const HTTPU_HEADER * hdr, const char *name)
{
    int i;

    NUTASSERT(hdr != NULL);

    /* Find the line with the given name. Without name the first line
       is returned. */
    for (i = 0; i < hdr->hdr_num; i++) {
        if (name == NULL || strcasecmp(hdr->hdr_name[i], name) == 0) {
            return hdr->hdr_value[i];
        }
    }
    /* For non-existing headers return an empty string. */
    return "";
}

int HttpuAddHeader(HTTPU_SESSION * s, const char *name, ...)
{
    va_list ap;
    int idx;
    char *str;
    int len;

    NUTASSERT(s != NULL);

    if (name) {
        /* Add header line, if name given. */
        len = strlen(name);
        idx = s->s_sndbuf.msg_len;
        memcpy(&s->s_sndbuf.msg_buff[idx], name, len);
        idx += len;
        s->s_sndbuf.msg_buff[idx++] = ':';
        s->s_sndbuf.msg_buff[idx++] = ' ';
    } else {
        /* Without name, create first header line. */
        memset(&s->s_sndbuf, 0, sizeof(s->s_sndbuf));
        idx = 0;
    }

    /* Add all arguments until a NULL pointer is reached. */
    va_start(ap, name);
    while ((str = va_arg(ap, char *)) != NULL) {
        len = strlen(str);
        if (len) {
            if (idx + len + 4 > sizeof(s->s_sndbuf.msg_buff)) {
                /* Buffer overflow. */
                idx = -1;
                break;
            }
            memcpy(&s->s_sndbuf.msg_buff[idx], str, len);
            idx += len;
        }
    }
    va_end(ap);

    if (idx >= 0) {
        /* Add end of line. */
        s->s_sndbuf.msg_buff[idx++] = '\r';
        s->s_sndbuf.msg_buff[idx++] = '\n';
        s->s_sndbuf.msg_len = idx;
        /* Add preventive end of header. */
        s->s_sndbuf.msg_buff[idx] = '\r';
        s->s_sndbuf.msg_buff[idx + 1] = '\n';
    }
    return idx;
}

int HttpuSend(HTTPU_SESSION * s, uint32_t ip, uint16_t port)
{
    NUTASSERT(s != NULL);
    NUTASSERT(ip != 0);
    NUTASSERT(port != 0);

#ifdef DEBUG_HTTPU
    printf("HTTPU Send(%d)\n%s", s->s_sndbuf.msg_len + 2, s->s_sndbuf.msg_buff);
#endif
    /* Add 2 more bytes for the preventive end of header. */
    return NutUdpSendTo(s->s_sock, ip, port, s->s_sndbuf.msg_buff, s->s_sndbuf.msg_len + 2);
}

int HttpuRespond(HTTPU_SESSION * s)
{
    NUTASSERT(s != NULL);

    /* Send response to the requester. */
    return HttpuSend(s, s->s_reqip, s->s_reqport);
}
