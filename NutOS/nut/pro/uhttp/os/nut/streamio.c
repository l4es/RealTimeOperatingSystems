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

#if !defined(WIN32) && !defined(__linux__)

#include <pro/uhttp/streamio.h>

#include <sys/version.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <memdebug.h>

#include <pro/uhttp/streamio.h>

int StreamInit(void)
{
    return 0;
}

int StreamClientAccept(HTTP_CLIENT_HANDLER handler, const char *params)
{
    int rc = -1;
    TCPSOCKET *sock;
    HTTP_STREAM *sp;
    unsigned short port = 80;
    uint32_t tmo = 1000;
    uint16_t mss = 0;
    uint16_t tcpbufsiz = 0;
    const char *pp = params;

    HTTP_ASSERT(handler != NULL);
    if (pp && *pp) {
        /* First parameter defines the port. */
        if (*pp != ':') {
            port = (unsigned short) atoi(pp);
            pp = strchr(pp, ':');
        }
        if (pp) {
            /* Second parameter defines receive time out. */
            if (*++pp != ':') {
                tmo = (uint32_t) atol(pp);
                pp = strchr(pp, ':');
            }
            if (pp) {
                /* Third parameter specifies TCP input buffer. */
                if (*++pp != ':') {
                    tcpbufsiz = (uint16_t) atoi(pp);
                    pp = strchr(pp, ':');
                }
                if (pp) {
                    /* Forth parameter specifies TCP segment size. */
                    mss = atol(pp);
                }
            }
        }
    }
    for (;;) {
        sock = NutTcpCreateSocket();
        if (sock) {
            if (mss) {
                NutTcpSetSockOpt(sock, TCP_MAXSEG, &mss, sizeof(mss));
            }
            if (tcpbufsiz) {
                NutTcpSetSockOpt(sock, SO_RCVBUF, &tcpbufsiz, sizeof(tcpbufsiz));
            }
            if (NutTcpAccept(sock, port) == 0) {

                NutTcpSetSockOpt(sock, SO_RCVTIMEO, &tmo, sizeof(tmo));

#ifdef HTTP_PLATFORM_STREAMS
                sp = calloc(1, sizeof(HTTP_STREAM));
                sp->strm_sock = sock;
                (*handler)(sp);
                free(sp);
#else
                /* Associate a binary stdio stream with the socket. */
                sp = _fdopen((int) ((uintptr_t) sock), "r+b");
                (*handler)(sp);
                fclose(sp);
#endif
                rc = 0;
            }
        }
        NutTcpCloseSocket(sock);
    }
    return rc;
}

int StreamReadUntilChars(HTTP_STREAM *sp, const char *delim, const char *ignore, char *buf, int siz)
{
    int rc = 0;
    int skip = 0;
    char ch;

    HTTP_ASSERT(sp != NULL);

    /* Do not read more characters than requested. */
    while (rc < siz) {
#ifdef HTTP_PLATFORM_STREAMS
        /* Check the current stream buffer. */
        if (sp->strm_ipos == sp->strm_ilen) {
            /* No more buffered data, re-fill the buffer. */
            int got = NutTcpReceive(sp->strm_sock, sp->strm_ibuf, sizeof(sp->strm_ibuf));
            if (got <= 0) {
                /* Broken connection or timeout. */
                if (got < 0) {
                    rc = -1;
                    skip = 0;
                }
                break;
            }
            sp->strm_ilen = got;
            sp->strm_ipos = 0;
        }
        ch = sp->strm_ibuf[sp->strm_ipos];
        sp->strm_ipos++;
#else
        {
            int c = fgetc(sp);

            if (c == EOF) {
                if (ferror(sp)) {
                    rc = -1;
                    skip = 0;
                }
                break;
            } else {
                ch = (char) c;
            }
        }
#endif
        if (rc == 0 && ch == ' ') {
            /* Skip leading spaces. */
            skip++;
        } else {
            rc++;
            if (delim && strchr(delim, ch)) {
                /* Delimiter found. */
                break;
            }
            if (buf && (ignore == NULL || strchr(ignore, ch) == NULL)) {
                /* Add valid character to application buffer. */
                *buf++ = ch;
            }
        }
    }
    if (buf) {
        *buf = '\0';
    }
    return rc + skip;
}

int StreamReadUntilString(HTTP_STREAM *sp, const char *delim, char *buf, int siz)
{
    int rc = 0;
    static int n;
#ifndef HTTP_PLATFORM_STREAMS
    static HTTP_STREAM *last_stream;
#endif
    int delen = strlen(delim);

    HTTP_ASSERT(sp != NULL);

#ifndef HTTP_PLATFORM_STREAMS
    if (!last_stream || last_stream != sp) {
        n = 0;
        last_stream = sp;
    }
#endif

    /* Do not read more characters than requested. */
    while (rc < siz) {
#ifdef HTTP_PLATFORM_STREAMS
        int i;

        /* Check if the delimiter fits in the current stream buffer. */
        while (sp->strm_ipos >= sp->strm_ilen - delen) {
            int got;
            /* Not enough data to fit the delimiter, re-fill the buffer. */
            sp->strm_ilen -= sp->strm_ipos;
            memcpy(sp->strm_ibuf, sp->strm_ibuf + sp->strm_ipos, sp->strm_ilen);
            sp->strm_ipos = 0;
            got = NutTcpReceive(sp->strm_sock, sp->strm_ibuf + sp->strm_ilen, sizeof(sp->strm_ibuf) - sp->strm_ilen);
            if (got <= 0) {
                /* Broken connection or timeout. */
                if (got < 0) {
                    rc = -1;
                }
                break;
            }
            sp->strm_ilen += got;
        }
        for (i = sp->strm_ipos, n = 0; i < sp->strm_ilen && rc + n < siz; i++, n++) {
            if (*delim == sp->strm_ibuf[i]) {
                if (i + delen >= sp->strm_ilen) {
                    break;
                }
                if (memcmp(&sp->strm_ibuf[i], delim, delen) == 0) {
                    break;
                }
            }
        }
        if (n) {
            memcpy(buf, sp->strm_ibuf + sp->strm_ipos, n);
            buf += n;
            rc += n;
            sp->strm_ipos += n;
        } else {
            break;
        }
#else
        int ch;

        ch = fgetc(sp);
        if (ch == EOF) {
            rc = -1;
            break;
        }
        *buf++ = (char) ch;
        rc++;
        if (ch == delim[n]) {
            if (++n >= delen) {
                rc -= n;
                rc = rc > 0 ? rc : 0;
                break;
            }
        } else {
            n = 0;
        }
#endif
    }
    return rc;
}

#ifdef HTTP_PLATFORM_STREAMS

#ifdef HTTP_CHUNKED_TRANSFER
static int send_chunked(TCPSOCKET *sock, const char *buf, int len)
{
    char *cs = NULL;

    asprintf(&cs, "%X\r\n", len);
    if (cs && NutTcpDeviceWrite(sock, cs, strlen(cs)) < 0) {
        free(cs);
        return -1;
    }
    free(cs);
    if (NutTcpDeviceWrite(sock, buf, len) < 0) {
        return -1;
    }
    if (NutTcpDeviceWrite(sock, "\r\n", 2) < 0) {
        return -1;
    }
    return len;
}
#endif

int s_set_flags(HTTP_STREAM *sp, unsigned int flags)
{
#ifdef HTTP_CHUNKED_TRANSFER
    sp->strm_flags |= flags;

    return 0;
#else
    (void)sp;
    (void)flags;
    return -1;
#endif
}

int s_clr_flags(HTTP_STREAM *sp, unsigned int flags)
{
#ifdef HTTP_CHUNKED_TRANSFER
    if (sp->strm_flags & S_FLG_CHUNKED) {
        NutTcpDeviceWrite(sp->strm_sock, "0\r\n\r\n", 5);
    }
    sp->strm_flags &= ~flags;

    return 0;
#else
    (void)sp;
    (void)flags;
    return -1;
#endif
}

int s_write(const void *buf, size_t size, size_t count, HTTP_STREAM *sp)
{
    HTTP_ASSERT(sp != NULL);
    HTTP_ASSERT(buf != NULL);

#ifdef HTTP_CHUNKED_TRANSFER
    if (sp->strm_flags & S_FLG_CHUNKED) {
        return send_chunked(sp->strm_sock, (const char *)buf, size * count);
    }
#endif
    return NutTcpDeviceWrite(sp->strm_sock, (const char *)buf, size * count);
}

int s_puts(const char *str, HTTP_STREAM *sp)
{
    int len;

    HTTP_ASSERT(sp != NULL);
    HTTP_ASSERT(str != NULL);

    len = strlen(str);
    if (len) {
#ifdef HTTP_CHUNKED_TRANSFER
        if (sp->strm_flags & S_FLG_CHUNKED) {
            return send_chunked(sp->strm_sock, str, len);
        }
#endif
        return NutTcpDeviceWrite(sp->strm_sock, str, len);
    }
    return 0;
}

int s_printf(HTTP_STREAM *sp, const char *fmt, ...)
{
    int rc = -1;
    char *buf = NULL;
    va_list ap;

    HTTP_ASSERT(sp != NULL);
    HTTP_ASSERT(fmt != NULL);

    va_start(ap, fmt);
    rc = vasprintf(&buf, fmt, ap);
    va_end(ap);
    if (buf) {
#ifdef HTTP_CHUNKED_TRANSFER
        if (sp->strm_flags & S_FLG_CHUNKED) {
            rc = send_chunked(sp->strm_sock, buf, rc);
        } else
#endif
        {
            rc = NutTcpDeviceWrite(sp->strm_sock, buf, rc);
        }
        free(buf);
    } else {
        rc = -1;
    }
    return rc;
}

int s_flush(HTTP_STREAM *sp)
{
    return NutTcpDeviceWrite(sp->strm_sock, NULL, 0);
}

#endif

int s_vputs(HTTP_STREAM *sp, ...)
{
    int rc = -1;
    int len;
    char *cp;
    char *buf;
    va_list ap;

    HTTP_ASSERT(sp != NULL);

    va_start(ap, sp);
    for (len = 0; (cp = va_arg(ap, char *)) != NULL; len += strlen(cp));
    va_end(ap);
    buf = malloc(len + 1);
    if (buf) {
        va_start(ap, sp);
        for (*buf = '\0'; (cp = va_arg(ap, char *)) != NULL; strcat(buf, cp));
        va_end(ap);
#ifdef HTTP_PLATFORM_STREAMS
#ifdef HTTP_CHUNKED_TRANSFER
        if (sp->strm_flags & S_FLG_CHUNKED) {
            rc = send_chunked(sp->strm_sock, buf, strlen(buf));
        } else
#endif
        {
            rc = NutTcpDeviceWrite(sp->strm_sock, buf, strlen(buf));
        }
#else
        if (fwrite(buf, 1, strlen(buf), sp) == strlen(buf)) {
            rc = 0;
        }
#endif
        free(buf);
    }
    return rc;
}

const char *StreamInfo(HTTP_STREAM *sp, int item)
{
    static char *env_value;

    free(env_value);
    env_value = NULL;
#ifdef HTTP_PLATFORM_STREAMS
    switch (item) {
    case SITEM_REMOTE_ADDR:
        env_value = strdup(inet_ntoa(sp->strm_sock->so_remote_addr));
        break;
    case SITEM_REMOTE_PORT:
        asprintf(&env_value, "%u", sp->strm_sock->so_remote_port);
        break;
    case SITEM_SERVER_NAME:
    case SITEM_SERVER_ADDR:
        env_value = strdup(inet_ntoa(sp->strm_sock->so_local_addr));
        break;
    case SITEM_SERVER_PORT:
        asprintf(&env_value, "%u", sp->strm_sock->so_local_port);
        break;
    }
#else
    (void)sp; 
    (void)item;
#endif
    if (env_value == NULL) {
        env_value = strdup("");
    }
    return env_value;
}

#endif
