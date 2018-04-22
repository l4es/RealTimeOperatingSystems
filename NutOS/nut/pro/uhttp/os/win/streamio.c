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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <process.h>

#include <pro/uhttp/streamio.h>

static int num_threads;

int StreamInit(void)
{
    WSADATA wsa;

    if (WSAStartup(MAKEWORD(2,2), &wsa)) {
        printf("Failed. Error Code : %d", WSAGetLastError());
    return -1;
    }
    return 0;
}

typedef struct _CLIENT_THREAD_PARAM {
    HTTP_STREAM *ctp_stream;
    HTTP_CLIENT_HANDLER ctp_handler;
} CLIENT_THREAD_PARAM;

void StreamClientThread(void *param)
{
    CLIENT_THREAD_PARAM *ctp = (CLIENT_THREAD_PARAM *) param;

    (*ctp->ctp_handler)(ctp->ctp_stream);
    closesocket(ctp->ctp_stream->strm_csock);
    free(ctp->ctp_stream);
    free(ctp);

    if (--num_threads == 0) {
        _CrtDumpMemoryLeaks();
    }
}

int StreamClientAccept(HTTP_CLIENT_HANDLER handler, const char *params)
{
    int rc = -1;
    SOCKET sock;
    SOCKET csock;
    struct sockaddr_in addr;
    struct sockaddr_in caddr;
    unsigned short port = 80;

    HTTP_ASSERT(handler != NULL);
    if (params) {
        port = (unsigned short)atoi(params);
    }

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (sock != INVALID_SOCKET) {
        memset((void*)&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
            if (listen(sock, 1) == 0) {
                socklen_t len = sizeof(caddr);

                for (;;) {
                    csock = accept(sock, (struct sockaddr*)&caddr, &len);
                    if (csock != INVALID_SOCKET) {
                        CLIENT_THREAD_PARAM *ctp;
                        unsigned int tmo = 1000;

                        setsockopt(csock, SOL_SOCKET, SO_RCVTIMEO, (char *) &tmo, sizeof(tmo));

                        ctp = malloc(sizeof(CLIENT_THREAD_PARAM));
                        ctp->ctp_handler = handler;
                        ctp->ctp_stream = calloc(1, sizeof(HTTP_STREAM));
                        ctp->ctp_stream->strm_ssock = sock;
                        ctp->ctp_stream->strm_csock = csock;
                        memcpy(&ctp->ctp_stream->strm_saddr, &addr, sizeof(ctp->ctp_stream->strm_saddr));
                        memcpy(&ctp->ctp_stream->strm_caddr, &caddr, sizeof(ctp->ctp_stream->strm_caddr));

                        num_threads++;
                        while (num_threads > 2) {
                            Sleep(100);
                        }
                        _beginthread(StreamClientThread, 0, ctp);
                    }
                }
            }
        }
        closesocket(sock);
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
        /* Check the current stream buffer. */
        if (sp->strm_ipos == sp->strm_ilen) {
            /* No more buffered data, re-fill the buffer. */
            int got = recv(sp->strm_csock, sp->strm_ibuf, 1460, 0);
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
    int n;
    int i;
    int delen = strlen(delim);

    HTTP_ASSERT(sp != NULL);

    /* Do not read more characters than requested. */
    while (rc < siz) {
        /* Check if the delimiter fits in the current stream buffer. */
        if (sp->strm_ipos >= sp->strm_ilen - delen) {
            int got;
            /* Not enough data to fit the delimiter, re-fill the buffer. */
            sp->strm_ilen -= sp->strm_ipos;
            memcpy(sp->strm_ibuf, sp->strm_ibuf + sp->strm_ipos, sp->strm_ilen);
            got = recv(sp->strm_csock, sp->strm_ibuf + sp->strm_ilen, sizeof(sp->strm_ibuf) - sp->strm_ilen, 0);
            if (got <= 0) {
                /* Broken connection or timeout. */
                if (got < 0) {
                    rc = -1;
                }
                break;
            }
            sp->strm_ilen += got;
            sp->strm_ipos = 0;
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
    }
    return rc;
}

#ifdef HTTP_CHUNKED_TRANSFER
static int send_chunked(SOCKET sock, const char *buf, int len, int flags)
{
    static const char *crlf = "\r\n";
    char cs[11];

    itoa(len, cs, 16);
    strcat(cs, crlf);
    if (send(sock, cs, strlen(cs), flags) > 0 && send(sock, buf, len, flags) > 0 && send(sock, crlf, 2, flags) == 2) {
        return len;
    }
    return -1;
}
#endif

int s_set_flags(HTTP_STREAM *sp, unsigned int flags)
{
#ifdef HTTP_CHUNKED_TRANSFER
    sp->strm_flags |= flags;

    return 0;
#else
    return -1;
#endif
}

int s_clr_flags(HTTP_STREAM *sp, unsigned int flags)
{
#ifdef HTTP_CHUNKED_TRANSFER
    if (sp->strm_flags & S_FLG_CHUNKED) {
        send(sp->strm_csock, "0\r\n\r\n", 5, flags);
    }
    sp->strm_flags &= ~flags;

    return 0;
#else
    return -1;
#endif
}

int s_write(const void *buf, size_t size, size_t count, HTTP_STREAM *sp)
{
    HTTP_ASSERT(sp != NULL);
    HTTP_ASSERT(buf != NULL);

#ifdef HTTP_CHUNKED_TRANSFER
    if (sp->strm_flags & S_FLG_CHUNKED) {
        return send_chunked(sp->strm_csock, (const char *)buf, size * count, 0);
    }
#endif
    return send(sp->strm_csock, (const char *)buf, size * count, 0);
}

int s_puts(const char *str, HTTP_STREAM *sp)
{
    HTTP_ASSERT(sp != NULL);
    HTTP_ASSERT(str != NULL);

#ifdef HTTP_CHUNKED_TRANSFER
    if (sp->strm_flags & S_FLG_CHUNKED) {
        return send_chunked(sp->strm_csock, str, strlen(str), 0);
    }
#endif
    return send(sp->strm_csock, str, strlen(str), 0);
}

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
#ifdef HTTP_CHUNKED_TRANSFER
        if (sp->strm_flags & S_FLG_CHUNKED) {
            rc = send_chunked(sp->strm_csock, buf, strlen(buf), 0);
        } else
#endif
        {
            rc = send(sp->strm_csock, buf, strlen(buf), 0);
        }
        free(buf);
    }
    return rc;
}

int s_printf(HTTP_STREAM *sp, const char *fmt, ...)
{
    int rc = -1;
    char *buf;
    va_list ap;

    HTTP_ASSERT(sp != NULL);
    HTTP_ASSERT(fmt != NULL);

    buf = malloc(1024);
    if (buf) {
        va_start(ap, fmt);
        rc = vsnprintf(buf, 1023, fmt, ap);
        va_end(ap);
#ifdef HTTP_CHUNKED_TRANSFER
        if (sp->strm_flags & S_FLG_CHUNKED) {
            rc = send_chunked(sp->strm_csock, buf, rc, 0);
        } else
#endif
        {
            rc = send(sp->strm_csock, buf, rc, 0);
        }
        free(buf);
    }
    return rc;
}

int s_flush(HTTP_STREAM *sp)
{
    return 0;
}

const char *StreamInfo(HTTP_STREAM *sp, int item)
{
    static char *env_value;
    char *vp = NULL;

    free(env_value);
    env_value = NULL;
    switch (item) {
    case SITEM_REMOTE_ADDR:
        env_value = strdup(inet_ntoa(sp->strm_caddr.sin_addr));
        break;
    case SITEM_REMOTE_PORT:
        env_value = malloc(16);
        sprintf(env_value, "%u", sp->strm_caddr.sin_port);
        break;
    case SITEM_SERVER_NAME:
    case SITEM_SERVER_ADDR:
        env_value = strdup(inet_ntoa(sp->strm_saddr.sin_addr));
        break;
    case SITEM_SERVER_PORT:
        env_value = malloc(16);
        sprintf(env_value, "%u", sp->strm_saddr.sin_port);
        break;
    }

    if (env_value == NULL) {
        env_value = strdup("");
    }
    return env_value;
}
