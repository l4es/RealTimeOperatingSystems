/*
 * Copyright (C) 2012 by egnite GmbH
 * Copyright (C) 2001-2004 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <cfg/http.h>
#if !defined(HTTPD_EXCLUDE_DATE)
#include <pro/rfctime.h>
#endif
#include <pro/uhttp/utils.h>
#include <pro/uhttp/modules/mod_cgi_func.h>
#include <pro/uhttp/modules/mod_ssi.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <memdebug.h>

#define SSI_TYPE_FILE    0x01
#define SSI_TYPE_VIRTUAL 0x02
#define SSI_TYPE_EXEC    0x03
#define SSI_TYPE_ECHO    0x04

#define HTTP_SSI_CMD_INCLUDE    1
#define HTTP_SSI_CMD_ECHO       2
#define HTTP_SSI_CMD_CONFIG     3
#define HTTP_SSI_CMD_SET        4
#define HTTP_SSI_CMD_IF         5
#define HTTP_SSI_CMD_ELIF       6
#define HTTP_SSI_CMD_ELSE       7
#define HTTP_SSI_CMD_ENDIF      8
#define HTTP_SSI_CMD_BLOCK      9
#define HTTP_SSI_CMD_ENDBLOCK   10


static const char* HttpSsiVarHandler(HTTPD_SESSION *hs, const char *name);

static HTTP_SSI_VARHANDLER ssivar_handler = HttpSsiVarHandler;

int HttpSsiProcessFile(HTTPD_SESSION *hs, int fd);

typedef struct _HTTP_SSI_PARAM HTTP_SSI_PARAM;

struct _HTTP_SSI_PARAM {
    const char *ipar_name;
    int ipar_namelen;
    const char *ipar_value;
    int ipar_valuelen;
};

typedef int (*HTTP_SSICMD_HANDLER) (HTTPD_SESSION*, HTTP_SSI_PARAM*);

int HttpSsiIncludeHandler(HTTPD_SESSION *hs, HTTP_SSI_PARAM *prm)
{
    int fd;

    if (prm->ipar_valuelen) {
        char *path = NULL;

        if (prm->ipar_namelen == 4 && memcmp(prm->ipar_name, "file", 4) == 0) {
            path = AllocConcatStringLen("", prm->ipar_value, prm->ipar_valuelen);
        }
        else if (prm->ipar_namelen == 7 && memcmp(prm->ipar_name, "virtual", 7) == 0) {
            path = AllocConcatStringLen(HTTP_ROOT, prm->ipar_value, prm->ipar_valuelen);
        }
        if (path) {
            fd = _open(path, _O_BINARY | _O_RDONLY);
            if (fd != -1) {
                HttpSsiProcessFile(hs, fd);
                _close(fd);
                free(path);
                return 0;
            }
            free(path);
        }
    }
    return -1;
}

int HttpSsiExecHandler(HTTPD_SESSION *hs, HTTP_SSI_PARAM *prm)
{
    char *path = NULL;
    char *args;
    char *argcpy = NULL;
    HTTP_REQUEST *orig_req;

    if (prm->ipar_valuelen) {
        if (prm->ipar_namelen == 3 && memcmp(prm->ipar_name, "cgi", 3) == 0) {
            path = AllocConcatStringLen("", prm->ipar_value, prm->ipar_valuelen);
        }
        if (path) {
            args = strchr(path, '?');
            if (args) {
                *args++ = '\0';
            }
            orig_req = malloc(sizeof(HTTP_REQUEST));
            if (orig_req) {
                memcpy(orig_req, &hs->s_req, sizeof(HTTP_REQUEST));
                hs->s_req.req_method = HTTP_METHOD_GET;
                hs->s_req.req_argn = NULL;
#if HTTP_VERSION >= 0x10
                hs->s_req.req_length = 0;
#endif
                if (args == NULL || hs->s_req.req_query == NULL) {
                    hs->s_req.req_query = args;
                }
                else {
                    char *qs = strstr(args, "$QUERY_STRING");
                    if (qs) {
                        int len;

                        *qs = '\0';
                        len = strlen(args) + strlen(hs->s_req.req_query) + strlen(qs + 13);

                        argcpy = malloc(len + 1);
                        if (argcpy) {
                            strcpy(argcpy, args);
                            strcat(argcpy, hs->s_req.req_query);
                            strcat(argcpy, qs + 13);
                        }
                        hs->s_req.req_query = argcpy;
                    } else {
                        hs->s_req.req_query = args;
                    }
                }
                HttpCgiFunctionHandler(hs, NULL, path);
                free(hs->s_req.req_argn);
                memcpy(&hs->s_req, orig_req, sizeof(HTTP_REQUEST));
                free(orig_req);
                free(argcpy);
            }
            free(path);
        }
    }
    return 0;
}

int HttpSsiEchoHandler(HTTPD_SESSION *hs, HTTP_SSI_PARAM *prm)
{
    static char *varname;
    const char *value = NULL;

    varname = realloc(varname, prm->ipar_valuelen + 1);
    memcpy(varname, prm->ipar_value, prm->ipar_valuelen);
    *(varname + prm->ipar_valuelen) = '\0';
    value = (*ssivar_handler)(hs, varname);
    s_puts(value, hs->s_stream);

    return 0;
}

typedef struct _HTTP_SSI_COMMAND HTTP_SSI_COMMAND;

struct _HTTP_SSI_COMMAND {
    char *icmd_name;
    int icmd_namelen;
    HTTP_SSICMD_HANDLER icmd_handler;
};

HTTP_SSI_COMMAND ssiCmdList[] = {
    //{ "set", 3, NULL },
    { "include", 7, HttpSsiIncludeHandler },
    //{ "if", 2, NULL },
    //{ "fsize", 5, NULL },
    //{ "flastmod", 8, NULL },
    { "exec", 4, HttpSsiExecHandler },
    //{ "endif", 5, NULL },
    //{ "else", 4, NULL },
    //{ "elif", 4, NULL },
    { "echo", 4, HttpSsiEchoHandler },
    //{ "config", 6, NULL }
};

#define HTTP_SSI_NUM_COMMANDS   (sizeof(ssiCmdList) / sizeof(HTTP_SSI_COMMAND))

int HttpSsiParse(HTTPD_SESSION *hs, const char *buf, int len)
{
    int i;
    //const char *var_name;
    //int var_name_len;
    //const char *var_value;
    //int var_value_len;
    HTTP_SSI_COMMAND *ssiCmd = NULL;
    HTTP_SSI_PARAM ssiPrm;

    memset(&ssiPrm, 0, sizeof(ssiPrm));
    while (len && isspace((int)*buf)) {
        buf++;
        len--;
    }
    for (i = 0; i < (int)HTTP_SSI_NUM_COMMANDS; i++) {
        if (len > ssiCmdList[i].icmd_namelen &&
            strncasecmp(buf, ssiCmdList[i].icmd_name, ssiCmdList[i].icmd_namelen) == 0) {
            ssiCmd = &ssiCmdList[i];
            len -= ssiCmd->icmd_namelen;
            buf += ssiCmd->icmd_namelen;
            break;
        }
    }
    if (i == HTTP_SSI_NUM_COMMANDS) {
        return -1;
    }
    while (len && isspace((int)*buf)) {
        buf++;
        len--;
    }
    if (len) {
        ssiPrm.ipar_name = buf;
        i = len;
        while (len) {
            if (!isalpha((int)*buf)) {
                ssiPrm.ipar_namelen = i - len;
                break;
            }
            buf++;
            len--;
        }
        if (len) {

            while (len && *buf != '=') {
                buf++;
                len--;
            }
            while (len && *buf != '"') {
                buf++;
                len--;
            }
            if (len) {
                buf++;
                len--;
                ssiPrm.ipar_value = buf;
                i = len;
                while (len) {
                    if (*buf == '"') {
                        ssiPrm.ipar_valuelen = i - len;
                        break;
                    }
                    buf++;
                    len--;
                }

            }
        }
    }
    ssiCmd->icmd_handler(hs, &ssiPrm);

    return 0;
}

int HttpSsiProcessFile(HTTPD_SESSION *hs, int fd)
{
    long avail;
    char *buf;
    char *bp = NULL;
    int bufsiz;
    int buflen;
    int off;
    char *csp;
    char *cep;
    int cmdlen;

    avail = _filelength(fd);
    bufsiz = avail < 1460 ? (int) avail : 1460;
    buf = malloc(bufsiz + 1);
    buflen = 0;
    do {
        if (buflen == 0) {
            bp = buf;
            buflen = _read(fd, bp, bufsiz);
            if (buflen <= 0) {
                break;
            }
            avail -= buflen;
            buf[buflen] = '\0';
        }
        /* Search for embedded SSI command. */
        csp = strstr(bp, "<!--#");
        if (csp) {
            /* Found SSI command start, send any preceding data. */
            off = (int) (csp - bp);
            if (off) {
                s_write(bp, 1, off, hs->s_stream);
                bp += off;
                buflen -= off;
            }
            /* Search end of SSI command. */
            cep = strstr(csp + 5, "-->");
            if (cep) {
                /* Found SSI command end, process it. */
                cmdlen = (int) (cep - csp) + 3;
                if (HttpSsiParse(hs, bp + 5, cmdlen - 3)) {
                    /* Bad SSI command, send it unprocessed. */
                    s_write(bp, 1, cmdlen, hs->s_stream);
                }
                buflen -= cmdlen;
                bp += cmdlen;
            }
            else {
                /* SSI command end not found. */
                if (csp != buf) {
                    /* If the command start was not at the beginning of
                       the buffer, then move the file pointer back to
                       the start of the command. */
                    _seek(fd, -strlen(csp), SEEK_CUR);
                } else {
                    /* SSI command doesn't fit in our buffer, send it
                       unprocessed. */
                    s_write(bp, 1, buflen, hs->s_stream);
                }
                /* Discard the buffer. */
                buflen = 0;
            }
        } else {
            /* No SSI command found. */
            if (bp != buf) {
                /* If our search was not started at the beginning of
                   the buffer, then read the last 4 bytes again. */
                off = buflen >= 4 ? 4 : buflen;
                _seek(fd, -off, SEEK_CUR);
                buflen -= off;
            }
            s_write(bp, 1, buflen, hs->s_stream);
            buflen = 0;
        }
    } while (1);
    s_flush(hs->s_stream);

    free(buf);

    return 0;
}

int HttpSsiHandler(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *filepath)
{
    int fd;

    fd = _open(filepath, _O_BINARY | _O_RDONLY);
    if (fd == -1) {
        HttpSendError(hs, 404);
        return 0;
    }
    HttpSendHeaderTop(hs, 200);
    s_puts("Cache-Control: no-cache, must-revalidate\r\n", hs->s_stream);
#if !defined(HTTPD_EXCLUDE_DATE)
    {
        time_t now = time(NULL);

        s_vputs(hs->s_stream, ct_Expires, ": ", Rfc1123TimeString(gmtime(&now)), " GMT\r\n", NULL);
    }
#endif
    HttpSendHeaderBottom(hs, mt->media_type, mt->media_subtype ? mt->media_subtype : mt->media_ext, -1);
    s_set_flags(hs->s_stream, S_FLG_CHUNKED);
    HttpSsiProcessFile(hs, fd);
    s_clr_flags(hs->s_stream, S_FLG_CHUNKED);
    _close(fd);

    return 0;
}

static const char* HttpSsiVarHandler(HTTPD_SESSION *hs, const char *name)
{
    (void)hs;
    return name;
}

HTTP_SSI_VARHANDLER HttpRegisterSsiVarHandler(HTTP_SSI_VARHANDLER handler)
{
    HTTP_SSI_VARHANDLER rc;

    rc = ssivar_handler;
    ssivar_handler = handler;

    return rc;
}
