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

#ifdef NUT_OS
#include <sys/version.h>
#include <dev/board.h>
#include <dev/urom.h>
#include <pro/dhcp.h>
#endif

#include <pro/uhttp/mediatypes.h>
#include <pro/uhttp/modules/mod_redir.h>
#include <pro/uhttp/modules/mod_cgi_func.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>

extern char *http_root_path;
#define HTTP_ROOT   (http_root_path ? http_root_path : HTTP_DEFAULT_ROOT)

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define MAX_UPSIZE  1460

static int CheckForPost(HTTP_REQUEST * req, int type)
{
    if (req->req_method != HTTP_METHOD_POST || req->req_type == NULL) {
        /* Bad method, POST expected. */
        return -1;
    }
    if (type == 0 && strncasecmp(req->req_type, "application/x-www-form-urlencoded", 33) == 0) {
        return 0;
    }
    if (strncasecmp(req->req_type, "multipart/form-data", 19) == 0) {
        return 0;
    }
    /* Bad content. */
    return -1;
}

static char *GetMultipartBoundary(HTTP_REQUEST *req)
{
    char *rp = NULL;
    const char *bptr;
    int blen;

    /* Make sure this is a multipart post. */
    if (CheckForPost(req, 1) == 0) {
        /* Retrieve the boundary string. */
        bptr = HttpArgValueSub(req->req_type, "boundary", &blen);
        if (bptr) {
            /* Build a delimiter string. */
            rp = malloc(blen + 3);
            if (rp) {
                rp[0] = '-';
                rp[1] = '-';
                memcpy(rp + 2, bptr, blen);
                rp[blen + 2] = '\0';
            }
        }
    }
    return rp;
}

static char *UploadFile(HTTPD_SESSION *hs, char *path)
{
    char *rp = NULL;
    char *upname = NULL;
    long avail;
    char *line;
    char *delim;
    const char *sub_ptr;
    int sub_len;
    int fd = -1;
    int got = 0;
    HTTP_STREAM *stream = hs->s_stream;
    HTTP_REQUEST *req = &hs->s_req;

    /* Retrieve the boundary string. */
    delim = GetMultipartBoundary(req);
    if (delim == NULL) {
        return NULL;
    }

    avail = req->req_length;
    line = malloc(MIN(avail, MAX_UPSIZE) + 1);
    if (line == NULL) {
        /* No memory. */
        free(delim);
        return NULL;
    }

    /* If we have a delimiter, then process the boundary content. */
    while (avail > 0) {
        /* Parse the next boundary header. */
        if (HttpParseMultipartHeader(hs, delim, &avail)) {
            /* Broken connection. */
            break;
        }
        /* Ignore headers without content disposition line. */
        if (req->req_bnd_dispo) {
            /* Retrieve the name of the form item. */
            sub_ptr = HttpArgValueSub(req->req_bnd_dispo, "name", &sub_len);
            if (sub_ptr) {
                /* The item named 'upload' contains the binary data of the file. */
                if (strncasecmp(sub_ptr, "image", sub_len) == 0) {
                    char *filename = NULL;
                    int fd = -1;
                    int eol = 0;

                    /* Get the upload file name. */
                    sub_ptr = HttpArgValueSub(req->req_bnd_dispo, "filename", &sub_len);
                    if (sub_ptr && sub_len) {
                        upname = malloc(sub_len + 1);
                        if (upname) {
                            memcpy(upname, sub_ptr, sub_len);
                            upname[sub_len] = 0;
                            /* Open the local file that the caller has provided. */
#ifdef NUT_OS
                            fd = _open(path, _O_CREAT | _O_TRUNC | _O_RDWR | _O_BINARY);
#else
                            fd = _open(path, _O_CREAT | _O_TRUNC | _O_RDWR | _O_BINARY, _S_IREAD | _S_IWRITE);
#endif
                            if (fd == -1) {
                                printf("Error %d opening %s\n", errno, path);
                            } else {
                                printf("Uploading %s\n", upname);
                            }
                        }
                    }
                    /* Recieve the binary data. */
                    while (avail) {
                        /* Read until the next boundary line. */
                        got = StreamReadUntilString(stream, delim, line, MIN(avail, MAX_UPSIZE));
                        if (got <= 0) {
                            break;
                        }
                        avail -= got;
                        /* Write data to the local file, if one had been opened. */
                        if (fd != -1) {
                            if (eol) {
                                _write(fd, "\r\n", 2);
                            }
                            if (got >= 2 && line[got - 2] == '\r' && line[got - 1] == '\n') {
                                eol = 1;
                                got -= 2;
                            }
                            _write(fd, line, got);
                        }
                    }
                    if (fd != -1) {
                        _close(fd);
                    }
                    free(filename);
                    if (got < 0) {
                        /* Broken connection. */
                        break;
                    }
                    rp = upname;
                }
                else if (strncasecmp(sub_ptr, "upload", sub_len) == 0) {
                    got = StreamReadUntilChars(hs->s_stream, "\n", "\r", line, MIN(avail, MAX_UPSIZE));
                    if (got <= 0) {
                        break;
                    }
                }
            }
        }
    }
    if (fd != -1) {
        _close(fd);
    }
    free(delim);
    free(line);

    return rp;
}

static int CgiUpload(HTTPD_SESSION *hs)
{
    char *upname;
    char *lclname;

    lclname = malloc(strlen(HTTP_ROOT) + sizeof("image.png"));
    strcat(strcpy(lclname, HTTP_ROOT), "image.png");
    upname = UploadFile(hs, lclname);
    HttpSendRedirection(hs, 303, "/index.html", NULL);

    return 0;
}

int main(void)
{
#ifdef NUT_OS
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
#endif

    puts("uHTTP upload sample\nBuild " __DATE__ " " __TIME__);

#ifdef NUT_OS
    NutRegisterDevice(&DEV_ETHER, 0, 0);
    NutDhcpIfConfig(DEV_ETHER_NAME, NULL, 60000);
    NutRegisterDevice(&devUrom, 0, 0);
#endif

    StreamInit();
    MediaTypeInitDefaults();
    HttpRegisterRedir("", "/index.html", 301);

    HttpRegisterCgiFunction("upload.cgi", CgiUpload);
    HttpRegisterMediaType("cgi", NULL, NULL, HttpCgiFunctionHandler);

    StreamClientAccept(HttpdClientHandler, NULL);

    puts("Exit");
#ifdef NUT_OS
    for (;;) ;
#endif

    return 0;
}
