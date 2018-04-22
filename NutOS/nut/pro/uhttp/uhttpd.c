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

#include <cfg/http.h>
#if !defined(HTTPD_EXCLUDE_DATE)
#include <pro/rfctime.h>
#endif
#include <pro/uhttp/utils.h>
#include <pro/uhttp/uhttpd.h>
#include <pro/uhttp/mediatypes.h>

#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>
#include <memdebug.h>

#ifndef HTTP_MAX_REQUEST_SIZE
#define HTTP_MAX_REQUEST_SIZE   64
#endif

/*! Constant string "GET". */
const char ct_GET[] = "GET";
/*! Constant string "HEAD". */
const char ct_HEAD[] = "HEAD";
/*! Constant string "POST". */
const char ct_POST[] = "POST";
/*! Constant string "Content-Disposition". */
const char ct_Content_Disposition[] = "Content-Disposition";
/*! Constant string "Content-Type". */
const char ct_Content_Type[] = "Content-Type";
/*! Constant string "Accept-Encoding". */
const char ct_Accept_Encoding[] = "Accept-Encoding";
/*! Constant string "Authorization". */
const char ct_Authorization[] = "Authorization";
/*! Constant string "Connection". */
const char ct_Connection[] = "Connection";
/*! Constant string "close". */
const char ct_close[] = "close";
/*! Constant string "keep-alive". */
const char ct_Keep_Alive[] = "keep-alive";
/*! Constant string "Content-Length". */
const char ct_Content_Length[] = "Content-Length";
/*! Constant string "Cookie". */
const char ct_Cookie[] = "Cookie";
/*! Constant string "Host". */
const char ct_Host[] = "Host";
/*! Constant string "If-Modified-Since". */
const char ct_If_Modified_Since[] = "If-Modified-Since";
/*! Constant string "Referer". */
const char ct_Referer[] = "Referer";
/*! Constant string "User-Agent". */
const char ct_User_Agent[] = "User-Agent";
/*! Constant string "Last-Modified". */
const char ct_Last_Modified[] = "Last-Modified";
/*! Constant string "Expires". */
const char ct_Expires[] = "Expires";
/*! Constant string "Date". */
const char ct_Date[] = "Date";
/*! Constant string "Content-Encoding". */
const char ct_Content_Encoding[] = "Content-Encoding";
/*! Constant string "Location". */
const char ct_Location[] = "Location";

char *http_root_path;

static int HttpAuthValidateAll(HTTPD_SESSION *req);
HTTP_AUTH_VALIDATOR httpd_auth_validator = HttpAuthValidateAll;

static int HttpLocationRedirNone(HTTPD_SESSION *hs);
HTTP_LOC_REDIRECTOR httpd_loc_redirector = HttpLocationRedirNone;

static int HttpAuthValidateAll(HTTPD_SESSION *req)
{
    (void)req;

    return 0;
}

static int HttpLocationRedirNone(HTTPD_SESSION *hs)
{
    (void)hs;

    return -1;
}

char *HttpArgParseFirst(HTTP_REQUEST * req)
{
    req->req_argp = req->req_query;

    return HttpArgParseNext(req);
}

char *HttpArgParseNext(HTTP_REQUEST * req)
{
    char *rp = NULL;
    char *cp;
    int len;

    if (req->req_argp) {
        cp = strchr(req->req_argp, '&');
        if (cp) {
            len = (int) (cp - req->req_argp);
            cp++;
            rp = malloc(len + 1);
            if (rp) {
                memcpy(rp, req->req_argp, len);
                *(rp + len) = '\0';
            }
        } else {
            rp = strdup(req->req_argp);
        }
        req->req_argp = cp;
        if (rp) {
            cp = strchr(rp, '=');
            if (cp) {
                *cp++ = '\0';
                HttpUrlUnescape(cp);
            }
            req->req_argv = cp;
        }
        free(req->req_argn);
        req->req_argn = rp;
        HttpUrlUnescape(rp);
    }
    return rp;
}

char *HttpArgReadNext(HTTPD_SESSION *hs, long *avail)
{
    int bufsiz;
    int got;
    char *cp;
    char *rp = NULL;
    char *buf = NULL;

    if (*avail) {
        bufsiz = *avail > 128 ? 128 : *avail;
        buf = malloc(bufsiz + 1);
        if (buf) {
            got = StreamReadUntilChars(hs->s_stream, "&", NULL, buf, bufsiz);
            if (got > 0) {
                *avail -= got;
                rp = strdup(buf);
                free(hs->s_req.req_argn);
                hs->s_req.req_argn = rp;
                cp = strchr(rp, '=');
                if (cp) {
                    *cp++ = '\0';
                    HttpUrlUnescape(cp);
                    hs->s_req.req_argv = cp;
                }
                HttpUrlUnescape(rp);
            }
            free(buf);
        }
    }
    return rp;
}

char *HttpArgValue(HTTP_REQUEST * req)
{
    return req->req_argv;
}

const char *HttpArgValueSub(const char *str, const char *name, int *len)
{
    const char *cp = str;
    size_t namelen = strlen(name);

    *len = 0;
    for (;;) {
        /* Move to the next disposition parameter. */
        cp = strchr(cp, ';');
        if (cp == NULL) {
            break;
        }
        /* Skip leading spaces. */
        while (*++cp == ' ');
        /* Check the parameter name. */
        if (strncasecmp(cp, name, namelen) == 0) {
            cp += namelen;
            /* Skip the name. */
            while (*cp && strchr("=\";", *cp) == NULL) {
                cp++;
            }
            /* Check if we have a name/value pair. */
            if (*cp == '=') {
                /* Skip leading spaces. */
                while (*++cp == ' ');
                /* Skip leading quote. */
                cp += *cp == '"';
                /* Determine the length of the value. */
                while (cp[*len] && strchr("\";", cp[*len]) == NULL) {
                    (*len)++;
                }
                break;
            }
        }
    }
    return cp;
}

#if HTTP_VERSION >= 0x10
int HttpParseMultipartHeader(HTTPD_SESSION *hs, const char *bnd, long *avail)
{
    int rc = -1;
    int got = 0;
    char *buf;
    char **strval;
    int bndlen = strlen(bnd);

    buf = malloc(MIN(*avail, HTTP_MAX_REQUEST_SIZE) + 1);
    if (buf == NULL) {
        return -1;
    }
    free(hs->s_req.req_bnd_type);
    hs->s_req.req_bnd_type = NULL;
    free(hs->s_req.req_bnd_dispo);
    hs->s_req.req_bnd_dispo = NULL;

    while (*avail) {
        /* Read the next line, expecting a boundary. */
        got = StreamReadUntilChars(hs->s_stream, "\n", "\r", buf, MIN(*avail, HTTP_MAX_REQUEST_SIZE));
        if (got <= 0) {
            /* Broken connection. */
            break;
        }
        *avail -= got;
        if (got > bndlen && strncmp(buf, bnd, bndlen) == 0) {
            /* Found a boundary. */
            break;
        }
    }
    /* Got boundary, check if it's the last one. */
    if (got < bndlen + 2 || strncmp(buf + bndlen, "--", 2)) {
        while (*avail) {
            /* Get next header name. */
            got = StreamReadUntilChars(hs->s_stream, ":\n", "\r", buf, MIN(*avail, HTTP_MAX_REQUEST_SIZE));
            if (got <= 0) {
                /* Broken connection, stop parsing. */
                break;
            }
            *avail -= got;
            if (*buf == '\0') {
                /* Empty line marks the end of the request header. */
                rc = 0;
                break;
            }
            strval = NULL;
            if (strcasecmp(buf, ct_Content_Disposition) == 0) {
                strval = &hs->s_req.req_bnd_dispo;
            }
            else if (strcasecmp(buf, ct_Content_Type) == 0) {
                strval = &hs->s_req.req_bnd_type;
            }
            else {
                got = StreamReadUntilChars(hs->s_stream, "\n", NULL, NULL, *avail);
                if (got <= 0) {
                    break;
                }
                *avail -= got;
            }
            if (strval) {
                got = StreamReadUntilChars(hs->s_stream, "\n", "\r", buf, MIN(*avail, HTTP_MAX_REQUEST_SIZE));
                if (got <= 0) {
                    break;
                }
                *avail -= got;
                *strval = strdup(buf);
            }
        }
    }
    free(buf);
    return rc;
}
#endif

int HttpParseHeader(HTTPD_SESSION *hs)
{
    int got;
    char *buf;
    char *cp;

    buf = malloc(HTTP_MAX_REQUEST_SIZE + 1);
    if (buf == NULL) {
        return -1;
    }
    memset(&hs->s_req, 0, sizeof(HTTP_REQUEST));

    /* Read the first word of the request. */
    got = StreamReadUntilChars(hs->s_stream, " \n", "\r", buf, HTTP_MAX_REQUEST_SIZE);
    if (got <= 0) {
        free(buf);
        return -1;
    }
    /* Expect a valid method. */
    if (strcasecmp(buf, ct_GET) == 0) {
        hs->s_req.req_method = HTTP_METHOD_GET;
    }
#if HTTP_VERSION >= 0x10
    else if (strcasecmp(buf, ct_HEAD) == 0) {
        hs->s_req.req_method = HTTP_METHOD_HEAD;
    }
    else if (strcasecmp(buf, ct_POST) == 0) {
        hs->s_req.req_method = HTTP_METHOD_POST;
    }
#endif
    else {
        /* Method not implemented. */
        HttpSendError(hs, 501);
        free(buf);
        return -1;
    }

    /* Read the second word of the request. */
    got = StreamReadUntilChars(hs->s_stream, " \n", "\r", buf, HTTP_MAX_REQUEST_SIZE);
    /* Expect a valid URI. */
    cp = strchr(buf, '?');
    if (cp) {
        *cp++ = '\0';
        hs->s_req.req_query = strdup(cp);
    }
    hs->s_req.req_url = UriUnescape(strdup(buf));

    /* Read the remaining part of the request. */
    got = StreamReadUntilChars(hs->s_stream, "\n", " \r", buf, HTTP_MAX_REQUEST_SIZE);
    /* If no HTTP version is provided, then assume HTTP/0.9. */
    if (strncasecmp(buf, "HTTP/", 5)) {
        hs->s_req.req_version = 0x09;
    } else {
        hs->s_req.req_version = buf[5] - '0';
        hs->s_req.req_version <<= 4;
        hs->s_req.req_version += buf[6] - '0';
    }

#if HTTP_VERSION >= 0x10
#if HTTP_VERSION >= 0x11 && HTTP_KEEP_ALIVE_REQ
    hs->s_req.req_connection = HTTP_CONN_KEEP_ALIVE;
#endif
    do {
        char **strval;

        /* Get next header name. */
        if (StreamReadUntilChars(hs->s_stream, ":\n", "\r", buf, HTTP_MAX_REQUEST_SIZE) <= 0) {
            /* Broken connection, stop parsing. */
            break;
        }
        if (*buf == '\0') {
            /* Empty line marks the end of the request header. */
            break;
        }
        strval = NULL;
        if (strcasecmp(buf, ct_Accept_Encoding) == 0) {
            strval = &hs->s_req.req_encoding;
        }
        else if (strcasecmp(buf, ct_Authorization) == 0) {
            strval = &hs->s_req.req_auth;
        }
#if HTTP_KEEP_ALIVE_REQ
        else if (strcasecmp(buf, ct_Connection) == 0) {
            got = StreamReadUntilChars(hs->s_stream, "\n", "\r", buf, HTTP_MAX_REQUEST_SIZE);
            if (strcasecmp(buf, ct_close) == 0) {
                hs->s_req.req_connection = HTTP_CONN_CLOSE;
            }
            else if (strcasecmp(buf, ct_Keep_Alive) == 0) {
                hs->s_req.req_connection = HTTP_CONN_KEEP_ALIVE;
            }
        }
#endif
        else if (strcasecmp(buf, ct_Content_Length) == 0) {
            got = StreamReadUntilChars(hs->s_stream, "\n", "\r", buf, HTTP_MAX_REQUEST_SIZE);
            hs->s_req.req_length = atol(buf);
        }
        else if (strcasecmp(buf, ct_Content_Type) == 0) {
            strval = &hs->s_req.req_type;
        }
        else if (strcasecmp(buf, ct_Cookie) == 0) {
            strval = &hs->s_req.req_cookie;
        }
        else if (strcasecmp(buf, ct_Host) == 0) {
            strval = &hs->s_req.req_host;
        }
#if !defined(HTTPD_EXCLUDE_DATE)
        else if (strcasecmp(buf, ct_If_Modified_Since) == 0) {
            got = StreamReadUntilChars(hs->s_stream, "\n", "\r", buf, HTTP_MAX_REQUEST_SIZE);
            hs->s_req.req_ims = RfcTimeParse(buf);
        }
#endif
        else if (strcasecmp(buf, ct_Referer) == 0) {
            strval = &hs->s_req.req_referer;
        }
        else if (strcasecmp(buf, ct_User_Agent) == 0) {
            strval = &hs->s_req.req_agent;
        }
        else {
            got = StreamReadUntilChars(hs->s_stream, "\n", NULL, NULL, 9999);
        }
        if (strval) {
            got = StreamReadUntilChars(hs->s_stream, "\n", "\r", buf, HTTP_MAX_REQUEST_SIZE);
            *strval = strdup(buf);
        }
    } while(1);
#endif

    free(buf);
    return 0;
}

int HttpRegisterRootPath(char *path)
{
    if (http_root_path) {
        free(http_root_path);
    }
    if (path) {
        http_root_path = strdup(path);
        if (http_root_path == NULL) {
            return -1;
        }
    } else {
        http_root_path = NULL;
    }
    return 0;
}

void HttpdClientHandler(HTTP_STREAM *sp)
{
    char *filename;
    HTTPD_SESSION *hs;
    HTTP_REQUEST *req;
    MEDIA_TYPE_ENTRY *mt;
    int err = 0;

    hs = malloc(sizeof(HTTPD_SESSION));
    if (hs) {
        do {
            hs->s_stream = sp;
            req = &hs->s_req;

            if (HttpParseHeader(hs)) {
                break;
            }
            if ((*httpd_auth_validator) (hs)) {
                err = 401;
            }
            else if ((*httpd_loc_redirector) (hs)) {
                /* No redirection available. */
                filename = AllocConcatStrings(HTTP_ROOT, req->req_url, NULL);
                if (filename) {
                    mt = GetMediaTypeEntry(filename);
                    if (mt == NULL) {
                        err = 404;
                    } else {
                        mt->media_handler(hs, mt, filename);
                    }
                    free(filename);
                } else {
                    err = 404;
                }
            }
            if (err) {
                HttpSendError(hs, err);
            }
            free(req->req_url);
            free(req->req_query);
            free(req->req_argp);
            free(req->req_argn);
#if HTTP_VERSION >= 0x10
            free(req->req_realm);
            free(req->req_type);
            free(req->req_cookie);
            free(req->req_auth);
            free(req->req_agent);
            free(req->req_referer);
            free(req->req_host);
            free(req->req_encoding);
            free(req->req_bnd_dispo);
            free(req->req_bnd_type);
#endif
        } while(req->req_connection == HTTP_CONN_KEEP_ALIVE);

        free(hs);
    }
}
