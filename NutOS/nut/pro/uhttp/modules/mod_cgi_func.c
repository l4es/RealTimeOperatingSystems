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

#include <pro/uhttp/modules/mod_cgi_func.h>

#include <stdlib.h>
#include <string.h>

static ISC_LIST(HTTP_CGI_FUNCTION) cgiFunctionList = ISC_LIST_INITIAL_TYPE(HTTP_CGI_FUNCTION);

HTTP_CGI_FUNCTION *HttpCgiFunctionLookup(const char *uri)
{
    HTTP_CGI_FUNCTION *cgi;
    int i;

    HTTP_ASSERT(uri != NULL);

    for (cgi = ISC_LIST_HEAD(cgiFunctionList); cgi; cgi = ISC_LIST_NEXT(cgi, cgi_link)) {
        i = strcasecmp(cgi->cgi_uri, uri);
        if (i <= 0) {
            if (i) {
                cgi = NULL;
            }
            break;
        }
    }
    return cgi;
}

int HttpRegisterCgiFunction(const char *uri, HTTP_CGI_HANDLER handler)
{
    int rc = -1;
    HTTP_CGI_FUNCTION *cur = NULL;
    int i = -1;

    HTTP_ASSERT(uri != NULL);

    for (cur = ISC_LIST_HEAD(cgiFunctionList); cur; cur = ISC_LIST_NEXT(cur, cgi_link)) {
        i = strcasecmp(cur->cgi_uri, uri);
        if (i <= 0) {
            break;
        }
    }
    if (i) {
        HTTP_CGI_FUNCTION *cgi;

        cgi = calloc(1, sizeof(HTTP_CGI_FUNCTION));
        if (cgi) {
            cgi->cgi_uri = strdup(uri);
            if (cgi->cgi_uri) {
                cgi->cgi_handler = handler;
                if (cur) {
                    ISC_LIST_INSERTBEFORE(cgiFunctionList, cur, cgi, cgi_link);
                } else {
                    ISC_LIST_APPEND(cgiFunctionList, cgi, cgi_link);
                }
                rc = 0;
            } else {
                free(cgi);
            }
        }
    }
    else if (handler) {
        /* Override registered handler. */
        cur->cgi_handler = handler;
    }
    else {
        /* Remove registered handler. */
        ISC_LIST_UNLINK_TYPE(cgiFunctionList, cur, cgi_link, HTTP_CGI_FUNCTION);
        free(cur->cgi_uri);
        free(cur);
    }
    return rc;
}

int HttpCgiFunctionHandler(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *filepath)
{
    HTTP_CGI_FUNCTION *cgi;
    const char *pp;

    (void)mt;

    HTTP_ASSERT(filepath != NULL);

    if (strncasecmp(filepath, HTTP_ROOT, strlen(HTTP_ROOT)) == 0) {
        pp = filepath + strlen(HTTP_ROOT);
    }
    else {
        pp = filepath + (*filepath == '/');
    }
    cgi = HttpCgiFunctionLookup(pp);
    if (cgi) {
        (*cgi->cgi_handler)(hs);
    }
    return 0;
}
