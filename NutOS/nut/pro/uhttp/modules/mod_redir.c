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

#include <pro/uhttp/modules/mod_redir.h>

#include <stdlib.h>
#include <string.h>

static ISC_LIST(HTTP_LOCATION) locationList = ISC_LIST_INITIAL_TYPE(HTTP_LOCATION);

HTTP_LOCATION *HttpLocationLookup(const char *uri)
{
    HTTP_LOCATION *loc;
    int i;

    for (loc = ISC_LIST_HEAD(locationList); loc; loc = ISC_LIST_NEXT(loc, loc_link)) {
        i = strcasecmp(loc->loc_uri, uri);
        if (i <= 0) {
            if (i) {
                loc = NULL;
            }
            break;
        }
    }
    return loc;
}

int HttpRegisterRedir(const char *uri, const char *redir, int response)
{
    int rc = -1;
    HTTP_LOCATION *loc;
    HTTP_LOCATION *cur;
    int i = -1;

    for (cur = ISC_LIST_HEAD(locationList); cur; cur = ISC_LIST_NEXT(cur, loc_link)) {
        i = strcasecmp(cur->loc_uri, uri);
        if (i == 0) {
            /* If uri is equal, override current redirection */
            free(cur->loc_redir);            /* Clear old redir */
            cur->loc_redir = strdup(redir);  /* Use new redirect */
            cur->loc_response = response;    /* Use new response */
            rc = 0;
            break;
        }
        if (i < 0) {
            break;
        }
    }
    if (i) {
        loc = calloc(1, sizeof(HTTP_LOCATION));
        if (loc) {
            loc->loc_uri = strdup(uri);
            if (loc->loc_uri) {
                loc->loc_redir = strdup(redir);
                if (loc->loc_redir) {
                    loc->loc_response = response;
                    if (cur) {
                        ISC_LIST_INSERTBEFORE(locationList, cur, loc, loc_link);
                    } else {
                        ISC_LIST_APPEND(locationList, loc, loc_link);
                    }
                    rc = 0;
                    httpd_loc_redirector = HttpLocationRedir;
                } else {
                    free(loc->loc_uri);
                }
            } else {
                free(loc);
            }
        }
    }
    return rc;
}

int HttpLocationRedir(HTTPD_SESSION *hs)
{
    HTTP_LOCATION *loc;

    HTTP_ASSERT(hs != NULL);

    loc = HttpLocationLookup(hs->s_req.req_url);
    if (loc) {
        HttpSendRedirection(hs, loc->loc_response, loc->loc_redir, NULL);
        return 0;
    }
    return -1;
}
