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

#include <pro/uhttp/streamio.h>
#include <pro/uhttp/utils.h>
#include <pro/uhttp/envvars.h>

#include <stdlib.h>
#include <string.h>
#include <memdebug.h>

/*@cond */
ISC_LIST(HTTP_ENVVAR_ENTRY) envVarList = ISC_LIST_INITIAL_TYPE(HTTP_ENVVAR_ENTRY);
/*@endcond */

const char* HttpSessionInfo(HTTPD_SESSION *hs, int item)
{
    static char *env_value;
    const char *vp = NULL;
#if !defined(HTTPD_EXCLUDE_DATE)
    time_t now;
#endif

    switch (item) {
#if !defined(HTTPD_EXCLUDE_DATE)
    case HSITEM_DATE_GMT:
        time(&now);
        vp = Rfc1123TimeString(gmtime(&now));
        break;
    case HSITEM_DATE_LOCAL:
        time(&now);
        vp = Rfc1123TimeString(localtime(&now));
        break;
#endif
    case HSITEM_DOCUMENT_NAME:
        vp = strrchr(hs->s_req.req_url, '/');
        if (vp) {
            vp++;
        } else {
            vp = hs->s_req.req_url;
        }
        break;
    case HSITEM_DOCUMENT_ROOT:
        vp = HTTP_ROOT;
        break;
#if HTTP_VERSION >= 0x10
    case HSITEM_HTTP_ACCEPT_ENCODING:
        vp = hs->s_req.req_encoding;
        break;
    case HSITEM_HTTP_CONNECTION:
        if (hs->s_req.req_connection == HTTP_CONN_KEEP_ALIVE) {
            vp = ct_Keep_Alive;
        } else {
            vp = ct_close;
        }
        break;
    case HSITEM_HTTP_COOKIE:
        break;
    case HSITEM_HTTP_HOST:
        vp = hs->s_req.req_host;
        break;
    case HSITEM_HTTP_REFERER:
        vp = hs->s_req.req_referer;
        break;
    case HSITEM_HTTP_USER_AGENT:
        vp = hs->s_req.req_agent;
        break;
#endif
    case HSITEM_QUERY_STRING:
        vp = hs->s_req.req_query;
        break;
    case HSITEM_QUERY_STRING_UNESCAPED:
        vp = hs->s_req.req_query;
        break;
    case HSITEM_REQUEST_METHOD:
        if (hs->s_req.req_method == HTTP_METHOD_GET) {
            vp = ct_GET;
        }
#if HTTP_VERSION >= 0x10
        else if (hs->s_req.req_method == HTTP_METHOD_HEAD) {
            vp = ct_HEAD;
        }
        else if (hs->s_req.req_method == HTTP_METHOD_POST) {
            vp = ct_POST;
        }
#endif
        break;
    case HSITEM_REQUEST_URI:
        vp = hs->s_req.req_url;
        break;
    }

    if (vp == NULL) {
        vp = "";
    }
    free(env_value);
    env_value = strdup(vp);
    if (item == HSITEM_QUERY_STRING) {
        HttpUrlUnescape(env_value);
    }
    return env_value;
}

const char* HttpStreamInfo(HTTPD_SESSION *hs, int item)
{
    return StreamInfo(hs->s_stream, item);
}

const char* EnvHandler(HTTPD_SESSION *hs, const char *name)
{
    static const char empty;
    HTTP_ENVVAR_ENTRY *env;
    const char *rp = &empty;
    int i;

    /* Locate the entry with the given name. */
    for (env = ISC_LIST_HEAD(envVarList); env; env = ISC_LIST_NEXT(env, env_link)) {
        i = strcasecmp(env->env_name, name);
        if (i <= 0) {
            if (i) {
                env = NULL;
            }
            break;
        }
    }
    if (env) {
        /* Found an entry, call the registered handler. */
        rp = (* env->env_handler)(hs, env->env_index < 0 ? -env->env_index : env->env_index);
    }
    return rp;
}
