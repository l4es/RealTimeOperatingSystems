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

#include <pro/uhttp/envvars.h>

extern ISC_LIST(HTTP_ENVVAR_ENTRY) envVarList;

/* Extensions must be sorted in reverse order. */
static HTTP_ENVVAR_ENTRY envvar_defaults[] = {
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "SERVER_PORT", HttpStreamInfo, -SITEM_SERVER_PORT },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "SERVER_NAME", HttpStreamInfo, -SITEM_SERVER_NAME },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "SERVER_ADDR", HttpStreamInfo, -SITEM_SERVER_ADDR },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "SCRIPT_NAME", HttpSessionInfo, -HSITEM_SCRIPT_NAME },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "SCRIPT_FILENAME", HttpSessionInfo, -HSITEM_SCRIPT_FILENAME },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "REQUEST_URI", HttpSessionInfo, -HSITEM_REQUEST_URI },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "REQUEST_METHOD", HttpSessionInfo, -HSITEM_REQUEST_METHOD },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "REMOTE_PORT", HttpStreamInfo, -SITEM_REMOTE_PORT },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "REMOTE_ADDR", HttpStreamInfo, -SITEM_REMOTE_ADDR },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "QUERY_STRING_UNESCAPED", HttpSessionInfo, -HSITEM_QUERY_STRING_UNESCAPED },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "QUERY_STRING", HttpSessionInfo, -HSITEM_QUERY_STRING },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "HTTP_USER_AGENT", HttpSessionInfo, -HSITEM_HTTP_USER_AGENT },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "HTTP_REFERER", HttpSessionInfo, -HSITEM_HTTP_REFERER },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "HTTP_HOST", HttpSessionInfo, -HSITEM_HTTP_HOST },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "HTTP_COOKIE", HttpSessionInfo, -HSITEM_HTTP_COOKIE },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "HTTP_CONNECTION", HttpSessionInfo, -HSITEM_HTTP_CONNECTION },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "HTTP_ACCEPT_ENCODING", HttpSessionInfo, -HSITEM_HTTP_ACCEPT_ENCODING },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "DOCUMENT_ROOT", HttpSessionInfo, -HSITEM_DOCUMENT_ROOT },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "DOCUMENT_NAME", HttpSessionInfo, -HSITEM_DOCUMENT_NAME },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "DATE_LOCAL", HttpSessionInfo, -HSITEM_DATE_LOCAL },
    { ISC_LINK_INITIAL(HTTP_ENVVAR_ENTRY), "DATE_GMT", HttpSessionInfo, -HSITEM_DATE_GMT }
};

#define ENVVAR_DEFAULTS (sizeof(envvar_defaults) / sizeof(HTTP_ENVVAR_ENTRY))

int EnvInitDefaults(void)
{
    int i;

    ISC_LIST_INIT(envVarList);
    for (i = 0; i < (int)ENVVAR_DEFAULTS; i++) {
    ISC_LIST_APPEND(envVarList, &envvar_defaults[i], env_link);
    }
    return 0;
}
