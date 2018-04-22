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

#include <stdlib.h>
#include <string.h>

#include <pro/uhttp/utils.h>
#include <pro/uhttp/modules/mod_auth_basic.h>

static ISC_LIST(AUTH_BASIC_ENTRY) authInfoList = ISC_LIST_INITIAL_TYPE(AUTH_BASIC_ENTRY);

const AUTH_BASIC_ENTRY *HttpAuthBasicLookup(const char *resource, const char *login, int best)
{
    AUTH_BASIC_ENTRY *auth;
    int i;

    HTTP_ASSERT(resource != NULL);

    /* Walk down the list, which is kept in descending order. */
    for (auth = ISC_LIST_HEAD(authInfoList); auth; auth = ISC_LIST_NEXT(auth, auth_link)) {
        if (best) {
            /* Check if the leading part of the resource fits. */
            i = strncasecmp(auth->auth_path, resource, strlen(auth->auth_path));
        } else {
            /* Check if path and resouce fit exactly. */
            i = strcasecmp(auth->auth_path, resource);
        }
        if (i < 0) {
            /* Items are in descending order. Thus, all items below
               will not match, not even partially. Give up! */
            auth = NULL;
            break;
        }
        if (i == 0) {
            /* Resource matches. */
            if (login) {
                /* Login has been given, check for exact macth. */
                if (strcmp(auth->auth_login, login) == 0) {
                    /* This is the requested item. */
                    break;
                }
            } else {
                /* No login provided, we found the requested item. */
                break;
            }
        }
    }
    return auth;
}

int HttpRegisterAuthBasic(const char *path, const char *login, const char *realm)
{
    int rc = -1;
    AUTH_BASIC_ENTRY *auth;
    AUTH_BASIC_ENTRY *cur = NULL;
    int i = -1;

    HTTP_ASSERT(path != NULL);

    for (cur = ISC_LIST_HEAD(authInfoList); cur; cur = ISC_LIST_NEXT(cur, auth_link)) {
        i = strcasecmp(cur->auth_path, path);
        if (i < 0) {
            /* Items are in descending order. Thus, all items below
               will not match, not even partially. */
            break;
        }
        if (i == 0) {
            if (login) {
                /* Verify provided login string. */
                i = strcmp(cur->auth_login, login);
                if (i <= 0) {
                    if (i == 0) {
                        /* Silently ignore duplicates. */
                        rc = 0;
                    }
                    break;
                }
            } else {
                /* No login provided, caller wants to unprotect the resource. */
                auth = cur;
                cur = ISC_LIST_PREV(cur, auth_link);
                ISC_LIST_UNLINK_TYPE(authInfoList, auth, auth_link, AUTH_BASIC_ENTRY);
                free(auth->auth_path);
                free(auth->auth_login);
                free(auth->auth_realm);
                free(auth);
                /* Indicate success, but continue searching additional
                   logins for this resource. */
                rc = 0;
            }
        }
    }
    if (login && i) {
        /* Create a new entry. */
        auth = calloc(1, sizeof(AUTH_BASIC_ENTRY));
        if (auth) {
            auth->auth_path = strdup(path);
            if (auth->auth_path) {
                auth->auth_login = strdup(login);
                if (auth->auth_login) {
                    if (realm) {
                        auth->auth_realm = strdup(realm);
                    } else {
                        auth->auth_realm = NULL;
                    }
                    if (cur) {
                        /* Insert entry, maintaining descending sort order. */
                        ISC_LIST_INSERTBEFORE(authInfoList, cur, auth, auth_link);
                    } else {
                        /* Append entry to the list. */
                        ISC_LIST_APPEND(authInfoList, auth, auth_link);
                    }
                    /* Indicate success and let the internal client handler
                       use our validator. */
                    rc = 0;
                    httpd_auth_validator = HttpAuthBasicValidate;
                } else {
                    free(auth->auth_path);
                }
            } else {
                free(auth);
            }
        }
    }
    return rc;
}

int HttpAuthBasicValidate(HTTPD_SESSION *hs)
{
#if HTTP_VERSION >= 0x10
    int rc = -1;
    const AUTH_BASIC_ENTRY *auth;

    HTTP_ASSERT(hs != NULL);

    /* Check if this resource is protected. */
    auth = HttpAuthBasicLookup(hs->s_req.req_url, NULL, 1);
    if (auth == NULL) {
        /* Unprotected resource, grant access. */
        rc = 0;
    } else {
        /* Protected resource, check authentication. */
        hs->s_req.req_realm = strdup(auth->auth_realm ? auth->auth_realm : auth->auth_path);
        if (hs->s_req.req_auth) {
            /* Accept basic authentication only. */
            if (strncasecmp(hs->s_req.req_auth, "Basic ", 6) == 0) {
                HttpDecodeBase64(hs->s_req.req_auth + 6);
                if (HttpAuthBasicLookup(auth->auth_path, hs->s_req.req_auth + 6, 0)) {
                    rc = 0;
                }
            }
        }
    }
    return rc;
#else
    return 0;
#endif
}
