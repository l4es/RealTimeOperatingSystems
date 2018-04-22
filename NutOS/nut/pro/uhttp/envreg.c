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

#include <stdlib.h>
#include <string.h>
#include <memdebug.h>

extern ISC_LIST(HTTP_ENVVAR_ENTRY) envVarList;

const HTTP_ENVVAR_ENTRY* EnvVarGetEntry(const char *name)
{
    HTTP_ENVVAR_ENTRY *env;
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
    return env;
}

int EnvRegisterVariable(char *name, HTTP_ENVVAR_HANDLER handler, int item)
{
    int rc = -1;
    HTTP_ENVVAR_ENTRY *cur;
    HTTP_ENVVAR_ENTRY *env;
    int i = -1;

    for (cur = ISC_LIST_HEAD(envVarList); cur; cur = ISC_LIST_NEXT(cur, env_link)) {
        i = strcasecmp(cur->env_name, name);
        if (i <= 0) {
            break;
        }
    }

    if (i == 0) {
        /* Existing entry. */
        if (handler) {
            /* Override entry. */
            cur->env_handler = handler;
        } else {
            /* Remove entry. */
            ISC_LIST_UNLINK_TYPE(envVarList, cur, env_link, HTTP_ENVVAR_ENTRY);
            if (cur->env_index < 0) {
                free(cur->env_name);
                free(cur);
            }
        }
        rc = 0;
    }
    if (handler && i) {
        /* New entry. */
        env = (HTTP_ENVVAR_ENTRY *) calloc(1, sizeof(HTTP_ENVVAR_ENTRY));
        if (env) {
            env->env_name = strdup(name);
            if (env->env_name) {
                env->env_handler = handler;
                env->env_index = item;
                if (cur) {
                    /* Insert entry, maintaining descending sort order. */
                    ISC_LIST_INSERTBEFORE(envVarList, cur, env, env_link);
                } else {
                    /* Append entry to the list. */
                    ISC_LIST_APPEND(envVarList, env, env_link);
                }
                /* Indicate success and let the internal client handler
                    use our validator. */
                rc = 0;
            }
        }
    }
    return rc;
}

