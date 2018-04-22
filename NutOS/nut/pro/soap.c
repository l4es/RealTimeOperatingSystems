/*
 * Copyright (C) 2012-2013 by egnite GmbH
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

#include <pro/soap.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

SOAP_PROCEDURE *SoapProcByName(SOAP_PROCEDURE *list, const char *name)
{
    while (list && strcasecmp(list->proc_name, name)) {
        list = list->proc_next;
    }
    return list;
}

SOAP_ARG *SoapArgByName(SOAP_PROCEDURE *proc, const char *name, int dir)
{
    SOAP_ARG *arg;

    for (arg = dir ? proc->proc_argo : proc->proc_argi; arg; arg = arg->arg_next) {
        if (strcasecmp(arg->arg_name, name) == 0) {
            break;
        }
    }
    return arg;
}

int SoapProcArgSet(SOAP_PROCEDURE *proc, const char *name, const char *value, int dir)
{
    int rc = -1;
    SOAP_ARG *arg = SoapArgByName(proc, name, dir);

    if (arg) {
        free(arg->arg_val);
        arg->arg_val = value ? strdup(value) : NULL;
        rc = 0;
    }
    return rc;
}

int SoapProcArgSetInt(SOAP_PROCEDURE *proc, const char *name, int value, int dir)
{
    static char str[16];

    sprintf(str, "%d", value);

    return SoapProcArgSet(proc, name, str, dir);
}

const char *SoapProcArgGet(SOAP_PROCEDURE *proc, const char *name, int dir)
{
    char *value = NULL;
    SOAP_ARG *arg = SoapArgByName(proc, name, dir);

    if (arg) {
        value = arg->arg_val;
    }
    return value;
}

int SoapProcArgGetInt(SOAP_PROCEDURE *proc, const char *name, int dir)
{
    int rc = 0;
    const char *value;

    value = SoapProcArgGet(proc, name, dir);
    if (value) {
        rc = atoi(value);
    }
    return rc;
}
