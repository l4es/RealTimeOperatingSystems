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

#include <pro/upnp.h>
#include <string.h>

/* Use for local debugging only. Do NOT include into NUTDEBUG.
#define DEBUG_UPNP
*/
#if defined(DEBUG_UPNP)
#include <stdio.h>
#endif

static const char *stype_names[] = {
    "?", "ui1", "ui2", "ui4", "i1", "i2", "i4", "int", "r4", "r8",
    "number", "fixed.14.4", "float", "char", "string", "date",
    "dateTime", "dateTime.tz", "boolean", "bin.base64", "bin.hex",
    "uri", "uuid"
};

void UpnpDumpDevice(const SSDP_DEVICE *sdev)
{
#if defined(DEBUG_UPNP)
    UPNP_DEVICE_INFO *udev = sdev->sdev_info;
    SSDP_SERVICE *ssvc;


    printf("Device ");
    if (udev) {
        puts(udev->udev_name);
    } else {
        puts("NoName");
    }
    printf("  UUID %s\n", sdev->sdev_uuid);
    printf("  %s:device:%s\n", sdev->sdev_domain, sdev->sdev_type);
    printf("  Description %s\n", sdev->sdev_url_desc);
    printf("  Cache %d\n", sdev->sdev_cache);

    for (ssvc = sdev->sdev_svc; ssvc; ssvc = ssvc->ssvc_next) {
        UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;

        printf("  Service %s:%s\n", ssvc->ssvc_domain, ssvc->ssvc_type);
        if (usvc) {
            SOAP_PROCEDURE *proc;
            SOAP_ARG *arg;

            printf("    Description %s\n", usvc->usvc_url_scpd);
            printf("    Control %s\n", usvc->usvc_url_ctrl);
            printf("    Event %s\n", usvc->usvc_url_event);
            for (proc = usvc->usvc_proc; proc; proc = proc->proc_next) {
                printf("    Procedure %s\n", proc->proc_name);
                for (arg = proc->proc_argi; arg; arg = arg->arg_next) {
                    UPNP_VARIABLE *uvar = arg->arg_info;

                    printf("      in:%s", arg->arg_name);
                    if (uvar) {
                        printf(" %s:%s=%s\n", UpnpVarTypeString(uvar->ustv_type), uvar->ustv_name, uvar->ustv_default);
                    } else {
                        puts(" unrelated");
                    }
                }
                for (arg = proc->proc_argo; arg; arg = arg->arg_next) {
                    UPNP_VARIABLE *uvar = arg->arg_info;

                    printf("      out:%s", arg->arg_name);
                    if (uvar) {
                        printf(" %s:%s=%s\n", UpnpVarTypeString(uvar->ustv_type), uvar->ustv_name, uvar->ustv_default);
                    } else {
                        puts(" unrelated");
                    }
                }
            }
        }
    }
#endif
}

int UpnpVarTypeIndex(const char *string)
{
    int idx;

    for (idx = 0; idx < sizeof(stype_names) / sizeof(stype_names[0]); idx++) {
        if (strcasecmp(stype_names[idx], string) == 0) {
            return idx;
        }
    }
    return 0;
}

const char *UpnpVarTypeString(int type)
{
    if (type < sizeof(stype_names) / sizeof(stype_names[0])) {
        return stype_names[type];
    }
    return stype_names[0];
}
