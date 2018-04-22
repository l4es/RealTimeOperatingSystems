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

#include <pro/uhttp/modules/mod_cgi_func.h>
#include <pro/upnp.h>

#include <string.h>

/* Use for local debugging only. Do NOT include into NUTDEBUG.
#define DEBUG_UPNPC
*/
#ifdef DEBUG_UPNPC
#include <stdio.h>
#endif

static SSDP_DEVICE *device_registration;

static void XmlHead(HTTP_STREAM *stream)
{
    HttpSendStreamHeaderTop(stream, 200);
    s_puts("SERVER: NutOS/5.0 UPnP/1.0 TestUPnP/1.0\r\n", stream);
    HttpSendStreamHeaderBottom(stream, "text", "xml", HTTP_CONN_CLOSE, -1);
    s_puts("<?xml version=\"1.0\"?>\r\n", stream);
}

static void WritePrepTag(HTTP_STREAM *stream, const char *tag, const char *val, const char *prepend)
{
    if (val) {
        s_printf(stream, "<%s>", tag);
        if (prepend && *val == '+') {
            val++;
            s_puts(prepend, stream);
        }
        s_printf(stream, "%s</%s>\r\n", val, tag);
    }
}

static void WriteTag(HTTP_STREAM *stream, const char *tag, const char *val)
{
    WritePrepTag(stream, tag, val, NULL);
}

static void SpecVersion(HTTP_STREAM *stream)
{
    s_puts("<specVersion>\r\n", stream);
    s_puts("<major>1</major>\r\n", stream);
    s_puts("<minor>0</minor>\r\n", stream);
    s_puts("</specVersion>\r\n", stream);
}

/*!
 * \brief Send device description.
 *
 * This uHTTP CGI function will be registered when UpnpRegisterDeviceTree()
 * is called for the first time.
 */
static int UpnpCgiDeviceDescription(HTTPD_SESSION *hs)
{
    HTTP_STREAM *stream = hs->s_stream;
    SSDP_DEVICE *sdev = NULL;
    SSDP_SERVICE *ssvc;
    char *type;

    type = HttpArgParseFirst(&hs->s_req);
    if (type) {
        for (sdev = device_registration; sdev; sdev = sdev->sdev_next) {
            if (strcmp(sdev->sdev_type, type) == 0) {
                break;
            }
        }
    }
    if (sdev) {
        UPNP_DEVICE_INFO *udev = sdev->sdev_info;

        XmlHead(hs->s_stream);
        s_puts("<root xmlns=\"urn:schemas-upnp-org:device-1-0\">\r\n", hs->s_stream);
        SpecVersion(stream);

        s_puts("<device>\r\n", stream);
        s_printf(stream, "<deviceType>urn:%s:device:%s:1</deviceType>\r\n", sdev->sdev_domain, sdev->sdev_type);
        WriteTag(stream, "friendlyName", udev->udev_name);
        WriteTag(stream, "manufacturer", udev->udev_mnf->umnf_name);
        WritePrepTag(stream, "manufacturerURL", udev->udev_mnf->umnf_url, "http://");
        WriteTag(stream, "modelDescription", udev->udev_mdl->umdl_desc);
        WriteTag(stream, "modelName", udev->udev_mdl->umdl_name);
        WriteTag(stream, "modelNumber", udev->udev_mdl->umdl_num);
        WritePrepTag(stream, "modelURL", udev->udev_mdl->umdl_url, "http://");
        WriteTag(stream, "UDN", sdev->sdev_uuid);
        if (sdev->sdev_svc) {
            s_puts("<serviceList>\r\n", stream);
            for (ssvc = sdev->sdev_svc; ssvc; ssvc = ssvc->ssvc_next) {
                UPNP_SERVICE_INFO *usvc = sdev->sdev_svc->ssvc_info;

                s_puts("<service>\r\n", stream);
                s_printf(stream, "<serviceType>urn:schemas-upnp-org:service:%s:1</serviceType>\r\n", sdev->sdev_svc->ssvc_type);
                s_printf(stream, "<serviceId>urn:upnp-org:serviceId:%s:1</serviceId>\r\n", sdev->sdev_svc->ssvc_type);
                s_printf(stream, "<SCPDURL>%s?%s=%s</SCPDURL>", usvc->usvc_url_scpd, sdev->sdev_type, ssvc->ssvc_type);
                s_printf(stream, "<controlURL>%s?%s=%s</controlURL>", usvc->usvc_url_ctrl, sdev->sdev_type, ssvc->ssvc_type);
                s_printf(stream, "<eventSubURL>%s?%s=%s</eventSubURL>", usvc->usvc_url_event, sdev->sdev_type, ssvc->ssvc_type);
                s_puts("</service>\r\n", stream);
            }
            s_puts("</serviceList>\r\n", stream);
        }
        WriteTag(stream, "presentationURL", udev->udev_presentation);
        s_puts("</device>\r\n", stream);
        s_puts("</root>\r\n", stream);
    }
    s_flush(stream);

    return 0;
}

/*!
 * \brief Send service description.
 *
 * This uHTTP CGI function will be registered when UpnpRegisterDeviceTree()
 * is called for the first time.
 */
static int UpnpCgiServiceDescription(HTTPD_SESSION *hs)
{
    SSDP_SERVICE *ssvc = NULL;
    SSDP_DEVICE *sdev = NULL;
    HTTP_STREAM *stream = hs->s_stream;
    const char *dev_type;

    dev_type = HttpArgParseFirst(&hs->s_req);
    if (dev_type) {
        const char *svc_type = HttpArgValue(&hs->s_req);

        if (dev_type) {
            for (sdev = device_registration; sdev; sdev = sdev->sdev_next) {
                if (strcmp(sdev->sdev_type, dev_type) == 0) {
                    for (ssvc = sdev->sdev_svc; ssvc; ssvc = ssvc->ssvc_next) {
                        if (strcmp(ssvc->ssvc_type, svc_type) == 0) {
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
    if (ssvc) {
        SOAP_PROCEDURE *act;
        UPNP_VARIABLE *stv;
        UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;

        XmlHead(hs->s_stream);
        s_puts("<scpd xmlns=\"urn:schemas-upnp-org:service-1-0\">\r\n", stream);
        SpecVersion(stream);

        act = usvc->usvc_proc;
        if (act) {
            s_puts("<actionList>\r\n", stream);
            do {
                s_puts("<action>\r\n", stream);
                WriteTag(stream, "name", act->proc_name);
                if (act->proc_argi || act->proc_argo) {
                    SOAP_ARG *arg;

                    s_puts("<argumentList>\r\n", stream);
                    for (arg = act->proc_argi; arg; arg = arg->arg_next) {
                        s_puts("<argument>\r\n", stream);
                        WriteTag(stream, "name", arg->arg_name);
                        WriteTag(stream, "relatedStateVariable", ((UPNP_VARIABLE *) arg->arg_info)->ustv_name);
                        WriteTag(stream, "direction", "in");
                        s_puts("</argument>\r\n", stream);
                    }
                    for (arg = act->proc_argo; arg; arg = (SOAP_ARG *) arg->arg_next) {
                        s_puts("<argument>\r\n", stream);
                        WriteTag(stream, "name", arg->arg_name);
                        WriteTag(stream, "relatedStateVariable", ((UPNP_VARIABLE *) arg->arg_info)->ustv_name);
                        WriteTag(stream, "direction", "out");
                        s_puts("</argument>\r\n", stream);
                    }
                    s_puts("</argumentList>\r\n", stream);
                }
                s_puts("</action>\r\n", stream);
            } while ((act = act->proc_next) != NULL);
            s_puts("</actionList>\r\n", stream);
        }

        stv = usvc->usvc_stv;
        if (stv) {
            s_puts("<serviceStateTable>\r\n", stream);
            do {
                s_printf(stream, "<stateVariable sendEvents=\"%s\">\r\n", stv->ustv_events ? "yes" : "no");
                WriteTag(stream, "name", stv->ustv_name);
                WriteTag(stream, "dataType", UpnpVarTypeString(stv->ustv_type));
                WriteTag(stream, "defaultValue", stv->ustv_default);
                s_puts("</stateVariable>\r\n", stream);
            } while((stv = stv->ustv_next) != NULL);
            s_puts("</serviceStateTable>\r\n", stream);
        }
        s_puts("</scpd>\r\n", stream);
    }
    s_flush(stream);

    return 0;
}

int UpnpRegisterDeviceTree(SSDP_DEVICE *parent, SSDP_DEVICE *sdev)
{
    int rc = 0;
    static uint_fast8_t initialized;

    if (parent) {
        parent->sdev_next = (SSDP_DEVICE *) sdev;
    } else {
        sdev->sdev_next = (SSDP_DEVICE *) device_registration;
        device_registration = sdev;
        if (!initialized) {
            HttpRegisterCgiFunction(sdev->sdev_url_desc, UpnpCgiDeviceDescription);
            HttpRegisterCgiFunction(((UPNP_SERVICE_INFO *) sdev->sdev_svc->ssvc_info)->usvc_url_scpd, UpnpCgiServiceDescription);
        }
        rc = SsdpRegisterDeviceTree(device_registration);
    }
    return rc;
}

