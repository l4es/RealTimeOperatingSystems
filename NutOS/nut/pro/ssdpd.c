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

#include <sys/confnet.h>
#include <sys/timer.h>
#include <arpa/inet.h>

#include <pro/ssdp.h>

#include <string.h>
#include <stdlib.h>

static SSDP_DEVICE *regdev_root;

static void AddResponseHeaders(HTTPU_SESSION *s, const char *location, const char *type)
{
    HttpuAddHeader(s, NULL, "HTTP/1.1 200 OK", NULL);
    HttpuAddHeader(s, "CACHE-CONTROL", "max-age=1800", NULL);
    HttpuAddHeader(s, "EXT", NULL);
    HttpuAddHeader(s, "LOCATION", "http://", inet_ntoa(confnet.cdn_ip_addr), "/", location, "?", type, NULL);
    HttpuAddHeader(s, "SERVER", "NutOS/5.0, UPnP/1.0, Ethernut/5.0", NULL);
}

static void AddNotifyHeaders(HTTPU_SESSION *s, const char *location, const char *type)
{
    HttpuAddHeader(s, NULL, "NOTIFY * HTTP/1.1", NULL);
    HttpuAddHeader(s, "HOST", ct_239_255_255_250, ":1900", NULL);
    HttpuAddHeader(s, "CACHE-CONTROL", "max-age=1800", NULL);
    HttpuAddHeader(s, "LOCATION", "http://", inet_ntoa(confnet.cdn_ip_addr), "/", location, "?", type, NULL);
    HttpuAddHeader(s, "SERVER", "NutOS/5.0, UPnP/1.0, Ethernut/5.0", NULL);
}

static void SendDeviceResponse(HTTPU_SESSION *s, SSDP_DEVICE *sdev, int mode)
{
    NutSleep(100);

    AddResponseHeaders(s, sdev->sdev_url_desc, sdev->sdev_type);
    if (mode == 1) {
        HttpuAddHeader(s, "ST", ct_upnp_rootdevice, NULL);
        HttpuAddHeader(s, "USN", ct_uuid_, sdev->sdev_uuid, "::", ct_upnp_rootdevice, NULL);
    }
    else if (mode == 2) {
        HttpuAddHeader(s, "ST", "urn:", sdev->sdev_domain, ":device:", sdev->sdev_type, ":1", NULL);
        HttpuAddHeader(s, "USN", ct_uuid_, sdev->sdev_uuid, "::urn:", sdev->sdev_domain, ":device:", sdev->sdev_type, ":1", NULL);
    }
    else {
        HttpuAddHeader(s, "ST", ct_uuid_, sdev->sdev_uuid, NULL);
        HttpuAddHeader(s, "USN", ct_uuid_, sdev->sdev_uuid, NULL);
    }
    HttpuRespond(s);
}

static void SendDeviceNotify(HTTPU_SESSION *s, SSDP_DEVICE *sdev, int mode)
{
    NutSleep(100);

    AddNotifyHeaders(s, sdev->sdev_url_desc, sdev->sdev_type);
    if (mode == 1) {
        HttpuAddHeader(s, "ST", ct_upnp_rootdevice, NULL);
        HttpuAddHeader(s, "USN", ct_uuid_, sdev->sdev_uuid, "::", ct_upnp_rootdevice, NULL);
    }
    else if (mode == 2) {
        HttpuAddHeader(s, "ST", "urn:", sdev->sdev_domain, ":device:", sdev->sdev_type, ":1", NULL);
        HttpuAddHeader(s, "USN", ct_uuid_, sdev->sdev_uuid, "::urn:", sdev->sdev_domain, ":device:", sdev->sdev_type, ":1", NULL);
    }
    else {
        HttpuAddHeader(s, "ST", ct_uuid_, sdev->sdev_uuid, NULL);
        HttpuAddHeader(s, "USN", ct_uuid_, sdev->sdev_uuid, NULL);
    }
    HttpuSend(s, inet_addr(ct_239_255_255_250), 1900);
}

static void SendServiceResponse(HTTPU_SESSION *s, SSDP_SERVICE *svc)
{
    AddResponseHeaders(s, svc->ssvc_dev->sdev_url_desc, svc->ssvc_dev->sdev_type);
    HttpuAddHeader(s, "ST", "urn:", svc->ssvc_domain, ":service:", svc->ssvc_type, ":1", NULL);
    HttpuAddHeader(s, "USN", ct_uuid_, svc->ssvc_dev->sdev_uuid, "::", "urn:", svc->ssvc_domain, ":service:", svc->ssvc_type, ":1", NULL);
    HttpuRespond(s);
}

static void SendServiceNotify(HTTPU_SESSION *s, SSDP_SERVICE *svc)
{
    AddNotifyHeaders(s, svc->ssvc_dev->sdev_url_desc, svc->ssvc_dev->sdev_type);
    HttpuAddHeader(s, "ST", "urn:", svc->ssvc_domain, ":service:", svc->ssvc_type, ":1", NULL);
    HttpuAddHeader(s, "USN", ct_uuid_, svc->ssvc_dev->sdev_uuid, "::", "urn:", svc->ssvc_domain, ":service:", svc->ssvc_type, ":1", NULL);
    HttpuSend(s, inet_addr(ct_239_255_255_250), 1900);
}

static void SendUuidResponse(HTTPU_SESSION *s, SSDP_DEVICE *tree, const char *uuid)
{
    SSDP_DEVICE *dev;

    for (dev = tree; dev; dev = dev->sdev_next) {
        if (strcmp(dev->sdev_uuid, uuid) == 0) {
            SendDeviceResponse(s, dev, 0);
        }
        SendUuidResponse(s, dev->sdev_embedded, uuid);
    }
}

static void SendDeviceTypeResponse(HTTPU_SESSION *s, SSDP_DEVICE *tree, const char *domain, const char *type)
{
    SSDP_DEVICE *dev;

    for (dev = tree; dev; dev = dev->sdev_next) {
        if (strcmp(dev->sdev_domain, domain) == 0 && strcmp(dev->sdev_type, type) == 0) {
            SendDeviceResponse(s, dev, 2);
        }
        SendDeviceTypeResponse(s, dev->sdev_embedded, domain, type);
    }
}

static void SendServiceTypeResponse(HTTPU_SESSION *s, SSDP_DEVICE *tree, const char *domain, const char *st)
{
    SSDP_DEVICE *dev;

    for (dev = tree; dev; dev = dev->sdev_next) {
        SSDP_SERVICE *svc;

        for (svc = dev->sdev_svc; svc; svc = svc->ssvc_next) {
            if (strcmp(svc->ssvc_domain, domain) == 0 && strcmp(svc->ssvc_type, st) == 0) {
                SendServiceResponse(s, svc);
            }
        }
        SendServiceTypeResponse(s, dev->sdev_embedded, domain, st);
    }
}

static void SendRootResponse(HTTPU_SESSION *s, SSDP_DEVICE *tree)
{
    SSDP_DEVICE *dev;

    for (dev = tree; dev; dev = dev->sdev_next) {
        SendDeviceResponse(s, dev, 1);
    }
}

static void SendAllResponse(HTTPU_SESSION *s, SSDP_DEVICE *tree)
{
    SSDP_DEVICE *dev;
    SSDP_SERVICE *svc;

    for (dev = tree; dev; dev = dev->sdev_next) {
        SendDeviceResponse(s, dev, 1);
        SendDeviceResponse(s, dev, 0);
        SendDeviceResponse(s, dev, 2);
        SendAllResponse(s, dev->sdev_embedded);
        for (svc = dev->sdev_svc; svc; svc = svc->ssvc_next) {
            SendServiceResponse(s, svc);
        }
    }
}

/*
 * This function implements an SSPD responder. It is called by the SSDP
 * background thread for each incoming M-SEARCH request.
 */
static void SendSearchResponse(HTTPU_SESSION *s)
{
    const char *st = HttpuGetHeader(&s->s_rcvhdr, "ST");

    if (strcmp(st, "ssdp:all") == 0) {
        SendAllResponse(s, regdev_root);
    }
    else if (strcmp(st, ct_upnp_rootdevice) == 0) {
        SendRootResponse(s, regdev_root);
    }
    else {
        char *stcomp[5];

        SsdpSplitWords(strdup(st), ':', stcomp, 5);
        if (strcmp(stcomp[0], "uuid") == 0) {
            SendUuidResponse(s, regdev_root, stcomp[1]);
        }
        else if (strcmp(stcomp[0], "urn") == 0) {
            if (strcmp(stcomp[2], "device") == 0) {
                SendDeviceTypeResponse(s, regdev_root, stcomp[1], stcomp[3]);
            }
            else if (strcmp(stcomp[2], "service") == 0) {
                SendServiceTypeResponse(s, regdev_root, stcomp[1], stcomp[3]);
            }
        }
        free(stcomp[0]);
    }
}

int SsdpSendNotifications(HTTPU_SESSION *s, char *note)
{
    static SSDP_DEVICE *sdev;
    static SSDP_SERVICE *ssvc;
    int rc = 0;

    if (ssvc == NULL) {
        if (sdev == NULL) {
            sdev = regdev_root;
            if (sdev) {
                SendDeviceNotify(s, sdev, 0);
                SendDeviceNotify(s, sdev, 1);
                SendDeviceNotify(s, sdev, 2);
                ssvc = sdev->sdev_svc;
            }
        } else {
            sdev = sdev->sdev_next;
            if (sdev) {
                SendDeviceNotify(s, sdev, 0);
                SendDeviceNotify(s, sdev, 1);
                SendDeviceNotify(s, sdev, 2);
                ssvc = sdev->sdev_svc;
            } else {
                rc = 900;
            }
        }
    } else {
        SendServiceNotify(s, ssvc);
        ssvc = ssvc->ssvc_next;
    }
    return rc;
}

int SsdpRegisterDeviceTree(SSDP_DEVICE *tree)
{
    regdev_root = tree;

    return SsdpRegisterResponder(SendSearchResponse);
}
