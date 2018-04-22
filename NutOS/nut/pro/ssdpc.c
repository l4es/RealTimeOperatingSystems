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

#include <arpa/inet.h>
#include <pro/ssdp.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct _SSDP_OBSERVER SSDP_OBSERVER;

struct _SSDP_OBSERVER {
    SSDP_OBSERVER *sobs_next;
    char *sobs_domain;
    char *sobs_type;
    SSDP_OBSERVER_FUNCTION sobs_cb;
};

static SSDP_DEVICE *ssdp_device_cache;
static SSDP_OBSERVER *ssdp_observer_root;

static int CallObservers(SSDP_SERVICE *ssvc, int_fast8_t removal)
{
    int rc;
    SSDP_OBSERVER *sobs;

    for (rc = 0, sobs = ssdp_observer_root; rc == 0 && sobs; sobs = sobs->sobs_next) {
        if (strcmp(ssvc->ssvc_domain, sobs->sobs_domain) == 0 && strcmp(ssvc->ssvc_type, sobs->sobs_type) == 0) {
            rc = (*sobs->sobs_cb)(ssvc, removal);
        }
    }
    return rc;
}

static char *SsdpDuplicateHeaderValue(const HTTPU_HEADER *hdr, const char *name)
{
    const char *val;

    val = HttpuGetHeader(hdr, name);
    if (*val) {
        return strdup(val);
    }
    return NULL;
}

static SSDP_DEVICE *CacheDevice(const char *uuid, const char *domain, const char *type, const HTTPU_HEADER *hdr)
{
    SSDP_DEVICE *sdev;

    for (sdev = ssdp_device_cache; sdev; sdev = sdev->sdev_next) {
        if (strcmp(sdev->sdev_uuid, uuid) == 0) {
            break;
        }
    }
    if (sdev == NULL) {
        sdev = calloc(1, sizeof(*sdev));
        if (sdev) {
            const char *cp;

            sdev->sdev_uuid = strdup(uuid);
            sdev->sdev_url_desc = SsdpDuplicateHeaderValue(hdr, "LOCATION");
            cp = strchr(HttpuGetHeader(hdr, "CACHE-CONTROL"), '=');
            if (cp) {
                sdev->sdev_cache = atoi(cp + 1);
            }
            sdev->sdev_next = ssdp_device_cache;
            ssdp_device_cache = sdev;
        }
    }
    if (sdev) {
        if (sdev->sdev_domain == NULL && domain) {
            sdev->sdev_domain = strdup(domain);
        }
        if (sdev->sdev_type == NULL && type) {
            sdev->sdev_type = strdup(type);
        }
    }
    return sdev;
}

static SSDP_DEVICE *CacheService(const char *uuid, const char *domain, const char *type, const HTTPU_HEADER *hdr)
{
    SSDP_DEVICE *sdev;

    sdev = CacheDevice(uuid, NULL, NULL, hdr);
    if (sdev) {
        SSDP_SERVICE *ssvc;

        for (ssvc = sdev->sdev_svc; ssvc; ssvc = ssvc->ssvc_next) {
            if (strcmp(ssvc->ssvc_domain, domain) == 0 && strcmp(ssvc->ssvc_type, type) == 0) {
                break;
            }
        }
        if (ssvc == NULL) {
            ssvc = calloc(1, sizeof(*ssvc));
            if (ssvc) {
                ssvc->ssvc_dev = sdev;
                ssvc->ssvc_domain = strdup(domain);
                ssvc->ssvc_type = strdup(type);
                /* Attach service to device and inform our observers. */
                ssvc->ssvc_next = sdev->sdev_svc;
                sdev->sdev_svc = ssvc;
                CallObservers(ssvc, 0);
            }
        }
    }
    return sdev;
}

static void RemoveDevice(const char *uuid)
{
    SSDP_DEVICE *sdev;
    SSDP_DEVICE **sdev_link = &ssdp_device_cache;

    for (sdev = ssdp_device_cache; sdev; sdev = sdev->sdev_next) {
        if (strcmp(sdev->sdev_uuid, uuid) == 0) {
            break;
        }
        sdev_link = &sdev->sdev_next;
    }
    if (sdev) {
        SSDP_SERVICE *ssvc = sdev->sdev_svc;

        while (ssvc) {
            SSDP_SERVICE *svc_next = ssvc->ssvc_next;

            CallObservers(ssvc, 1);
            free(ssvc->ssvc_domain);
            free(ssvc->ssvc_type);
            free(ssvc);
            ssvc = svc_next;
        }
        free(sdev->sdev_uuid);
        free(sdev->sdev_url_desc);
        free(sdev->sdev_domain);
        free(sdev->sdev_type);
        *sdev_link = sdev->sdev_next;
        free(sdev);
    }
}

/*!
 * \brief Retrieve UUID from HTTPU header.
 */
static const char *SsdpUuidFromHeader(const HTTPU_HEADER *hdr)
{
    static char *uuid;
    const char *cp;

    free(uuid);
    uuid = NULL;
    cp = HttpuGetHeader(hdr, "USN");
    if (*cp) {
        if (strncmp(cp, ct_uuid_, 5) == 0) {
            cp += 5;
        }
        uuid = strchr(cp, ':');
        if (uuid) {
            int len = uuid - cp;

            uuid = malloc(len + 1);
            memcpy(uuid, cp, len);
            *(uuid + len) = '\0';
        } else {
            uuid = strdup(cp);
        }
    }
    return uuid;
}

int SsdpDiscoverDevices(const char *target, int_fast8_t mxwait)
{
    int rc = -1;
    HTTPU_SESSION *s;

    /* Create a new HTTPU session. */
    s = HttpuSessionCreate(0);
    if (s) {
        /* Build and transmit an M-SEARCH header. */
        HttpuAddHeader(s, NULL, "M-SEARCH * HTTP/1.1", NULL);
        HttpuAddHeader(s, "Host", ct_239_255_255_250, ":1900", NULL);
        HttpuAddHeader(s, "MAN", "\"ssdp:discover\"", NULL);
        {
            char valbuf[4];
            sprintf(valbuf, "%d", mxwait);
            HttpuAddHeader(s, "MX", valbuf, NULL);
        }
        HttpuAddHeader(s, "ST", target, NULL);
        HttpuSend(s, inet_addr(ct_239_255_255_250), 1900);

        /* Receive all responses within maximum wait time. */
        while (mxwait) {
            rc = HttpuReceive(s, 1000);
            if (rc < 0) {
                break;
            }
            else if (rc == 0) {
                mxwait--;
            } else {
                const char *hdr;

                /* Get the first header line, which should be
                   HTTP/1.x 200 OK. */
                hdr = HttpuGetHeader(&s->s_rcvhdr, NULL);
                if (atoi(hdr + 9) == 200) {
                    const char *uuid = SsdpUuidFromHeader(&s->s_rcvhdr);

                    if (uuid) {
                        const char *cp = HttpuGetHeader(&s->s_rcvhdr, "ST");

                        if (strcasecmp(cp, ct_upnp_rootdevice) == 0) {
                            CacheDevice(uuid, NULL, NULL, &s->s_rcvhdr);
                        }
                        else if (strncasecmp(cp, "urn:", 4) == 0) {
                            char *comp[5];

                            SsdpSplitWords(strdup(cp), ':', comp, 5);
                            if (strcmp(comp[2], "device") == 0) {
                                CacheDevice(uuid, comp[1], comp[3], &s->s_rcvhdr);
                            }
                            else if (strcmp(comp[2], "service") == 0) {
                                CacheService(uuid, comp[1], comp[3], &s->s_rcvhdr);
                            }
                            free(comp[0]);
                        }
                    }
                }
                rc = 0;
            }
        }
        /* Terminate this session. */
        HttpuSessionDestroy(s);
    }
    return rc;
}

/*!
 * \brief Process notification messages from the network.
 */
static void NotificationListener(HTTPU_HEADER *hdr)
{
    const char *nts;
    const char *nt;
    const char *uuid;
    char *comp[5];

    nt = HttpuGetHeader(hdr, "NT");
    nts = HttpuGetHeader(hdr, "NTS");
    uuid = SsdpUuidFromHeader(hdr);
    if (*nt == '\0' || *nts == '\0' || uuid == NULL) {
        return;
    }

    if (strcmp(nts, "ssdp:byebye") == 0) {
        RemoveDevice(uuid);
    }
    else if (strcmp(nts, "ssdp:alive") == 0) {
        SsdpSplitWords(strdup(nt), ':', comp, 5);
        if (strcmp(comp[0], "urn") == 0 && strcmp(comp[2], "service") == 0) {
            SSDP_OBSERVER *sobs;

            for (sobs = ssdp_observer_root; sobs; sobs = sobs->sobs_next) {
                if (strcmp(comp[1], sobs->sobs_domain) == 0 && strcmp(comp[3], sobs->sobs_type) == 0) {
                    CacheService(uuid, comp[1], comp[3], hdr);
                    break;
                }
            }
        }
        free(comp[0]);
    }
}

int SsdpRegisterServiceObserver(SSDP_OBSERVER_FUNCTION cb, const char *domain, const char *type, int_fast8_t mxwait)
{
    int rc = -1;
    SSDP_OBSERVER *sobs;

    sobs = malloc(sizeof(*sobs));
    if (sobs) {
        sobs->sobs_domain = strdup(domain);
        sobs->sobs_type = strdup(type);
        if (sobs->sobs_domain && sobs->sobs_type) {
            char *st;

            sobs->sobs_cb = cb;
            sobs->sobs_next = ssdp_observer_root;
            ssdp_observer_root = sobs;

            st = malloc(4 + strlen(domain) + 9 + strlen(type) + 3);
            if (st) {
                sprintf(st, "urn:%s:service:%s:1", domain, type);
                rc = SsdpDiscoverDevices(st, mxwait);
                free(st);
                if (rc == 0) {
                    rc = SsdpRegisterListener(NotificationListener);
                }
            }
        } else {
            free(sobs->sobs_domain);
            free(sobs->sobs_type);
            free(sobs);
        }
    }
    return rc;
}
