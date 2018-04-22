#ifndef _PRO_SSDP_H_
#define _PRO_SSDP_H_

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

/*!
 * \file pro/httpu.h
 * \brief HTTP over UDP library.
 *
 * \verbatim File version $Id: ssdp.h 4920 2013-01-04 12:37:38Z haraldkipp $ \endverbatim
 */

#include <pro/httpu.h>

typedef struct _SSDP_DEVICE SSDP_DEVICE;
typedef struct _SSDP_SERVICE SSDP_SERVICE;

struct _SSDP_DEVICE {
    /*! \brief Pointer to next sibling device. */
    SSDP_DEVICE *sdev_next;
    /*! \brief Linked list of embedded devices. */
    SSDP_DEVICE *sdev_embedded;
    /*! \brief Linked list of services. */
    SSDP_SERVICE *sdev_svc;
    /*! \brief Universally unique identifier. */
    char *sdev_uuid;
    /*! \brief Location of the device description. */
    char *sdev_url_desc;
    /*! \brief Device type domain. */
    char *sdev_domain;
    /*! \brief Device type name. */
    char *sdev_type;
    /*! \brief Expiration time. */
    int sdev_cache;
    /*! \brief Pointer to upper layer device information. */
    void *sdev_info;
};

struct _SSDP_SERVICE {
    /*! \brief Linked list of device services. */
    SSDP_SERVICE *ssvc_next;
    /*! \brief Pointer to associated device. */
    SSDP_DEVICE *ssvc_dev;
    /*! \brief Service type domain. */
    char *ssvc_domain;
    /*! \brief Service type name. */
    char *ssvc_type;
    /*! \brief Pointer to upper layer service information. */
    void *ssvc_info;
};

typedef int (*SSDP_OBSERVER_FUNCTION)(SSDP_SERVICE *ssvc, int_fast8_t action);
typedef void (*SSDP_LISTENER_FUNCTION)(HTTPU_HEADER *hdr);
typedef void (*SSDP_RESPONDER_FUNCTION)(HTTPU_SESSION *s);

extern const char ct_uuid_[];
extern const char ct_upnp_rootdevice[];
extern const char ct_239_255_255_250[];



/*!
 * \brief Register a local SSDP device tree.
 *
 * To be called by applications that implement an SSDP device.
 *
 * \param sdev   Pointer to an initialized device tree structure, including
 *               all provided services and UPnP informations.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SsdpRegisterDeviceTree(SSDP_DEVICE *sdev);

/*!
 * \brief Send SSDP notifications.
 *
 * This function is regularly called by the SSDP background thread.
 */
extern int SsdpSendNotifications(HTTPU_SESSION *s, char *note);

/*!
 * \brief Search network for SSDP devices.
 *
 * \param target Specifies the search target. Either set to a specific
 *               device type or set to "ssdp:all" to search for all devices
 *               or "upnp:rootdevice" to search for all root devices. In
 *               any case, only root devices are collected.
 * \param mxwait Maximum wait time in seconds. Should be within 1 and 120
 *               seconds, typical values are 3 to 5.
 *
 * \return Linked list of discovered root devices or NULL if none were found.
 */
extern int SsdpDiscoverDevices(const char *target, int_fast8_t mxwait);

/*!
 * \brief Register SSDP service observer.
 *
 * This function will initiate a service discovery for the given number of
 * seconds and will normally not return before the discovery has finished.
 * During this time however, the SSDP background receiver may already call
 * the observer function for each discovered service.
 *
 * \param cb     This function will be called after adding a service to the
 *               cache or before removing a service from the cache.
 * \param domain Type domain of the service to observe.
 * \param type   Type name of the service to observe.
 * \param mxwait Maximum discovery time in seconds, which should be within
 *               1 to 120. Recommended values are within 3 to 5.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SsdpRegisterServiceObserver(SSDP_OBSERVER_FUNCTION cb, const char *domain, const char *type, int_fast8_t mxwait);

/*!
 * \brief Register SSDP notification listener.
 *
 * When this function or SsdpRegisterResponder() is called for the
 * first time, then a background thread is started, which listens
 * for HTTPU requests on port 1900.
 *
 * Typical applications will not register any SSDP listener. Note, that
 * only a single listener can be registered, which is normally occupied
 * by the SSDP client.
 *
 * \param callback This function will be called when a notification has
 *                 been received from the network.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SsdpRegisterListener(SSDP_LISTENER_FUNCTION callback);

/*!
 * \brief Register SSDP search responder.
 *
 * When this function or SsdpRegisterListener() is called for the
 * first time, then a background thread is started, which listens
 * for HTTPU requests on port 1900.
 *
 * Typical applications will not register any SSDP responder. Note, that
 * only a single entry is available, which is normally occupied by the
 * SSDP daemon.
 *
 * \param callback This function will be called when a search request
 *                 has been received from the network.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SsdpRegisterResponder(SSDP_RESPONDER_FUNCTION callback);

/*!
 * \brief Update UUID with MAC address.
 *
 * \todo This function should be moved to a more general place.
 *
 * \param uuid Identifier to update.
 * \param mac  MAC address to fill in.
 */
extern void SsdpUuidSetMac(char *uuid, const uint8_t *mac);

/*!
 * \brief Split string into words.
 *
 * \todo This function should be moved to a more general place.
 *
 * \param str   String to split.
 * \param delim Word delimiting character.
 * \param words Array to fill with pointers to words.
 * \param n     Maximum number of words to split.
 */
extern void SsdpSplitWords(char *str, char delim, char **words, int n);

#endif
