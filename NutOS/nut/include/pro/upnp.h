#ifndef _PRO_UPNP_H_
#define _PRO_UPNP_H_

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
 * \file pro/upnp.h
 * \brief UPnP library.
 *
 * \verbatim File version $Id: upnp.h 4921 2013-01-04 14:11:21Z haraldkipp $ \endverbatim
 */

#include <pro/soap.h>
#include <pro/ssdp.h>

#define UPNP_STYPE_UI1          1
#define UPNP_STYPE_UI2          2
#define UPNP_STYPE_UI4          3
#define UPNP_STYPE_I1           4
#define UPNP_STYPE_I2           5
#define UPNP_STYPE_I4           6
#define UPNP_STYPE_INT          7
#define UPNP_STYPE_R4           8
#define UPNP_STYPE_R8           9
#define UPNP_STYPE_NUMBER       10
#define UPNP_STYPE_FIXED_14_4   11
#define UPNP_STYPE_FLOAT        12
#define UPNP_STYPE_CHAR         13
#define UPNP_STYPE_STRING       14
#define UPNP_STYPE_DATE         15
#define UPNP_STYPE_DATETIME     16
#define UPNP_STYPE_DATETIME_TZ  17
#define UPNP_STYPE_BOOLEAN      18
#define UPNP_STYPE_BIN_BASE64   19
#define UPNP_STYPE_BIN_HEX      20
#define UPNP_STYPE_URI          21
#define UPNP_STYPE_UUID         22
#define UPNP_STYPE_LAST         UPNP_STYPE_UUID

typedef struct _UPNP_DEVICE_INFO UPNP_DEVICE_INFO;
typedef struct _UPNP_MODEL_INFO UPNP_MODEL_INFO;
typedef struct _UPNP_MANUFACTURER_INFO UPNP_MANUFACTURER_INFO;
typedef struct _UPNP_SERVICE_INFO UPNP_SERVICE_INFO;
typedef struct _UPNP_VARIABLE UPNP_VARIABLE;

struct _UPNP_VARIABLE {
    int ustv_type;
    char *ustv_name;
    int ustv_events;
    char *ustv_default;
    UPNP_VARIABLE *ustv_next;
};

struct _UPNP_SERVICE_INFO {
    char *usvc_url_scpd;
    char *usvc_url_ctrl;
    char *usvc_url_event;
    SOAP_PROCEDURE *usvc_proc;
    UPNP_VARIABLE *usvc_stv;
};

struct _UPNP_MANUFACTURER_INFO {
    /*! \brief Manufacturer name (required). */
    char *umnf_name;
    /*! \brief Manufacturer URL (optional). */
    char *umnf_url;
};

struct _UPNP_MODEL_INFO {
    /*! \brief Model name (required). */
    char *umdl_name;
    /*! \brief Model number (recommended). */
    char *umdl_num;
    /*! \brief Model description (recommended). */
    char *umdl_desc;
    /*! \brief Model URL (optional). */
    char *umdl_url;
};

struct _UPNP_DEVICE_INFO {
    /*! \brief Friendly device name (required).
     * Short description for end user in less than 64 characters.
     */
    char *udev_name;
    /*! \brief Presentation URL (recommended). */
    char *udev_presentation;
    /*! \brief Model information. */
    UPNP_MODEL_INFO *udev_mdl;
    /*! \brief Manufacturer information. */
    UPNP_MANUFACTURER_INFO *udev_mnf;
};

/*!
 * \brief Register a local UPnP device tree.
 *
 * \param parent Pointer to the parent device structure. This is typically
 *               a NULL pointer and only set for embedded devices.
 * \param sdev   Pointer to an initialized device tree structure, including
 *               all provided services and UPnP informations.
 *
 * \return 0 on success or -1 on failure.
 */
extern int UpnpRegisterDeviceTree(SSDP_DEVICE *parent, SSDP_DEVICE *sdev);

/*!
 * \brief Register UPnP service observer.
 *
 * This function will initiate a service discovery for the given number of
 * seconds and will normally not return before the discovery has finished.
 * During this time however, the SSDP background receiver may already call
 * the observer function for each discovered service.
 *
 * The following code fragment demonstrates how to implement a simple
 * network service discovery:
 * \code
 * #include <pro/upnp.h>
 *
 * int RendererObserver(SSDP_SERVICE *ssvc, int_fast8_t removal)
 * {
 *     UPNP_DEVICE_INFO *udev = ssvc->ssvc_dev->sdev_info;
 *
 *     if (removal) {
 *         printf("%s disappears\n", udev->udev_name);
 *     } else {
 *         printf("%s discovered\n", udev->udev_name);
 *     }
 *     return 0;
 * }
 *
 * UpnpRegisterServiceObserver(RendererObserver, "schemas-upnp-org", "RenderingControl", 5);
 * \endcode
 *
 * \param cb     This observer function will be called when a service of
 *               the specified type appears or disappears.
 * \param domain Type domain of the service to observe.
 * \param type   Type name of the service to observe.
 * \param mxwait Maximum discovery time in seconds, which should be within
 *               1 to 120. Recommended values are within 3 to 5.
 *
 * \return 0 on success or -1 on failure.
 */
extern int UpnpRegisterServiceObserver(SSDP_OBSERVER_FUNCTION cb, const char *domain, const char *type, int_fast8_t mxwait);

/*!
 * \brief Get UPnP service procedure by name.
 *
 * \param ssvc Specifies the service.
 * \param name Procedure name.
 *
 * \return Pointer to the procedure or NULL if not found.
 */
extern SOAP_PROCEDURE *UpnpServiceProcByName(const SSDP_SERVICE *ssvc, const char *name);

/*!
 * \brief Call UPnP service.
 *
 * \param ssvc Specifies the service to call.
 * \param proc Specifies the procedure to call.
 * \param tmo  Maximum number of milliseconds to wait for response.
 *
 * \return 0 on success or -1 on failure.
 */
extern int UpnpServiceProcCall(SSDP_SERVICE *ssvc, SOAP_PROCEDURE *proc, uint32_t tmo);

/*!
 * \brief Get index of a specified UPnP variable type name.
 *
 * \param string Pointer to the type name.
 *
 * \return Type index (UPNP_STYPE) or 0 for unknown types.
 */
extern int UpnpVarTypeIndex(const char *string);

/*!
 * \brief Get string of a specified UPnP variable type.
 *
 * \param type Any UPNP_STYPE.
 *
 * \return Pointer to the type name. Unknown types will be returned
 *         as a question mark.
 */
extern const char *UpnpVarTypeString(int type);

extern void UpnpDumpDevice(const SSDP_DEVICE *sdev);

#endif
