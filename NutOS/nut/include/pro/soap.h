#ifndef _PRO_SOAP_H_
#define _PRO_SOAP_H_

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
 * \file pro/soap.h
 * \brief SOAP remote call library.
 *
 * \verbatim File version $Id: soap.h 4919 2013-01-04 11:49:36Z haraldkipp $ \endverbatim
 */

#include <cfg/http.h>
#include <pro/uhttp/uhttpd.h>
#include <stdint.h>

#ifndef SOAP_MAX_TAG_SIZE
#define SOAP_MAX_TAG_SIZE       128
#endif

#ifndef SOAP_MAX_TAG_ATTRIBUTES
#define SOAP_MAX_TAG_ATTRIBUTES 8
#endif

/*! \brief Select input argument. */
#define SOAP_ARG_IN  0
/*! \brief Select output argument. */
#define SOAP_ARG_OUT 1

#define SOAPTYPE_EOTAG       0x40
#define SOAPTYPE_ETAG        0x80

typedef struct _SOAP_TAG SOAP_TAG;
typedef struct _SOAP_NAME SOAP_NAME;
typedef struct _SOAP_ATTRIBUTE SOAP_ATTRIBUTE;

typedef struct _SOAP_ARG SOAP_ARG;
typedef struct _SOAP_PROCEDURE SOAP_PROCEDURE;

struct _SOAP_ARG {
    SOAP_ARG *arg_next;
    char *arg_name;
    char *arg_val;
    void *arg_info;
};

struct _SOAP_PROCEDURE {
    SOAP_PROCEDURE *proc_next;
    char *proc_name;
    SOAP_ARG *proc_argi;
    SOAP_ARG *proc_argo;
};

struct _SOAP_NAME {
    char *_namespace;
    char *_name;
};

struct _SOAP_ATTRIBUTE {
    SOAP_NAME attr_name;
    char *attr_value;
};

struct _SOAP_TAG {
    uint8_t soap_ttf;
    SOAP_NAME soap_name;
    SOAP_ATTRIBUTE soap_attr[SOAP_MAX_TAG_ATTRIBUTES];
    char soap_buff[SOAP_MAX_TAG_SIZE + 1];
};

/*!
 * \brief Get SOAP procedure by name.
 *
 * \param list  Linked list of procedures.
 * \param name  Procedure name.
 *
 * \return Pointer to the procedure or NULL if not found.
 */
extern SOAP_PROCEDURE *SoapProcByName(SOAP_PROCEDURE *list, const char *name);

/*!
 * \brief Get SOAP argument by name.
 *
 * \param proc  Specifies the SOAP procedure.
 * \param name  Name of the argument to find.
 * \param dir   Must be \ref SOAP_ARG_IN for input or \ref SOAP_ARG_OUT
 *              for output arguments.
 *
 * \return Pointer to the procedure or NULL if not found.
 */
extern SOAP_ARG *SoapArgByName(SOAP_PROCEDURE *proc, const char *name, int dir);

/*!
 * \brief Set SOAP procedure parameter.
 *
 * \param proc  Specifies the SOAP procedure.
 * \param name  Parameter name.
 * \param value Parameter value.
 * \param dir   Must be \ref SOAP_ARG_IN for input or \ref SOAP_ARG_OUT
 *              for output arguments.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SoapProcArgSet(SOAP_PROCEDURE *proc, const char *name, const char *value, int dir);

/*!
 * \brief Set SOAP procedure integer parameter.
 *
 * \param proc  Specifies the SOAP procedure.
 * \param name  Parameter name.
 * \param value Parameter value.
 * \param dir   Must be \ref SOAP_ARG_IN for input or \ref SOAP_ARG_OUT
 *              for output arguments.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SoapProcArgSetInt(SOAP_PROCEDURE *proc, const char *name, int value, int dir);

/*!
 * \brief Get SOAP procedure parameter.
 *
 * \param proc Specifies the SOAP procedure.
 * \param name Parameter name.
 * \param dir  Must be \ref SOAP_ARG_IN for input or \ref SOAP_ARG_OUT
 *             for output arguments.
 *
 * \return Pointer to the requested parameter value. If not available,
 *         then NULL is returned.
 */
extern const char *SoapProcArgGet(SOAP_PROCEDURE *proc, const char *name, int dir);

/*!
 * \brief Get SOAP procedure integer parameter.
 *
 * \param proc Specifies the SOAP procedure.
 * \param name Parameter name.
 * \param dir  Must be \ref SOAP_ARG_IN for input or \ref SOAP_ARG_OUT
 *             for output arguments.
 *
 * \return Value of the requested parameter. If not available, then 0
 *         is returned.
 */
extern int SoapProcArgGetInt(SOAP_PROCEDURE *proc, const char *name, int dir);

/*!
 * \brief Parse SOAP call request.
 *
 * \param stream uHTTP stream of the connection.
 * \param avail  Number of bytes available in the request.
 * \param list   Linked list of supported SOAP procedures.
 *
 * \return Pointer to the requested procedure. NULL is returned in case
 *         of an error
 */
extern SOAP_PROCEDURE *SoapParseCallRequest(HTTP_STREAM *stream, int avail, SOAP_PROCEDURE *list);

/*!
 * \brief Send SOAP call response.
 *
 * Values of any output arguments must have be set by the caller in the
 * procedure structure.
 *
 * The following code fragment shows how to handle a SOAP remote call.
 * \code
 * #include <pro/uhttp/uhttpd.h>
 * #include <pro/uhttp/modules/mod_cgi_func.h>
 * #include <pro/uhttp/mediatypes.h>
 * #include <pro/soap.h>
 *
 * int status;
 *
 * int CgiSwitchLight(HTTPD_SESSION *hs)
 * {
 *   const char *domain = "schemas-upnp-org";
 *   const char *type = "BinaryLight";
 *   HTTP_STREAM *stream = hs->s_stream;
 *   int req_len = (int) hs->s_req.req_length;
 *   SOAP_PROCEDURE *proc;
 *
 *   proc = SoapParseCallRequest(stream, req_len, switchPowerProcs);
 *   if (proc) {
 *     if (strcmp(proc->proc_name, "GetStatus") == 0) {
 *       SoapProcArgSetInt(proc, "ResultStatus", status, SOAP_ARG_OUT);
 *       SoapSendCallResponse(hstream, proc, domain, type);
 *     }
 *     else if (strcmp(proc->proc_name, "SetTarget") == 0) {
 *       status = SoapProcArgGetInt(proc, "newTargetValue", SOAP_ARG_IN);
 *       SoapSendCallResponse(stream, proc, domain, type);
 *     }
 *   }
 *   return 0;
 * }
 *
 * StreamInit();
 * MediaTypeInitDefaults();
 * HttpRegisterMediaType("cgi", NULL, NULL, HttpCgiFunctionHandler);
 * HttpRegisterCgiFunction("switchlight.cgi", CgiSwitchLight);
 * \endcode
 *
 * \param stream uHTTP stream of the connection.
 * \param proc   Pointer to SOAP procedure structure.
 * \param domain Type domain of the service.
 * \param type   Type name of the service.
 *
 * \return Pointer to the requested procedure. NULL is returned in case
 *         of an error
 */
extern int SoapSendCallResponse(HTTP_STREAM *stream, SOAP_PROCEDURE *proc, const char *domain, const char *type);

/*!
 * \brief Call SOAP procedure resource.
 *
 * The following code fragment shows how to do a SOAP remote call.
 * \code
 * #include <pro/soap.h>
 *
 * char *url = "http://192.0.0.2/dlna/renderer";
 * char *uri = "renderer";
 * char *urn = "schemas-upnp-org:service:RenderingControl:1";
 * SOAP_PROCEDURE *proc;
 *
 * proc = SoapProcByName(tv_control, "GetVolume");
 * SoapProcArgSetInt(proc, "InstanceID", 0, SOAP_ARG_IN);
 * SoapProcArgSet(proc, "Channel", "Master", SOAP_ARG_IN);
 * if (SoapProcCallResource(proc, url, uri, urn, 2000) == 0) {
 *     int vol = SoapProcArgGetInt(proc, "CurrentVolume", SOAP_ARG_OUT);
 * }
 * \endcode
 *
 * \param proc Specifies the SOAP procedure.
 * \param url  HTTP resource locator.
 * \param uri  HTTP resource identifier.
 * \param urn  SOAP resource name.
 * \param tmo  Maximum number of milliseconds to wait for response.
 *
 * \return 0 on success or -1 on failure.
 */
extern int SoapProcCallResource(SOAP_PROCEDURE *proc, const char *url, const char *uri, const char *urn, uint32_t tmo);

#endif
