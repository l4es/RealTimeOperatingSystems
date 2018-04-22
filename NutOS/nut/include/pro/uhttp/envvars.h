#ifndef _PRO_UHTTP_ENVVARS_H_
#define _PRO_UHTTP_ENVVARS_H_

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

/*
 * $Id$
 */

#include <isc/list.h>

#include <pro/uhttp/uhttpd.h>

/*!
 * \addtogroup xgUHTTPEnvVars Environment variables
 * \ingroup xgUHTTP
 */
/*@{*/

/*! \name Default environment variables */
/*@{*/
/*! \brief The current date and time in Greenwich Mean Time. */
#define HSITEM_DATE_GMT                 1
/*! \brief The current date and time in the server's timezone. */
#define HSITEM_DATE_LOCAL               2
/*! \brief The file name of the document requested by the client. */
#define HSITEM_DOCUMENT_NAME            3
/*! \brief The root directory of this site. */
#define HSITEM_DOCUMENT_ROOT            4
/*! \brief The MIME types the requestor will accept. */
#define HSITEM_HTTP_ACCEPT_ENCODING     5
/*! \brief The type of connection. */
#define HSITEM_HTTP_CONNECTION          6
/*! \brief The value of any cookie. */
#define HSITEM_HTTP_COOKIE              7
/*! \brief The base URL of the host. */
#define HSITEM_HTTP_HOST                8
/*! \brief The URL of the page that made this request. */
#define HSITEM_HTTP_REFERER             9
/*! \brief The browser id or user-agent string identifying the browser. */
#define HSITEM_HTTP_USER_AGENT          10
/*! \brief The argument string for this request. */
#define HSITEM_QUERY_STRING             11
/*! \brief The argument string for this request. */
#define HSITEM_QUERY_STRING_UNESCAPED   12
/*! \brief The method used for this request. */
#define HSITEM_REQUEST_METHOD           13
/*! \brief The URI for this request. */
#define HSITEM_REQUEST_URI              14
/*! \brief The path to the script being executed. */
#define HSITEM_SCRIPT_FILENAME          15
/*! \brief The file name of the script being executed. */
#define HSITEM_SCRIPT_NAME              16
/*@}*/

/*! \brief Environment handler function type. */
typedef const char* (*HTTP_ENV_HANDLER) (HTTPD_SESSION*, const char *name);

/*! \brief Environment variable handler function type. */
typedef const char* (*HTTP_ENVVAR_HANDLER) (HTTPD_SESSION*, int);

/*! \brief Environment variable entry structure type. */
typedef struct _HTTP_ENVVAR_ENTRY HTTP_ENVVAR_ENTRY;

/*! \brief Environment variable entry structure. */
struct _HTTP_ENVVAR_ENTRY {
    /*! \brief Chain link. */
    ISC_LINK(HTTP_ENVVAR_ENTRY) env_link;
    /*! \brief Name of the variable. */
    char *env_name;
    /*! \brief Registered handler function. */
    HTTP_ENVVAR_HANDLER env_handler;
    int env_index;
};

/*!
 * \brief Initialize the default environment variables.
 *
 * \return 0 on success or -1 on error.
 */
extern int EnvInitDefaults(void);

/*!
 * \brief Register an environment variable.
 *
 * \param name Name of the environment variable.
 * \param handler This function will be called to retrieve the variable's value.
 * \param item Numeric identifier that will be passed to the handler.
 *
 * \return 0 on success or -1 on error.
 */
extern int EnvRegisterVariable(char *name, HTTP_ENVVAR_HANDLER handler, int item);

/*!
 * \brief Retrieve entry of a registered environment variable.
 */
extern const HTTP_ENVVAR_ENTRY* EnvVarGetEntry(const char *name);

/*!
 * \brief Default environment handler.
 *
 * \param hs Pointer to the session info structure.
 * \param name Name of the environment variable.
 *
 * \return Pointer to the value. If the variable does not exists, then
 *         NULL is returned.
 */
extern const char *EnvHandler(HTTPD_SESSION *hs, const char *name);

/*!
 * \brief Default handler of stream related environment variables.
 *
 * \param hs Pointer to the session info structure.
 * \param item Numeric identifier of the variable.
 *
 * \return Pointer to the value.
 */
extern const char *HttpStreamInfo(HTTPD_SESSION *hs, int item);

/*!
 * \brief Default handler of session related environment variables.
 *
 * \param hs Pointer to the session info structure.
 * \param item Numeric identifier of the variable.
 *
 * \return Pointer to the value.
 */
extern const char *HttpSessionInfo(HTTPD_SESSION *hs, int item);

/*@}*/

#endif

