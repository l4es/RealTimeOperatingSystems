#ifndef _PRO_UHTTP_MODULES_MOD_CGI_H_
#define _PRO_UHTTP_MODULES_MOD_CGI_H_

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

#include <isc/list.h>
#include <pro/uhttp/mediatypes.h>

/*!
 * \addtogroup xgUHTTPModCgiFunc CGI functions
 * \ingroup xgUHTTPModules
 */
/*@{*/

/*!
 * \brief Common gateway interface function entry type.
 */
typedef struct _HTTP_CGI_FUNCTION HTTP_CGI_FUNCTION;

typedef int (*HTTP_CGI_HANDLER) (HTTPD_SESSION*);

/*!
 * \brief Common gateway interface function entry structure.
 *
 * Be prepared, that the layout of this structure may change in future
 * versions.
 */
struct _HTTP_CGI_FUNCTION {
    /*! \brief Chain link. */
    ISC_LINK(HTTP_CGI_FUNCTION) cgi_link;
    /*! \brief URI of the CGI function. */
    char *cgi_uri;
    /*! \brief Registered CGI function. */
    HTTP_CGI_HANDLER cgi_handler;
};

/*!
 * \brief Register a common gateway interface function.
 *
 *
 * Usage example:
 *
 * \code
 * #include <pro/uhttp/modules/mod_cgi_func.h>
 *
 * int CgiTest(HTTPD_SESSION *hs, const HTTP_CGI_FUNCTION *cgi)
 * {
 *     ...
 *     return 0
 * }
 *
 * if (HttpRegisterCgiFunction("cgi/test.cgi", CgiTest) == 0) {
 *     puts("CGI registered.");
 *     ...
 * } else {
 *     puts("Failed to register CGI.");
 *     ...
 * }
 *
 * if (HttpRegisterCgiFunction("cgi/test.cgi", NULL) == 0) {
 *     puts("CGI unregistered.");
 *     ...
 * } else {
 *     puts("Failed to unregister CGI.");
 *     ...
 * }
 *
 * if (HttpRegisterMediaType("cgi", NULL, NULL, HttpCgiFunctionHandler) == 0) {
 *     puts("Media type CGI registered.");
 *     ...
 * } else {
 *     puts("Failed to register CGI media type.");
 *     ...
 * }
 * \endcode
 *
 * \param uri     Case insensitive URI of this CGI. Previously registered
 *                functions may be overridden by subsequent calls using
 *                the same URI.
 * \param handler Pointer to the function that will be called if the given
 *                URI is requested, or NULL to unregister the CGI function.
 *
 * \return 0 on success or -1 on error.
 */
extern int HttpRegisterCgiFunction(const char *uri, HTTP_CGI_HANDLER handler);

/*!
 * \brief Default media type handler for CGI functions.
 */
extern int HttpCgiFunctionHandler(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *filepath);

/*@}*/
#endif
