#ifndef _PRO_UHTTP_MODULES_MOD_REDIR_H_
#define _PRO_UHTTP_MODULES_MOD_REDIR_H_

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
#include <pro/uhttp/uhttpd.h>

/*!
 * \addtogroup xgUHTTPModRedir Redirection
 * \ingroup xgUHTTPModules
 */
/*@{*/

/*!
 * \brief HTTP redirection entry type.
 */
typedef struct _HTTP_LOCATION HTTP_LOCATION;

/*!
 * \brief HTTP redirection information structure.
 */
struct _HTTP_LOCATION {
    /*! \brief Chain link */
    ISC_LINK(HTTP_LOCATION) loc_link;
    /*! \brief Redirected resource. */
    char *loc_uri;
    /*! \brief Redirection target. */
    char *loc_redir;
    /*! \brief HTTP response code. */
    int loc_response;
    /*! \brief Not used? */
    const char *loc_resptext;
};

/*!
 * \brief Register an HTTP redirection.
 *
 * The following sample permanently redirects an empty resource to
 * the index page index.html.
 *
 * \code
 * #include <pro/uhttp/modules/mod_redir.h>
 *
 * HttpRegisterRedir("", "/index.html", 301);
 * \endcode
 *
 * This function will automatically set the default redirection handler
 * HttpLocationRedir().
 *
 * \param url      The resource that will be redirected.
 * \param redir    The redirection target.
 * \param response The HTTP response code that will be send with the
 *                 redirection.
 *
 * \return 0 if the redirection has been successfully registered.
 *         Otherwise -1 is returned.
 */
extern int HttpRegisterRedir(const char *url, const char *redir, int response);

/*!
 * \brief Default HTTP redirection handler.
 *
 * This function is automatically called by the default client handler
 * function HttpdClientHandler() after at least one redirection entry
 * had been registered.
 *
 * \param hs Pointer to the session info structure.
 *
 * \return 0 if a redirection response had been sent. -1 is returned if
 *         no redirection exists, in which case the caller should handle
 *         the original request.
 */
extern int HttpLocationRedir(HTTPD_SESSION *hs);

/*!
 * \brief Retrieve redirection entry for a given resource.
 *
 * This routine may be used by custom redirection handlers.
 */
HTTP_LOCATION *HttpLocationLookup(const char *uri);

/*@}*/
#endif

