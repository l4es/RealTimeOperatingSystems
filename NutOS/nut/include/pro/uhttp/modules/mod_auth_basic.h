#ifndef _PRO_UHTTP_MODULES_MOD_AUTH_BASIC_H_
#define _PRO_UHTTP_MODULES_MOD_AUTH_BASIC_H_

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
 * \addtogroup xgUHTTPModBasicAuth Basic access authentication
 * \ingroup xgUHTTPModules
 */
/*@{*/

/*!
 * \brief Basic authorization entry type.
 */
typedef struct _AUTH_BASIC_ENTRY AUTH_BASIC_ENTRY;

/*!
 * \brief Basic authorization entry structure.
 *
 * Be prepared, that the layout of this structure may change in future
 * versions.
 */
struct _AUTH_BASIC_ENTRY {
    /*! \brief Chain link. */
    ISC_LINK(AUTH_BASIC_ENTRY) auth_link;
    /*! \brief URL of protected area. */
    char *auth_path;
    /*! \brief Login user and password, separated by a colon. */
    char *auth_login;
    /*! \brief Description of the resource. */
    char *auth_realm;
};

/*!
 * \brief Register a basic authorization.
 *
 * Protect a specified URL from unauthorized access. Resources, which
 * are not registered by this function are accessible by anyone.
 *
 * It is allowed to specify several different logins for the same resource.
 *
 * Alternatively it is possible to unprotect a previously protected resource
 * by passing a NULL pointer instead of a login string.
 *
 * Usage example:
 *
 * \code
 * #include <pro/uhttp/modules/mod_auth_basic.h>
 *
 * if (HttpRegisterAuthBasic("dir", "User:Pass") == 0) {
 *     puts("Resource is protected.");
 *     ...
 * } else {
 *     puts("Failed to protect resource.");
 *     ...
 * }
 *
 * if (HttpRegisterAuthBasic("dir", NULL) == 0) {
 *     puts("Resource is unprotected.");
 *     ...
 * } else {
 *     puts("Failed to remove protection.");
 *     ...
 * }
 * \endcode
 *
 * \param path  Path to the protected resource.
 * \param login Required login to access the given resource or NULL to
 *              remove any previously registered protection for the given
 *              resource . To protect a resource, this string must contain
 *              a user name, followed by a colon followed by an unencrypted
 *              password.
 * \param realm Description of the protected resource. This optional
 *              parameter can be a NULL pointer, in which case the path
 *              is used instead.
 *
 * \return 0 on success or -1 on error. Trying to add duplicate entries
 *         will be silently ignored.
 */
extern int HttpRegisterAuthBasic(const char *path, const char *login, const char *realm);

/*!
 * \brief Validate an authentication for a specified realm.
 *
 * If the requested resource had been previously protected by a call to
 * HttpRegisterAuthBasic() and if the client of the specified session
 * hasn't provided a valid authentication, then access is rejected. In
 * the this case the caller should return a 401 response code to the
 * client. This will typically prompt the user to enter a valid
 * user/password pair.
 *
 * \param hs Pointer to a \ref _HTTPD_SESSION structure, which
 *                should contain the requested resource and a valid
 *                authentication.
 *
 * \return 0 if access is granted, -1 if not.
 */
extern int HttpAuthBasicValidate(HTTPD_SESSION *hs);

/*!
 * \brief Look up a basic authorization entry.
 *
 * This low level routine can be used to retrieve a previously
 * registered authorization entry. Note, that the structure
 * layout of authorization entries may change in future versions.
 *
 * Usage example:
 *
 * \code
 * #include <pro/uhttp/modules/mod_auth_basic.h>
 *
 * if (HttpAuthBasicLookup("/dir/index.html", NULL, 1) == NULL) {
 *     puts("Resource is unprotected.");
 *     ...
 * } else {
 *     puts("Resource is protected.");
 *     ...
 * }
 *
 * if (HttpAuthBasicLookup("/dir/index.html", "User:Pass", 1)) {
 *     puts("Access is granted.");
 *     ...
 * } else {
 *     puts("Access is rejected.");
 *     ...
 * }
 * \endcode
 *
 * \param realm Requested resource realm, case insensitive.
 * \param login Requested authentication user and password, separated
 *              by a colon. This pointer may be NULL, if the caller
 *              only wants to check if the specified resource is
 *              protected or not.
 * \param best  Set to 1 to find the best matching realm. If 0, then
 *              an exact match is requested.
 *
 * \return Pointer to the AUTH_BASIC_ENTRY structure or NULL if the
 *         requested entry doesn't exists.
 */
extern const AUTH_BASIC_ENTRY *HttpAuthBasicLookup(const char *realm, const char *login, int best);


/*@}*/
#endif
