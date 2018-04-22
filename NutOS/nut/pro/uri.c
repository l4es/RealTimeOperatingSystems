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

//#include <netdb.h>

#include <stdlib.h>
#include <string.h>

#include <pro/uri.h>

static const char *scheme_names[] = {
    "ftp://", "https://", "mailto:", "pop://", "snmp://", "telnet://", NULL
};

static const unsigned short default_ports[] = {
    20, 443, 25, 110, 161, 23, 80
};

/* [name:][<user>[:<password>]@]<host>[:<port>][/<path>] */
/*!
 * \brief Create a scheme structure from a URI.
 *
 * \param uri URI to parse. The expected format is
 * \code
 * [name:][<user>[:<password>]@]<host>[:<port>][/<path>]
 * \endcode
 *
 * \return Pointer to an allocated \ref URI_SCHEME "URI scheme"
 *         structure. If this structure is no longer used, the caller
 *         must call UriSchemeRelease(). In case of an error, NULL is
 *         returned.
 */
URI_SCHEME *UriSchemeSplit(const char *uri)
{
    URI_SCHEME *schm = NULL;
    char *cp;

    if (uri && *uri) {
        /* Create a blank scheme structure. */
        schm = calloc(1, sizeof(*schm));
        if (schm) {
            int nidx;

            /* Determine the scheme name. */
            for (nidx = 0; scheme_names[nidx]; nidx++) {
                int len = strlen(scheme_names[nidx]);
                if (strncmp(uri, scheme_names[nidx], len) == 0) {
                    uri += len;
                    break;
                }
            }
            /* Create a local copy of the URI string. */
            schm->schm_uri = strdup(uri);
            if (schm->schm_uri) {
                /* Split the local copy. */
                schm->schm_host = schm->schm_uri;
                for (cp = schm->schm_uri; *cp; cp++) {
                    if (*cp == ':') {
                        *cp = '\0';
                        schm->schm_port = cp + 1;
                    }
                    else if (*cp == '/') {
                        *cp = 0;
                        schm->schm_path = cp + 1;
                        break;
                    }
                    else if (*cp == '@') {
                        *cp = 0;
                        schm->schm_user = schm->schm_host;
                        schm->schm_pass = schm->schm_port;
                        schm->schm_host = cp + 1;
                        schm->schm_port = NULL;
                    }
                }
                if (schm->schm_port) {
                    schm->schm_portnum = (unsigned short) atoi(schm->schm_port);
                }
                else {
                    schm->schm_portnum = default_ports[nidx];
                }
                return schm;
            }
        }
        UriSchemeRelease(schm);
    }
    return NULL;
}

/*!
 * \brief Release a HTTP scheme structure.
 *
 * \param schm Pointer to a \ref HttpSchemeParse "previously allocated"
 *             HTTP scheme structure.
 */
void UriSchemeRelease(URI_SCHEME *schm)
{
    free(schm->schm_uri);
    free(schm);
}
