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

#include <pro/uhttp/modules/mod_ssi.h>
#include <pro/uhttp/mediatypes.h>

#include <stdlib.h>
#include <string.h>
#include <memdebug.h>

extern ISC_LIST(MEDIA_TYPE_ENTRY) mediaTypeList;

int HttpRegisterMediaType(char *ext, char *type, char *subtype, MEDIATYPE_HANDLER handler)
{
    int rc = -1;
    MEDIA_TYPE_ENTRY *cur;
    MEDIA_TYPE_ENTRY *mt;
    int i = -1;

    for (cur = ISC_LIST_HEAD(mediaTypeList); cur; cur = ISC_LIST_NEXT(cur, media_link)) {
        i = strcasecmp(cur->media_ext, ext);
        if (i <= 0) {
            break;
        }
    }

    if (i == 0) {
        /* Existing entry. */
        if (handler) {
            /* Override entry. */
            cur->media_handler = handler;
            if (type && subtype) {
                if ((cur->media_flags & MTFLAG_INITIAL) == 0) {
                    free(cur->media_type);
                    free(cur->media_subtype);
                }
                cur->media_type = strdup(type);
                cur->media_subtype = strdup(subtype);
                cur->media_flags = 0;
            }
        } else {
            /* Remove entry. */
            ISC_LIST_UNLINK_TYPE(mediaTypeList, cur, media_link, MEDIA_TYPE_ENTRY);
            if (cur->media_flags == 0) {
                free(cur->media_type);
                free(cur->media_subtype);
                free(cur);
            }
        }
        rc = 0;
    }
    if (handler && i) {
        /* New entry. */
        mt = (MEDIA_TYPE_ENTRY *) calloc(1, sizeof(MEDIA_TYPE_ENTRY));
        if (mt) {
            strncpy(mt->media_ext, ext, sizeof(mt->media_ext) - 1);
            if (type) {
                mt->media_type = strdup(type);
            }
            if (subtype) {
                mt->media_subtype = strdup(subtype);
            }
            mt->media_handler = handler;
            if (cur) {
                /* Insert entry, maintaining descending sort order. */
                ISC_LIST_INSERTBEFORE(mediaTypeList, cur, mt, media_link);
            } else {
                /* Append entry to the list. */
                ISC_LIST_APPEND(mediaTypeList, mt, media_link);
            }
            /* Indicate success. */
            rc = 0;
        }
    }
    return rc;
}

