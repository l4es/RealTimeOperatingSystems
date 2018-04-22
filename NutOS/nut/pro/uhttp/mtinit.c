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

extern ISC_LIST(MEDIA_TYPE_ENTRY) mediaTypeList;

static char mtc_text[] = "text";
static char mtc_image[] = "image";
char mtc_application[] = "application";

/* Extensions must be sorted in reverse order. */
static MEDIA_TYPE_ENTRY mt_defaults[] = {
#ifdef HTTP_MEDIATYPE_XML
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_text, NULL, MTFLAG_INITIAL, MediaTypeHandlerText, "xml" },
#endif
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_text, "plain", MTFLAG_INITIAL, MediaTypeHandlerText, "txt" },
#ifdef HTTP_MEDIATYPE_SVG
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_image, "svg+xml", MTFLAG_INITIAL, MediaTypeHandlerBinary, "svg" },
#endif
#ifdef HTTP_MEDIATYPE_SHTML
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_text, "html", MTFLAG_INITIAL, HttpSsiHandler, "shtml" },
#endif
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_image, NULL, MTFLAG_INITIAL, MediaTypeHandlerBinary, "png" },
#ifdef HTTP_MEDIATYPE_PDF
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_application, NULL, MTFLAG_INITIAL, MediaTypeHandlerBinary, "pdf" },
#endif
#ifdef HTTP_MEDIATYPE_JS
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY),  mtc_text, "javascript", MTFLAG_INITIAL, MediaTypeHandlerText, "js" },
#endif
#ifdef HTTP_MEDIATYPE_JPG
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_image, "jpeg", MTFLAG_INITIAL, MediaTypeHandlerBinary, "jpg" },
#endif
#ifdef HTTP_MEDIATYPE_JAR
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_application, "x-java-archive", MTFLAG_INITIAL, MediaTypeHandlerBinary, "jar" },
#endif
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_text, NULL, MTFLAG_INITIAL, MediaTypeHandlerText, "html" },
#ifdef HTTP_MEDIATYPE_HTM
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_text, "html", MTFLAG_INITIAL, MediaTypeHandlerText, "htm" },
#endif
#ifdef HTTP_MEDIATYPE_GIF
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_image, NULL, MTFLAG_INITIAL, MediaTypeHandlerBinary, "gif" },
#endif
#ifdef HTTP_MEDIATYPE_CSS
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_text, NULL, MTFLAG_INITIAL, MediaTypeHandlerText, "css" },
#endif
#ifdef HTTP_MEDIATYPE_BMP
    { ISC_LINK_INITIAL(MEDIA_TYPE_ENTRY), mtc_image, NULL, MTFLAG_INITIAL, MediaTypeHandlerBinary, "bmp" }
#endif
};

#define MT_DEFAULTS     (sizeof(mt_defaults) / sizeof(MEDIA_TYPE_ENTRY))

int MediaTypeInitDefaults(void)
{
    int i;

    ISC_LIST_INIT(mediaTypeList);
    for (i = 0; i < (int)MT_DEFAULTS; i++) {
        ISC_LIST_APPEND(mediaTypeList, &mt_defaults[i], media_link);
    }
    return 0;
}
