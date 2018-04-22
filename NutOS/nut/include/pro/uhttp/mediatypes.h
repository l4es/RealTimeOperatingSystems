#ifndef _PRO_UHTTP_MEDIATYPES_H_
#define _PRO_UHTTP_MEDIATYPES_H_

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
#include <pro/uhttp/streamio.h>

#define MEDIATYPE_EXT_MAXLEN    8

/*!
 * \addtogroup xgUHTTPMediaTypes Media types
 * \ingroup xgUHTTP
 */
/*@{*/

#define MTFLAG_INITIAL  1

/*! \brief Media type entry structure type. */
typedef struct _MEDIATYPE MEDIA_TYPE_ENTRY;

typedef int (*MEDIATYPE_HANDLER) (HTTPD_SESSION*, const MEDIA_TYPE_ENTRY*, const char*);

/*! \brief Media type entry structure. */
struct _MEDIATYPE {
    /*! \brief Chain link. */
    ISC_LINK(MEDIA_TYPE_ENTRY)  media_link;
    /*! \brief Name of this type. */
    char *media_type;
    /*! \brief Sub-name of this type. */
    char *media_subtype;
    /*! \brief Attribute flags. */
    unsigned int media_flags;
    /*! \brief handler function. */
    MEDIATYPE_HANDLER media_handler;
    /*! \brief File name extension. */
    char media_ext[MEDIATYPE_EXT_MAXLEN + 1];
};

/*!
 * \brief Initialize the default media types.
 *
 * \return 0 on success or -1 on error.
 */
extern int MediaTypeInitDefaults(void);

/*!
 * \brief Register a media type.
 *
 * \param ext File name extension.
 * \param type Name of this media type.
 * \param type Sub-name of this media type.
 * \param handler This function will be called to handle the request.
 *
 * \return 0 on success or -1 on error.
 */
extern int HttpRegisterMediaType(char *ext, char *type, char *subtype, MEDIATYPE_HANDLER handler);

/*!
 * \brief Retrieve the media type entry for a given file name.
 *
 * \param path Path name of the file.
 *
 * \return Pointer to the entry or NULL, if no such entry exists.
 */
extern MEDIA_TYPE_ENTRY *GetMediaTypeEntry(char *path);

/*!
 * \brief Default handler for text files.
 *
 * \param hs Pointer to the session info structure.
 * \param mt Pointer to the media type entry.
 * \param path Path name of the file.
 *
 * \return 0 on success or -1 on error.
 */
extern int MediaTypeHandlerText(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *path);

/*!
 * \brief Default handler for binary files.
 *
 * \param hs Pointer to the session info structure.
 * \param mt Pointer to the media type entry.
 * \param path Path name of the file.
 *
 * \return 0 on success or -1 on error.
 */
extern int MediaTypeHandlerBinary(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *path);

/*@}*/
#endif
