#ifndef _CFG_HTTP_H_
#define _CFG_HTTP_H_

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
 * \file cfg/http.h
 * \brief HTTP configuration.
 *
 * When using the Configurator, a file with the same name may be created in the
 * build tree, which replaces this file.
 *
 * \verbatim
 *
 * $Id: http.h 2154 2008-07-08 13:28:44Z haraldkipp $
 *
 * \endverbatim
 */

#if defined(_WIN32)

#ifndef HTTP_MAJOR_VERSION
#define HTTP_MAJOR_VERSION      1
#endif

#ifndef HTTP_MINOR_VERSION
#define HTTP_MINOR_VERSION      1
#endif

#ifndef HTTP_DEFAULT_ROOT
#define HTTP_DEFAULT_ROOT   "htdocs/"
#endif

#ifndef HTTP_MAX_REQUEST_SIZE
#define HTTP_MAX_REQUEST_SIZE   256
#endif

#ifndef HTTP_FILE_CHUNK_SIZE
#define HTTP_FILE_CHUNK_SIZE    512
#endif

#ifndef HTTP_KEEP_ALIVE_REQ
#define HTTP_KEEP_ALIVE_REQ     1
#endif

//#define HTTPD_EXCLUDE_DATE

/* Default media types. */
#define HTTP_MEDIATYPE_XML      1
#define HTTP_MEDIATYPE_SVG      1
#define HTTP_MEDIATYPE_SHTML    1
#define HTTP_MEDIATYPE_PDF      1
#define HTTP_MEDIATYPE_JPG      1
#define HTTP_MEDIATYPE_JS       1
#define HTTP_MEDIATYPE_JAR      1
#define HTTP_MEDIATYPE_HTM      1
#define HTTP_MEDIATYPE_GIF      1
#define HTTP_MEDIATYPE_CSS      1
#define HTTP_MEDIATYPE_BMP      1

#endif /* _WIN32 */

#endif
