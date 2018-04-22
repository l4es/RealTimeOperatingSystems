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

#include <io.h>
#include <fcntl.h>

#include <cfg/http.h>
#if !defined(HTTPD_EXCLUDE_DATE)
#include <time.h>
#endif
#include <sys/stat.h>

#include <isc/list.h>

#if !defined(HTTPD_EXCLUDE_DATE)
#include <pro/rfctime.h>
#endif
#include <pro/uhttp/uhttpd.h>
#include <pro/uhttp/streamio.h>
#include <pro/uhttp/modules/mod_ssi.h>
#include <pro/uhttp/mediatypes.h>

#include <stdlib.h>
#include <string.h>
#include <memdebug.h>

ISC_LIST(MEDIA_TYPE_ENTRY) mediaTypeList = ISC_LIST_INITIAL_TYPE(MEDIA_TYPE_ENTRY);

MEDIA_TYPE_ENTRY *MediaTypesFindByExt(const char *ext)
{
    MEDIA_TYPE_ENTRY *mt;
    int i;

    for (mt = ISC_LIST_HEAD(mediaTypeList); mt; mt = ISC_LIST_NEXT(mt, media_link)) {
        i = strcasecmp(mt->media_ext, ext);
        if (i <= 0) {
            if (i) {
                mt = NULL;
            }
            break;
        }
    }
    return mt;
}

MEDIA_TYPE_ENTRY *GetMediaTypeEntry(char *name)
{
    MEDIA_TYPE_ENTRY *mt;

    if (name == NULL || *name == '\0') {
        mt = MediaTypesFindByExt("html");
    } else {
        char *cp = strrchr(name, '.');
        if (cp == NULL || (mt = MediaTypesFindByExt(cp + 1)) == NULL) {
            mt = MediaTypesFindByExt("txt");
        }
    }
    return mt;
}

int MediaTypeHandlerBinary(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *filepath)
{
    int fd;
    int got;
    char *data = malloc(1460);
    long fsize = -1;
#if HTTP_VERSION >= 0x10
    struct stat s;
#if !defined(HTTPD_EXCLUDE_DATE)
    time_t mtime;
#endif
#endif

    fd = _open(filepath, _O_BINARY | _O_RDONLY);
    if (fd == -1) {
        HttpSendError(hs, 404);
    } else {
#if HTTP_VERSION >= 0x10
        int rc = fstat(fd, &s);
        if (rc) {
            /* File system doesn't support fstat. */
            fsize = _filelength(fd);
        } else {
            fsize = s.st_size;
        }
#if !defined(HTTPD_EXCLUDE_DATE)
        if (rc) {
            mtime = RfcTimeParse("Fri " __DATE__ " " __TIME__) + _timezone;
        } else {
            mtime = s.st_mtime;
        }
        /* Check if-modified-since condition. */
        if (hs->s_req.req_ims && s.st_mtime <= hs->s_req.req_ims) {
            HttpSendHeaderTop(hs, 304);
            HttpSendHeaderDate(hs, mtime);
            HttpSendHeaderBottom(hs, NULL, NULL, 0);
        } else
#endif
#endif
        {
            HttpSendHeaderTop(hs, 200);
#if !defined(HTTPD_EXCLUDE_DATE) && (HTTP_VERSION >= 0x10)
            HttpSendHeaderDate(hs, mtime);
#endif
            HttpSendHeaderBottom(hs, mt->media_type, mt->media_subtype ? mt->media_subtype : mt->media_ext, fsize);
            do {
                got = _read(fd, data, 1460);
                if (got > 0) {
                    s_write(data, 1, got, hs->s_stream);
                }
            } while(got == 1460);
        }
        _close(fd);
    }
    s_flush(hs->s_stream);
    free(data);

    return 0;
}

int MediaTypeHandlerText(HTTPD_SESSION *hs, const MEDIA_TYPE_ENTRY *mt, const char *filepath)
{
    return MediaTypeHandlerBinary(hs, mt, filepath);
}
