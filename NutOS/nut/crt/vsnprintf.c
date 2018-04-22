/*
 * Copyright (C) 2001-2003 by egnite Software GmbH. All rights reserved.
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
 *
 */

/*
 * $Log$
 * Revision 1.4  2009/01/17 11:26:38  haraldkipp
 * Getting rid of two remaining BSD types in favor of stdint.
 * Replaced 'u_int' by 'unsinged int' and 'uptr_t' by 'uintptr_t'.
 *
 * Revision 1.3  2006/03/02 19:57:33  haraldkipp
 * ICCARM insists on a (void *) typecast for the second parameter of memcpy().
 *
 * Revision 1.2  2004/03/16 16:48:27  haraldkipp
 * Added Jan Dubiec's H8/300 port.
 *
 * Revision 1.1.1.1  2003/05/09 14:40:35  haraldkipp
 * Initial using 3.2.1
 *
 * Revision 1.1  2003/02/04 17:49:09  harald
 * *** empty log message ***
 *
 */

#include "nut_io.h"

#include <string.h>

#define min(a, b) ((a)<(b)?(a):(b))

/*!
 * \addtogroup xgCrtStdio
 */
/*@{*/

static int _snputb(int fd, const void *buffer, size_t count)
{
    size_t n = 0;    
    struct __memstream *msp = (struct __memstream *) ((uintptr_t) fd);

    if (msp->size > 0) {
        n = min(msp->size, count);

        memcpy(*(msp->spp), buffer, n);
        *(msp->spp) += n;
        msp->size -= n;
    }
    return n;
}

/*!
 * \brief Write argument list to a string using a given format.
 *
 * \param buffer Pointer to a buffer that receives the output string.
 * \param size   Maximum number of characters to be written, including the trailing \0
 * \param fmt    Format string containing conversion specifications.
 * \param ap     List of arguments.
 *
 * \return The number of characters written or a negative value to
 *         indicate an error.
 */
int vsnprintf(char *buffer, size_t size, const char *fmt, va_list ap)
{
    int rc = 0;
    struct __memstream ms;

    if (size > 0) { 
        ms.spp  = &buffer;
        ms.size = size - 1;

        rc = _putf(_snputb,
    #ifdef __HARVARD_ARCH__
                   _snputb_P,
    #endif
                   (int) ((uintptr_t) &ms), fmt, ap);
        rc = min(rc, size);
        *buffer = 0;
    }

    return rc;
}

/*@}*/
