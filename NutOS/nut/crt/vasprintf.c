/*
 * Copyright (C) 2013 by egnite Software GmbH
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
 * $Id: vasprintf.c 5354 2013-09-27 09:57:40Z u_bonnes $
 */

#include "nut_io.h"
#include <sys/nutdebug.h>

#include <stdlib.h>

/*!
 * \addtogroup xgCrtStdio
 */
/*@{*/

/* Dummy function for calculating the output length. */
static int _nputb(int fd, const void *buffer, size_t count)
{
    return count;
}
#ifdef __HARVARD_ARCH__
static int _nputb_P(int fd, PGM_P buffer_P, size_t count)
{
    return count;
}
#endif
/*!
 * \brief Write argument list to a string using a given format.
 *
 * This function is similar to \ref vsprintf, but internally allocates
 * the required buffer using \ref malloc. If no longer used, the caller
 * must release this buffer by calling \ref free.
 *
 * \param strp Pointer to a character pointer that receives the address
 *             of a newly allocated output string.
 * \param fmt  Format string containing conversion specifications.
 * \param ap   List of arguments.
 *
 * \return The number of characters written. A negative value is
 *         returned to indicate an error. In this case no buffer
 *         is allocated and the pointer is set to NULL.
 */
int vasprintf(char **strp, const char *fmt, va_list ap)
{
    int rc;

    NUTASSERT(strp != NULL);
    NUTASSERT(fmt != NULL);

    /* Determine the length of the output string. */
    rc = _putf(_nputb,
#ifdef __HARVARD_ARCH__
               _nputb_P,
#endif
               0, fmt, ap);
    if (rc >= 0) {
        *strp = (char *) malloc(rc + 1);
        if (*strp) {
            vsprintf(*strp, fmt, ap);
        } else {
            rc = -1;
        }
    } else {
        *strp = NULL;
    }
    return rc;
}

/*@}*/
