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
 * $Id: asprintf.c 4993 2013-02-20 10:33:34Z haraldkipp $
 */

#include "nut_io.h"
#include <sys/nutdebug.h>

/*!
 * \addtogroup xgCrtStdio
 */
/*@{*/

/*!
 * \brief Write formatted data to a string.
 *
 * This function is similar to \ref sprintf, but internally allocates
 * the required buffer using \ref malloc. If no longer used, the caller
 * must release this buffer by calling \ref free.
 *
 * \param strp Pointer to a character pointer that receives the address
 *             of a newly allocated output string.
 * \param fmt  Format string containing conversion specifications.
 * \param ...  List of arguments.
 *
 * \return The number of characters written. A negative value is
 *         returned to indicate an error. In this case no buffer
 *         is allocated and the pointer is set to NULL.
 */
int asprintf(char **strp, const char *fmt, ...)
{
    int rc;
    va_list ap;

    NUTASSERT(strp != NULL);
    NUTASSERT(fmt != NULL);

    va_start(ap, fmt);
    /* All is done in this function. */
    rc = vasprintf(strp, fmt, ap);
    va_end(ap);

    return rc;
}

/*@}*/
