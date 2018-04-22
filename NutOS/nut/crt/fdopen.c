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
 * Revision 1.3  2009/02/13 14:52:05  haraldkipp
 * Include memdebug.h for heap management debugging support.
 *
 * Revision 1.2  2008/08/11 06:59:40  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.1.1.1  2003/05/09 14:40:24  haraldkipp
 * Initial using 3.2.1
 *
 * Revision 1.1  2003/02/04 17:49:04  harald
 * *** empty log message ***
 *
 */

#include "nut_io.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <memdebug.h>
#include <sys/device.h>


/*!
 * \addtogroup xgCrtStdio
 */
/*@{*/

/*!
 * \brief Open a stream associated with a file, device or socket descriptor.
 *
 * \param fd   Descriptor of a previously opened file, device or
 *             connected socket.
 * \param mode Specifies the access mode.
 *         \li \c "r"  Read only.
 *         \li \c "w"  Write only.
 *         \li \c "a"  Write only at the end of file.
 *         \li \c "r+" Read and write existing file.
 *         \li \c "w+" Read and write, destroys existing file contents.
 *         \li \c "a+" Read and write, preserves existing file contents.
 *         \li \c "b"  May be appended to any of the above strings to
 *                     specify binary access.
 *
 * \return A pointer to the open stream or a null pointer to indicate
 *         an error.
 */
FILE *_fdopen(int fd, const char *mode)
{
    int mflags;
    uint_fast8_t i;

    /*
     * Translate file mode.
     */
    if ((mflags = _fmode(mode)) == EOF)
        return 0;


    /************************* HACK ALERT!!! *************************/
    /* TCP Sockets are hacked into this pseudo
     * file scheme. If a stream shall be connected to this socket,
     * _fdopen() is called with a casted pointer to the socket struct
     * instead of passing a real file descriptor. Therefore we have
     * to handle such pointers in a special way...
     *
     * I really would like to see a new socket API, where we directly
     * use file descriptors (integers in the range of 0 ... FOPEN_MAX-1
     * instead of struct pointers
     */
     /************************* HACK ALERT!!! *************************/

    if ((unsigned int)fd >= FOPEN_MAX) {
        /* We expect "normal" filedescriptors to be positive integers in
         * a range of 0 ... FOPEN_MAX-1. Any other value will be sanity
         * checked and then manually added to the __fds[] array. The
         * resulting integer file descriptor will then be used to
         * connect the stream like is is done with normal file descriptors
         * as well.
         */

        NUTVIRTUALDEVICE *vdv = (NUTVIRTUALDEVICE *) fd;

        if ((vdv->vdv_zero == NULL) && (vdv->vdv_type == IFTYP_TCPSOCK)) {
            /* Seems the user passed a TCPSOCK struct pointer... */
            /* Search the next free filedescriptor */

            for (i = 0; __fds[i];) {
                if (++i >= FOPEN_MAX) {
                    errno = EMFILE;
                    return NULL;
                }
            }

            /* Reserve the entry */
            __fds[i] = (NUTFILE*) fd;
            fd = i;
        }
        /************************ HACK ALERT END *************************/
    }

    /*
     * Find an empty slot.
     */
    for (i = 0; __iob[i];) {
        if (++i >= FOPEN_MAX) {
            errno = ENFILE;
            return NULL;
        }
    }

    if ((__iob[i] = malloc(sizeof(FILE))) != 0) {
        __iob[i]->iob_fd = fd;
        __iob[i]->iob_mode = mflags;
        __iob[i]->iob_flags = 0;
        __iob[i]->iob_unget = 0;
    } else
        errno = ENOMEM;


    return __iob[i];
}

/*@}*/
