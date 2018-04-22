/*
 * Copyright (C) 2013 by egnite GmbH
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
 * $Id: funopen.c 5472 2013-12-06 00:16:28Z olereinhardt $
 */
#include <sys/device.h>
#include <sys/file.h>
#include <fs/fs.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "nut_io.h"

/*!
 * \addtogroup xgCrtStdio
 */
/*@{*/

typedef struct _USERDEVIF USERDEVIF;

struct _USERDEVIF {
    int (*ifc_read) (void *, char *, int);
    int (*ifc_write) (void *, const char *, int);
    long (*ifc_seek)(void *, long, int);
    int (*ifc_close)(void *);
};

static int UserDevRead(NUTFILE *nfp, void *buffer, int size)
{
    USERDEVIF *ifc;

    ifc = (USERDEVIF *) nfp->nf_dev->dev_icb;
    return ifc->ifc_read(nfp->nf_fcb, (char *) buffer, size);
}

static int UserDevWrite(NUTFILE *nfp, const void *buffer, int len)
{
    USERDEVIF *ifc = (USERDEVIF *) nfp->nf_dev->dev_icb;

    return ifc->ifc_write(nfp->nf_fcb, (const char *) buffer, len);
}

static int UserDevClose(NUTFILE *nfp)
{
    int rc;
    USERDEVIF *ifc = (USERDEVIF *) nfp->nf_dev->dev_icb;

    rc = ifc->ifc_close(nfp->nf_fcb);
    free(nfp->nf_dev->dev_icb);
    free(nfp->nf_dev);
    free(nfp);

    return rc;
}

static int UserDevIoCtl(NUTDEVICE * dev, int req, void *conf)
{
    if (req == FS_FILE_SEEK) {
        IOCTL_ARG3 *args = (IOCTL_ARG3 *) conf;
        NUTFILE *nfp = (NUTFILE *) args->arg1;
        USERDEVIF *ifc = (USERDEVIF *) nfp->nf_dev->dev_icb;

        ifc->ifc_seek(nfp->nf_fcb, *((long *) args->arg2), (int) args->arg3);
    }
    return -1;
}

/*!
 * \brief Create a stream that is associated to user functions.
 *
 * \param cookie  A generic pointer that is passed to the user functions
 *                instead of a file descriptor.
 * \param readfn  User function for reading.
 * \param writefn User function for writing.
 * \param seekfn  User function for moving the file pointer.
 * \param closefn User function for closing the stream.
 *
 * \return A pointer to the open stream or a null pointer to indicate
 *         an error.
 */
FILE *funopen(void *cookie,
    int (*readfn) (void *, char *, int),
    int (*writefn) (void *, const char *, int),
    long (*seekfn)(void *, long, int),
    int (*closefn)(void *))
{
    NUTFILE *nfp;
    USERDEVIF *ifc;
    NUTDEVICE *dev;
    int fd;
    int i;

    /*
     * Find an empty slot.
     */
    for (i = 0; __iob[i];) {
        if (++i >= FOPEN_MAX) {
            errno = ENFILE;
            return NULL;
        }
    }

    for (fd = 0; __fds[fd];) {
        if (++fd >= FOPEN_MAX) {
            errno = EMFILE;
            return NULL;
        }
    }

    __iob[i] = (FILE *) calloc(1, sizeof(FILE));
    nfp = (NUTFILE *) malloc(sizeof(NUTFILE));
    dev = (NUTDEVICE *) calloc(1, sizeof(NUTDEVICE));
    ifc = (USERDEVIF *) malloc(sizeof(USERDEVIF));
    if (__iob[i] && nfp && dev && ifc) {
        ifc->ifc_read = readfn;
        ifc->ifc_write = writefn;
        ifc->ifc_seek = seekfn;
        ifc->ifc_close = closefn;

        dev->dev_icb = ifc;
        dev->dev_ioctl = UserDevIoCtl;
        dev->dev_read = UserDevRead;
        dev->dev_write = UserDevWrite;
        dev->dev_close = UserDevClose;

        nfp->nf_dev = dev;
        nfp->nf_fcb = cookie;

        __iob[i]->iob_fd = (int) nfp;
        __fds[fd] = nfp;
        return __iob[i];
    }
    free(__iob[i]);
    __iob[i] = NULL;

    errno = ENOMEM;
    return NULL;
}

/*@}*/
