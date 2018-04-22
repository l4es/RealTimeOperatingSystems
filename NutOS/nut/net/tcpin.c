/*
 * Copyright (C) 2001-2003 by egnite Software GmbH
 * Copyright (c) 1993 by Digital Equipment Corporation
 * Copyright (c) 1983, 1993 by The Regents of the University of California
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
 * \file net/tcpin.c
 * \brief TCP input functions.
 *
 * \verbatim
 * $Id: tcpin.c 5505 2014-01-01 11:15:16Z mifi $
 * \endverbatim
 */

#include <cfg/os.h>
#include <sys/thread.h>

#include <netinet/ip.h>
#include <sys/socket.h>
#include <netinet/tcp.h>

#ifdef NUTDEBUG
#include <net/netdebug.h>
#endif

/*!
 * \addtogroup xgTCP
 */
/*@{*/


/*!
 * \brief Process incoming TCP segments from IP layer.
 *
 * \warning The caller must take care not to pass broadcast
 *          or multicast segments.
 *
 * \note This routine is called by the IP layer on incoming
 *       TCP  segments. Applications typically do not call
 *       this function.
 *
 */
int NutTcpInput(NUTDEVICE * dev, NETBUF * nb)
{
    TCPHDR *th = (TCPHDR *) nb->nb_tp.vp;

    (void)dev;

    /* Process unicasts only. */
    if (th && (nb->nb_flags & NBAF_UNICAST) != 0) {
        uint_fast8_t hdrlen = th->th_off * 4;

        /* Check the header length. */
        if (hdrlen >= sizeof(TCPHDR) && hdrlen <= nb->nb_tp.sz) {
            nb->nb_ap.sz = nb->nb_tp.sz - hdrlen;
            if (nb->nb_ap.sz) {
                nb->nb_ap.vp = ((uint32_t *) th) + th->th_off;
            }
            nb->nb_tp.sz = hdrlen;

            /*
            ** According to RFC1122 we MUST check the checksum
            ** on incoming segments. Anyway, we rely on lower
            ** level checksums to save processing resources.
            **
            ** However, we may combine checksum calculation
            ** with moving data to the receiver buffer.
            **/

            NutTcpStateMachine(nb);

            return 0;
        }
    }
    /* Silently discard this segment. */
    NutNetBufFree(nb);

    return 0;
}

/*@}*/
