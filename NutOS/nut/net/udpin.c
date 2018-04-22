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
 * \file net/udpin.c
 * \brief UDP input functions.
 *
 * \verbatim
 * $Id: udpin.c 5505 2014-01-01 11:15:16Z mifi $
 * \endverbatim
 */

#include <sys/event.h>

#include <netinet/udp.h>
#include <sys/socket.h>
#include <netinet/ip_icmp.h>
#include <netinet/icmp.h>

/*!
 * \addtogroup xgUDP
 */
/*@{*/

/*!
 * \brief Handle incoming UDP packets.
 *
 * \note This routine is called by the IP layer on
 *       incoming UDP packets. Applications typically do
 *       not call this function.
 *
 * \param nb    Network buffer structure containing the UDP packet.
 */
int NutUdpInput(NUTDEVICE * dev, NETBUF * nb)
{
    UDPHDR *uh;
    UDPSOCKET *sock;

    (void)dev;

    uh = (UDPHDR *) nb->nb_tp.vp;
    /* Make sure that the datagram contains a full header. */
    if (uh == NULL || nb->nb_tp.sz < sizeof(UDPHDR)) {
        NutNetBufFree(nb);
        return 0;
    }

    nb->nb_ap.sz = nb->nb_tp.sz - sizeof(UDPHDR);
    nb->nb_tp.sz = sizeof(UDPHDR);
    if (nb->nb_ap.sz) {
        nb->nb_ap.vp = uh + 1;
    }

    /*
     * Find a port. If none exists and if this datagram hasn't been
     * broadcasted, return an ICMP unreachable.
     */
    if ((sock = NutUdpFindSocket(uh->uh_dport)) == 0) {
        if ((nb->nb_flags & NBAF_UNICAST) == 0 ||
            NutIcmpResponse(ICMP_UNREACH, ICMP_UNREACH_PORT, 0, nb) == 0) {
            NutNetBufFree(nb);
        }
        return 0;
    }

    /* if buffer size is defined, use packet queue */
    if (sock->so_rx_bsz) {
        /* New packet fits into the buffer? */
        if (sock->so_rx_cnt + nb->nb_ap.sz > sock->so_rx_bsz) {
            /* No, so discard it */
            NutNetBufFree(nb);
            return 0;
        } else {
            /* if a first packet is already in the queue, find the end
             * and add the new packet */
            if (sock->so_rx_nb) {
                NETBUF *snb;
                for (snb = sock->so_rx_nb; snb->nb_next != 0; snb = snb->nb_next);
                snb->nb_next = nb;
            } else
                sock->so_rx_nb = nb;

            /* increment input buffer count */
            sock->so_rx_cnt += nb->nb_ap.sz;
        };
    } else {                    /* no packet queue */
        /* if a packet is still buffered, discard it */
        if (sock->so_rx_nb) {
            NutNetBufFree(sock->so_rx_nb);
        }
        sock->so_rx_nb = nb;
        sock->so_rx_cnt = nb->nb_ap.sz; /* set input buffer count to size of new packet */
    };

    /* post the event only, if one thread is waiting */
    if (sock->so_rx_rdy)
        NutEventPost(&sock->so_rx_rdy);
    return 0;
}

/*@}*/
