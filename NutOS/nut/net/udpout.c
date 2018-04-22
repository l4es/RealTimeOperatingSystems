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
 * \file net/udpout.c
 * \brief UDP output functions.
 *
 * \verbatim
 * $Id: udpout.c 3686 2011-12-04 14:20:38Z haraldkipp $
 * \endverbatim
 */

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/ipcsum.h>
#include <net/route.h>
#include <sys/socket.h>

/*!
 * \addtogroup xgUDP
 */
/*@{*/

/*!
 * \brief Send a UDP packet.
 *
 * \param sock  Socket descriptor. This pointer must have been
 *              retrieved by calling NutUdpCreateSocket().
 * \param daddr IP address of the remote host in network byte order.
 * \param port  Remote port number in host byte order.
 * \param nb    Network buffer structure containing the datagram.
 *              This buffer will be released if the function returns
 *              an error.
 *
 * \note Applications typically do not call this function but
 *       use the UDP socket interface.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutUdpOutput(UDPSOCKET * sock, uint32_t daddr, uint16_t port, NETBUF * nb)
{
    uint32_t saddr;
    uint32_t csum;
    UDPHDR *uh;
    NUTDEVICE *dev;
    IFNET *nif;

    if ((nb = NutNetBufAlloc(nb, NBAF_TRANSPORT, sizeof(UDPHDR))) == 0)
        return -1;

    uh = nb->nb_tp.vp;
    uh->uh_sport = sock->so_local_port;
    uh->uh_dport = htons(port);
    uh->uh_ulen = htons((uint16_t)(nb->nb_tp.sz + nb->nb_ap.sz));

    /*
     * Get local address for this destination.
     */
    if ((dev = NutIpRouteQuery(daddr, &saddr)) != 0) {
        nif = dev->dev_icb;
        saddr = nif->if_local_ip;
    } else
        saddr = 0;

    uh->uh_sum = 0;
    csum = NutIpPseudoChkSumPartial(saddr, daddr, IPPROTO_UDP, uh->uh_ulen);
    csum = NutIpChkSumPartial(csum, uh, sizeof(UDPHDR));
    uh->uh_sum = NutIpChkSum(csum, nb->nb_ap.vp, nb->nb_ap.sz);

    return NutIpOutput(IPPROTO_UDP, daddr, nb);
}

/*@}*/
