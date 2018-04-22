/*
 * Copyright (C) 2001-2004 by egnite Software GmbH
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
 * \file net/icmpin.c
 * \brief ICMP input functions.
 *
 * \verbatim
 * $Id: icmpin.c 6135 2015-09-28 11:47:06Z olereinhardt $
 * \endverbatim
 */

#include <netinet/ip_icmp.h>
#include <netinet/ipcsum.h>
#include <netinet/icmp.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/in.h>
#include <errno.h>

/*!
 * \addtogroup xgICMP
 */
/*@{*/

/*!
 * \brief Translation table from icmp error code to errno.
 */

static const int icmp_code2errno[16] =
{
    ENETUNREACH,
    EHOSTUNREACH,
    ENOPROTOOPT,
    ECONNREFUSED,
    EMSGSIZE,
    EOPNOTSUPP,
    ENETUNREACH,
    EHOSTDOWN,
    ENETUNREACH,
    ENETUNREACH,
    EHOSTUNREACH,
    ENETUNREACH,
    EHOSTUNREACH,
    EHOSTUNREACH,
    EHOSTUNREACH,
    EHOSTUNREACH,
};

/*
 * Send out ICMP echo response.
 */
static int NutIcmpReflect(NUTDEVICE * dev, uint8_t type, NETBUF * nb)
{
    IPHDR *ip;
    uint32_t dest;
    IFNET *nif;

    ip = nb->nb_nw.vp;
    dest = ip->ip_src;
    nif = dev->dev_icb;
    ip->ip_src = nif->if_local_ip;
    ip->ip_ttl = MAXTTL;

    return NutIcmpOutput(type, dest, nb);
}

/*
 * Process incoming ICMP messages for destination unreachable.
 */
static int NutIcmpUnreach(NETBUF * nb, int icmp_code)
{
    IPHDR *ih;

    if (nb->nb_ap.sz < sizeof(IPHDR) + 8)
        return -1;

    ih = nb->nb_ap.vp;

    switch (ih->ip_p) {
        case IPPROTO_TCP:
        {
            TCPHDR *th;
            TCPSOCKET *sock_tcp;

            th = (TCPHDR *) (((char *) ih) + sizeof(IPHDR));
            sock_tcp = NutTcpFindSocket(th->th_dport, th->th_sport, ih->ip_src);
            if (sock_tcp == 0)
                return -1;

            if (sock_tcp->so_state != TCPS_SYN_SENT && sock_tcp->so_state != TCPS_ESTABLISHED)
                return -1;

            NutTcpAbortSocket(sock_tcp, icmp_code2errno[icmp_code]);
        }
        break;

#ifndef NUT_UDP_ICMP_EXCLUDE
        case IPPROTO_UDP:
        {
            UDPHDR *uh;
            UDPSOCKET *sock_udp;

            uh = (UDPHDR *) (((char *) ih) + sizeof(IPHDR));
            sock_udp = NutUdpFindSocket(uh->uh_dport);

            if (sock_udp == NULL)
                return -1;

            if (NutUdpSetSocketError(sock_udp, ih->ip_dst, uh->uh_dport, icmp_code2errno[icmp_code]))
                return -1;
        }
        break;
#endif

        default:
            return -1;
    }


    return 0;
}

/*!
 * \brief Handle incoming ICMP packets.
 *
 * Incoming ICMP packets are processed in the background.
 * NutNet currently handles echo request and destination
 * unreachable packets. Any other packet type is silently
 * discarded.
 *
 * \note This routine is called by the IP layer on incoming
 *       ICMP datagrams. Applications typically do not call
 *       this function.
 *
 * \param dev Identifies the device that received the packet.
 * \param nb  Pointer to a network buffer structure containing
 *            the ICMP datagram.
 */
void NutIcmpInput(NUTDEVICE * dev, NETBUF * nb)
{
    ICMPHDR *icp = (ICMPHDR *) nb->nb_tp.vp;

    /* Silently discard packets, which are too small. */
    if (icp == NULL || nb->nb_tp.sz < sizeof(ICMPHDR)) {
        NutNetBufFree(nb);
        return;
    }

    nb->nb_ap.sz = nb->nb_tp.sz - sizeof(ICMPHDR);
    if (nb->nb_ap.sz) {
        nb->nb_ap.vp = icp + 1;
        nb->nb_tp.sz = sizeof(ICMPHDR);
    }

    switch (icp->icmp_type) {
    case ICMP_ECHO:
        if (NutIcmpReflect(dev, ICMP_ECHOREPLY, nb)) {
            break;
        }
    case ICMP_UNREACH:
        NutIcmpUnreach(nb, icp->icmp_code);
    default:
        NutNetBufFree(nb);
    }
}

/*@}*/
