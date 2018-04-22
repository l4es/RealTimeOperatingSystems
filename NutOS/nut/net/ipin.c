/*
 * Copyright (C) 2008-2012 by egnite GmbH
 * Copyright (C) 2001-2007 by egnite Software GmbH
 * Copyright (c) 1983, 1993 by The Regents of the University of California
 * Copyright (c) 1993 by Digital Equipment Corporation
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
 * \file net/ipin.c
 * \brief IP input functions.
 *
 * \verbatim
 * $Id: ipin.c 4636 2012-09-20 09:02:23Z haraldkipp $
 * \endverbatim
 */

#include <net/route.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/icmp.h>
#include <netinet/ip_icmp.h>
#include <netinet/igmp.h>
#include <netinet/udp.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/*!
 * \addtogroup xgIP
 */
/*@{*/

static NutIpFilterFunc NutIpFilter;

/*!
 * \brief Set filter function for incoming IP datagrams.
 *
 * The callbackFunc is called by the IP layer on every incoming IP
 * datagram. Thus it must not block. The implementer returns 0 for
 * allow, -1 for deny.
 *
 * It is recommended to set the filer after DHCP has done its thing,
 * just in case your DHCP server is on a different subnet for example.
 *
 * \param callbackFunc Pointer to callback function to filter IP packets.
 *                     Set to 0 to disable the filter again.
 */
void NutIpSetInputFilter(NutIpFilterFunc callbackFunc)
{
    NutIpFilter = callbackFunc;
}

/*!
 * \brief Pointer to an optional demultiplexer.
 *
 * This pointer will be set on the first call to NutRegisterIpHandler().
 */
int (*ip_demux) (NUTDEVICE *, NETBUF *);

/*!
 * \brief Process incoming IP datagrams.
 * \internal
 *
 * Datagrams addressed to other destinations and datagrams
 * whose version number is not 4 are silently discarded.
 *
 * This routine is called by the Ethernet layer on incoming IP datagrams.
 *
 * \param dev Identifies the device that received this datagram.
 * \param nb  The network buffer received.
 */
void NutIpInput(NUTDEVICE * dev, NETBUF * nb)
{
    IPHDR *ip;
    uint_fast8_t hdrlen;
    uint32_t dst;
    uint_fast8_t bcast = 0;
    IFNET *nif;

    ip = nb->nb_nw.vp;

    /*
     * Silently discard datagrams of different IP version as well as
     * fragmented or filtered datagrams.
     */
    if (ip->ip_v != IPVERSION ||        /* Version check. */
        (ntohs(ip->ip_off) & (IP_MF | IP_OFFMASK)) != 0 ||      /* Fragmentation. */
        (NutIpFilter && NutIpFilter(ip->ip_src))) {     /* Filter. */
        NutNetBufFree(nb);
        return;
    }

    /*
    ** IP header length is given in 32-bit fields. Calculate the size in
    ** bytes and make sure that the header we know will fit in. Check
    ** further, that the header length is not larger than the bytes we
    ** received.
    */
    hdrlen = ip->ip_hl * 4;
    if (hdrlen < sizeof(IPHDR) || hdrlen > nb->nb_nw.sz) {
        NutNetBufFree(nb);
        return;
    }

#if NUT_IP_INCHKSUM
    /* Optional checksum calculation on incoming datagrams. */
#endif

    dst = ip->ip_dst;
    nif = dev->dev_icb;

    /*
     * Check for limited broadcast.
     */
    if (dst == INADDR_BROADCAST) {
        bcast = 1;
    }

    /*
     * Check for multicast.
     */
    else if (IN_MULTICAST(dst)) {
        MCASTENTRY *mca;

        for (mca = nif->if_mcast; mca; mca = mca->mca_next) {
            if (dst == mca->mca_ip) {
                break;
            }
        }
        if (mca == NULL) {
            NutNetBufFree(nb);
            return;
        }
        bcast = 2;
    }

    /*
     * Check device's local IP address.
     */
    else if (nif->if_local_ip != 0) {
        /*
         * Check for unicast.
         */
        if (dst == nif->if_local_ip) {
            nb->nb_flags |= NBAF_UNICAST;
        }

        /*
         * Check for net-directed broadcast.
         */
        else if (~nif->if_mask && (dst & ~nif->if_mask) == ~nif->if_mask) {
            bcast = 1;
        }

        /*
         * Not for us, discard silently.
         */
        else {
            NutIpForward(nb);
            NutNetBufFree(nb);
            return;
        }
    }

    /*
     * Calculate IP data length.
     */
    nb->nb_tp.sz = htons(ip->ip_len);
    if (nb->nb_tp.sz < hdrlen || nb->nb_tp.sz > nb->nb_nw.sz) {
        NutNetBufFree(nb);
        return;
    }
    nb->nb_nw.sz = hdrlen;
    nb->nb_tp.sz -= hdrlen;
    if (nb->nb_tp.sz) {
        nb->nb_tp.vp = ((char *) ip) + hdrlen;
    }

    /*
     * Route valid datagram to the related handler.
     */
    if (ip_demux == NULL || (*ip_demux) (dev, nb)) {
        switch (ip->ip_p) {
        case IPPROTO_ICMP:
            NutIcmpInput(dev, nb);
            break;
        case IPPROTO_IGMP:
            NutIgmpInput(dev, nb);
            break;
        default:
            /* Unknown protocol, send ICMP destination (protocol)
            * unreachable message.
            */
            if (bcast || !NutIcmpResponse(ICMP_UNREACH, ICMP_UNREACH_PROTOCOL, 0, nb))
                NutNetBufFree(nb);
            break;
        }
    }
}

/*@}*/
