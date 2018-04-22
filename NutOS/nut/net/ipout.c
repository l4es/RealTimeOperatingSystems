/*
 * Copyright (C) 2001-2007 by egnite Software GmbH
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
 * \file net/ipout.c
 * \brief IP output functions.
 *
 * \verbatim
 * $Id: ipout.c 5505 2014-01-01 11:15:16Z mifi $
 * \endverbatim
 */

#include <string.h>

#include <sys/types.h>
#include <net/ether.h>
#include <net/route.h>
#include <netinet/if_ether.h>
#include <netinet/if_ppp.h>
#include <netinet/ipcsum.h>
#include <netinet/ip.h>
#include <netinet/in.h>
/*!
 * \addtogroup xgIP
 */
/*@{*/

/*!
 * \brief Send IP datagram.
 *
 * Route an IP datagram to the proper interface.
 *
 * The function will not return until the data has been stored
 * in the network device hardware for transmission. If the
 * device is not ready for transmitting a new packet, the
 * calling thread will be suspended until the device becomes
 * ready again. If the hardware address of the target host needs
 * to be resolved the function will be suspended too.
 *
 * \param proto Protocol type.
 * \param dest  Destination IP address. The function will determine
 *              the proper network interface by checking the routing
 *              table. It will also perform any necessary hardware
 *              address resolution.
 * \param nb    Network buffer structure containing the datagram.
 *              This buffer will be released if the function returns
 *              an error.
 *
 * \return 0 on success, -1 otherwise.
 *
 * \bug Broadcasts to multiple network devices will fail after the
 *      first device returns an error.
 */
int NutIpOutput(uint8_t proto, uint32_t dest, NETBUF * nb)
{
    uint8_t ha[6];
    IPHDR_OPT *ip;
    NUTDEVICE *dev;
    IFNET *nif;
    uint32_t gate;

    if (proto != IPPROTO_IGMP) {
        if ((nb = NutNetBufAlloc(nb, NBAF_NETWORK, sizeof(IPHDR))) == 0)
            return -1;
    } else {
        if ((nb = NutNetBufAlloc(nb, NBAF_NETWORK, sizeof(IPHDR_OPT))) == 0)
            return -1;
    }

    /*
     * Set those items in the IP header, which are common for
     * all interfaces.
     */
    ip = nb->nb_nw.vp;
    ip->ip_v = 4;
    if (proto == IPPROTO_IGMP) {
        ip->ip_hl = sizeof(IPHDR_OPT) / 4;
    } else {
        ip->ip_hl = sizeof(IPHDR) / 4;
    }
    ip->ip_tos = 0;
    ip->ip_len = htons(nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz);
    ip->ip_off = 0;
    if (proto == IPPROTO_IGMP) {
        ip->ip_ttl = 1;
    } else {
        ip->ip_ttl = 0x40;
    }
    ip->ip_p = proto;
    ip->ip_dst = dest;

    if (proto == IPPROTO_IGMP) {
        /* Router Alert Option */
        ip->ip_option = htonl(0x94040000);
    }


    /*
     * Limited broadcasts are sent on all network interfaces.
     * See RFC 919.
     */
    if ((dest == 0xffffffff) || (IP_IS_MULTICAST(dest))) {
        if (dest == 0xffffffff) {
            /* Broadcast */
            memset(ha, 0xff, sizeof(ha));
        } else {
            /* Multicast */
            ha[0] = 0x01;
            ha[1] = 0x00;
            ha[2] = 0x5E;
            ha[3] = ((uint8_t *) & dest)[1] & 0x7f;
            ha[4] = ((uint8_t *) & dest)[2];
            ha[5] = ((uint8_t *) & dest)[3];
        }

        for (dev = nutDeviceList; dev; dev = dev->dev_next) {
            if (dev->dev_type == IFTYP_NET) {
                /*
                 * Set remaining IP header items using a NETBUF clone and calculate
                 * the checksum.
                 */
                int rc = 0;
                NETBUF *nb_clone = NutNetBufClonePart(nb, NBAF_NETWORK);

                nif = dev->dev_icb;
                ip = nb_clone->nb_nw.vp;
                ip->ip_id = htons(nif->if_pkt_id);
                nif->if_pkt_id++;
                ip->ip_src = nif->if_local_ip;
                ip->ip_ttl = 1;
                ip->ip_sum = 0;
                ip->ip_sum = NutIpChkSum(0, nb_clone->nb_nw.vp, nb_clone->nb_nw.sz);
                if (nif->if_type == IFT_ETHER)
                    rc = (*nif->if_output) (dev, ETHERTYPE_IP, ha, nb_clone);
                else if (nif->if_type == IFT_PPP)
                    rc = (*nif->if_output) (dev, PPP_IP, 0, nb_clone);
                if (rc == 0) {
                    NutNetBufFree(nb_clone);
                }
            }
        }
        return 0;
    }

    /*
     * Get destination's route. This will also return the proper
     * interface.
     */
    if ((dev = NutIpRouteQuery(dest, &gate)) == 0) {
        NutNetBufFree(nb);
        return -1;
    }

    /*
     * Set remaining IP header items and calculate the checksum.
     */
    nif = dev->dev_icb;
    ip->ip_id = htons(nif->if_pkt_id);
    nif->if_pkt_id++;
    ip->ip_src = nif->if_local_ip;
    ip->ip_sum = 0;
    ip->ip_sum = NutIpChkSum(0, nb->nb_nw.vp, nb->nb_nw.sz);

    /*
     * On Ethernet we query the MAC address of our next hop,
     * which might be the destination or the gateway to this
     * destination.
     */
    if (nif->if_type == IFT_ETHER) {
        /*
         * Detect directed broadcasts for the local network. In this
         * case don't send ARP queries, but send directly to MAC broadcast
         * address.
         */
        if ((gate == 0) && ((dest | nif->if_mask) == 0xffffffff)) {
            memset(ha, 0xff, sizeof(ha));
        } else if (NutArpCacheQuery(dev, gate ? gate : dest, ha)) {
            /* Note, that a failed ARP request is not considered a
               transmission error. It might be caused by a simple
               packet loss. */
            return 0;
        }
        return (*nif->if_output) (dev, ETHERTYPE_IP, ha, nb);
    } else if (nif->if_type == IFT_PPP)
        return (*nif->if_output) (dev, PPP_IP, 0, nb);

    NutNetBufFree(nb);
    return -1;
}

/*!
 * \brief Forward IP datagram.
 *
 * Forwards IP datagram to the proper destination if NET_IP_FORWARD has
 * been configured.
 *
 * \param nb    Network buffer structure containing the datagram.
 *              This buffer will be released if the function returns
 *              an error.
 *
 * \return Always 0.
 */
int NutIpForward(NETBUF *nb)
{
#ifdef NUT_IP_FORWARDING

    NUTDEVICE *dev;
    IFNET *nif;
    NETBUF *r_nb;
    IPHDR *ip;
    uint32_t dest;

    /* Check the time to live. */
    ip = nb->nb_nw.vp;
    if (ip->ip_ttl <= 1) {
        return 0;
    }

    /* Retrieve interface and gateway to the destination. */
    dev = NutIpRouteQuery(ip->ip_dst, &dest);
    if (dev == NULL) {
        return 0;
    }
    if (dest == 0) {
        dest = ip->ip_dst;
    }
    nif = dev->dev_icb;

    /* Allocate a new NETBUF and copy the payload to it. */
    r_nb = NutNetBufAlloc(NULL, NBAF_TRANSPORT, nb->nb_tp.sz);
    if (r_nb == NULL) {
        return 0;
    }
    memcpy(r_nb->nb_tp.vp, nb->nb_tp.vp, nb->nb_tp.sz);

    /* Copy the IP header to the new NETBUF. */
    r_nb = NutNetBufAlloc(r_nb, NBAF_NETWORK, nb->nb_nw.sz);
    if (r_nb == NULL) {
        return 0;
    }
    memcpy(r_nb->nb_nw.vp, nb->nb_nw.vp, nb->nb_nw.sz);

    ip = r_nb->nb_nw.vp;
    ip->ip_ttl--;
    ip->ip_sum = 0;
    ip->ip_sum = NutIpChkSum(0, ip, sizeof(IPHDR));

    /* Forward the new NETBUF to the interface. */
    if (nif->if_type == IFT_ETHER) {
        uint8_t ha[6];

        if (NutArpCacheQuery(dev, dest, ha)) {
            return 0;
        }
        if ((*nif->if_output) (dev, ETHERTYPE_IP, ha, r_nb)) {
            return 0;
        }
    }
    else if (nif->if_type == IFT_PPP) {
        if ((*nif->if_output) (dev, PPP_IP, 0, r_nb)) {
            return 0;
        }
    }
    NutNetBufFree(r_nb);
#else
    (void)nb;
#endif
    return 0;
}

/*@}*/
