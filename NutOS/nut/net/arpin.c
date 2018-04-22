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
 *
 */

/*!
 * \file net/arpin.c
 * \brief ARP input functions.
 *
 * \verbatim
 * $Id: arpin.c 3686 2011-12-04 14:20:38Z haraldkipp $
 * \endverbatim
 */

#include <sys/types.h>
#include <net/if_var.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
/*!
 * \addtogroup xgARP
 *
 * \todo Response may reuse received ARP packet.
 */
/*@{*/


/*!
 * \brief Handle incoming ARP packets.
 *
 * Packets not destined to us or packets with unsupported
 * address type or item length are silently discarded.
 *
 * \note This routine is called by the Ethernet layer on
 *       incoming ARP packets. Applications typically do
 *       not call this function.
 *
 * \param dev Identifies the device that received the packet.
 * \param nb  Pointer to a network buffer structure containing
 *            the ARP packet.
 */
void NutArpInput(NUTDEVICE * dev, NETBUF * nb)
{
    ETHERARP *ea;
    ARPHDR *ah;
    IFNET *nif;

    if (nb->nb_nw.sz < sizeof(ETHERARP)) {
        NutNetBufFree(nb);
        return;
    }

    ea = (ETHERARP *) nb->nb_nw.vp;
    ah = (ARPHDR *) & ea->ea_hdr;

    /*
     * Silently discard packets with unsupported
     * address types and lengths.
     */
    if (ntohs(ah->ar_hrd) != ARPHRD_ETHER ||
        ntohs(ah->ar_pro) != ETHERTYPE_IP ||
        ah->ar_hln != 6 || ah->ar_pln != 4) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * Silently discard packets for other destinations.
     */
    nif = dev->dev_icb;
    if (ea->arp_tpa != nif->if_local_ip) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * TODO: Silently discard packets with our own
     * source address.
     */

    /*
     * TODO: Discard packets with broadcast source
     * address.
     */

    /*
     * TODO: Discard packets if source IP address
     * equals our address. We should send a reply.
     */

    /*
     * Add the sender to our ARP cache. Note, that we don't do
     * this with replies only, but also with requests on our
     * address. The assumption is that if someone is requesting
     * our address, they are probably intending to talk to us,
     * so it saves time if we cache their address.
     */
    NutArpCacheUpdate(dev, ea->arp_spa, ea->arp_sha);

    /*
     * Reply to ARP requests.
     */
    if (htons(ea->ea_hdr.ar_op) == ARPOP_REQUEST) {
        NETBUF *nbr =
            NutArpAllocNetBuf(ARPOP_REPLY, ea->arp_spa, ea->arp_sha);

        if (nbr) {
            if (!NutArpOutput(dev, nbr))
                NutNetBufFree(nbr);
        }
    }
    NutNetBufFree(nb);
}

/*@}*/
