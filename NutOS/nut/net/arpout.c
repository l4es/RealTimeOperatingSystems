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
 * \file net/arpout.c
 * \brief ARP output functions.
 *
 * \verbatim
 * $Id: arpout.c 3686 2011-12-04 14:20:38Z haraldkipp $
 * \endverbatim
 */

#include <string.h>

#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/types.h>
#include <net/if_var.h>
#include <netinet/if_ether.h>
#include <net/ether.h>
#include <netinet/in.h>
/*!
 * \addtogroup xgARP
 */
/*@{*/

/*!
 * \brief Allocate an ARP network buffer structure.
 *
 * \param type Type of ARP packet.
 * \param ip   Target IP address.
 * \param mac  Target MAC address, null pointer for broadcast.
 *
 * \return Pointer to the allocated network buffer structure
 *         or 0 on failure.
 */
NETBUF *NutArpAllocNetBuf(uint16_t type, uint32_t ip, uint8_t * mac)
{
    NETBUF *nb;
    ETHERARP *ea;
    ARPHDR *ah;

    if ((nb = NutNetBufAlloc(0, NBAF_NETWORK, sizeof(ETHERARP))) == 0)
        return 0;

    ea = nb->nb_nw.vp;
    ah = (ARPHDR *) & ea->ea_hdr;

    /*
     * Set ARP header.
     */
    ah->ar_hrd = htons(ARPHRD_ETHER);
    ah->ar_pro = htons(ETHERTYPE_IP);
    ah->ar_hln = 6;
    ah->ar_pln = 4;
    ah->ar_op = htons(type);

    /*
     * Set ARP destination data.
     */
    if (mac)
        memcpy(ea->arp_tha, mac, 6);
    else
        memset(ea->arp_tha, 0xff, 6);
    ea->arp_tpa = ip;

    return nb;
}

/*!
 * \brief Send an ARP packet.
 *
 * \note Applications typically do not call this function.
 *
 * \param dev   Identifies the device to use.
 * \param nb    Network buffer structure containing the packet to be sent.
 *              The structure must have been allocated by a previous
 *              call NutNetBufAlloc().
 *
 * \return 0 on success, -1 in case of any errors.
 */
int NutArpOutput(NUTDEVICE * dev, NETBUF * nb)
{
    ETHERARP *ea;
    IFNET *nif;

    ea = nb->nb_nw.vp;

    /*
     * Set ARP source data.
     */
    nif = dev->dev_icb;
    memcpy(ea->arp_sha, nif->if_mac, 6);
    ea->arp_spa = nif->if_local_ip;

    return (*nif->if_output)(dev, ETHERTYPE_ARP, ea->arp_tha, nb);
}

/*@}*/
