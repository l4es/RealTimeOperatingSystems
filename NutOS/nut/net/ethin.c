/*
 * Copyright (C) 2008 by egnite GmbH
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
 * \file net/ethin.c
 * \brief Ethernet input functions.
 *
 * \verbatim
 * $Id: ethin.c 4407 2012-08-05 17:06:22Z haraldkipp $
 * \endverbatim
 */

#include <sys/types.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <net/ether.h>

/*!
 * \addtogroup xgEthernet
 */
/*@{*/

/*!
 * \brief Pointer to an optional demultiplexer.
 *
 * This pointer will be set on the first call to NutRegisterEthHandler().
 */
int (*ether_demux) (NUTDEVICE *, NETBUF *);

/*!
 * \brief Handle incoming Ethernet frames.
 *
 * Splits the Ethernet frame into the data link and the
 * network part. Then the frame is routed to the proper
 * handler, based on the type field in the Ethernet header.
 *
 * If the frame neither contains an IP nor an ARP type telegram and if
 * no registered handler exists or accepts the frame, then it is silently
 * discarded.
 *
 * \note This routine is called by the device driver on
 *       incoming Ethernet packets. Applications typically do
 *       not call this function.
 *
 * \param dev Identifies the device that received the frame.
 * \param nb  Pointer to a network buffer structure containing
 *            the Ethernet frame.
 */
void NutEtherInput(NUTDEVICE * dev, NETBUF * nb)
{
    ETHERHDR *eh;

    /*
     * Split the Ethernet frame.
     */
    eh = (ETHERHDR *) nb->nb_dl.vp;
    nb->nb_nw.vp = eh + 1;
    nb->nb_nw.sz = nb->nb_dl.sz - sizeof(ETHERHDR);
    nb->nb_dl.sz = sizeof(ETHERHDR);

    /* Route frame to the correct handler. Process registered handlers
    ** first, IP next and ARP last. */
    if (ether_demux == NULL || (*ether_demux) (dev, nb)) {
        switch (ntohs(eh->ether_type)) {
        case ETHERTYPE_IP:
            NutIpInput(dev, nb);
            break;
        case ETHERTYPE_ARP:
            NutArpInput(dev, nb);
            break;
        default:
            /* No handler found. Silently discard the frame. */
            NutNetBufFree(nb);
#ifdef NUT_PERFMON
            {
                IFNET *nif = (IFNET *) dev->dev_icb;
                nif->if_in_unknown_protos++;
            }
#endif
            break;
        }
    }
}

/*@}*/
