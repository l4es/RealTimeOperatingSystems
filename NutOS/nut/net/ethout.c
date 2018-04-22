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
 * \file net/ethout.c
 * \brief Ethernet output functions.
 *
 * \verbatim
 * $Id: ethout.c 4407 2012-08-05 17:06:22Z haraldkipp $
 * \endverbatim
 */

#include <string.h>
#include <sys/types.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <net/ether.h>

/*!
 * \addtogroup xgEthernet
 */
/*@{*/


/*!
 * \brief Send Ethernet frame.
 *
 * Send an Ethernet frame of a given type using the specified device.
 *
 * \param dev   Identifies the device to use.
 * \param type  Type of this frame, either ETHERTYPE_IP or ETHERTYPE_ARP.
 * \param ha    MAC address of the destination, set null for broadcasts.
 * \param nb    Network buffer structure containing the packet to be sent.
 *              The structure must have been allocated by a previous
 *              call NutNetBufAlloc() and will be freed if this function
 *              returns with an error.
 *
 * \return 0 on success, -1 in case of any errors.
 */
int NutEtherOutput(NUTDEVICE * dev, uint16_t type, uint8_t * ha, NETBUF * nb)
{
    ETHERHDR *eh;
    IFNET *nif;

    if (NutNetBufAlloc(nb, NBAF_DATALINK, sizeof(ETHERHDR)) == 0)
        return -1;

    eh = (ETHERHDR *) nb->nb_dl.vp;
    nif = dev->dev_icb;
    memcpy(eh->ether_shost, nif->if_mac, 6);
    if (ha) {
        memcpy(eh->ether_dhost, ha, 6);
        NUT_PERFMON_INC(nif->if_out_ucast_pkts);
    } else {
        memset(eh->ether_dhost, 0xff, 6);
        NUT_PERFMON_INC(nif->if_out_n_ucast_pkts);
    }
    eh->ether_type = htons(type);

    /*
     * Call the network device output routine.
     */
    if((*nif->if_send) (dev, nb)) {
        NutNetBufFree(nb);
        return -1;
    }
    return 0;
}

/*@}*/
