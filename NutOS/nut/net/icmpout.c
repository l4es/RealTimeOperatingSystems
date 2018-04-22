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
 * \file net/icmpout.c
 * \brief ICMP output functions.
 *
 * \verbatim
 * $Id: icmpout.c 3686 2011-12-04 14:20:38Z haraldkipp $
 * \endverbatim
 */

#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <netinet/ipcsum.h>
#include <netinet/icmp.h>
#include <string.h>

/*!
 * \addtogroup xgICMP
 */
/*@{*/
/*!
 * \brief Send an ICMP datagram.
 *
 * Known ICMP types are:
 *
 * - #ICMP_ECHOREPLY Echo reply
 * - #ICMP_UNREACH Destination unreachable
 * - #ICMP_SOURCEQUENCH Packet lost
 * - #ICMP_REDIRECT Shorter route
 * - #ICMP_ECHO Echo reply
 * - #ICMP_ROUTERADVERT Router advertisement
 * - #ICMP_ROUTERSOLICIT Router solicitation
 * - #ICMP_TIMXCEED Time exceeded
 * - #ICMP_PARAMPROB Bad IP header
 * - #ICMP_TSTAMP Timestamp request
 * - #ICMP_TSTAMPREPLY Timestamp reply
 * - #ICMP_IREQ Information request
 * - #ICMP_IREQREPLY Information reply
 * - #ICMP_MASKREQ Address mask request
 * - #ICMP_MASKREPLY Address mask reply
 *
 * \param type Type of the ICMP message.
 * \param dest Destination IP address.
 * \param nb   Network buffer structure containing the datagram.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutIcmpOutput(uint8_t type, uint32_t dest, NETBUF * nb)
{
    ICMPHDR *icp;
    uint16_t csum;

    icp = (ICMPHDR *) nb->nb_tp.vp;
    icp->icmp_type = type;
    icp->icmp_cksum = 0;

    csum = NutIpChkSumPartial(0, nb->nb_tp.vp, nb->nb_tp.sz);
    icp->icmp_cksum = NutIpChkSum(csum, nb->nb_ap.vp, nb->nb_ap.sz);

    return NutIpOutput(IPPROTO_ICMP, dest, nb);
}

/*!
 * \brief Send an ICMP message to a given destination.
 *
 * \param type Type of the ICMP message. See NutIcmpOutput() for
 *             a list of valid types.
 * \param code Type subcode.
 * \param spec Type specific data item.
 * \param dest IP address of the target.
 * \param nb   Network buffer structure containing the message to be sent.
 *             The structure must have been allocated by a previous
 *             call NutNetBufAlloc() and will be freed if this function
 *             returns with an error.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutIcmpReply(uint8_t type, uint8_t code, uint32_t spec, uint32_t dest, NETBUF * nb)
{
    ICMPHDR *icp;

    if ((nb = NutNetBufAlloc(nb, NBAF_TRANSPORT, sizeof(ICMPHDR))) == 0)
        return -1;

    icp = (ICMPHDR *) nb->nb_tp.vp;
    icp->icmp_code = code;
    icp->icmp_spec = spec;

    return NutIcmpOutput(type, dest, nb);
}

/*!
 * \brief Send an ICMP message as a response to a given destination.
 *
 * \param type Type of the ICMP message. See NutIcmpOutput() for
 *             a list of valid types.
 * \param code Type subcode.
 * \param spec Type specific data item.
 * \param nb   Network buffer structure containing the previously received
 *             network packet. According to RFC792 the complete IP header
 *             and the first 8 bytes of the transport netbuf is used as the
 *             application data for the response. If this function returns
 *             with an error, the buffer is freed. The destination address is
 *             taken from the IP header.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutIcmpResponse(uint8_t type, uint8_t code, uint32_t spec, NETBUF * nb)
{
    IPHDR *ip;
    uint32_t dest;

    ip = nb->nb_nw.vp;
    dest = ip->ip_src;

    if ((nb = NutNetBufAlloc(nb, NBAF_APPLICATION, sizeof(IPHDR) + 8)) == 0)
        return -1;

    memcpy(nb->nb_ap.vp, nb->nb_nw.vp, sizeof(IPHDR));
    memcpy((char *)nb->nb_ap.vp + sizeof(IPHDR), nb->nb_tp.vp, 8);

    return NutIcmpReply(type, code, spec, dest, nb);
}


/*@}*/
