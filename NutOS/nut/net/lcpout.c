/*
 * Copyright (C) 2001-2004 by egnite Software GmbH
 * Copyright (c) 1989 by Carnegie Mellon University
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
 * \file net/lcpout.c
 * \brief PPP LCP output functions.
 *
 * \verbatim
 * $Id: lcpout.c 4602 2012-09-13 09:39:43Z haraldkipp $
 * \endverbatim
 */

#include <string.h>

#include <dev/ppp.h>
#include <sys/types.h>
#include <sys/heap.h>
#include <netinet/in.h>
#include <net/if_var.h>
#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>
#include <net/ppp.h>
#include <dev/usart.h>
#include <io.h>

/*!
 * \addtogroup xgLCP
 */
/*@{*/

extern uint32_t new_magic;

/*!
 * \brief Send a LCP packet.
 *
 * \note Applications typically do not call this function.
 *
 * \param dev   Identifies the device to use.
 * \param code  Type subcode.
 * \param id    Exchange identifier.
 * \param nb    Network buffer structure containing the packet to send
 *              or null if the packet contains no information.
 *              The structure must have been allocated by a previous
 *              call NutNetBufAlloc() and will be freed when this function
 *              returns.
 *
 * \return 0 on success, -1 in case of any errors.
 */
int NutLcpOutput(NUTDEVICE * dev, uint8_t code, uint8_t id, NETBUF * nb)
{
    XCPHDR *lcp;

    if ((nb = NutNetBufAlloc(nb, NBAF_NETWORK, sizeof(XCPHDR))) == 0)
        return -1;

    lcp = nb->nb_nw.vp;
    lcp->xch_code = code;
    lcp->xch_id = id;
    lcp->xch_len = htons(nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz);

    if (NutPppOutput(dev, PPP_LCP, 0, nb) == 0)
        NutNetBufFree(nb);

    return 0;
}

static INLINE void LcpResetOptions(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

    dcb->dcb_compr = 0;
    if ((dcb->dcb_user && *dcb->dcb_user) || (dcb->dcb_pass && *dcb->dcb_pass)) {
        dcb->dcb_auth = PPP_PAP;
    } else {
        dcb->dcb_auth = 0;
    }
    dcb->dcb_neg_magic = new_magic;
    dcb->dcb_loc_magic = 0;
    dcb->dcb_rem_magic = 0;
    dcb->dcb_accm = 0xFFffFFff;
    dcb->dcb_loc_mru = 1500;

    _ioctl(dcb->dcb_fd, HDLC_SETTXACCM, &dcb->dcb_accm );
}

/*
 * Send a Configure-Request.
 */
void LcpTxConfReq(NUTDEVICE * dev, uint8_t id, uint8_t rejected)
{
    PPPDCB *dcb = dev->dev_dcb;
    XCPOPT *xcpo;
    NETBUF *nb;

    /*
     * Not currently negotiating, reset options.
     */
    if (dcb->dcb_lcp_state != PPPS_REQSENT && dcb->dcb_lcp_state != PPPS_ACKRCVD && dcb->dcb_lcp_state != PPPS_ACKSENT) {
        LcpResetOptions(dev);
        dcb->dcb_lcp_naks = 0;
    }
    dcb->dcb_acked = 0;
    dcb->dcb_retries = 0;

    /*
     * Create the request.
     */
    if ((nb = NutNetBufAlloc(0, NBAF_APPLICATION, rejected ? 6 : 12)) != 0) {
        xcpo = nb->nb_ap.vp;
        xcpo->xcpo_type = LCP_ASYNCMAP;
        xcpo->xcpo_len = 6;
        xcpo->xcpo_.ul = htonl(LCP_DEFOPT_ASYNCMAP); /* Should this be "= 0;" instead? */

        /*
         * This is a temporary hack. In the initial version
         * we sent the ASYNCMAP only and never expected any
         * rejects. The MAGICNUMBER had been added later
         * to support echo requests, but some servers reject
         * this option. Now we still do not provide full
         * reject processing but blindly assume, that the
         * MAGICNUMBER is the rejected option.
         */
        if (!rejected) {
            xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
            xcpo->xcpo_type = LCP_MAGICNUMBER;
            xcpo->xcpo_len = 6;
            xcpo->xcpo_.ul = dcb->dcb_neg_magic;
        }

        NutLcpOutput(dev, XCP_CONFREQ, id, nb);
    }
}

/*
 * Send a Protocol-Reject for some protocol.
 */
void LcpTxProtRej(NUTDEVICE * dev, uint16_t protocol, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;
    NETBUF *nbr;
    uint16_t *sp;

    if ((nbr = NutNetBufAlloc(0, NBAF_APPLICATION, nb->nb_nw.sz)) != 0) {
        sp = nbr->nb_ap.vp;
        *sp++ = htons(protocol);
        memcpy(sp, nb->nb_nw.vp, nb->nb_nw.sz - 2);
        NutNetBufFree(nb);
        NutLcpOutput(dev, LCP_PROTREJ, ++dcb->dcb_rejid, nbr);
    } else {
        NutNetBufFree(nb);
    }
}

/*@}*/
