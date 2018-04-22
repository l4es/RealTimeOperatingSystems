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
 * \file net/papout.c
 * \brief PPP PAP output functions.
 *
 * \verbatim
 * $Id: papout.c 5505 2014-01-01 11:15:16Z mifi $
 * \endverbatim
 */

#include <string.h>

#include <dev/ppp.h>

#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/types.h>
#include <net/if_var.h>
#include <net/ppp.h>
#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>
#include <netinet/in.h>
/*!
 * \addtogroup xgPAP
 */
/*@{*/

/*!
 * \brief Send a PAP packet.
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
int NutPapOutput(NUTDEVICE * dev, uint8_t code, uint8_t id, NETBUF * nb)
{
    XCPHDR *xch;

    if ((nb = NutNetBufAlloc(nb, NBAF_NETWORK, sizeof(XCPHDR))) == 0)
        return -1;

    xch = nb->nb_nw.vp;
    xch->xch_code = code;
    xch->xch_id = id;
    xch->xch_len = htons(nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz);

    if (NutPppOutput(dev, PPP_PAP, 0, nb) == 0)
        NutNetBufFree(nb);

    return 0;
}

/*
 * Send a Configure-Request.
 */
void PapTxAuthReq(NUTDEVICE *dev, uint8_t id)
{
    PPPDCB *dcb = dev->dev_dcb;
    NETBUF *nb;
    char *cp;
    int len;

    (void)id;

    /*
     * Create the request.
     */
    len = 2;
    if(dcb->dcb_user)
        len += strlen((char*)dcb->dcb_user);
    if(dcb->dcb_pass)
        len += strlen((char*)dcb->dcb_pass);
    if ((nb = NutNetBufAlloc(0, NBAF_APPLICATION, len)) != 0) {
        cp = nb->nb_ap.vp;
        *cp = dcb->dcb_user ? (char)strlen((char*)dcb->dcb_user) : 0;
        if(*cp)
            memcpy(cp + 1, dcb->dcb_user, *cp);

        cp += *cp + 1;
        *cp = dcb->dcb_pass ? (char)strlen((char*)dcb->dcb_pass) : 0;
        if(*cp)
            memcpy(cp + 1, dcb->dcb_pass, *cp);

        dcb->dcb_auth_state = PAPCS_AUTHREQ;
        NutPapOutput(dev, XCP_CONFREQ, ++dcb->dcb_reqid, nb);
    }
}

/*@}*/
