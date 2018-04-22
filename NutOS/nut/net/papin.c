/*
 * Copyright (C) 2001-2003 by egnite Software GmbH
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
 * \file net/papin.c
 * \brief PPP PAP output functions.
 *
 * \verbatim
 * $Id: papin.c 5505 2014-01-01 11:15:16Z mifi $
 * \endverbatim
 */

#include <dev/ppp.h>
#include <sys/types.h>
#include <net/if_var.h>
#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>
#include <netinet/in.h>
/*!
 * \addtogroup xgPAP
 *
 */
/*@{*/


/*
 * No server support.
 */
void PapRxAuthReq(NUTDEVICE *dev, uint8_t id, NETBUF *nb)
{
    NutPapOutput(dev, XCP_CONFACK, id, nb);
}

void PapRxAuthAck(NUTDEVICE *dev, uint8_t id, NETBUF *nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    (void)id;
    (void)nb;

    if(dcb->dcb_auth_state == PAPCS_AUTHREQ) {
        /*
         * Flag us open and start the network.
         */
        dcb->dcb_auth_state = PAPCS_OPEN;
        IpcpLowerUp(dev);
    }
}

void PapRxAuthNak(NUTDEVICE *dev, uint8_t id, NETBUF *nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    (void)id;
    (void)nb;

    if(dcb->dcb_auth_state == PAPCS_AUTHREQ) {
        dcb->dcb_auth_state = PAPCS_BADAUTH;
        IpcpLowerDown(dev);
    }
}

/*!
 * \brief Handle incoming PAP packets.
 *
 *
 * \param dev Identifies the device that received the packet.
 * \param nb  Pointer to a network buffer structure containing
 *            the PAP packet.
 */
void NutPapInput(NUTDEVICE * dev, NETBUF * nb)
{
    XCPHDR *pap;
    uint16_t len;

    /*
     * Drop packets with illegal lengths.
     */
    if (nb->nb_nw.sz < sizeof(XCPHDR)) {
        NutNetBufFree(nb);
        return;
    }
    pap = (XCPHDR *) nb->nb_nw.vp;
    if ((len = htons(pap->xch_len)) < sizeof(XCPHDR) || len > nb->nb_nw.sz) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * Split the PAP packet.
     */
    nb->nb_ap.vp = pap + 1;
    nb->nb_ap.sz = htons(pap->xch_len) - sizeof(XCPHDR);

    /*
     * Action depends on code.
     */
    switch (pap->xch_code) {
    case XCP_CONFREQ:
        PapRxAuthReq(dev, pap->xch_id, nb);
        break;

    case XCP_CONFACK:
        PapRxAuthAck(dev, pap->xch_id, nb);
        break;

    case XCP_CONFNAK:
        PapRxAuthNak(dev, pap->xch_id, nb);
        break;

    default:
        break;
    }
    NutNetBufFree(nb);
}

/*@}*/
