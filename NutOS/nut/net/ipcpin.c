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
 * \file net/ipcin.c
 * \brief PPP IPC input functions.
 *
 * \verbatim
 * $Id: ipcpin.c 5505 2014-01-01 11:15:16Z mifi $
 * \endverbatim
 */

#include <sys/event.h>
#include <sys/types.h>
#include <net/if_var.h>
#include <dev/ppp.h>

#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>
#include <netinet/in.h>

/*!
 * \addtogroup xgIPCP
 */
/*@{*/

static uint16_t IpcpValidateIpReq(uint32_t *expected_ip, uint32_t *requested_ip)
{
    if (*expected_ip == 0 && *requested_ip) {
        *expected_ip = *requested_ip;
    }
    else if (*expected_ip != *requested_ip) {
        *requested_ip = *expected_ip;
        return 6;
    }
    return 0;
}

/*
 * Received Configure-Request.
 */
static void IpcpRxConfReq(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;
    int rc = XCP_CONFACK;
    XCPOPT *xcpo;
    uint16_t xcpl;
    XCPOPT *xcpr;
    uint16_t xcps;
    uint16_t len = 0;
    uint_fast8_t i;
    uint32_t ip;

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSED:
        /*
         * Go away, we're closed.
         */
        NutNetBufFree(nb);
        NutIpcpOutput(dev, XCP_TERMACK, id, 0);
        return;

    case PPPS_CLOSING:
    case PPPS_STOPPING:
        /*
         * Ignore if we are going down.
         */
        NutNetBufFree(nb);
        return;

    case PPPS_OPENED:
        /*
         * Go down and restart negotiation.
         */
        IpcpLowerDown(dev);
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        break;

    case PPPS_STOPPED:
        /*
         * Negotiation started by our peer.
         */
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;
    }

    /*
     * Check if there is anything to reject.
     */
    xcpo = nb->nb_ap.vp;
    xcpl = nb->nb_ap.sz;
    xcpr = nb->nb_ap.vp;
    xcps = 0;
    while (xcpl >= 2) {
        len = xcpo->xcpo_len;
        if (len > xcpl)
            len = xcpl;
        else {
            switch (xcpo->xcpo_type) {
            case IPCP_MS_DNS1:
                if (xcpo->xcpo_.ul == 0 && dcb->dcb_ip_dns1 == 0) {
                    break;
                }
            case IPCP_MS_DNS2:
                if (xcpo->xcpo_.ul == 0 && dcb->dcb_ip_dns2 == 0) {
                    break;
                }
            case IPCP_ADDR:
                if (xcpo->xcpo_len == 6)
                    len = 0;
                break;
            }
        }

        if (len) {
            if (xcpr != xcpo) {
                xcpr->xcpo_type = xcpo->xcpo_type;
                xcpr->xcpo_len = len;
                for (i = 0; i < len - 2; i++)
                    xcpr->xcpo_.uc[i] = xcpo->xcpo_.uc[i];
            }
            xcpr = (XCPOPT *) ((char *) xcpr + len);
            xcps += len;
        }
        xcpl -= xcpo->xcpo_len;
        xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
    }

    if (xcps) {
        nb->nb_ap.sz = xcps;
        rc = XCP_CONFREJ;
    }

    /*
     * Check if there is anything to negotiate.
     */
    else {
        xcpo = nb->nb_ap.vp;
        xcpl = nb->nb_ap.sz;
        xcpr = nb->nb_ap.vp;
        xcps = 0;
        len = 0;
        while (xcpl >= 2) {
            ip = xcpo->xcpo_.ul;
            switch (xcpo->xcpo_type) {
            case IPCP_ADDR:
                len = IpcpValidateIpReq(&dcb->dcb_remote_ip, &ip);
                break;
            case IPCP_MS_DNS1:
                len = IpcpValidateIpReq(&dcb->dcb_ip_dns1, &ip);
                break;
            case IPCP_MS_DNS2:
                len = IpcpValidateIpReq(&dcb->dcb_ip_dns2, &ip);
                break;
            }

            if (len) {
                if (xcpr != xcpo) {
                    xcpr->xcpo_type = xcpo->xcpo_type;
                    xcpr->xcpo_len = len;
                }
                xcpr->xcpo_.ul = ip;
                xcpr = (XCPOPT *) ((char *) xcpr + len);
                xcps += len;
                len = 0;
            }
            xcpl -= xcpo->xcpo_len;
            xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
        }
        if (xcps) {
            nb->nb_ap.sz = xcps;
            rc = XCP_CONFNAK;
        }
    }

    NutIpcpOutput(dev, rc, id, nb);

    if (rc == XCP_CONFACK) {
        if (dcb->dcb_ipcp_state == PPPS_ACKRCVD) {
            dcb->dcb_ipcp_state = PPPS_OPENED;
            NutEventPost(&dcb->dcb_state_chg);
        } else
            dcb->dcb_ipcp_state = PPPS_ACKSENT;
        dcb->dcb_ipcp_naks = 0;
    } else if (dcb->dcb_ipcp_state != PPPS_ACKRCVD)
        dcb->dcb_ipcp_state = PPPS_REQSENT;
}

/*
 * Configure-Ack received.
 */
static void IpcpRxConfAck(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;
    XCPOPT *xcpo;
    uint16_t xcpl;

    /*
     * Ignore, if we are not expecting this id.
     */
    if (id != dcb->dcb_reqid || dcb->dcb_acked) {
        return;
    }

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSED:
    case PPPS_STOPPED:
        /*
         * Go away, we're closed.
         */
        NutNetBufFree(nb);
        NutIpcpOutput(dev, XCP_TERMACK, id, 0);
        return;

    case PPPS_REQSENT:
        dcb->dcb_ipcp_state = PPPS_ACKRCVD;
        dcb->dcb_retries = 0;
        break;

    case PPPS_ACKRCVD:
        /* Huh? an extra valid Ack? oh well... */
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;

    case PPPS_ACKSENT:
        dcb->dcb_ipcp_state = PPPS_OPENED;
        dcb->dcb_retries = 0;
        NutEventPost(&dcb->dcb_state_chg);
        break;

    case PPPS_OPENED:
        /* Go down and restart negotiation */
        IpcpLowerDown(dev);
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;
    }

    xcpo = nb->nb_ap.vp;
    xcpl = nb->nb_ap.sz;
    while (xcpl >= 2) {
        switch (xcpo->xcpo_type) {
        case IPCP_ADDR:
            if (xcpo->xcpo_.ul)
                dcb->dcb_local_ip = xcpo->xcpo_.ul;
            break;
        case IPCP_COMPRESSTYPE:
            break;
        case IPCP_MS_DNS1:
            if (xcpo->xcpo_.ul)
                dcb->dcb_ip_dns1 = xcpo->xcpo_.ul;
            break;
        case IPCP_MS_DNS2:
            /* Fixed secondary DNS bug, thanks to Tarmo Fimberg
               and Jelle Martijn Kok. */
            if (xcpo->xcpo_.ul)
                dcb->dcb_ip_dns2 = xcpo->xcpo_.ul;
            break;
        }
        xcpl -= xcpo->xcpo_len;
        xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
    }

    dcb->dcb_acked = 1;
    NutNetBufFree(nb);
}

/*
 * Configure-Nak or Configure-Reject received.
 */
static void IpcpRxConfNakRej(NUTDEVICE * dev, uint8_t id, NETBUF * nb, uint8_t rejected)
{
    PPPDCB *dcb = dev->dev_dcb;
    XCPOPT *xcpo;
    uint16_t xcpl;

    /*
     * Ignore, if we are not expecting this id.
     */
    if (id != dcb->dcb_reqid || dcb->dcb_acked) {
        NutNetBufFree(nb);
        return;
    }

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSED:
    case PPPS_STOPPED:
        /*
         * Go away, we're closed.
         */
        NutNetBufFree(nb);
        NutIpcpOutput(dev, XCP_TERMACK, id, 0);
        return;

    case PPPS_REQSENT:
    case PPPS_ACKSENT:
    case PPPS_ACKRCVD:
    case PPPS_OPENED:
        break;

    default:
        NutNetBufFree(nb);
        return;
    }

    dcb->dcb_acked = 1;

    xcpo = nb->nb_ap.vp;
    xcpl = nb->nb_ap.sz;
    while (xcpl >= 2) {
        switch (xcpo->xcpo_type) {
        case IPCP_ADDR:
            if (xcpo->xcpo_.ul)
                dcb->dcb_local_ip = xcpo->xcpo_.ul;
            break;
        case IPCP_COMPRESSTYPE:
            break;
        case IPCP_MS_DNS1:
            if (rejected)
                dcb->dcb_rejects |= REJ_IPCP_DNS1;
            else if (xcpo->xcpo_.ul)
                dcb->dcb_ip_dns1 = xcpo->xcpo_.ul;
            break;
        case IPCP_MS_DNS2:
            if (rejected)
                dcb->dcb_rejects |= REJ_IPCP_DNS2;
            else if (xcpo->xcpo_.ul)
                dcb->dcb_ip_dns2 = xcpo->xcpo_.ul;
            break;
        }
        xcpl -= xcpo->xcpo_len;
        xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
    }
    NutNetBufFree(nb);

    switch (dcb->dcb_ipcp_state) {

    case PPPS_REQSENT:
    case PPPS_ACKSENT:
        /* They didn't agree to what we wanted - try another request */
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        break;

    case PPPS_ACKRCVD:
        /* Got a Nak/reject when we had already had an Ack?? oh well... */
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;

    case PPPS_OPENED:
        /* Go down and restart negotiation */
        IpcpLowerDown(dev);
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;
    }
}

/*
 * \brief Terminate request received.
 */
static void IpcpRxTermReq(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    NutNetBufFree(nb);
    switch (dcb->dcb_ipcp_state) {
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        dcb->dcb_ipcp_state = PPPS_STOPPING;
        break;
    }
    NutIpcpOutput(dev, XCP_TERMACK, id, 0);
}

/*
 * Terminate-Ack received.
 */
static void IpcpRxTermAck(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    (void)id;
    (void)nb;

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSING:
        dcb->dcb_ipcp_state = PPPS_CLOSED;
        break;
    case PPPS_STOPPING:
        dcb->dcb_ipcp_state = PPPS_STOPPED;
        break;

    case PPPS_ACKRCVD:
        dcb->dcb_ipcp_state = PPPS_REQSENT;
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        IpcpTxConfReq(dev, ++dcb->dcb_reqid);
        break;
    }
}

/*
 * Peer doesn't speak this protocol.
 *
 * Treat this as a catastrophic error (RXJ-).
 */
void IpcpRxProtRej(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

    switch (dcb->dcb_ipcp_state) {
    case PPPS_CLOSING:
    case PPPS_CLOSED:
        dcb->dcb_ipcp_state = PPPS_CLOSED;
        break;

    case PPPS_STOPPING:
    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
    case PPPS_STOPPED:
        dcb->dcb_ipcp_state = PPPS_STOPPED;
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
        dcb->dcb_ipcp_state = PPPS_STOPPING;
        break;
    }
}

/*
 * Receive an Code-Reject.
 */
static void IpcpRxCodeRej(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    (void)id;

    NutNetBufFree(nb);
    if (dcb->dcb_ipcp_state == PPPS_ACKRCVD)
        dcb->dcb_ipcp_state = PPPS_REQSENT;
}



/*!
 * \brief Handle incoming IPCP packets.
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
void NutIpcpInput(NUTDEVICE * dev, NETBUF * nb)
{
    XCPHDR *xch;
    PPPDCB *dcb = dev->dev_dcb;
    uint16_t len;

    /*
     * Discard packets with illegal lengths.
     */
    if (nb->nb_nw.sz < sizeof(XCPHDR)) {
        NutNetBufFree(nb);
        return;
    }
    xch = (XCPHDR *) nb->nb_nw.vp;
    if ((len = htons(xch->xch_len)) < sizeof(XCPHDR) || len > nb->nb_nw.sz) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * Discard all packets while we are in initial or starting state.
     */
    if (dcb->dcb_ipcp_state == PPPS_INITIAL || dcb->dcb_ipcp_state == PPPS_STARTING) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * Split the IPCP packet.
     */
    nb->nb_ap.vp = xch + 1;
    nb->nb_ap.sz = htons(xch->xch_len) - sizeof(XCPHDR);

    /*
     * Action depends on code.
     */
    switch (xch->xch_code) {
    case XCP_CONFREQ:
        IpcpRxConfReq(dev, xch->xch_id, nb);
        break;

    case XCP_CONFACK:
        IpcpRxConfAck(dev, xch->xch_id, nb);
        break;

    case XCP_CONFNAK:
        IpcpRxConfNakRej(dev, xch->xch_id, nb, 0);
        break;

    case XCP_CONFREJ:
        IpcpRxConfNakRej(dev, xch->xch_id, nb, 1);
        break;

    case XCP_TERMREQ:
        IpcpRxTermReq(dev, xch->xch_id, nb);
        break;

    case XCP_TERMACK:
        IpcpRxTermAck(dev, xch->xch_id, nb);
        break;

    case XCP_CODEREJ:
        IpcpRxCodeRej(dev, xch->xch_id, nb);
        break;

    default:
        /*
         * TODO: Send code reject.
         */
        NutNetBufFree(nb);
        break;
    }
}

/*@}*/
