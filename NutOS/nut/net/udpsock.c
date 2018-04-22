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
 * \file net/udpsock.c
 * \brief UDP socket interface.
 *
 * \verbatim
 * $Id: udpsock.c 5509 2014-01-02 20:46:54Z mifi $
 * \endverbatim
 */

#include <sys/heap.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <memdebug.h>

/*!
 * \addtogroup xgUdpSocket
 */
/*@{*/


UDPSOCKET *udpSocketList;       /*!< Global linked list of all UDP sockets. */
static uint16_t last_local_port;  /* Unassigned local port. */
static uint_fast8_t registered;

/*!
 * \brief Create a UDP socket.
 *
 * \param  port Server applications provide the local port number
 *              with this parameter. Client applications should
 *              pass zero.
 *
 * \return Socket descriptor of the newly created UDP socket or
 *         0 if there is not enough memory left.
 *
 */
UDPSOCKET *NutUdpCreateSocket(uint16_t port)
{
    UDPSOCKET *sock;
    uint16_t   ticks;

    if (!registered) {
        if (NutRegisterIpHandler(IPPROTO_UDP, NutUdpInput)) {
            return NULL;
        }
        registered = 1;
    }
    if (port == 0) {
        do {
            /* Each time a new socket is created the local port number in incremented
               by a more or less randomized value between 1 and 15 (the lowest 4 bit of
               NutGetMillis() | 1). The highest two bits are always set to 1.
               This way a port range of 49152 to 65535 is used according to the IANA
               suggestions for ephemeral port usage.
             */
            ticks = (uint16_t) NutGetMillis();
            if (last_local_port) {
                last_local_port += (uint16_t) ((ticks & 0x000F) | 1);
            } else {
                last_local_port = ticks;
            }

            last_local_port |= 0xC000;

            port = htons(last_local_port);
            sock = udpSocketList;
            while (sock) {
                if (sock->so_local_port == port)
                    break;
                sock = sock->so_next;
            }
        } while (sock);
        port = last_local_port;
    }
    if ((sock = calloc(1, sizeof(UDPSOCKET))) != 0) {
        sock->so_local_port = htons(port);
        sock->so_next = udpSocketList;
        udpSocketList = sock;
    }
    return sock;
}

/*!
 * \brief Send a UDP datagram.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutUdpCreateSocket().
 * \param addr IP address of the remote host in network byte order.
 * \param port Remote port number in host byte order.
 * \param data Pointer to a buffer containing the data to send.
 * \param len  Number of bytes to be sent.
 *
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutUdpError().
 */

int NutUdpSendTo(UDPSOCKET * sock, uint32_t addr, uint16_t port, void *data, int len)
{
    int rc;
    NETBUF *nb;

#ifndef NUT_UDP_ICMP_EXCLUDE
    if (sock->so_last_error)
        return -1;
#endif

    if ((nb = NutNetBufAlloc(0, NBAF_APPLICATION, len)) == 0) {
        sock->so_last_error = ENOMEM;
        return -1;
    }
    memcpy(nb->nb_ap.vp, data, len);

    /* Bugfix by Ralph Mason. We should not free the NETBUF in case of an error. */
    if ((rc = NutUdpOutput(sock, addr, port, nb)) == 0)
        NutNetBufFree(nb);

    return rc;
}

/*!
 * \brief Receive a UDP datagram.
 *
 * \param sock    Socket descriptor. This pointer must have been
 *                retrieved by calling NutUdpCreateSocket().
 * \param addr    IP address of the remote host in network byte order.
 * \param port    Remote port number in host byte order.
 * \param data    Pointer to the buffer that receives the data.
 * \param size    Size of the buffer that receives the data.
 * \param timeout Maximum number of milliseconds to wait.
 *
 * \return The number of bytes received, if successful. The return
 *         value < 0 indicates an error. The specific error code
 *         can be retrieved by calling NutUdpError().
 *
 * \note Timeout is limited to the granularity of the system timer.
 */
 /* @@@ 2003-10-24: modified by OS for udp packet queue */
int NutUdpReceiveFrom(UDPSOCKET * sock, uint32_t * addr, uint16_t * port, void *data, int size, uint32_t timeout)
{
    IPHDR *ip;
    UDPHDR *uh;
    NETBUF *nb;

#ifndef NUT_UDP_ICMP_EXCLUDE
    /* The ICMP handler might have set an error condition. */
    if (sock->so_last_error)
        return -1;
#endif

    if (sock->so_rx_nb == 0)
        NutEventWait(&sock->so_rx_rdy, timeout);

#ifndef NUT_UDP_ICMP_EXCLUDE
    /* An ICMP message might have posted the rx event. So check again */
    if (sock->so_last_error)
        return -1;
#endif

    if ((nb = sock->so_rx_nb) == 0)
        return 0;

    /* forward the queue's head to the next packet */
    sock->so_rx_nb = nb->nb_next;

    ip = nb->nb_nw.vp;
    *addr = ip->ip_src;

    uh = nb->nb_tp.vp;
    *port = htons(uh->uh_sport);

    if (size > nb->nb_ap.sz)
        size = nb->nb_ap.sz;

    sock->so_rx_cnt -= nb->nb_ap.sz;    /* decrement input buffer count */

    memcpy(data, nb->nb_ap.vp, size);
    NutNetBufFree(nb);

    return size;
}

/*!
 * \brief Close UDP socket.
 *
 * The memory occupied by the socket is immediately released
 * after calling this function. The application  must not use
 * the socket after this call.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutUdpCreateSocket().
 *
 * \return 0 on success, -1 otherwise.
 */
 /* @@@ 2003-10-24: modified by OS for udp packet queue */
int NutUdpDestroySocket(UDPSOCKET * sock)
{
    UDPSOCKET *sp;
    UDPSOCKET **spp;
    int rc = -1;
    NETBUF *nb;

    spp = &udpSocketList;
    sp = udpSocketList;

    while (sp) {
        if (sp == sock) {
            *spp = sock->so_next;
            /* packets may have arrived that the application
               did not read before closing the socket. */
            while ((nb = sock->so_rx_nb) != 0) {
                sock->so_rx_nb = nb->nb_next;
                NutNetBufFree(nb);
            }
            free(sock);
            rc = 0;
            break;
        }
        spp = &sp->so_next;
        sp = sp->so_next;
    }
    return rc;
}

/*!
 * \brief Return specific code of the last error and the IP address / port of
 *        the host to which the communication failed
 *
 * Possible error codes are:
 * - ENOTSOCK: Socket operation on non-socket
 * - EMSGSIZE: Message too long
 * - ENOPROTOOPT: Protocol not available
 * - EOPNOTSUPP: Operation not supported on socket
 * - ENETUNREACH: Network is unreachable
 * - ECONNREFUSED: Connection refused
 * - EHOSTDOWN: Host is down
 * - EHOSTUNREACH: No route to host
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutUdpCreateSocket().
 * \param addr IP address of the remote host in network byte order.
 * \param port Remote port number in host byte order.
 *
 * \todo Not all error codes are properly set right now. Some socket
 *       functions return an error without setting an error code.
 */

int NutUdpError(UDPSOCKET * sock, uint32_t * addr, uint16_t * port)
{
    int rc;

    if (sock == 0) {
        addr = 0;
        port = 0;
        return ENOTSOCK;
    }

    if (sock->so_last_error) {
        rc = sock->so_last_error;
        *port = ntohs(sock->so_remote_port);
        *addr = sock->so_remote_addr;

        sock->so_last_error  = 0;
        sock->so_remote_port = 0;
        sock->so_remote_addr = 0;
        return rc;
    }
    return 0;
}

/*!
 * \brief Find a matching socket.
 *
 * Loop through all sockets and find a matching one.
 *
 * \note Applications typically do not need to call this function.
 *
 * \param port  Local port number.
 *
 * \return Socket descriptor.
 */
UDPSOCKET *NutUdpFindSocket(uint16_t port)
{
    UDPSOCKET *sp;
    UDPSOCKET *sock = 0;

    for (sp = udpSocketList; sp; sp = sp->so_next) {
        if (sp->so_local_port == port) {
            sock = sp;
            break;
        }
    }
    return sock;
}

/*!
 * \brief Set value of a UDP socket option.
 *
 * The following values can be set:
 *
 * - #SO_RCVBUF   Socket input buffer size (#uint16_t).
 *
 * \param sock    Socket descriptor. This pointer must have been
 *                retrieved by calling NutUdpCreateSocket().
 * \param optname Option to set.
 * \param optval  Pointer to the value.
 * \param optlen  Length of the value.
 * \return 0 on success, -1 otherwise.
 */
int NutUdpSetSockOpt(UDPSOCKET * sock, int optname, const void *optval, int optlen)
{
    int rc = -1;

    if (sock == 0)
        return -1;
    switch (optname) {

    case SO_RCVBUF:
        if (optval != 0 && optlen == sizeof(uint16_t)) {
            sock->so_rx_bsz = *((uint16_t *) optval);
            rc = 0;
        }
        break;

    default:
        /* sock->so_last_error = ENOPROTOOPT; */
        break;
    }
    return rc;
}

/*!
 * \brief Get a UDP socket option value.
 *
 * The following values can be set:
 *
 * - #SO_RCVBUF   Socket input buffer size (#uint16_t).
 *
 * \param sock    Socket descriptor. This pointer must have been
 *                retrieved by calling NutUdpCreateSocket().
 * \param optname Option to get.
 * \param optval  Points to a buffer receiving the value.
 * \param optlen  Length of the value buffer.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutUdpGetSockOpt(UDPSOCKET * sock, int optname, void *optval, int optlen)
{
    int rc = -1;

    if (sock == 0)
        return -1;
    switch (optname) {

    case SO_RCVBUF:
        if (optval != 0 && optlen == sizeof(uint16_t)) {
            *((uint16_t *) optval) = sock->so_rx_bsz;
            rc = 0;
        }
        break;

    default:
        /* sock->so_last_error = ENOPROTOOPT; */
        break;
    }
    return rc;
}

/*!
 * \brief Set a UDP socket error.
 *
 * This function should only be used together (and from) the ICMP input routine
 *
 * The following values can be set:
 *
 * - #EHOSTUNREACH   Host is unreachable
 *
 * \param sock        Socket descriptor. This pointer must have been
 *                    retrieved by calling NutUdpCreateSocket().
 * \param remote_addr Remote IP address in network byte order
 * \param remote_port Remote port in network byte order
 * \param error       Error number.
 *
 * \return 0 on success, -1 otherwise.
 */

int NutUdpSetSocketError(UDPSOCKET * sock, uint32_t remote_addr, uint16_t remote_port, uint16_t error)
{
    if (sock == 0)
        return -1;

    sock->so_remote_addr = remote_addr;
    sock->so_remote_port = remote_port;
    sock->so_last_error  = error;

    /* post the event only, if a thread is waiting */
    if (sock->so_rx_rdy)
        NutEventPost(&sock->so_rx_rdy);

    return 0;
}


/*@}*/
