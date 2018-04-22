/*
 * Copyright (C) 2001-2005 by egnite Software GmbH
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
 * \file net/tcpsock.c
 * \brief TCP socket interface.
 *
 * \verbatim
 * $Id: tcpsock.c 6625 2017-04-13 14:05:46Z u_bonnes $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/crt.h>
#include <cfg/tcp.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <net/route.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/icmp.h>
#include <netinet/ip_icmp.h>
#include <netinet/ipcsum.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <io.h>
#include <memdebug.h>

#ifdef NUTDEBUG
#include <net/netdebug.h>
#endif

/*!
 * \addtogroup xgTcpSocket
 */
/*@{*/

TCPSOCKET *tcpSocketList = 0;   /*!< Global linked list of all TCP sockets. */

static uint16_t last_local_port; /* Unassigned local port. */

static uint_fast8_t registered;

void NutTcpDiscardBuffers(TCPSOCKET * sock)
{
    NETBUF *nb;
    while ((nb = sock->so_rx_buf) != 0) {
        sock->so_rx_buf = nb->nb_next;
        NutNetBufFree(nb);
    }
    while ((nb = sock->so_tx_nbq) != 0) {
        sock->so_tx_nbq = nb->nb_next;
        NutNetBufFree(nb);
    }
    while ((nb = sock->so_rx_nbq) != 0) {
        sock->so_rx_nbq = nb->nb_next;
        NutNetBufFree(nb);
    }
}

/*!
 * \brief Destroy a previously allocated socket.
 *
 * Release occupied socket memory.
 *
 * Applications must not call this function. It is automatically called
 * by a timer after the socket has been closed by NutTcpCloseSocket().
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 */
void NutTcpDestroySocket(TCPSOCKET * sock)
{
    //@@@printf ("[%04X] Calling destroy.\n", (u_short) sock);

    /*
     * Free all memory occupied by the socket.
     */
    NutTcpDiscardBuffers(sock);
    if (sock->so_devocnt)
    {
        free(sock->so_devobuf);
        sock->so_devocnt = 0;
    }
    memset(sock, 0, sizeof(TCPSOCKET));
    free(sock);
}

/*!
 * \brief Find a matching socket.
 *
 * Loop through all sockets and find a matching connection (preferred)
 * or a listening socket.
 *
 * Applications typically do not call this function.
 *
 * \param lport Local port number.
 * \param rport Remote port number.
 * \param raddr Remote IP address in network byte order.
 *
 * \return Socket descriptor.
 */
TCPSOCKET *NutTcpFindSocket(uint16_t lport, uint16_t rport, uint32_t raddr)
{
    TCPSOCKET *sp;
    TCPSOCKET *sock = 0;

    /*
     * Try to find an exact match for the remote
     * address and port first.
     */
    for (sp = tcpSocketList; sp; sp = sp->so_next) {
        if (sp->so_local_port == lport) {
            if (sp->so_remote_addr == raddr && sp->so_remote_port == rport && sp->so_state != TCPS_CLOSED && sp->so_state != TCPS_DESTROY) {
                sock = sp;
                break;
            }
        }
    }

    /*
     * If no exact match exists, try a listening socket.
     * This part had been totally wrong, because it
     * didn't check the local port number and accepted
     * incoming requests on any port. Thanks to
     * Alejandro Lopez, who pointed this out.
     */
    if (sock == 0) {
        for (sp = tcpSocketList; sp; sp = sp->so_next) {
            if (sp->so_state == TCPS_LISTEN && sp->so_local_port == lport) {
                sock = sp;
                break;
            }
        }
    }

    return sock;
}


/*!
 * \brief Create a TCP socket.
 *
 * Allocates a TCPSOCKET structure from heap memory, initializes
 * it and returns a pointer to that structure.
 *
 * The very first call will also start the TCP state machine,
 * which is running in a separate thread.
 *
 * \return Socket descriptor of the newly created TCP socket or
 *         0 if there is not enough memory left.
 */
TCPSOCKET *NutTcpCreateSocket(void)
{
    TCPSOCKET *sock = 0;

    if (!registered) {
        if (NutRegisterIpHandler(IPPROTO_TCP, NutTcpInput) || NutTcpInitStateMachine()) {
            return NULL;
        }
        registered = 1;
    }
    if ((sock = calloc(1, sizeof(TCPSOCKET))) != 0) {
        sock->so_state = TCPS_CLOSED;

        /*
         * Initialize the virtual device interface.
         */
        sock->so_devtype = IFTYP_TCPSOCK;
        sock->so_devread = NutTcpDeviceRead;
        sock->so_devwrite = NutTcpDeviceWrite;
#ifdef __HARVARD_ARCH__
        sock->so_devwrite_P = NutTcpDeviceWrite_P;
#endif
        sock->so_devioctl = NutTcpDeviceIOCtl;
        sock->so_devselect = NutTcpDeviceSelect;

        sock->so_tx_isn = NutGetTickCount();  /* Generate the ISN from the nut_ticks counter */
        sock->so_tx_una = sock->so_tx_isn;
        sock->so_tx_nxt = sock->so_tx_isn;
        sock->so_rx_bsz = sock->so_rx_win = TCP_WINSIZE;

        sock->so_mss = TCP_MSS;
        sock->so_rtto = 1000; /* Initial retransmission time out */
#if !defined(RFC793)
        sock->so_rtsv = 1000;
#endif

        sock->so_next = tcpSocketList;

        sock->so_devobsz = TCP_MSS; /* Default output buffer size is TCP_MSS bytes */

        tcpSocketList = sock;
    }
    return sock;
}

/*!
 * \brief Set value of a TCP socket option.
 *
 * The following values can be set:
 *
 * - #TCP_MAXSEG  Maximum segment size (#uint16_t). Can only be set if
                  socket is not yet connected.
 * - #SO_SNDTIMEO Socket send timeout (#uint32_t).
 * - #SO_RCVTIMEO Socket receive timeout (#uint32_t).
 * - #SO_SNDBUF   Socket output buffer size (#uint16_t).
 *
 * \param sock    Socket descriptor. This pointer must have been
 *                retrieved by calling NutTcpCreateSocket().
 * \param optname Option to set.
 * \param optval  Pointer to the value.
 * \param optlen  Length of the value.
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutTcpError().
 */
int NutTcpSetSockOpt(TCPSOCKET * sock, int optname, const void *optval, int optlen)
{
    int rc = -1;

    if (sock == 0)
        return -1;
    switch (optname) {

    case TCP_MAXSEG:
        if (optval == 0 || optlen != sizeof(uint16_t))
            sock->so_last_error = EINVAL;
        else if (sock->so_state != TCPS_CLOSED)
            sock->so_last_error = EISCONN;
        else {
            sock->so_mss = *((uint16_t *) optval);
            rc = 0;
        }
        break;

    case SO_RCVBUF:
        if (optval == 0 || optlen != sizeof(uint16_t))
            sock->so_last_error = EINVAL;
        else {
            sock->so_rx_bsz = *((uint16_t *) optval);
            sock->so_rx_win = sock->so_rx_bsz;
            rc = 0;
        }
        break;

    case SO_SNDTIMEO:
        if (optval == 0 || optlen != sizeof(uint32_t))
            sock->so_last_error = EINVAL;
        else {
            sock->so_write_to = *((uint32_t *) optval);
            rc = 0;
        }
        break;

    case SO_RCVTIMEO:
        if (optval == 0 || optlen != sizeof(uint32_t))
            sock->so_last_error = EINVAL;
        else {
            sock->so_read_to = *((uint32_t *) optval);
            rc = 0;
        }
        break;

    case SO_SNDBUF:
        if (optval == 0 || optlen != sizeof(uint16_t))
            sock->so_last_error = EINVAL;
        else {
            NutTcpDeviceWrite(sock, 0, 0);
            sock->so_devobsz = *((uint16_t *) optval);
            rc = 0;
        }
        break;

    default:
        sock->so_last_error = ENOPROTOOPT;
        break;
    }
    return rc;
}

/*!
 * \brief Get a TCP socket option value.
 *
 * The following values can be set:
 *
 * - #TCP_MAXSEG  Maximum segment size (#uint16_t).
 * - #SO_SNDTIMEO Socket send timeout (#uint32_t).
 * - #SO_RCVTIMEO Socket receive timeout (#uint32_t).
 * - #SO_SNDBUF   Socket output buffer size (#uint16_t).
 *
 * \param sock    Socket descriptor. This pointer must have been
 *                retrieved by calling NutTcpCreateSocket().
 * \param optname Option to get.
 * \param optval  Points to a buffer receiving the value.
 * \param optlen  Length of the value buffer.
 *
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutTcpError().
 */
int NutTcpGetSockOpt(TCPSOCKET * sock, int optname, void *optval, int optlen)
{
    int rc = -1;

    if (sock == 0)
        return -1;
    switch (optname) {

    case TCP_MAXSEG:
        if (optval == 0 || optlen != sizeof(uint16_t))
            sock->so_last_error = EINVAL;
        else {
            *((uint16_t *) optval) = sock->so_mss;
            rc = 0;
        }
        break;

    case SO_RCVBUF:
        if (optval == 0 || optlen != sizeof(uint16_t))
            sock->so_last_error = EINVAL;
        else {
            *((uint16_t *) optval) = sock->so_rx_bsz;
            rc = 0;
        }
        break;

    case SO_SNDTIMEO:
        if (optval == 0 || optlen != sizeof(uint32_t))
            sock->so_last_error = EINVAL;
        else {
            *((uint32_t *) optval) = sock->so_write_to;
            rc = 0;
        }
        break;

    case SO_RCVTIMEO:
        if (optval == 0 || optlen != sizeof(uint32_t))
            sock->so_last_error = EINVAL;
        else {
            *((uint32_t *) optval) = sock->so_read_to;
            rc = 0;
        }
        break;

    case SO_SNDBUF:
        if (optval == 0 || optlen != sizeof(uint16_t))
            sock->so_last_error = EINVAL;
        else {
            *((uint16_t *) optval) = sock->so_devobsz;
            rc = 0;
        }
        break;
    default:
        sock->so_last_error = ENOPROTOOPT;
        break;
    }
    return rc;
}

/*!
 * \brief Connect to a remote socket.
 *
 * This function tries to establish a connection to the specified
 * remote port of the specified remote server. The calling thread
 * will be suspended until a connection is successfully established
 * or an error occurs.
 *
 * If a write timeout is set for this socket, the function will return
 * an error, if the connection could not be established before the
 * timeout. In this case the last error is set to ETIMEOUT.
 *
 * This function is typically used by TCP client applications.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 * \param addr IP address of the host to connect (network byte order).
 * \param port Port number to connect (host byte order).
 *
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutTcpError().
 */
int NutTcpConnect(TCPSOCKET * sock, uint32_t addr, uint16_t port)
{
    TCPSOCKET *sp;
    NUTDEVICE *dev;
    uint16_t   ticks;

    if (sock == 0)
        return -1;
    /*
     * Despite RFC793 we do not allow a passive
     * open to become active.
     */
    if (sock->so_state == TCPS_LISTEN) {
        sock->so_last_error = EOPNOTSUPP;
        return -1;
    } else if (sock->so_state != TCPS_CLOSED) {
        sock->so_last_error = EISCONN;
        return -1;
    }

    /*
     * Find an unused local port.
     */
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

        sp = tcpSocketList;
        while (sp) {
            /* Thanks to Ralph Mason for fixing the byte order bug. */
            if (sp->so_local_port == htons(last_local_port))
                break;
            sp = sp->so_next;
        }
    } while (sp);

    /*
     * OK - we've got a new port. Now fill
     * remaining parts of the socket structure.
     */
    sock->so_local_port = htons(last_local_port);
    sock->so_remote_port = htons(port);
    sock->so_remote_addr = addr;

    /*
     * Get local address for this destination.
     */
    if ((dev = NutIpRouteQuery(addr, 0)) != 0) {
        IFNET *nif = dev->dev_icb;
        sock->so_local_addr = nif->if_local_ip;
    } else {
        sock->so_last_error = EHOSTUNREACH;
        return -1;
    }

    /*
     * Trigger active open event for the state machine.
     * This will suspend the thread until synchronized.
     */
    return NutTcpStateActiveOpenEvent(sock);
}

/*!
 * \brief Wait for incoming connect from a remote socket.
 *
 * The calling thread will be suspended until until an
 * incoming connection request is received.
 *
 * This function is typically used by TCP server applications.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 * \param port Port number to listen to (host byte order).
 *
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutTcpError().
 */
int NutTcpAccept(TCPSOCKET * sock, uint16_t port)
{
    sock->so_local_port = htons(port);

    return NutTcpStatePassiveOpenEvent(sock);
}

/*!
 * \brief Send data on a connected TCP socket.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket(). In
 *             addition a connection must have been established
 *             by calling NutTcpConnect or NutTcpAccept.
 * \param data Pointer to a buffer containing the data to send.
 * \param len  Number of bytes to be sent.
 *
 * \return If successful, the number of bytes added to the socket transmit
 *         buffer. This is limited to the maximum segment size of the
 *         connection and thus may be less than the specified number of
 *         bytes to send. The return value -1 indicates a fatal error.
 *         On time out, a value of 0 is returned.
 */
int NutTcpSend(TCPSOCKET * sock, const void *data, int len)
{
    uint16_t unacked;

    /*
     * Check parameters.
     */
#ifdef TCP_REENABLE_THREADYIELDS
    NutThreadYield();
#endif

    if (sock == 0)
        return -1;
    if (data == 0 || len == 0)
        return 0;

    /*
     * Limit the transmission size to our maximum segment size.
     */
    if (len > sock->so_mss)
        len = sock->so_mss;

    for (;;) {
        /*
         * We can only send on an established connection.
         */
        if (sock->so_state != TCPS_ESTABLISHED) {
            sock->so_last_error = ENOTCONN;
            return -1;
        }

        /*
         * Limit the size of unacknowledged data to four full segments.
         * Also wait for peer's window open wide enough to take all our
         * data. This also avoids silly window syndrome on our side.
         */
        unacked = sock->so_tx_nxt - sock->so_tx_una;
        if ((unacked >> 2) < sock->so_mss && len <= sock->so_tx_win - unacked) {
            break;
        }
        if (NutEventWait(&sock->so_tx_tq, sock->so_write_to)) {
            return 0;
        }
    }
    /*
     * The segment will be automatically retransmitted if not
     * acknowledged in time. If this returns an error, it's a
     * fatal one.
     */
    sock->so_tx_flags |= SO_ACK;
    if (NutTcpOutput(sock, data, len))
        return -1;
    return len;
}

/*!
 * \brief Receive data on a connected TCP socket.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket(). In
 *             addition a connection must have been established
 *             by calling NutTcpConnect or NutTcpAccept.
 * \param data Pointer to the buffer that receives the data.
 * \param size Size of the buffer.
 *
 * \return If successful, the number of received data bytes
 *         is returned. This may be less than the specified
 *         size of the buffer. The return value 0 indicates
 *         a timeout, while -1 is returned in case of an error
 *         or broken connection. Call NutTcpError() to determine
 *         the specific error code.
 */
int NutTcpReceive(TCPSOCKET * sock, void *data, int size)
{
    int i;

#ifdef TCP_REENABLE_THREADYIELDS
    NutThreadYield();
#endif
    /*
     * Check parameters.
     */
    if (sock == 0)
        return -1;
    if (sock->so_state != TCPS_ESTABLISHED && sock->so_state != TCPS_CLOSE_WAIT) {
        sock->so_last_error = ENOTCONN;
        return -1;
    }
    if (data == 0 || size == 0)
        return 0;

    /*
     * Wait until any data arrived, a timeout occurs
     * or the connection terminates.
     */
    while (sock->so_rx_cnt - sock->so_rd_cnt == 0) {
        if (sock->so_state != TCPS_ESTABLISHED) {
            sock->so_last_error = ENOTCONN;
            return -1;
        }
        if (NutEventWait(&sock->so_rx_tq, sock->so_read_to))
            return 0;
    }

    if (size > sock->so_rx_cnt - sock->so_rd_cnt)
        size = sock->so_rx_cnt - sock->so_rd_cnt;
    if (size) {
        NETBUF *nb;
        int rd_cnt;         /* Bytes read from NETBUF. */
        int nb_cnt;         /* Bytes left in NETBUF. */
        int ab_cnt;         /* Total bytes in app buffer. */
        int mv_cnt;         /* Bytes to move to app buffer. */

        rd_cnt = sock->so_rd_cnt;

        ab_cnt = 0;
        while (ab_cnt < size) {
            nb = sock->so_rx_buf;
            nb_cnt = nb->nb_ap.sz - rd_cnt;
            mv_cnt = size - ab_cnt;
            if (mv_cnt > nb_cnt)
                mv_cnt = nb_cnt;
            memcpy((char *) data + ab_cnt, (char *) (nb->nb_ap.vp) + rd_cnt, mv_cnt);
            ab_cnt += mv_cnt;
            rd_cnt += mv_cnt;
            if (mv_cnt >= nb_cnt) {
                sock->so_rx_buf = nb->nb_next;
                sock->so_rx_cnt -= rd_cnt;
                NutNetBufFree(nb);
                sock->so_rx_apc--;
                nb = sock->so_rx_buf;
                rd_cnt = 0;
            }
        }
        sock->so_rd_cnt = rd_cnt;

        /*
         * Update our receive window.
         */
        if (sock->so_state == TCPS_ESTABLISHED) {
            i = sock->so_rx_win;
            if ((i += size) > sock->so_rx_bsz)
                i = sock->so_rx_bsz;

            if (sock->so_rx_win <= sock->so_mss && i > sock->so_mss) {
                /* Force immediate window update. */
                sock->so_rx_win = i;
                NutTcpStateWindowEvent(sock);
            } else {
                /* Lazy window update. */
                sock->so_rx_win = i;
                sock->so_tx_flags |= SO_ACK;
            }
        }
    }
    return size;
}

/*!
 * \brief Close TCP socket.
 *
 * Note, that the socket may not be immediately destroyed
 * after calling this function. However, the application
 * must not use the socket after this call.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 *
 * \return 0 on success, -1 otherwise.
 */
int NutTcpCloseSocket(TCPSOCKET * sock)
{
    /* Flush buffer first */
    //@@@printf ("[%04X] Calling close\n", (u_short) sock);
    NutTcpDeviceWrite(sock, 0, 0);
    return NutTcpStateCloseEvent(sock);
}

/*!
 * \brief Return specific code of the last error.
 *
 * Possible error codes (net/errno.h) are:
 *
 * - EWOULDBLOCK: Operation would block
 * - EINPROGRESS: Operation now in progress
 * - EALREADY: Operation already in progress
 * - ENOTSOCK: Socket operation on non-socket
 * - EDESTADDRREQ: Destination address required
 * - EMSGSIZE: Message too long
 * - EPROTOTYPE: Protocol wrong type for socket
 * - ENOPROTOOPT: Protocol not available
 * - EPROTONOSUPPORT: Protocol not supported
 * - ESOCKTNOSUPPORT: Socket type not supported
 * - EOPNOTSUPP: Operation not supported on socket
 * - EPFNOSUPPORT: Protocol family not supported
 * - EAFNOSUPPORT: Address family not supported by protocol family
 * - EADDRINUSE: Address already in use
 * - EADDRNOTAVAIL: Can't assign requested address
 * - ENETDOWN: Network is down
 * - ENETUNREACH: Network is unreachable
 * - ENETRESET: Network dropped connection on reset
 * - ECONNABORTED: Software caused connection abort
 * - ECONNRESET: Connection reset by peer
 * - ENOBUFS: No buffer space available
 * - EISCONN: Socket is already connected
 * - ENOTCONN: Socket is not connected
 * - ESHUTDOWN: Can't send after socket shutdown
 * - ETOOMANYREFS: Too many references: can't splice
 * - ETIMEDOUT: Connection timed out
 * - ECONNREFUSED: Connection refused
 * - ELOOP: Too many levels of symbolic links
 * - ENAMETOOLONG: File name too long
 * - EHOSTDOWN: Host is down
 * - EHOSTUNREACH: No route to host
 * - ENOTEMPTY: Directory not empty
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 *
 * \note Applications must not call this function to retrieve the
 *       error code if NutTcpCloseSocket() or NutTcpDestroySocket()
 *       failed.
 *
 * \todo Not all error codes are properly set right now. Some socket
 *       functions return an error without setting an error code.
 */
int NutTcpError(TCPSOCKET * sock)
{
    if (sock == 0)
        return ENOTSOCK;
    return sock->so_last_error;
}

/*!
 * \brief Read from virtual socket device.
 *
 * TCP sockets can be used like other Nut/OS devices. This routine
 * is part of the virtual socket device driver.
 *
 * This function is called by the low level input routines of the
 * \ref xrCrtLowio "C runtime library", using the _NUTDEVICE::dev_read
 * entry.
 *
 * \param sock   Socket descriptor. This pointer must have been
 *               retrieved by calling NutTcpCreateSocket().
 * \param buffer Pointer to the buffer that receives the data.
 * \param size   Maximum number of bytes to read.
 *
 * \return The number of bytes read, which may be less than the number
 *         of bytes specified. A return value of -1 indicates an error,
 *         while zero is returned in case of a timeout.
 */
int NutTcpDeviceRead(TCPSOCKET * sock, void *buffer, int size)
{
    return NutTcpReceive(sock, buffer, size);
}

static int SendBuffer(TCPSOCKET * sock, const void *buffer, int size)
{
    int rc;
    int bite;

    for (rc = 0; rc < size; rc += bite) {
        if ((bite = NutTcpSend(sock, (uint8_t *) buffer + rc, size - rc)) <= 0) {
            return -1;
        }
    }
    return rc;
}

/*!
 * \brief Write to a socket.
 *
 * TCP sockets can be used like other Nut/OS devices. This routine
 * is part of the virtual socket device driver.
 *
 * This function is called by the low level output routines of the
 * \ref xrCrtLowio "C runtime library", using the
 * \ref _NUTDEVICE::dev_write entry.
 *
 * In contrast to NutTcpSend() this routine provides some buffering.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 * \param buf  Pointer to the data to be written.
 * \param size Number of bytes to write. If zero, then the output buffer
 *             will be flushed.
 *
 * \return The number of bytes written. A return value of -1 indicates
 *         an error.
 *
 */
int NutTcpDeviceWrite(TCPSOCKET * sock, const void *buf, int size)
{
    int rc;
    int sz;
    /* hack alert for ICCAVR */
    uint8_t *buffer = (uint8_t*) buf;

    /*
     * Check parameters.
     */
    if (sock == 0)
        return -1;
    if (sock->so_state != TCPS_ESTABLISHED) {
        sock->so_last_error = ENOTCONN;
        return -1;
    }

    /* Flush buffer? */
    if (size == 0) {
        if (sock->so_devocnt) {
            if (SendBuffer(sock, sock->so_devobuf, sock->so_devocnt) < 0) {
                free(sock->so_devobuf);
                sock->so_devocnt = 0;
                return -1;
            }
            free(sock->so_devobuf);
            sock->so_devocnt = 0;
        }
        return 0;
    }

    /* If we don't have a buffer so far... */
    if (sock->so_devocnt == 0) {
        /* If new data block is bigger or equal than buffer size
         * send first part of data to NIC and store remaining
         * bytes in buffer
         */
        if (size >= sock->so_devobsz) {
            rc = size % sock->so_devobsz;
            if (SendBuffer(sock, buffer, size - rc) < 0)
                return -1;
            buffer += size - rc;
        } else
            rc = size;

        /* If there are some remainings bytes, allocate buffer
         * and store them
         */
        if (rc) {
            if (!(sock->so_devobuf = malloc(sock->so_devobsz)))
                return -1;
            memcpy(sock->so_devobuf, buffer, rc);
            sock->so_devocnt = rc;
        }
        return size;
    }

    /* Check if new data fully fits in output buffer */
    if (sock->so_devocnt + size < sock->so_devobsz) {
        memcpy(sock->so_devobuf + sock->so_devocnt, buffer, size);
        sock->so_devocnt += size;
        return size;
    }

    /* Otherwise store first bytes of new data in buffer and flush
     * the buffer
     */
    sz = sock->so_devobsz - sock->so_devocnt;
    memcpy(sock->so_devobuf + sock->so_devocnt, buffer, sz);
    buffer += sz;
    if (SendBuffer(sock, sock->so_devobuf, sock->so_devobsz) < 0) {
        free(sock->so_devobuf);
        sock->so_devocnt = 0;
        return -1;
    }

    /* If remaining data is bigger or equal than buffer size
     * send first part of data to NIC and later store remaining
     * bytes in buffer
     */
    sz = size - sz;
    if (sz >= sock->so_devobsz) {
        rc = sz % sock->so_devobsz;
        if (SendBuffer(sock, buffer, sz - rc) < 0) {
            free(sock->so_devobuf);
            sock->so_devocnt = 0;
            return -1;
        }
        buffer += sz - rc;
    } else
        rc = sz;

    /* If there are some remaining bytes, store them in buffer
     */
    if (rc)
        memcpy(sock->so_devobuf, buffer, rc);
    else                        /* Otherwise free buffer */
        free(sock->so_devobuf);
    sock->so_devocnt = rc;

    return size;
}

/*!
 * \brief Write to device.
 *
 * This function is implemented for CPUs with Harvard Architecture
 * only.
 *
 * TCP sockets can be used like other Nut/OS devices. This routine
 * is part of the virtual socket device driver and similar to
 * NutTcpDeviceWrite() except that the data is located in program
 * memory.
 *
 * This function is called by the low level output routines of the
 * \ref xrCrtLowio "C runtime library", using the
 * \ref _NUTDEVICE::dev_write_P entry.
 *
 * \param sock   Socket descriptor. This pointer must have been
 *               retrieved by calling NutTcpCreateSocket().
 * \param buffer Pointer to the data in program space to be written.
 * \param size   Number of bytes to write.
 *
 * \warning Inefficient implementation. No buffering has been
 *          implemented. Thus, each call will result in a separate
 *          TCP segment.
 */
#ifdef __HARVARD_ARCH__
int NutTcpDeviceWrite_P(TCPSOCKET * sock, PGM_P buffer, int size)
{
    int rc;
    char *rp = 0;

    /*
     * Hack alert. Neither do we handle out of memory correctly
     * nor do we pass the PGM pointer to lower levels.
     */
    if (size && (rp = NutHeapAlloc(size)) != 0)
        memcpy_P(rp, buffer, size);
    rc = NutTcpDeviceWrite(sock, rp, size);
    if (rp)
        NutHeapFree(rp);

    return rc;
}
#endif

/*!
 * \brief Driver control function.
 *
 * Used by the virtual device driver to modify or query device specific
 * settings.
 *
 * \param sock  Socket descriptor. This pointer must have been
 *              retrieved by calling NutTcpCreateSocket().
 * \param cmd   Requested control function. May be set to one of the
 *              following constants:
 *              - \ref IOCTL_GETFILESIZE
 *              - \ref IOCTL_GETINBUFCOUNT
 *              - \ref IOCTL_GETOUTBUFCOUNT
 *
 * \param param Points to a buffer that contains any data required for
 *              the given control function or receives data from that
 *              function.
 * \return 0 on success, -1 otherwise.
 */
int NutTcpDeviceIOCtl(TCPSOCKET * sock, int cmd, void *param)
{
    uint32_t *lvp = (uint32_t *) param;
    int rc = 0;

    switch (cmd) {
    case IOCTL_GETFILESIZE:
    case IOCTL_GETINBUFCOUNT:
        *lvp = (sock->so_rx_cnt - sock->so_rd_cnt);
        break;
    case IOCTL_GETOUTBUFCOUNT:
        *lvp = (sock->so_devocnt);
        break;
    default:
        rc = -1;
    }

    return rc;
}

#ifndef CRT_DISABLE_SELECT_POLL
/*!
 * \brief Callback function called by select routine to check, if read
 *        or write to this socket would block
 * \internal
 *
 * This function is called by select_scan routine of the C runtime library
 * as part of the select() implementation. If select if currently waiting
 * on a waitqueue, this function will be called to check the current state
 * as soon as the select waitqueues got signalled by the tcp statemachine
 *
 * \param sock  Socket descriptor. This pointer must have been
 *              retrieved by calling NutTcpCreateSocket().
 * \param flags Flags representing what we are waiting for (read / write / exception)
 * \param wq    Waitqueue, which should be added to the drivers waitqueue list. The
 *              thread that called select is waiting on this waitqueue
 * \param cmd   Internal command, that is passed to NutSelectManageWq
 *
 * \return Flags representing the current filedescriptor state
 */
int NutTcpDeviceSelect (TCPSOCKET * sock, int flags, HANDLE *wq, select_cmd_t cmd)
{
    int rflags = 0;

    /* Manage the wait queue lists for the select call */
    NutSelectManageWq(&sock->so_rx_wq_list, wq, flags & WQ_FLAG_READ, cmd);
    NutSelectManageWq(&sock->so_tx_wq_list, wq, flags & WQ_FLAG_WRITE, cmd);

    if (sock->so_state == TCPS_ESTABLISHED) {
        /* Check if there is some data available for reading (compare to NutTcpReceive) */
        if (sock->so_rx_cnt - sock->so_rd_cnt > 0) {
            rflags |= WQ_FLAG_READ;
        }

        /* Check if there is some free space in the output buffer. */
        if (sock->so_devocnt < sock->so_devobsz) {
            rflags |= WQ_FLAG_WRITE;
        }
    }
    if ((sock->so_state == TCPS_CLOSED) || (sock->so_state == TCPS_CLOSE_WAIT) || (sock->so_state == TCPS_DESTROY)) {
        /* The socket has been closed. Signal for read and write.
           Send and receive routines will then return EOF.
         */
        rflags |= WQ_FLAG_READ | WQ_FLAG_WRITE;
    }

    rflags &= flags;

    return rflags;
}
#endif
/*@}*/
