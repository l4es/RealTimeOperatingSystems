/*
 * Copyright (C) 2012-2013 by egnite GmbH
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

#include <arpa/inet.h>
#include <netdb.h>

#include <pro/tcphost.h>


int TcpHostConnect(TCPSOCKET *sock, const char * host, uint16_t port)
{
    uint32_t ip;

    /* Get remote host IP address. */
    ip = inet_addr(host);
    if (ip == (uint32_t)-1 || ip == 0) {
        /* Doesn't look like an address, try host name. */
        ip = NutDnsGetHostByName((uint8_t *)host);
        if (ip == 0) {
            /* Give up. */
            return -1;
        }
    }
    /* Got a valid address, connect to it. */
    return NutTcpConnect(sock, ip, port);
}

FILE *TcpHostConnectStream(TCPSOCKET *sock, const char *host, uint16_t port, uint32_t tmo)
{
    /* Set socket options. Failures are ignored. */
    NutTcpSetSockOpt(sock, SO_RCVTIMEO, &tmo, sizeof(tmo));

    /* Connect the stream server. */
    if (TcpHostConnect(sock, host, port)) {
        return NULL;
    }
    /* Associate a binary stream with the socket. */
    return _fdopen((int) ((intptr_t) sock), "r+b");
}
