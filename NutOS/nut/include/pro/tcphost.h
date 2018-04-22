#ifndef _PRO_TCPHOST_H_
#define _PRO_TCPHOST_H_

/*
 * Copyright (C) 2011-2013 by egnite GmbH
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
 * \file pro/tcphost.h
 * \brief TCP host API.
 *
 * \verbatim File version $Id: tcphost.h 4918 2013-01-03 18:24:05Z haraldkipp $ \endverbatim
 */

#include <sys/socket.h>
#include <stdio.h>

/*!
 * \brief Connect a TCP remote host.
 *
 * This function similiar to the Nut/Net API function NutTcpConnect.
 * However, instead of an IP address, it also allows to specify a host
 * by its name.
 *
 * \param sock  Socket descriptor. This pointer must have been
 *              retrieved by calling NutTcpCreateSocket().
 * \param rhost Name or IP address of the host to connect.
 * \param port  Port number to connect to, given in host byte order.
 *
 * \return 0 on success, -1 otherwise.
 */
extern int TcpHostConnect(TCPSOCKET *sock, const char * host, uint16_t port);

/*!
 * \brief Connect a stdio stream to a TCP remote host.
 *
 * Convenience function for stream based TCP transfers.
 *
 * \param sock Socket descriptor. This pointer must have been
 *             retrieved by calling NutTcpCreateSocket().
 * \param host Name or IP address of the host to connect.
 * \param port Port number to connect to, given in host byte order.
 * \param tmo  Timeout in milliseconds.
 *
 * \return FILE pointer on success, NULL otherwise.
 */
extern FILE *TcpHostConnectStream(TCPSOCKET *sock, const char *host, uint16_t port, uint32_t tmo);

#endif
