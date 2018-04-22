#ifndef _PRO_HTTPU_H_
#define _PRO_HTTPU_H_

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

/*!
 * \file pro/httpu.h
 * \brief HTTP over UDP library.
 */

#include <cfg/http.h>
#include <sys/socket.h>

/*!
 * \addtogroup xgHTTPU HTTPU
 * \ingroup xgUserPro
 * \brief Hypertext transfer protocol over UDP library.
 */
/*@{*/

/*!
 * \brief Maximum datagram size.
 *
 * For each session a total number of 4 datagram buffers are allocated
 * from the heap.
 */
#ifndef HTTPU_MAX_DATAGRAM_SIZE
#define HTTPU_MAX_DATAGRAM_SIZE     508
#endif

/*!
 * \brief Maximum number of received header lines.
 *
 * Transmit header lines are only limited by
 * \ref HTTPU_MAX_DATAGRAM_SIZE.
 */
#ifndef HTTPU_MAX_HEADER_LINES
#define HTTPU_MAX_HEADER_LINES      16
#endif

/*! \brief HTTPU session structure type. */
typedef struct _HTTPU_SESSION HTTPU_SESSION;

/*! \brief HTTPU header structure type. */
typedef struct _HTTPU_HEADER HTTPU_HEADER;

/*! \brief HTTPU message structure type. */
typedef struct _HTTPU_MESSAGE HTTPU_MESSAGE;

/*! \brief HTTPU header structure. */
struct _HTTPU_HEADER {
    /*! \brief Number of header lines. */
    int hdr_num;
    /*! \brief Header names. */
    char *hdr_name[HTTPU_MAX_HEADER_LINES];
    /*! \brief Header values. */
    char *hdr_value[HTTPU_MAX_HEADER_LINES];
};

/*! \brief HTTPU message structure. */
struct _HTTPU_MESSAGE {
    /*! \brief Message content. */
    char msg_buff[HTTPU_MAX_DATAGRAM_SIZE + 1];
    /*! \brief Number of bytes in the buffer. */
    int msg_len;
};

/*! \brief HTTPU message structure. */
struct _HTTPU_SESSION {
    /*! \brief UDP socket. */
    UDPSOCKET *s_sock;
    /*! \brief IP address of requesting client. */
    uint32_t s_reqip;
    /*! \brief Source port of a request. */
    unsigned short s_reqport;
    /*! \brief Buffer for outgoing datagrams. */
    HTTPU_MESSAGE s_sndbuf;
    /*! \brief Buffer for incoming datagrams. */
    HTTPU_MESSAGE s_rcvbuf;
    /*! \brief Headers of incoming datagrams. */
    HTTPU_HEADER s_rcvhdr;
};

/*!
 * \brief Create HTTPU session.
 *
 *
 *
 * \param  port Server applications provide the local port number
 *              with this parameter. Client applications may
 *              pass zero to let the system select an available
 *              port.
 *
 * \return Session descriptor of the newly created HTTPU session or
 *         NULL in case of any failure.
 */
extern HTTPU_SESSION *HttpuSessionCreate(uint16_t port);

/*!
 * \brief Close HTTPU session.
 *
 * The memory occupied by the session is immediately released
 * after calling this function. The application  must not use
 * the session descriptor after this call.
 *
 * \param s Session descriptor, obtained by a previous call to
 *          HttpuSessionCreate().
 */
extern void HttpuSessionDestroy(HTTPU_SESSION *s);

/*!
 * \brief Receive HTTPU message.
 *
 * Note, that the total size of the HTTPU message is limited by
 * the configuration macro \ref HTTPU_MAX_DATAGRAM_SIZE.
 *
 * \param s   Session descriptor, obtained by a previous call to
 *            HttpuSessionCreate().
 * \param tmo Maximum number of milliseconds to wait.
 *
 * \return The number of bytes received, if successful. A negative
 *         return value indicates an error, while zero indicates
 *         a time out.
 */
extern int HttpuReceive(HTTPU_SESSION *s, uint32_t tmo);

/*!
 * \brief Send HTTPU message to a given destination.
 *
 * This function can be used by client applications to send an HTTPU
 * request.
 *
 * \param s    Session descriptor, obtained by a previous call to
 *             HttpuSessionCreate().
 * \param ip   Destination IP address.
 * \param port Destination UDP port.
 *
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutUdpError().
 */
extern int HttpuSend(HTTPU_SESSION *s, uint32_t ip, uint16_t port);

/*!
 * \brief Send HTTPU response.
 *
 * This function can be used by server applications to send an HTTPU
 * response.
 *
 * \param s   Session descriptor, obtained by a previous call to
 *            HttpuSessionCreate().
 *
 * \return 0 on success, -1 otherwise. The specific error code
 *         can be retrieved by calling NutUdpError().
 */
extern int HttpuRespond(HTTPU_SESSION *s);

/*!
 * \brief Add header line to HTTPU message.
 *
 * This function is used to create the header of a HTTPU request or
 * response.
 *
 * Note, that the total size of an HTTPU message, header plus body, is
 * limited by the configuration macro \ref HTTPU_MAX_DATAGRAM_SIZE.
 *
 * \param s    Session descriptor, obtained by a previous call to
 *             HttpuSessionCreate().
 * \param name Name of the header. If set to NULL, then the header
 *             value is used as the first line.
 * \param ...  Any number of strings that will be concatenated to
 *             form the header value. The last argument must be
 *             a NULL pointer.
 *
 * \return Accumulated size of this line or -1 in case of buffer
 *         overflow.
 */
extern int HttpuAddHeader(HTTPU_SESSION *s, const char *name, ...);

/*!
 * \brief Get HTTPU header value by name.
 *
 * This function can be used to query header values of incoming
 * messages.
 *
 * \param hdr  Pointer to the session'request s header storage,
 *             typically HTTPU_SESSION::s_rcvhdr.
 * \param name Name of the header.
 *
 * \return Pointer to the header value. If no header with the given
 *         name is available, then a pointer to an empty string is
 *         returned. Note, that the total number of HTTPU header
 *         lines for incoming telegrams is limited by the
 *         configuration macro \ref HTTPU_MAX_HEADER_LINES.
 */
extern const char *HttpuGetHeader(const HTTPU_HEADER *hdr, const char *name);

#endif

