#ifndef PRO_POP3C_H
#define PRO_POP3C_H

/*
 * Copyright 2010 by egnite GmbH
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
 * \file pro/pop3c.h
 * \brief POP3 client functions.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <sys/socket.h>
#include <stdio.h>

#ifndef POP3_BUFSIZ
/*! \brief Size of the POP3 buffer. */
#define POP3_BUFSIZ     256
#endif

/*!
 * \brief POP3 session structure type.
 */
typedef struct _POP3CLIENTSESSION {
    /*! \brief Socket of this session. */
    TCPSOCKET *pop3_sock;
    /*! \brief Stream of this session. */
    FILE *pop3_stream;
    /*! \brief Server timestamp. */
    char *pop3_stamp;
    /*! \brief POP3 buffer. */
    char pop3_buff[POP3_BUFSIZ];
} POP3CLIENTSESSION;

extern POP3CLIENTSESSION *NutPop3Connect(uint32_t ip, uint16_t port);
extern void NutPop3Disconnect(POP3CLIENTSESSION * si);
extern int NutPop3Login(POP3CLIENTSESSION * si, char *user, char *pass);

extern int NutPop3RetrieveMsg(POP3CLIENTSESSION * si, int msg);
extern int NutPop3DeleteMsg(POP3CLIENTSESSION * si, int msg);
extern const char *NutPop3SendCommand(POP3CLIENTSESSION * si, const char *fmt, ...);
extern const char *NutPop3ReceiveResponse(POP3CLIENTSESSION * si);

#endif
