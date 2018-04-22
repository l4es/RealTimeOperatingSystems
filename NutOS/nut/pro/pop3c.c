/*
 * Copyright 2012 by egnite GmbH
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

/*
 * \file pro/pop3c.c
 * \brief Post office protocol client.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <gorp/md5.h>

#include <stdlib.h>
#include <string.h>

#include <pro/pop3c.h>

#ifndef POP3_TIMEOUT
#define POP3_TIMEOUT    600000
#endif

/*!
 * \brief Read a response line from the server.
 *
 * \return Pointer to a buffer containing the response. In case of a
 *         broken connection or a line overflow, a NULL pointer is
 *         returned.
 */
const char *NutPop3ReceiveResponse(POP3CLIENTSESSION * si)
{
    char *cp;

    if (fgets(si->pop3_buff, sizeof(si->pop3_buff), si->pop3_stream)) {
        cp = strchr(si->pop3_buff, '\r');
        if (cp == NULL) {
            cp = strchr(si->pop3_buff, '\n');
        }
        if (cp) {
            *cp = '\0';
            return si->pop3_buff;
        }
        /* Line overflow. */
    }
    return NULL;
}

static int CheckResponse(const char *response)
{
    return (response && *response == '+') ? 0 : -1;
}

/*!
 * \brief Send command to the server and return the first response line.
 *
 * If a multi-line response is expected, the caller may use
 * NutPop3ReceiveResponse() to receive additional response lines.
 *
 * \param si  Pointer to the \ref POP3CLIENTSESSION structure, obtained
 *            from a previous call to NutPop3Connect().
 * \param fmt Format string containing conversion specifications like
 *            printf.
 *
 * \return Pointer to a buffer containing the response. If an error
 *         occurred, then a NULL pointer is returned.
 */
const char *NutPop3SendCommand(POP3CLIENTSESSION * si, const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(si->pop3_stream, (char *) fmt, ap);
    va_end(ap);
    fputs("\r\n", si->pop3_stream);
    fflush(si->pop3_stream);

    return NutPop3ReceiveResponse(si);
}

/*!
 * \brief Terminate an POP3 session.
 *
 * Gracefully closes the POP3 connection.
 *
 * \param si Pointer to the \ref POP3CLIENTSESSION structure, obtained
 *           from a previous call to NutPop3Connect().
 */
void NutPop3Disconnect(POP3CLIENTSESSION * si)
{
    if (si->pop3_sock) {
        if (si->pop3_stream) {
            NutPop3SendCommand(si, "QUIT");
            fclose(si->pop3_stream);
        }
        NutTcpCloseSocket(si->pop3_sock);
    }
    free(si->pop3_stamp);
    free(si);
}

/*!
 * \brief Start an POP3 session.
 *
 * Applications may use the following basic sequence to retrieve an email:
 *
 * \code
 * #include <pro/pop3c.h>
 *
 * POP3CLIENTSESSION *pop3;
 * char *line;
 *
 * pop3 = NutPop3Connect(ip, 110);
 * if (pop3) {
 *   NutPop3Login(pop3, "luser", "secret");
 *   if (NutPop3RetrieveMsg(pop3, 1) == 0) {
 *     do {
 *       line = NutPop3ReceiveResponse(pop3);
 *     } while (line && strcmp(line, "."));
 *   }
 *   NutPop3Disconnect(pop3);
 * }
 * \endcode
 *
 * \param ip   IP address of the host to connect.
 * \param port Port number to connect. Typically port 110 is used by POP3.
 *
 * \return A pointer to a newly create \ref POP3CLIENTSESSION structure,
 *         if the server is connected and ready to accept commands.
 *         Otherwise a NULL pointer is returned.
 */
POP3CLIENTSESSION *NutPop3Connect(uint32_t ip, uint16_t port)
{
    POP3CLIENTSESSION *si;

    si = calloc(1, sizeof(*si));
    if (si) {
        si->pop3_sock = NutTcpCreateSocket();
        if (si->pop3_sock && NutTcpConnect(si->pop3_sock, ip, port) == 0) {
            uint32_t tmo = POP3_TIMEOUT;
            NutTcpSetSockOpt(si->pop3_sock, SO_RCVTIMEO, &tmo, sizeof(tmo));
            si->pop3_stream = _fdopen((int) ((intptr_t) si->pop3_sock), "r+b");
            if (si->pop3_stream) {
                const char *rsp = NutPop3ReceiveResponse(si);
                if (rsp) {
                    char *cp = strchr(rsp, '<');
                    if (cp) {
                        si->pop3_stamp = strdup(cp);
                        if (si->pop3_stamp) {
                            cp = strchr(si->pop3_stamp, '>');
                            if (cp) {
                                *++cp = '\0';
                            } else {
                                free(si->pop3_stamp);
                                si->pop3_stamp = NULL;
                            }
                        }
                    }
                    return si;
                }
            }
        }
        NutPop3Disconnect(si);
        free(si);
    }
    return NULL;
}

/*!
 * \brief Identify the POP3 client to the server.
 *
 * \param si   Pointer to the \ref POP3CLIENTSESSION structure, obtained
 *             from a previous call to NutPop3Connect().
 * \param user Login name.
 * \param pass Login password.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutPop3Login(POP3CLIENTSESSION * si, char *user, char *pass)
{
    int rc = -1;

    if (si->pop3_stamp) {
        size_t len = strlen(si->pop3_stamp) + strlen(pass);
        char *buff = malloc(len + 1);
        MD5CONTEXT *ctx = calloc(1, sizeof(*ctx));
        uint8_t *digest = malloc(16);

        if (buff && ctx && digest) {
            strcpy(buff, si->pop3_stamp);
            strcat(buff, pass);
            NutMD5Init(ctx);
            NutMD5Update(ctx, (uint8_t *) buff, len);
            NutMD5Final(ctx, digest);

            rc = CheckResponse(NutPop3SendCommand(si, "APOP %s %s", user, digest));
        }
        free(buff);
        free(ctx);
        free(digest);
    } else {
        if (CheckResponse(NutPop3SendCommand(si, "USER %s", user)) == 0) {
            rc = CheckResponse(NutPop3SendCommand(si, "PASS %s", pass));
        }
    }
    return rc;
}

/*!
 * \brief Start reading a message.
 *
 * When this function returns 0, then the caller should use
 * NutPop3ReceiveResponse() to retrieve the contents of the message line
 * by line until a line containing a single dot is returned. If the
 * requested message is not available, then -1 is returned.
 *
 * \param si  Pointer to the \ref POP3CLIENTSESSION structure, obtained
 *            from a previous call to NutPop3Connect().
 * \param msg Index of the message to retrieve, starting at 1.
 *
 * \return 0 on success, the caller may use NutPop3ReceiveResponse()
 *         to retrieve the contents of the message line by line until
 *         a line with a single dot is returned. If an error occurs or
 *         if the requested message is not available, then -1 is
 *         returned.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutPop3RetrieveMsg(POP3CLIENTSESSION * si, int msg)
{
    return CheckResponse(NutPop3SendCommand(si, "RETR %d", msg));
}

/*!
 * \brief Delete a message.
 *
 * \param si  Pointer to the \ref POP3CLIENTSESSION structure, obtained
 *            from a previous call to NutPop3Connect().
 * \param msg Index of the message to remove, starting at 1.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutPop3DeleteMsg(POP3CLIENTSESSION * si, int msg)
{
    return CheckResponse(NutPop3SendCommand(si, "DELE %d", msg));
}
