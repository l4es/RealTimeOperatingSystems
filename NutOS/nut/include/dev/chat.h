#ifndef _DEV_CHAT_H_
#define _DEV_CHAT_H_

/*
 * Copyright (C) 2001-2004 by egnite Software GmbH. All rights reserved.
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
 * $Id: chat.h 4610 2012-09-17 10:48:26Z haraldkipp $
 */

#include <cfg/os.h>
#include <cfg/chat.h>
#include <stdint.h>

#define CHAT_ARG_SEND           0
#define CHAT_ARG_ABORT          1
#define CHAT_ARG_TIMEOUT        2
#define CHAT_ARG_REPORT         3

/*!
 * \brief Maximum number of abort strings.
 */
#ifndef CHAT_MAX_ABORTS
#define CHAT_MAX_ABORTS         10
#endif

/*!
 * \brief Maximum size of report strings.
 */
#ifndef CHAT_MAX_REPORT_SIZE
#define CHAT_MAX_REPORT_SIZE    32
#endif

/*!
 * \brief Default timeout.
 */
#ifndef CHAT_DEFAULT_TIMEOUT
#define CHAT_DEFAULT_TIMEOUT    45
#endif

typedef struct {
    int chat_fd;
    uint8_t chat_arg;
    uint8_t chat_aborts;
    char *chat_abort[CHAT_MAX_ABORTS];
    uint8_t chat_abomat[CHAT_MAX_ABORTS];
    char *chat_report_search;
    uint8_t chat_repmat;
    char chat_report_state;
} NUTCHAT;

extern uint8_t *chat_report;

#ifdef NUTDEBUG
#include <stdio.h>
#endif

#ifdef NUTDEBUG
void NutTraceChat(FILE * stream, uint8_t flags);
#endif

int NutChatExpectString(NUTCHAT *ci, char *str);
int NutChatExpect(NUTCHAT *ci, char *str);
int NutChatSend(NUTCHAT *ci, char *str);
NUTCHAT *NutChatCreate(int fd);
void NutChatDestroy(NUTCHAT *ci);
int NutChat(int fd, const char *script);
#ifdef __HARVARD_ARCH__
int NutChat_P(int fd, PGM_P script);
#endif

#endif
