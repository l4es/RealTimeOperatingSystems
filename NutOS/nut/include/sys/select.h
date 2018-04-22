#ifndef _SYS_SELECT_H_
#define _SYS_SELECT_H_

/*
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 *
 */

/*!
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include "cfg/crt.h"
#include <compiler.h>
#include <io.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <stdint.h>

/*!
 * \addtogroup xgCrtLowio
 */
/*@{*/

/*!
 * \brief waitqueue flags: wait for read / write / exception
 */

#define WQ_FLAG_READ    0x01
#define WQ_FLAG_WRITE   0x02
#define WQ_FLAG_EXCEPT  0x04

/*!
 * \brief waitqueue list structure type.
 */
typedef struct _WQLIST WQLIST;

/*!
 * \struct _WQLIST device.h sys/device.h
 * \brief File waitqueue list structure.
 */

struct _WQLIST {
    WQLIST      *next;
    HANDLE      *wq;
    uint_fast8_t flags;
};

/*!
 * \brief Select commands, internally used.
 */

typedef enum {
	SELECT_CMD_NOP  = 0,
	SELECT_CMD_INIT,
	SELECT_CMD_CLEANUP,
} select_cmd_t;

/* FD_SET used for select */

/* Define the maximum number of filedescriptors handled by select at the same
   time. This value is FOPEN_MAX rounded to the next higher power of two.
 */
#define FD_SETSIZE    ((FOPEN_MAX+7) & (~0x07))

/* File descriptor set manipulation and test functions */
#define FD_SET(n, p)  ((p)->fd_bits[(n) >> 3] |=  (1 << ((n) & 0x07)))
#define FD_CLR(n, p)  ((p)->fd_bits[(n) >> 3] &= ~(1 << ((n) & 0x07)))
#define FD_ISSET(n,p) ((p)->fd_bits[(n) >> 3] &   (1 << ((n) & 0x07)))
#define FD_ZERO(p)    memset((void*)(p), 0, sizeof(*(p)))

typedef struct fd_set {
	uint8_t fd_bits [(FD_SETSIZE+7)/8];
} fd_set;

/*@}*/

#ifndef CRT_DISABLE_SELECT_POLL

extern void NutSelectWakeup(WQLIST *wq_list, uint_fast8_t flags);
extern void NutSelectWakeupFromIrq(WQLIST *wq_list, uint_fast8_t flags);
extern void NutSelectManageWq(WQLIST **wq_list, HANDLE *wq, int flags, select_cmd_t cmd);

extern int select(int n, fd_set *rfds, fd_set *wfds, fd_set *exfds, struct timeval *timeout);

#else

#define NutSelectWakeup(wq_list, flags) {}
#define NutSelectWakeupFromIrq(wq_list, flags) {}
#define NutSelectManageWq(wq_list, wq, flags, cmd) {}

#endif

#endif
