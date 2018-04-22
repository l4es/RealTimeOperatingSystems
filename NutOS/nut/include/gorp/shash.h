#ifndef _SHASH_H_
#define _SHASH_H_

/*
 * Copyright (c) 1998-1999 Jeremie Miller <jer@jabber.org>
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 *
 */

/* This code is based on
 * Based on BSD licensed mdnsd implementation by Jer <jer@jabber.org>
 * http://dotlocal.org/mdnsd/
 *
 * Unfortunately this site is now longer alive. You can still find it at:
 * http://web.archive.org/web/20080705131510/http://dotlocal.org/mdnsd/
 *
 * mdnsd - embeddable Multicast DNS Daemon
 * =======================================
 *
 * "mdnsd" is a very lightweight, simple, portable, and easy to integrate
 * open source implementation of Multicast DNS (part of Zeroconf, also called
 * Rendezvous by Apple) for developers. It supports both acting as a Query and
 * a Responder, allowing any software to participate fully on the .localnetwork
 * just by including a few files and calling a few functions.  All of the
 * complexity of handling the Multicast DNS retransmit timing, duplicate
 * suppression, probing, conflict detection, and other facets of the DNS
 * protocol is hidden behind a very simple and very easy to use interface,
 * described in the header file. The single small c source file has almost no
 * dependencies, and is portable to almost any embedded platform.
 * Multiple example applications and usages are included in the download,
 * including a simple persistent query browser and a tool to advertise .local
 * web sites.
 *
 * The code is licensed under both the GPL and BSD licenses, for use in any
 * free software or commercial application. If there is a licensing need not
 * covered by either of those, alternative licensing is available upon request.
 *
 */

/*!
 * \file include/gorp/shash.h
 * \brief Simple hastable implementation of a string ==> void* hashtable
 *        Minimal and efficient
 *
 * \verbatim
 *
 * $Id$
 *
 * \endverbatim
 */


/*!
 * \addtogroup xgSHash
 */
/*@{*/


/* Simple hastable implementation of a string ==> void* hashtable */
typedef struct string_hash_table_struct *SHASH;

/*@}*/

/* Hash creation and cleanup functions */
extern SHASH SHashInit(int prime);
extern void  SHashFree(SHASH hash);

/* Hash manipulation and query functions */
extern void  SHashSet(SHASH hash, char *key, void *val);
extern void  SHashStore(SHASH hash, const char *key, int key_len, void *val, int value_len);
extern void *SHashGet(SHASH hash, const char *key);

/* Routine to iterate over the hash and call a callback for each entry */
typedef void (*SHASH_CB)(SHASH hash, const char *key, void *val, void *arg);
extern void SHashForEach(SHASH hash, SHASH_CB cb, void *arg);

#endif

