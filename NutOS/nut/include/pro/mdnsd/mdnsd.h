#ifndef _MDNSD_H_
#define _MDNSD_H_

/*
 * Copyright (C) 2003 Jeremie Miller <jer@jabber.org>
 * Copyright (c) 2009 Simon Budig <simon@budig.org>
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
 * \file include/pro/mdnsd.h
 * \brief Multicast DNS Deamon
 *
 * \verbatim
 *
 * $Id$
 *
 * \endverbatim
 */

#include <pro/mdnsd/rfc1035.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timer.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

#define MDNS_PORT         5353
#define MDNS_MULTICAST_IP "224.0.0.251"

/* Size of query/publish hashes */
//#define SPRIME 108
#define SPRIME 19


/* size of cache hash */
//#define LPRIME 1009
#define LPRIME 41

/* brute force garbage cleanup frequency, rarely needed (daily default) */
//#define GC     86400
#define GC     3600


/*!
 * \brief MDNS answer data
 */
typedef struct _mdnsda_struct  TMdnsdAnswer;
struct _mdnsda_struct
{
    char    *name;
    uint16_t type;
    uint32_t ttl;
    uint16_t rdlen;
    uint8_t *rdata;
    uint32_t ip;       // A
    char    *rdname;   // NS/CNAME/PTR/SRV
    struct {
        uint16_t priority;
        uint16_t weight;
        uint16_t port;
    } srv; // SRV
};


/*!
 * \brief MDNS record entry
 */
typedef struct _mdnsdr_struct  TMdnsdRecord;
struct _mdnsdr_struct
{
    TMdnsdAnswer rr;
    int8_t unique; // # of checks performed to ensure
    int    tries;
    void  (*conflict)(TMdnsdRecord *record,char *name, int type, void *arg);
    void   *arg;
    TMdnsdRecord *next;
    TMdnsdRecord *list;
};


/*!
 * \brief MDNS query struct
 */
typedef struct _query_struct   TQuery;
struct _query_struct
{
    char    *name;
    int      type;
    uint32_t nexttry;
    int      tries;
    int (*answer)(TMdnsdAnswer *answer, void *arg);
    void    *arg;
    TQuery  *next;
    TQuery  *list;
};


/*!
 * \brief Unicast record data
 */
typedef struct _unicast_struct TUnicast;
struct _unicast_struct
{
    int       id;
    uint32_t  to_ip;
    uint16_t  port;
    TMdnsdRecord *record;
    TUnicast *next;
};


/*!
 * \brief Cached record entry struct
 */
typedef struct _cached_struct  TCached;
struct _cached_struct
{
    TMdnsdAnswer rr;
    TQuery      *query;
    TCached     *next;
};


/*!
 * \brief Main MDNS deamon data
 */
typedef struct _mdnsd_struct   TMdnsd;
struct _mdnsd_struct
{
    int8_t shutdown;
    uint32_t expireall;
	uint32_t checkqlist;
    struct timeval now;
    struct timeval sleep;
    struct timeval pause;
    struct timeval probe;
    struct timeval publish;
    int    class;
    int    frame;
    TCached      *cache[LPRIME];
    TMdnsdRecord *published[SPRIME];
    TMdnsdRecord *probing;
    TMdnsdRecord *a_now;
    TMdnsdRecord *a_pause;
    TMdnsdRecord *a_publish;
    TUnicast     *uanswers;
    TQuery       *queries[SPRIME];
    TQuery       *qlist;
};

/* Global functions */
TMdnsd *MdnsdNew(int class, int frame);
void MdnsdShutdown(TMdnsd *mdnsd);
void MdnsdFree(TMdnsd *mdnsd);

/* I/O functions */
void MdnsdInput(TMdnsd *mdnsd, DNSMESSAGE *m, uint32_t ip, uint16_t port);
int MdnsdOutput(TMdnsd *mdnsd, DNSMESSAGE *m, uint32_t *ip, uint16_t *port);
struct timeval *MdnsdGetMaxSleepTime(TMdnsd *mdnsd);

/* Qery / Answer functions */
void MdnsdQuery(TMdnsd *mdnsd, char *host, int type, int (*answer)(TMdnsdAnswer *a, void *arg), void *arg);
TMdnsdAnswer *MdnsdListCachedAnswers(TMdnsd *mdnsd, char *host, int type, TMdnsdAnswer *last);

/* Publishing functions */
TMdnsdRecord *MdnsdAllocUnique(TMdnsd *mdnsd, char *host, int type, uint32_t ttl, void (*conflict)(TMdnsdRecord *record, char *host, int type, void *arg), void *arg);
TMdnsdRecord *MdnsdAllocShared(TMdnsd *mdnsd, char *host, int type, uint32_t ttl);
void MdnsdDone(TMdnsd *mdnsd, TMdnsdRecord *record);

/* These all set/update the data for the given record, nothing is published until they are called */
void MdnsdSetRaw(TMdnsd *mdnsd, TMdnsdRecord *record, uint8_t *data, int len);
void MdnsdSetHost(TMdnsd *mdnsd, TMdnsdRecord *record, char *name);
void MdnsdSetIp(TMdnsd *mdnsd, TMdnsdRecord *record, uint32_t ip);
void MdnsdSetSrv(TMdnsd *mdnsd, TMdnsdRecord *record, int priority, int weight, uint16_t port, char *name);

/* MDNS Deamon service routines */

int MdnsDeamonStart(char *eth_dev, uint32_t ip, uint16_t port, char *hostname, char *servicetype, int param_count, char **params);

#endif
