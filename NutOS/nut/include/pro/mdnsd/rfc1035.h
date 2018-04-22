#ifndef _RFC1035_H_
#define _RFC1035_H_

/*
 * Copyright (C) 2003 Jer <jer@jabber.org>
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
 * \file include/pro/rfc1035.h
 * \brief Standalone DNS parsing functions
 *
 * Implementation follows RF1035 [http://www.rfc-editor.org/rfc/rfc1035.txt] and
 * includes decoding functions for svr rr's code type 33. See RFC2782
 * [http://www.rfc-editor.org/rfc/rfc2782.txt]*
 *
 * \verbatim
 *
 * $Id$
 *
 * \endverbatim
 */

#include <stdint.h>

/*!
 * \addtogroup xgMulticastDns
 */
/*@{*/

#define PACKET_BUFFER_LEN  1500//4096
#define MAX_DNS_PACKET_LEN 1500//4000
#define MAX_LABEL_SIZE     256
#define MAX_LABEL          20

#define QTYPE_A           1
#define QTYPE_NS          2
#define QTYPE_CNAME       5
#define QTYPE_PTR         12
#define QTYPE_TXT         16
#define QTYPE_SRV         33
#define QTYPE_ANY         255

#define RRTYPE_A          QTYPE_A
#define RRTYPE_NS         QTYPE_NS
#define RRTYPE_CNAME      QTYPE_CNAME
#define RRTYPE_PTR        QTYPE_PTR
#define RRTYPE_TXT        QTYPE_TXT
#define RRTYPE_SRV        QTYPE_SRV


/*!
 * \brief DNS question structure type.
 *
 * Contains the name, query type and class
 */
typedef struct
{
    char    *name;
    uint16_t type;
    uint16_t class;
} DNSQUESTION;


/*!
 * \brief DNS ressource record structure type.
 */
typedef struct
{
    char    *name;
    uint16_t type;
    uint16_t class;
    uint32_t ttl;
    uint16_t rdlength;
    uint8_t *rdata;
    union {
        struct {
            uint32_t ip;
            char *name;
        } a;
        struct {
            char *name;
        } ns;
        struct {
            char *name;
        } cname;
        struct {
            char *name;
        } ptr;
        struct {
            uint16_t priority;
            uint16_t weight;
            uint16_t port;
            char *name;
        } srv;
    } known;
} DNSRESOURCE;


/*!
 * \brief DNS message structure type.
 */
typedef struct
{
    /* external data */
    uint16_t id;
    struct {
        uint16_t qr:1;
        uint16_t opcode:4;
        uint16_t aa:1;
        uint16_t tc:1;
        uint16_t rd:1;
        uint16_t ra:1;
        uint16_t z:3;
        uint16_t rcode:4;
    } header;
    uint16_t     qdcount;
    uint16_t     ancount;
    uint16_t     nscount;
    uint16_t     arcount;
    DNSQUESTION *qd;
    DNSRESOURCE *an;
    DNSRESOURCE *ns;
    DNSRESOURCE *ar;

    /* internal variables */
    uint8_t *buf;
    uint8_t *labels[MAX_LABEL];
    int len;
    int label;

    /* Padding data */
    uint8_t packet[MAX_DNS_PACKET_LEN];
} DNSMESSAGE;

/* Conversion of the network byte order buffer content into uint16_t or uint32_t values.
 * Buffer pointer is incremented accordingly
 */
uint16_t DnsNet2Short(uint8_t **buf);
uint32_t DnsNet2Long(uint8_t **buf);

/* Copies a short or long value in host byte order to the buffer and convert it to network byte order */
void DnsShort2Net(uint16_t val, uint8_t **buf);
void DnsLong2net(uint32_t val, uint8_t **buf);

/* Parsing function to parse a DNS packet into message format. Therfor the packet
   must be at least MAX_PACKET_LEN bytes in size and must be allocated clean (zeroed)
 */
void DnsParseMsg(DNSMESSAGE *msg, uint8_t *packet);

/* Create a message for sending out on the network. */
DNSMESSAGE *DnsCreateMsg(void);

/* Append a question to the message buffer */
void DnsMsgAdd_qd(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class);

/* Append a resource record to the message. Call these functions in the below order. */
void DnsMsgAdd_an(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl);
void DnsMsgAdd_ns(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl);
void DnsMsgAdd_ar(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl);

/* Append resource data types blocks */
void DnsMsgAdd_rdata_long(DNSMESSAGE *msg, uint32_t val);
void DnsMsgAdd_rdata_name(DNSMESSAGE *msg, char *name);
void DnsMsgAdd_rdata_srv(DNSMESSAGE *msg, uint16_t priority, uint16_t weight, uint16_t port, char *name);
void DnsMsgAdd_rdata_raw(DNSMESSAGE *msg, uint8_t *rdata, uint16_t rdlength);

/* Generate the message packet to be send out and return the length */
uint8_t *DnsMsg2Pkt(DNSMESSAGE *msg);
int DnsMsgLen(DNSMESSAGE *msg);


/*@}*/
#endif
