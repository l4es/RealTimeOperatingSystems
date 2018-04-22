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
 * \file pro/rfc1035.c
 * \brief Standalone DNS parsing functions
 *
 * Implementation follows RF1035 [http://www.rfc-editor.org/rfc/rfc1035.txt] and
 * includes decoding functions for svr rr's code type 33. See RFC2782
 * [http://www.rfc-editor.org/rfc/rfc2782.txt]
 *
 * \verbatim
 *
 * $Id$
 *
 * \endverbatim
 */

#include <string.h>
#include <stdio.h>
#include <pro/mdnsd/rfc1035.h>

/*!
 * \addtogroup xgMulticastDns
 */
/*@{*/


/*!
 * \brief Conversion of the network byte order buffer content into uint16_t value.
 *
 * This functions increments the buffer pointer accordingly
 *
 * \param buf   buffer
 *
 * \return      Converted value
 */
uint16_t DnsNet2Short(uint8_t **buf)
{
    uint16_t val;
    val  = (*((*buf) ++)) << 8;
    val |=  *((*buf) ++);
    return val;
}


/*!
 * \brief Conversion of the network byte order buffer content into uint32_t value.
 *
 * This functions increments the buffer pointer accordingly
 *
 * \param buf   buffer
 *
 * \return      Converted value
 */
uint32_t DnsNet2Long(uint8_t **buf)
{
    uint32_t val;
    val  = (uint32_t)(*((*buf) ++)) << 24;
    val |= (uint32_t)(*((*buf) ++)) << 16;
    val |= (uint32_t)(*((*buf) ++)) <<  8;
    val |= (uint32_t) *((*buf) ++);
    return val;
}


/*!
 * \brief Conversion of uint16_t value into buffer content in network byte order
 *
 * This functions increments the buffer pointer accordingly
 *
 * \param val   Value
 * \param buf   buffer
 */
void DnsShort2Net(uint16_t val, uint8_t **buf)
{
    *((*buf) ++) = (uint8_t) (val >> 8);
    *((*buf) ++) = (uint8_t) val;
}

/*!
 * \brief Conversion of uint32_t value into buffer content in network byte order
 *
 * This functions increments the buffer pointer accordingly
 *
 * \param val   Value
 * \param buf   buffer
 */
void DnsLong2Net(uint32_t val, uint8_t **buf)
{
    *((*buf) ++) = (uint8_t) (val >> 24);
    *((*buf) ++) = (uint8_t) (val >> 16);
    *((*buf) ++) = (uint8_t) (val >> 8);
    *((*buf) ++) = (uint8_t) val;
}


/*!
 * \brief "Decompress" a label by calculating the offset of the original label string
 *
 * This functions increments the buffer pointer accordingly
 *
 * \param ptr   Pointer to the compressed label
 *
 * \return      Offset of the original label string
 */
static uint16_t LabelDecompress(uint8_t *ptr)
{
    uint16_t val;
    val = 0xc0 ^ ptr[0];
    val <<= 8;
    val |= ptr[1];
    if (val > PACKET_BUFFER_LEN - 1) {
        val = PACKET_BUFFER_LEN - 1;
    }
    return val;
}


/*!
 * \brief Extracts the label from the buffer.
 *
 * In case the label is compressed, it will be decompressed.
 * This functions increments the buffer pointer accordingly.
 * If the label is not yet cached, it will be added to the cached labels
 *
 * \param msg   DNS message
 * \param buf   Pointer to the package data buffer
 * \param namep Pointer to the string buffer, where the label will be copied to
 */
static void ExtractLabel(DNSMESSAGE *msg, uint8_t **buf, char **namep)
{
    uint8_t *label;
    char *name;
    int x;

    /* Set the name pointer to the end of the data block */
    name = (char*)(msg->packet + msg->len);
    *namep = name;

    /* loop storing label in the block */
    label = *buf;
    while (*label != 0) {
        /* Skip any compression pointer until end encountered */
        while (*label & 0xc0) {
            label = msg->buf + LabelDecompress(label);
            if (*(label) == 0) {
                break;
            }
        }

        /* Check the limits */
        if (((name + *label) - *namep > 255) ||
            (msg->len + ((name + *label) - *namep) > PACKET_BUFFER_LEN -1 )) {
            return;
        }

        /* Copy label to the name buffer */
        memcpy(name, label+1, *label);
        name[*label] = '.';

        name  += *label + 1;
        label += *label + 1;
    }

    /* Advance in the buffer */
    for (label = *buf; (*label != 0) && (!((*label & 0xc0) && label++)); label += *label + 1);
    *buf = label + 1;

    /* Add a traling \0 and check if the name is yet cached */
    *name = '\0';

    for (x = 0; x < MAX_LABEL; x++) {
        if (msg->labels[x]) {
            if (strcmp(*namep, (char*)msg->labels[x]) != 0) {
                continue;
            }

            *namep = (char*)msg->labels[x];
            return;
        }
    }

    /* The label was not yet cached, so cache it if there is still room */
    if ((x < MAX_LABEL) && (msg->labels[x] == 0)) {
        msg->labels[x] = (uint8_t*)*namep;
    }
    msg->len += (name - *namep) + 1;
}

/*!
 * \brief Check if labels are matching
 *
 * If labels are compressed, they will be decompressed first.
 *
 * \param msg       DNS message
 * \param label1    Pointer to label1
 * \param label1    Pointer to label2
 *
 * \return          1 in case they are matching, else 0
 */
static int MatchLabel(DNSMESSAGE *msg, uint8_t *label1, uint8_t *label2)
{
    int len;

    /* If we were calles with a pointer, call MatchLabel with dereferenced pointer again */
    if (*label1 & 0xc0) {
        return MatchLabel(msg, msg->buf + LabelDecompress(label1), label2);
    }

    if (*label2 & 0xc0) {
        return MatchLabel(msg, label1, msg->buf + LabelDecompress(label2));
    }

    /* Return in case of a match */
    if (label1 == label2) {
        return 1;
    }

    /* Compare the label */
    if (*label1 != *label2){
        return 0;
    }

    for (len = 1; len <= *label1; len++) {
        if (label1[len] != label2[len]) {
            return 0;
        }
    }

    /* Get the new labels */
    label1 += *label1 + 1;
    label2 += *label2 + 1;

    /* Finally all labels should be matched */
    if ((*label1 == 0) && (*label2 == 0)) {
        return 1;
    }

    /* Recursivly call match with the next labels */
    return MatchLabel(msg, label1, label2);
}


/*!
 * \brief Convert host name to label using compression
 *
 * \param msg       DNS message
 * \param buf       Pointer to the buffer where the label shall be placed
 * \param name      Hostname
 *
 * \return          Length of the label
 */
static int Host2Label(DNSMESSAGE *msg, uint8_t **buf, char *name)
{
    uint8_t  label[MAX_LABEL_SIZE];
    uint8_t *l;
    int len = 0;
    int x = 1;
    int y = 0;
    int last = 0;

    if (name == NULL) {
        return 0;
    }

    /* Let's make the label */
    while(name[y]) {
        if (name[y] == '.') {
            if (name[y + 1] == 0) {
                break;
            }
            label[last] = x - (last + 1);
            last = x;
        } else {
            label[x] = name[y];
        }
        if (x++ == MAX_LABEL_SIZE - 1) {
            return 0;
        }
        y++;
    }

    label[last] = x - (last + 1);

    if (x == 1) {
		/* special case, bad names, but handle correctly */
        x--;
    }
    len = x + 1;

    /* \0 terminate the label */
    label[x] = 0;

    /* Check each label against all msg->labels for a match */
    for(x = 0; label[x]; x += label[x] + 1) {
        for( y = 0; msg->labels[y]; y++) {
            if (MatchLabel(msg, label+x, msg->labels[y])) {
                /* If the label matches, create the pointer */
                l = label + x;
                DnsShort2Net(msg->labels[y] - msg->packet, &l);
                label[x] |= 0xc0;
                len = x + 2;
                break;
            }
        }

        if (label[x] & 0xc0) {
            break;
        }
    }

    /* Copy the label into the buffer and let the buffer pointer point to the label */
    memcpy(*buf, label, len);
    l = *buf;
    *buf += len;

    /* Save the label location of each new label for future compression */
    for (x = 0; l[x]; x += l[x] + 1) {
        if(l[x] & 0xc0) {
            break;
        }

        if (msg->label + 1 >= MAX_LABEL - 1) {
            break;
        }

        msg->labels[msg->label++] = l + x;
    }

    return len;
}


/*!
 * \brief Parse a ressource record packet and fill the rr struct
 *
 * \param msg       DNS message
 * \param rr        Pointer to ressource record struct where the informations shall be stored
 * \param count     Number of ressource records in the package
 * \param buf       Package buffer
 *
 * \return          0: success, -1 in case of an error
 */
static int ParseRR(DNSMESSAGE *msg, DNSRESOURCE *rr, int count, uint8_t **buf)
{
    int i;

    for (i=0; i < count; i++) {
        /* Extract the name, type, class, tt and record data length from the
           buffer. The buffer pointer is automatically incremented by each call
           tp nut2xxx
         */
        ExtractLabel(msg, buf, &(rr[i].name));
        rr[i].type     = DnsNet2Short(buf);
        rr[i].class    = DnsNet2Short(buf);
        rr[i].ttl      = DnsNet2Long(buf);
        rr[i].rdlength = DnsNet2Short(buf);

        /* Sanity checking the length */
        if ((rr[i].rdlength + (*buf - msg->buf) > MAX_DNS_PACKET_LEN) ||
           (msg->len + rr[i].rdlength > MAX_DNS_PACKET_LEN)) {
            return -1;
        }

        /* Length was ok, make a copy of the data */
        rr[i].rdata = msg->packet + msg->len;

        msg->len += rr[i].rdlength;
        memcpy(rr[i].rdata, *buf, rr[i].rdlength);

        /* Parse known message types */
        switch (rr[i].type) {
            case RRTYPE_A:
                if(msg->len + 16 > MAX_DNS_PACKET_LEN) {
                    return -1;
                }
                rr[i].known.a.name = (char*)(msg->packet + msg->len);
                msg->len += 16;
                sprintf(rr[i].known.a.name, "%d.%d.%d.%d", (*buf)[0], (*buf)[1], (*buf)[2], (*buf)[3]);
                rr[i].known.a.ip = DnsNet2Long(buf);
                break;

            case RRTYPE_NS:
                ExtractLabel(msg, buf, &(rr[i].known.ns.name));
                break;

            case RRTYPE_CNAME:
                ExtractLabel(msg, buf, &(rr[i].known.cname.name));
                break;

            case RRTYPE_PTR:
                ExtractLabel(msg, buf, &(rr[i].known.ptr.name));
                break;

            case RRTYPE_SRV:
                rr[i].known.srv.priority = DnsNet2Short(buf);
                rr[i].known.srv.weight   = DnsNet2Short(buf);
                rr[i].known.srv.port     = DnsNet2Short(buf);
                ExtractLabel(msg, buf, &(rr[i].known.srv.name));
                break;

            default:
                *buf += rr[i].rdlength;
        }
    }

    return 0;
}


/*!
 * \brief Parse DNS packet into message format
 *
 * The packet must be at least MAX_PACKET_LEN bytes in size and must be allocated clean (zeroed)
 *
 * \param msg       DNS message
 * \param packet    Package buffer
 */
void DnsParseMsg(DNSMESSAGE *msg, uint8_t *packet)
{
    uint8_t *buf;
    int i;

    if((packet == 0) || (msg == 0)) {
        return;
    }

    /* Bit decoding of the record header */
    buf = packet;
    msg->buf = buf;
    msg->id = DnsNet2Short(&buf);


    if (buf[0] & 0x80) {
        msg->header.qr = 1;
    }

    /* Shift opcode field 3 bits right for easy comparision */
    msg->header.opcode = (buf[0] & 0x78) >> 3;

    if (buf[0] & 0x01) {
        msg->header.rd = 1;
    }

    if (buf[0] & 0x02) {
        msg->header.tc = 1;
    }

    if (buf[0] & 0x04) {
        msg->header.aa = 1;
    }

    if (buf[1] & 0x80) {
        msg->header.ra = 1;
    }

    msg->header.z = (buf[1] & 0x70) >> 4;
    msg->header.rcode = buf[1] & 0x0F;
    buf += 2;

    msg->qdcount = DnsNet2Short(&buf);
    if (msg->len + (sizeof(DNSQUESTION) * msg->qdcount) > MAX_DNS_PACKET_LEN - 8) {
        msg->qdcount = 0;
        return;
    }

    /* Get number of AN records */
    msg->ancount = DnsNet2Short(&buf);
    if (msg->len + (sizeof(DNSRESOURCE) * msg->ancount) > MAX_DNS_PACKET_LEN - 8) {
        msg->ancount = 0;
        return;
    }

    /* Get number of NS records */
    msg->nscount = DnsNet2Short(&buf);
    if (msg->len + (sizeof(DNSRESOURCE) * msg->nscount) > MAX_DNS_PACKET_LEN - 8) {
        msg->nscount = 0;
        return;
    }

    /* Get number of AR records */
    msg->arcount = DnsNet2Short(&buf);
    if (msg->len + (sizeof(DNSRESOURCE) * msg->arcount) > MAX_DNS_PACKET_LEN - 8) {
        msg->arcount = 0;
        return;
    }

    /* Process the questions */
    while (msg->len & 0x07) msg->len++;
    /* Process AN record */
    msg->qd= (DNSQUESTION *) (msg->packet + msg->len);
    msg->len += sizeof(DNSQUESTION) * msg->qdcount;

    for(i = 0; i < msg->qdcount; i++) {
        ExtractLabel(msg, &buf, &(msg->qd[i].name));
        msg->qd[i].type  = DnsNet2Short(&buf);
        msg->qd[i].class = DnsNet2Short(&buf);
    }


    /* Process Ressource records and keep us always at a 8 byte boundary */

    /* Align... */
    while (msg->len & 0x07) msg->len++;
    /* Process AN record */
    msg->an = (DNSRESOURCE *) (msg->packet + msg->len);
    msg->len += sizeof(DNSRESOURCE) * msg->ancount;

    /* Align... */
    while (msg->len & 0x07) msg->len++;
    /* Process NS record */
    msg->ns = (DNSRESOURCE *) (msg->packet + msg->len);
    msg->len += sizeof(DNSRESOURCE) * msg->nscount;

    /* Align... */
    while (msg->len & 0x07) msg->len++;
    /* Process AR record */
    msg->ar = (DNSRESOURCE *) (msg->packet + msg->len);
    msg->len += sizeof(DNSRESOURCE) * msg->arcount;

    /* Parse AN record */
    if (ParseRR(msg, msg->an, msg->ancount, &buf)) {
        /* Size limit checking failed */
        return;
    }

    /* Parse NS record */
    if (ParseRR(msg, msg->ns, msg->nscount, &buf)) {
        /* Size limit checking failed */
        return;
    }

    /* Parse AR record */
    if (ParseRR(msg, msg->ar, msg->arcount, &buf)) {
        /* Size limit checking failed */
        return;
    }
}


/*!
 * \brief Append a ressource record to the message
 *
 * \param msg       DNS message
 * \param name      Hostname
 * \param type      Query type
 * \param class     Query class
 * \param ttl       TTL value
 */
static void DnsAppendRR(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl)
{
    if (msg->buf == 0) {
        /* Initialise the buffer pointer */
        msg->buf = msg->packet + 12;
    }
    Host2Label(msg, &(msg->buf), name);
    DnsShort2Net(type, &(msg->buf));
    DnsShort2Net(class, &(msg->buf));
    DnsLong2Net(ttl, &(msg->buf));
}


/*!
 * \brief Append a QD ressource record to the message
 *
 * Should be called first before calling DnsMsgAdd_an.
 *
 * \param msg       DNS message
 * \param name      Hostname
 * \param type      Query type
 * \param class     Query class
 */
void DnsMsgAdd_qd(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class)
{
    msg->qdcount++;
    if (msg->buf == 0) {
        /* Initialise the buffer pointer */
        msg->buf = msg->packet + 12;
    }

    Host2Label(msg, &(msg->buf), name);
    DnsShort2Net(type, &(msg->buf));
    DnsShort2Net(class, &(msg->buf));
}


/*!
 * \brief Append a AN ressource record to the message
 *
 * Should be called first before calling DnsMsgAdd_ns.
 *
 * \param msg       DNS message
 * \param name      Hostname
 * \param type      Query type
 * \param class     Query class
 */
void DnsMsgAdd_an(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl)
{
    msg->ancount++;
    DnsAppendRR(msg, name, type, class, ttl);
}


/*!
 * \brief Append a NS ressource record to the message
 *
 * Should be called first before calling DnsMsgAdd_ar.
 *
 * \param msg       DNS message
 * \param name      Hostname
 * \param type      Query type
 * \param class     Query class
 */
void DnsMsgAdd_ns(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl)
{
    msg->nscount++;
    DnsAppendRR(msg, name, type, class, ttl);
}


/*!
 * \brief Append a AR ressource record to the message
 *
 * \param msg       DNS message
 * \param name      Hostname
 * \param type      Query type
 * \param class     Query class
 */
void DnsMsgAdd_ar(DNSMESSAGE *msg, char *name, uint16_t type, uint16_t class, uint32_t ttl)
{
    msg->arcount++;
    DnsAppendRR(msg, name, type, class, ttl);
}


/*!
 * \brief Append resource data types block: uint32_t value
 *
 * \param msg       DNS message
 * \param val       value
 */
void DnsMsgAdd_rdata_long(DNSMESSAGE *msg, uint32_t val)
{
    DnsShort2Net(4, &(msg->buf));
    DnsLong2Net(val, &(msg->buf));
}


/*!
 * \brief Append resource data types block: Hostname
 *
 * \param msg       DNS message
 * \param name      Hostname
 */
void DnsMsgAdd_rdata_name(DNSMESSAGE *msg, char *name)
{
    uint8_t *tmp_buf = msg->buf;
    msg->buf += 2;
    DnsShort2Net(Host2Label(msg, &(msg->buf), name), &tmp_buf);
}

/*!
 * \brief Append resource data types block: Service data
 *
 * \param msg       DNS message
 * \param priority  Priority of the target host: lower value means more preferred.
 * \param weight    Relative weight for records with the same priority.
 * \param port      TCP / UDP port number of the service
 * \param name      The canonical hostname of the machine providing the service.
 */
void DnsMsgAdd_rdata_srv(DNSMESSAGE *msg, uint16_t priority, uint16_t weight, uint16_t port, char *name)
{
    uint8_t *tmp_buf = msg->buf;
    msg->buf += 2;
    DnsShort2Net(priority, &(msg->buf));
    DnsShort2Net(weight, &(msg->buf));
    DnsShort2Net(port, &(msg->buf));
    DnsShort2Net(Host2Label(msg, &(msg->buf), name) + 6, &tmp_buf);
}


/*!
 * \brief Append resource data types block: Raw data
 *
 * \param msg       DNS message
 * \param rdata     Pointer to the raw data buffer
 * \param rdlength  Length of the raw data block
 */
void DnsMsgAdd_rdata_raw(DNSMESSAGE *msg, uint8_t *rdata, uint16_t rdlength)
{
    if ((msg->buf - msg->packet) + rdlength > MAX_DNS_PACKET_LEN) {
        rdlength = 0;
    }
    DnsShort2Net(rdlength, &(msg->buf));
    memcpy(msg->buf, rdata, rdlength);
    msg->buf += rdlength;
}


/*!
 * \brief Generate the message packet to be send out.
 *
 * \return      Packet buffer
 */
uint8_t *DnsMsg2Pkt(DNSMESSAGE *msg)
{
    uint8_t c, *buf = msg->buf;
    msg->buf = msg->packet;
    DnsShort2Net(msg->id, &(msg->buf));

    if (msg->header.qr) {
        msg->buf[0] |= 0x80;
    }

    if ((c = msg->header.opcode)) {
        msg->buf[0] |= (c << 3);
    }
    if (msg->header.aa) {
        msg->buf[0] |= 0x04;
    }

    if (msg->header.tc) {
        msg->buf[0] |= 0x02;
    }

    if (msg->header.rd) {
        msg->buf[0] |= 0x01;
    }

    if (msg->header.ra) {
        msg->buf[1] |= 0x80;
    }

    if ((c = msg->header.z)) {
        msg->buf[1] |= (c << 4);
    }

    if (msg->header.rcode) {
        msg->buf[1] |= msg->header.rcode;
    }

    msg->buf += 2;

    DnsShort2Net(msg->qdcount, &(msg->buf));
    DnsShort2Net(msg->ancount, &(msg->buf));
    DnsShort2Net(msg->nscount, &(msg->buf));
    DnsShort2Net(msg->arcount, &(msg->buf));

    /* Restore the packet pointer. It is necessary for DnsMsgLen() */
    msg->buf = buf;

    /* Return the modified packet */
    return msg->packet;
}


/*!
 * \brief Calculate message packet length
 *
 * \return      Message packet length
 */

int DnsMsgLen(DNSMESSAGE *msg)
{
    if(msg->buf == 0) return 12;
    return msg->buf - msg->packet;
}

/*@}*/
