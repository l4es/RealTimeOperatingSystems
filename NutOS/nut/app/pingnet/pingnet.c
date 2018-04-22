/*
 * Copyright (C) 2012 by egnite GmbH
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

/*!
 * $Id$
 */

/*!
 * \example pingnet/pingnet.c
 *
 * Nut/Net does not offer ICMP sockets or any similar API to send out
 * ICMP echo requests and receive ICMP echo answers.
 *
 * This sample shows how to implement Ping by using NutIcmpReply()
 * to transmit and registering an IP callback to receive ICMP packets.
 * The program will regularly send echo requests to all nodes in the
 * local network and print out any status change on the serial port.
 */

/* Device specific definitions. */
#include <dev/board.h>

/* OS specific definitions. */
#include <sys/version.h>
#include <sys/confnet.h>
#include <sys/heap.h>
#include <sys/timer.h>
#include <sys/event.h>

/* Network specific definitions. */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/icmp.h>
#include <netinet/ip_icmp.h>
#include <pro/dhcp.h>

/* Standard C header files. */
#include <stdlib.h>
#include <stdio.h>
#include <io.h>


/* Version of this application sample. */
#define APP_VERSION     "1.0.0"

/* NETBUF queue to pass packets from handler to application. */
static HANDLE sign_queue;
static NETBUF *nbuf_queue;
static int nbuf_count;

/* Node status list. */
static uint8_t *upnodes;
#define NODE_UP(n)      { upnodes[(n) / 8] |= 1 << ((n) & 7); }
#define NODE_DOWN(n)    { upnodes[(n) / 8] &= ~(1 << ((n) & 7)); }
#define NODE_IS_UP(n)   ((upnodes[(n) / 8] & (1 << ((n) & 7))) != 0)


/*
 * Halt the application on fatal errors.
 */
static void FatalError(char *msg)
{
    while(1) {
        /* Print a message ... */
        puts(msg);
        /* Give Debugger a chancd to attach */
        NutSleep(1000);
    }
}

/*
 * This handler will be called on every incoming ICMP packet
 * and handle all echo replies.
 *
 * We must return 0, if we processed the packet, or -1 if we
 * are not interested. In the latter case the packet will be
 * passed to other registered handlers or handled by Nut/Net.
 */
static int IcmpCallback(NUTDEVICE * dev, NETBUF * nb)
{
    ICMPHDR *icp;

    /* ICMP header is in the transport part. */
    icp = (ICMPHDR *) nb->nb_tp.vp;
    /* Make sure we have a valid echo reply packet, ... */
    if (icp && nb->nb_tp.sz >= sizeof(ICMPHDR)
        && icp->icmp_type == ICMP_ECHOREPLY
        /* ... but limit the number of unprocessed packets. */
        && nbuf_count < 8) {
        /* Add the packet to the queue and wake up the main thread. */
        nb->nb_next = nbuf_queue;
        nbuf_queue = nb;
        NutEventPost(&sign_queue);

        return 0;
    }
    return -1;
}

/*
 * This function will send an ICMP echo request to the specified destionation.
 */
static int IcmpSendPing(uint32_t dest, uint16_t id, uint16_t seq, int len)
{
    NETBUF *nb;
    int i;
    uint8_t *dp;
    uint32_t spec;

    /* Create a new NETBUF. */
    nb = NutNetBufAlloc(NULL, NBAF_APPLICATION, len);
    if (nb) {
        /* Fill the data area with sample characters. */
        dp = (uint8_t *) nb->nb_ap.vp;
        for (i = 0; i < len; i++) {
            *dp++ = 'a' + (i & 15);
        }
        /* Set up the echo request ID and sequence number. */
        spec = id;
        spec <<= 16;
        spec |= seq;
        /* Send out the packet. The name of this function is misleading. */
        if (NutIcmpReply(ICMP_ECHO, 0, htonl(spec), dest, nb) == 0) {
            /* Funny Nut/Net expects us to release the packet only if
               it has been successfully processed. */
            NutNetBufFree(nb);
            return 0;
        }
    }
    /* Return an error. */
    return -1;
}


/*
 * Main application routine.
 *
 * Nut/OS automatically calls this entry after initialization.
 */
int main(void)
{
    uint32_t baud = 115200;
    /* IP of the local network in host byte order. */
    uint32_t net_ip;
    /* Currently scanned IP in host byte order. */
    uint32_t scan_ip;
    /* Currently scanned IP in network byte order. */
    uint32_t dest;
    /* Host portion mask in host byte order. */
    uint32_t host_mask;
    /* Current echo request sequence number. */
    uint16_t seq = 0;

    /*
     * Assign stdout to the DEBUG device.
     */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);

    /*
     * Print out our version information.
     */
    printf("\n\nNut/OS %s\n", NutVersionString());
    printf("PingNet %s " __DATE__ " " __TIME__ "\n", APP_VERSION);

    /*
     * Configure the network interface. It is assumed, that
     * we got a valid configuration in non-volatile memory.
     *
     * For alternatives see
     * http://www.ethernut.de/nutwiki/Network_Configuration
     */
    printf("Configure %s...", DEV_ETHER_NAME);
    if (NutRegisterDevice(&DEV_ETHER, 0, 0)) {
        FatalError("failed");
    }
    if (NutDhcpIfConfig("eth0", 0, 60000)) {
        FatalError("no valid network configuration");
    }
    printf("%s ready\n", inet_ntoa(confnet.cdn_ip_addr));
    /* Some Ethernet device drivers seem to return from initialization
       even if they are not fully up and running. Typically this is no
       problem, because upper layers will do retries. However, in our
       special case we'd lose some nodes during the first scan. So just
       take a short nap to give the driver some more time. */
    NutSleep(2000);

    /*
     * Register an ICMP callback function to inspect all incoming
     * ICMP packets.
     */
    if (NutRegisterIpHandler(IPPROTO_ICMP, IcmpCallback)) {
        FatalError("failed to register ICMP callback");
    }

    /*
     * Determine the host portion mask and the IP of the local
     * network from our IP configuration. Note, that Nut/Net
     * stores all IP address items in network byte order.
     */
    host_mask = ~ntohl(confnet.cdn_ip_mask);
    net_ip = ntohl(confnet.cdn_ip_addr) & ~host_mask;

    /*
     * Allocate a bit field to store the state of all possible
     * nodes of our local network. Limit this to 2^16 (class B)
     * and also reject point to point configurations.
     */
    if (host_mask > 0xFFFF || host_mask < 0x0003) {
        FatalError("Bad network size");
    }
    upnodes = calloc((host_mask + 1) / 8, 1);
    if (upnodes == NULL) {
        FatalError("out of memory");
    }

    /*
     * Scan the whole network endlessly.
     */
    for (;;) {
        int retries = 0;

        seq++;
        scan_ip = net_ip;
        /*
         * Scan node by node.
         */
        for (;;) {
            int got = 0;

            /* If this is not a retry, move to the next possible node
               address. */
            if (retries == 0) {
                scan_ip++;
            }
            /* All nodes processed if we reached the broadcast address. */
            if ((scan_ip & host_mask) == host_mask) {
                break;
            }
            /* Send an echo request to the current IP (network byte order). */
            dest = htonl(scan_ip);
            printf("\r%s ", inet_ntoa(dest));
            if (IcmpSendPing(dest, 1, seq, 32) == 0) {
                /* Wait until our ICMP handler signals new packets. */
                while (got == 0 && NutEventWait(&sign_queue, 100) == 0) {
                    /* Inspect all queued packets. */
                    while (nbuf_queue) {
                        NETBUF *nb;
                        IPHDR *ip = (IPHDR *) nbuf_queue->nb_nw.vp;

                        /* Check if this packet is from the currently scanned
                           node address. We may additionally check ID and
                           sequence number, but actually anything from that
                           interface will be just fine to mark it as up. */
                        got += (ip->ip_src == dest);
                        nb = nbuf_queue;
                        nbuf_queue = nb->nb_next;
                        NutNetBufFree(nb);
                    }
                }
                if (NODE_IS_UP(scan_ip & host_mask)) {
                    /* If the node has been up and is now down, then
                       do a few retries first to be sure that it is
                       really not responding anymore. */
                    if (got == 0) {
                        if (retries < 3) {
                            retries++;
                        } else {
                            retries = 0;
                            NODE_DOWN(scan_ip & host_mask);
                            puts("down");
                        }
                    }
                }
                else if (got) {
                    /* New node detected. */
                    NODE_UP(scan_ip & host_mask);
                    puts("up");
                }
            } else {
                /* Failed to send out the request. */
                puts("ICMP transmit error");
            }
        }
        /* Sleep one minute before scanning the network again. */
        printf("\rSleeping                ");
        NutSleep(60000);
    }
    /* Never reached, but required to suppress compiler warning. */
    return 0;
}
