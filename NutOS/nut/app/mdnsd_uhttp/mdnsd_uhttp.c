/*
 * Copyright (C) 2014 Ole Reinhardt <ole.reinhardt@embedded-it.de>
 *              
 * Based on uhttp_tiny sample, (C) 2012 Egnite GmbH
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
 * $Id$
 */

/* Include Nut/OS specific headers. */
#include <sys/version.h>
#include <dev/board.h>
#include <sys/confnet.h>
#include <sys/heap.h>
#include <dev/urom.h>
#include <arpa/inet.h>
#include <pro/dhcp.h>

#include <pro/uhttp/modules/mod_redir.h>
#include <pro/uhttp/mediatypes.h>
#include <pro/mdnsd/mdnsd.h>

#include <stdio.h>

#define HOSTNAME "ethernut"

#define MIN_FREE_RAM  (16*1024)

/*!
 * \example mdnsd_uhttp/mdnsd_uhttp.c
 *
 * Multicast DNS server sample.
 *
 * This demo runs a simple HTTP server (based on uhttp_tiny sample) 
 * and starts the multicast DNS server in background. It promotes
 * its own services (webserver) and IP address using the multicast DNS
 * protocol, also known as "Bonjour (Apple)", "Avahi (Linux)" or 
 * "Zeroconf"
 *
 * You can connect to the webserver by just entering 
 *
 * http://ethernut.local 
 * 
 * As URL into your browser. This should at least work on most modern
 * operating systems out of the box. If not, multicast DNS support
 * needs to be installed or enabled.
 *
 * The Webserver itself is very simplistic. Only a single server 
 * thread is running. Concurrent requests may fail.
 */

void error(char *err_txt)
{
    printf("ERROR: %s\n", err_txt);
    fflush(stdout);
    for (;;);
}

int main(void)
{
    /* Register the default console device. */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    /* Assign the console device to stdout. */
    freopen(DEV_CONSOLE.dev_name, "w", stdout);

    puts("\nMulticast DNS mini HTTP deamon demo\n");

    /* Register the UROM file system. */
    printf("\nRegister file system...");
    if (NutRegisterDevice(&devUrom, 0, 0)) {
        error("Registering filesystem failed");
    }

    /* Register the default network device. */
    printf("OK\nRegister %s...", DEV_ETHER_NAME);
    if (NutRegisterDevice(&DEV_ETHER, 0, 0)) {
        error("Registering Ethernet device failed");
    }

    /* Configure the network interface. */
    printf("OK\nConfigure %s...", DEV_ETHER_NAME);
    if (NutDhcpIfConfig(DEV_ETHER_NAME, NULL, 60000)) {
        error("Configuring Network using DHCP failed");
    }
    puts(inet_ntoa(confnet.cdn_ip_addr));

    if (NutHeapAvailable() < MIN_FREE_RAM) {
        error("This demo needs more RAM. The MDNSd should have > 16K Free ram available");
    }

    /* Write a banner to our console. */
    puts("\nMulticast DNS sample running");

    /* ------- MDNS Deamon -------- */

    MdnsDeamonStart(DEV_ETHER_NAME, confnet.cdn_ip_addr, 80, HOSTNAME, "_http._tcp", 0, NULL);
    printf("MDNS: Announcing as %s.local\n", HOSTNAME);

    /* ------- Webserver -------- */

    /* Register media type defaults. These are configurable in
     * include/cfg/http.h or the Nut/OS Configurator. */
    MediaTypeInitDefaults();

    /* Register redirection to /index.html if no file is specified */       
    HttpRegisterRedir("", "/index.html", 301);

    /* Wait for a client (browser) and handle its request. This function
       will only return on unrecoverable errors. */
    StreamClientAccept(HttpdClientHandler, NULL);

    /* Typically this point will be never reached. */
    return 0;
}
