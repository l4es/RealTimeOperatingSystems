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
 */

/*!
 * $Id$
 */

#ifdef NUT_OS
/* Include Nut/OS specific headers. */
#include <sys/version.h>
#include <dev/board.h>
#include <sys/confnet.h>
#include <dev/urom.h>
#include <arpa/inet.h>
#include <pro/dhcp.h>
#endif

#include <pro/uhttp/mediatypes.h>
#include <stdio.h>

/*!
 * \example uhttpd_tiny/httpd_tiny.c
 *
 * Minimalist HTTP server sample.
 *
 * This is a simple HTTP server based on the MicroHTTP library. It
 * should compile and run without modification on any operating system
 * that is supported by the MicroHTTP library.
 *
 * It does not include any redirection. Thus, it is required to specify
 * the full URL in the browser request, e.g. 127.0.0.1/index.html.
 *
 * Only a single server thread is running. Concurrent requests may
 * fail.
 *
 * When used on Nut/OS, make sure, that the macro NUT_OS has been
 * defined on the compile command, which can done by adding
 * HWDEF += -DNUT_OS
 * either in the Makefile or in app/UserConf.mk. It is further required,
 * that a valid network configuration exists in non-volatile memory. You
 * can run the sample application editconf to check this.
 */

/*
 * Target specific initialization.
 */
static int TargetInit(void)
{
    /* On Nut/OS we need to register some drivers, assign stdout to the
     * default console and configure the network interface. */
#ifdef NUT_OS
    /* Register the default console device. */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    /* Assign the console device to stdout. */
    freopen(DEV_CONSOLE.dev_name, "w", stdout);

    /* Register the UROM file system. */
    printf("\nRegister file system...");
    if (NutRegisterDevice(&devUrom, 0, 0)) {
        return -1;
    }

    /* Register the default network device. */
    printf("OK\nRegister %s...", DEV_ETHER_NAME);
    if (NutRegisterDevice(&DEV_ETHER, 0, 0)) {
        return -1;
    }

    /* Configure the network interface. */
    printf("OK\nConfigure %s...", DEV_ETHER_NAME);
    if (NutDhcpIfConfig(DEV_ETHER_NAME, NULL, 60000)) {
        return -1;
    }
    puts(inet_ntoa(confnet.cdn_ip_addr));
#endif
    return 0;
}

int main(void)
{
    /* Target-specific initialization. */
    if (TargetInit()) {
        puts("failed");
        for (;;);
    }

    /* Write a banner to our console. */
    puts("\nTiny uHTTP sample running");

    /* Initialize the TCP socket interface. */
    StreamInit();

    /* Register media type defaults. These are configurable in
     * include/cfg/http.h or the Nut/OS Configurator. */
    MediaTypeInitDefaults();

    /* Wait for a client (browser) and handle its request. This function
       will only return on unrecoverable errors. */
    StreamClientAccept(HttpdClientHandler, NULL);

    /* Typically this point will be never reached. */
    return 0;
}
