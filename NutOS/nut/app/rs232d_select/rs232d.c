/*!
 * Copyright (C) 2013 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 * Copyright (C) 2001-2005 by egnite Software GmbH. 
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
 * $Id: rs232d.c 4999 2013-02-22 10:59:22Z olereinhardt $
 */

/*!
 * \example rs232d_select/rs232d_select.c
 *
 * Simple RS232 server implemented using select(). No extra threads are 
 * needed as we can wait for activity on both filedescriptors (uart and 
 * socket) at the same time using the select() call.
 *
 * Use a serial cable to connect the RS232 port
 * of the Ethernut Board with a COM port of a PC. Start a terminal
 * program and a telnet client on the PC. Telnet should connect to
 * the Ethernut Board.
 *
 * Characters typed in the telnet window will appear in the terminal
 * program window and vice versa. Baudrate is 9600.
 *
 */

#include <dev/board.h>

#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/timer.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/confnet.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>

#include <arpa/inet.h>
#include <net/if_var.h>
#include <pro/dhcp.h>

#define BUFFERSIZE  128
#define TCPPORT     23

#define max(a,b) ((a)>=(b)?(a):(b))

typedef struct {
    FILE *cd_rs232;
    FILE *cd_tcpip;
} CHANNEL;

/*
 * Transfer data from input stream to output stream. Wait for incomming data 
 * on both streams using the select() function.
 */
void StreamCopy(FILE * stream_sock, FILE * stream_uart)
{
    fd_set  rfds;
    int     rc;
    char   *buff;
    size_t  cnt;

    buff = malloc(BUFFERSIZE);
    for(;;) {
        FD_ZERO(&rfds);
        /* Request read notification for the socket and the uart stream*/
        FD_SET(_fileno(stream_sock), &rfds);
        FD_SET(_fileno(stream_uart), &rfds);
        
        /* Wait for notification on the rfds descriptor set. Wait with infinite timeout.
           If the socket is closed by the foreign host, select will return a notification
           of stream_sock as well, but fread will return 0, which indicates the closed socket. 
         */
        rc = select(max(_fileno (stream_sock), _fileno (stream_uart)) + 1, &rfds, NULL, NULL, NULL);
        if (rc > 0) {
            if (FD_ISSET(_fileno(stream_sock), &rfds)) {
                /* Copy the data from socket to uart. Return if an error occured during read or write. */
                if ((cnt = fread(buff, 1, BUFFERSIZE, stream_sock)) == 0)
                    break;
                if ((cnt = fwrite(buff, 1, cnt, stream_uart)) == 0)
                    break;
                if (fflush(stream_uart))
                    break;
            }
            if (FD_ISSET(_fileno(stream_uart), &rfds)) {
                /* Copy the data from uart to socket. Return if an error occured during read or write. */
                if ((cnt = fread(buff, 1, BUFFERSIZE, stream_uart)) == 0)
                    break;
                if ((cnt = fwrite(buff, 1, cnt, stream_sock)) == 0)
                    break;
                if (fflush(stream_sock))
                    break;
            }
        } else
        if (rc < 0) {
            /* An error occured during select. Free allocated buffer and return... */
            free(buff);
            return;
        }
    }
    /* Free allocated buffer */
    free(buff);
}

/*
 * Main application routine.
 *
 * Nut/OS automatically calls this entry after initialization.
 */
int main(void)
{
    TCPSOCKET *sock;
    CHANNEL cd;
    uint32_t baud = 9600;

    /*
     * Register our devices.
     */
    NutRegisterDevice(&DEV_UART, 0, 0);
#ifndef DEV_ETHER
    for (;;);
#else
    NutRegisterDevice(&DEV_ETHER, 0x8300, 5);

    /*
     * Setup the uart device.
     */
    cd.cd_rs232 = fopen(DEV_UART_NAME, "r+b");
    _ioctl(_fileno(cd.cd_rs232), UART_SETSPEED, &baud);
    
    /*
     * Setup the ethernet device. Try DHCP first. If this is
     * the first time boot with empty EEPROM and no DHCP server
     * was found, use hardcoded values.
     */
    if (NutDhcpIfConfig(DEV_ETHER_NAME, 0, 60000)) {
        /* No valid EEPROM contents, use hard coded MAC. */
        uint8_t my_mac[] = { 0x00, 0x06, 0x98, 0x20, 0x00, 0x00 };

        if (NutDhcpIfConfig("eth0", my_mac, 60000)) {
            /* No DHCP server found, use hard coded IP address. */
            uint32_t ip_addr = inet_addr("192.168.192.100");
            uint32_t ip_mask = inet_addr("255.255.255.0");

            NutNetIfConfig("eth0", my_mac, ip_addr, ip_mask);
            /* If not in a local network, we must also call
               NutIpRouteAdd() to configure the routing. */
        }
    }

    /*
     * Now loop endless for connections.
     */
    for (;;) {
        /*
         * Create a socket and listen for a client.
         */
        sock = NutTcpCreateSocket();
        NutTcpAccept(sock, TCPPORT);

        /*
         * Open a stdio stream assigned to the connected socket.
         */
        cd.cd_tcpip = _fdopen((int) sock, "r+b");

        /*
         * Call Socket <-> RS232 copy routine. On return we will be
         * disconnected again.
         */
        StreamCopy(cd.cd_tcpip, cd.cd_rs232);

        /*
         * Close the stream.
         */
        fclose(cd.cd_tcpip);

        /*
         * Close our socket.
         */
        NutTcpCloseSocket(sock);
    }
#endif
    return 0;
}
