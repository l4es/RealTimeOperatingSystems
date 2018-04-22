/*
 * Copyright (C) 2001-2006 by egnite Software GmbH
 * Copyright (C) 2009 by egnite GmbH
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

/*
 * $Id: webdemo.c 5545 2014-01-12 19:47:48Z olereinhardt $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

#include <stdio.h>
#include <string.h>

#include <dev/nicrtl.h>
#include <dev/lanc111.h>
#include <dev/urom.h>

#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/confnet.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <net/route.h>
#include <pro/httpd.h>
#include <pro/dhcp.h>
#include <sys/atom.h>

#include "basemon.h"
#include "webdemo.h"

#ifdef __AVR__ /* TODO: This function is called from webserver thread!!! Broken for other CPUs? */
static char *states[] = { "TRM",
    "<FONT COLOR=#CC0000>RUN</FONT>",
    "<FONT COLOR=#339966>RDY</FONT>",
    "SLP"
};


/*
 * Thread list CGI.
 */

static int ShowThreads(FILE * stream, REQUEST * req)
{
    NUTTHREADINFO *tdp = nutThreadList;
    static prog_char head_P[] = "<HTML><HEAD><TITLE>Nut/OS Threads</TITLE>" "</HEAD><BODY><h1>Nut/OS Threads</h1>\r\n";
    static prog_char ttop_P[] = "<TABLE BORDER><TR><TH>Handle</TH>"
        "<TH>Name</TH><TH>Priority</TH>"
        "<TH>Status</TH><TH>Event<BR>Queue</TH>" "<TH>Timer</TH><TH>Stack-<BR>pointer</TH>" "<TH>Free<BR>Stack</TH></TR>\r\n";
    static prog_char tbot_P[] = "</TABLE></BODY></HTML>";
    static prog_char tfmt_P[] = "<TR><TD>%04X</TD><TD>%s</TD><TD>%u</TD>"
        "<TD>%s</TD><TD>%04X</TD><TD>%04X</TD>" "<TD>%04X</TD><TD>%u</TD><TD>%s</TD></TR>\r\n";

    NutHttpSendHeaderTop(stream, req, 200, "Ok");
    NutHttpSendHeaderBottom(stream, req, "text/html", -1);

    fputs_P(head_P, stream);

    fputs_P(ttop_P, stream);
    while (tdp) {
        fprintf_P(stream, tfmt_P, (unsigned int)tdp, tdp->td_name, tdp->td_priority,
                  states[tdp->td_state], (unsigned int)tdp->td_queue, (unsigned int)tdp->td_timer,
                  (unsigned int)tdp->td_sp,
                  (unsigned int) tdp->td_sp - (unsigned int) tdp->td_memory, *((uint32_t *) tdp->td_memory) != DEADBEEF ? "Corr" : "OK");
        tdp = tdp->td_next;
    }
    fputs_P(tbot_P, stream);
    fflush(stream);

    return 0;
}

/*
 * Timer list CGI.
 */
static int ShowTimer(FILE * stream, REQUEST * req)
{
    NUTTIMERINFO *tnp;
    uint32_t ticks_left;
    uint32_t crystal;
    static prog_char head_P[] = "<HTML><HEAD><TITLE>Nut/OS Timers</TITLE>" "</HEAD><BODY>";
    static prog_char cfmt_P[] = "\r\nCPU running at %u.%04u MHz<br>\r\n";
    static prog_char ttop_P[] = "<TABLE BORDER><TR><TH>Handle</TH>"
        "<TH>Countdown</TH><TH>Tick Reload</TH>" "<TH>Callback<BR>Address</TH>" "<TH>Callback<BR>Argument</TH></TR>\r\n";
    static prog_char tfmt_P[] = "<TR><TD>%04X</TD><TD>%lu</TD><TD>%lu</TD>" "<TD>%04X</TD><TD>%04X</TD></TR>\r\n";
    static prog_char tbot_P[] = "</TABLE></BODY></HTML>";

    NutHttpSendHeaderTop(stream, req, 200, "Ok");
    NutHttpSendHeaderBottom(stream, req, "text/html", -1);


    fputs_P(head_P, stream);

    crystal = NutGetCpuClock();
    fprintf_P(stream, cfmt_P, (int) (crystal / 1000000UL), (int) ((crystal - (crystal / 1000000UL) * 1000000UL) / 100));

    if ((tnp = nutTimerList) != 0) {
        fputs_P(ttop_P, stream);
        ticks_left = 0;
        while (tnp) {
            ticks_left += tnp->tn_ticks_left;
            fprintf_P(stream, tfmt_P, (unsigned int)tnp, ticks_left, tnp->tn_ticks, (unsigned int)tnp->tn_callback, (unsigned int)tnp->tn_arg);
            tnp = tnp->tn_next;
        }
    }

    fputs_P(tbot_P, stream);
    fflush(stream);

    return 0;
}

/*
 * Socket list CGI.
 */
static int ShowSockets(FILE * stream, REQUEST * req)
{
    extern TCPSOCKET *tcpSocketList;
    TCPSOCKET *ts;
    static prog_char head_P[] = "<HTML><HEAD><TITLE>Show Threads</TITLE>"
        "</HEAD><BODY><TABLE BORDER><TR>" "<TH>Handle</TH><TH>Type</TH><TH>Local</TH>" "<TH>Remote</TH><TH>Status</TH></TR>\r\n";
    static prog_char fmt1_P[] = "<TR><TD>%04X</TD><TD>TCP</TD><TD>%s:%u</TD>";
    static prog_char fmt2_P[] = "<TD>%s:%u</TD><TD>";
    static prog_char estb_P[] = "<FONT COLOR=#CC0000>ESTABL</FONT>";
    static prog_char tbot_P[] = "</TABLE></BODY></HTML>\r\n";

    NutHttpSendHeaderTop(stream, req, 200, "Ok");
    NutHttpSendHeaderBottom(stream, req, "text/html", -1);

    fputs_P(head_P, stream);

    /* TODO: Its a bad idea to loop through the socket list while there are scheduling points in this loop.
             It may lead to a race condition.
     */
    NutEnterCritical();
    for (ts = tcpSocketList; ts; ts = ts->so_next) {
        fprintf_P(stream, fmt1_P, (unsigned int)ts, inet_ntoa(ts->so_local_addr), ntohs(ts->so_local_port));
        fprintf_P(stream, fmt2_P, inet_ntoa(ts->so_remote_addr), ntohs(ts->so_remote_port));
        switch (ts->so_state) {
        case TCPS_LISTEN:
            fputs("LISTEN", stream);
            break;
        case TCPS_SYN_SENT:
            fputs("SYNSENT", stream);
            break;
        case TCPS_SYN_RECEIVED:
            fputs("SYNRCVD", stream);
            break;
        case TCPS_ESTABLISHED:
            fputs_P(estb_P, stream);
            break;
        case TCPS_FIN_WAIT_1:
            fputs("FINWAIT1", stream);
            break;
        case TCPS_FIN_WAIT_2:
            fputs("FINWAIT2", stream);
            break;
        case TCPS_CLOSE_WAIT:
            fputs("CLOSEWAIT", stream);
            break;
        case TCPS_CLOSING:
            fputs("CLOSING", stream);
            break;
        case TCPS_LAST_ACK:
            fputs("LASTACK", stream);
            break;
        case TCPS_TIME_WAIT:
            fputs("TIMEWAIT", stream);
            break;
        case TCPS_CLOSED:
            fputs("CLOSED", stream);
            break;
        case TCPS_DESTROY:
            fputs("DESTROY", stream);
            break;
        default:
            fputs("?UNK?", stream);
            break;
        }
        fputs("</TD></TR>\r\n", stream);
    }
    NutExitCritical();
    fputs_P(tbot_P, stream);
    fflush(stream);

    return 0;
}

static void DoCheckboxes(FILE * stream, char * name, uint8_t val)
{
    uint8_t i;
    static prog_char ttop_P[] = "<tr><td>%s</td>";
    static prog_char tfmt_P[] = "<td><input type=\"checkbox\"" " name=\"%s\" value=\"%u\" ";
    static prog_char tchk_P[] = " checked=\"checked\"";

    fprintf_P(stream, ttop_P, name);
    for (i = 8; i-- > 0;) {
        fprintf_P(stream, tfmt_P, name, i);
        if (val & _BV(i))
            fputs_P(tchk_P, stream);
        fputs("></td>\r\n", stream);
    }
    fputs("</tr>\r\n", stream);
}

/*
 * Socket list CGI.
 */
static int ShowPorts(FILE * stream, REQUEST * req)
{
    static prog_char ttop_P[] = "<HTML><HEAD><TITLE>Show Ports</TITLE>"
        "</HEAD><BODY>"
        "<form action=\"cgi-bin/setports.cgi\" "
        "enctype=\"text/plain\"> <TABLE BORDER>"
        "<tr><td>Bit</td><td>7</td><td>6</td>" "<td>5</td><td>4</td><td>3</td><td>2</td>" "<td>1</td><td>0</td></tr>\r\n";
#if defined (__AVR__)
    static prog_char trow_P[] = "<tr></tr>";
#endif
    static prog_char tbot_P[] = "</table></form>\r\n</body>\r\n</html>";

    NutHttpSendHeaderTop(stream, req, 200, "Ok");
    NutHttpSendHeaderBottom(stream, req, "text/html", -1);

    fputs_P(ttop_P, stream);

#if defined (__AVR__)
    DoCheckboxes(stream, "DDRA", inb(DDRA));
    DoCheckboxes(stream, "PINA", inb(PINA));
    DoCheckboxes(stream, "PORTA", inb(PORTA));

    fputs_P(trow_P, stream);
    DoCheckboxes(stream, "DDRB", inb(DDRB));
    DoCheckboxes(stream, "PINB", inb(PINB));
    DoCheckboxes(stream, "PORTB", inb(PORTB));

    fputs_P(trow_P, stream);
    DoCheckboxes(stream, "PORTC", inb(PORTC));

    fputs_P(trow_P, stream);
    DoCheckboxes(stream, "DDRD", inb(DDRD));
    DoCheckboxes(stream, "PIND", inb(PIND));
    DoCheckboxes(stream, "PORTD", inb(PORTD));

    fputs_P(trow_P, stream);
    DoCheckboxes(stream, "DDRE", inb(DDRE));
    DoCheckboxes(stream, "PINE", inb(PINE));
    DoCheckboxes(stream, "PORTE", inb(PORTE));

    fputs_P(trow_P, stream);
    DoCheckboxes(stream, "PINF", inb(PINF));
#endif

    fputs_P(tbot_P, stream);
    fflush(stream);

    return 0;
}

#endif /* #ifdef __AVR__ */

THREAD(WebDemo, arg)
{
#ifdef __AVR__
    TCPSOCKET *sock;
    FILE *stream;
    IFNET *ifn = 0;
    uint32_t ip_addr;             /* ICCAVR bugfix */
    static prog_char netfail_P[] = "\nFailed to configure network " /* */
                                   "interface: Ethernut stopped!\n\x07";
    static prog_char dhcpfail_P[] = "\nFailed to configure network " /* */
                                    "via DHCP: Ethernut stopped!\n\x07";
    /*
     * Register Realtek controller at address 8300 hex
     * and interrupt 5.
     */
#if defined(__AVR__)
    if(nic == 1)
        NutRegisterDevice(&devEth0, 0x8300, 5);
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega2561__)
    else
        NutRegisterDevice(&devSmsc111, 0, 0);
#endif
#endif

    /*
     * Configure lan interface.
     */
    ip_addr = inet_addr(my_ip);
    if (ip_addr) {
        if (NutNetIfConfig("eth0", my_mac, ip_addr, inet_addr(my_mask))) {
            printf_P(netfail_P);
            if (uart_bs >= 0) {
                for (;;)
                    NutSleep(1000);
            } else {
#if defined (__AVR__)
                asm("cli");
                asm("call 0");
#endif
            }
        }
    } else if (NutDhcpIfConfig("eth0", my_mac, 60000)) {
        printf_P(dhcpfail_P);
        if (uart_bs >= 0) {
            for (;;)
                NutSleep(1000);
        } else {
#if defined (__AVR__)
            asm("cli");
            asm("call 0");
#endif
        }
    }

    printf("MAC  %02X-%02X-%02X-%02X-%02X-%02X\nIP   %s",
                   confnet.cdn_mac[0], confnet.cdn_mac[1],
                   confnet.cdn_mac[2], confnet.cdn_mac[3], confnet.cdn_mac[4], confnet.cdn_mac[5],
                   inet_ntoa(confnet.cdn_ip_addr));
    printf("\nMask %s", inet_ntoa(confnet.cdn_ip_mask));

    /*
     * Add optional default route.
     */
    if (inet_addr(my_ip) && inet_addr(my_gate)) {
        if (uart_bs >= 0)
            printf("\nAdd gateway %s", my_gate);
#if defined (__AVR__)
        NutIpRouteAdd(0, 0, inet_addr(my_gate), &devEth0);
#endif
    }

    else if (confnet.cdn_gateway && uart_bs >= 0)
        printf("\nGate %s", inet_ntoa(confnet.cdn_gateway));
#if defined (__AVR__)
    if(nic == 1)
        ifn = (IFNET *) (devEth0.dev_icb);
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega2561__)
    else
        ifn = (IFNET *) (devSmsc111.dev_icb);
#endif
#endif
    printf("\nHTTP server running. URL http://%s/\n", inet_ntoa(ifn->if_local_ip));

    /*
     * Register our device for the file system.
     */
    NutRegisterDevice(&devUrom, 0, 0);
    /*
     * Register CGI routines.
     */
    NutRegisterCgi("threads.cgi", ShowThreads);
    NutRegisterCgi("timer.cgi", ShowTimer);
    NutRegisterCgi("sockets.cgi", ShowSockets);
    NutRegisterCgi("ports.cgi", ShowPorts);
    /*
     * Now loop endless for connections.
     */
    for (;;) {
        /*
         * Create a socket.
         */
        sock = NutTcpCreateSocket();
        /*
         * Listen on port 80. If we return,
         * we got a client.
         */
        NutTcpAccept(sock, 80);
        /*
         * Create a stream from the socket, so we can use stdio.
         */
        stream = _fdopen((int) sock, "r+b");
        /*
         * Process http request.
         */
        NutHttpProcessRequest(stream);
        /*
         * Destroy our device.
         */
        fclose(stream);
        /*
         * Close our socket.
         */
        NutTcpCloseSocket(sock);
    }
#endif
    while(1); /* Keep compiler happy */
}

