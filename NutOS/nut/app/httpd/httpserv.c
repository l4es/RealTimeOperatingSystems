/*
 * Copyright (C) 2009, 2013 by egnite GmbH
 * Copyright (C) 2001-2004 by egnite Software GmbH
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
 * $Id: httpserv.c 6617 2017-03-21 17:25:39Z u_bonnes $
 */

/*!
 * \example httpd/httpserv.c
 *
 * This multithreaded webserver demo uses the old Nut/Net HTTP Library.
 * Note, that there is a more recent microHTTP API (uhttp) available.
 *
 * You will find the main() function at the bottom.
 */

/* Configuration headers. */
#include <cfg/os.h>
#include <cfg/http.h>

#include <dev/board.h>
#include <dev/urom.h>

#include <sys/version.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/timer.h>
#include <sys/confnet.h>
#include <sys/socket.h>

#include <arpa/inet.h>

#include <pro/httpd.h>
#include <pro/dhcp.h>
#include <pro/ssi.h>
#include <pro/asp.h>

#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>

/* Server thread stack size. */
#ifndef HTTPD_SERVICE_STACK
#ifndef NUT_THREAD_MAINSTACK
#define NUT_THREAD_MAINSTACK 1024
#endif
#define HTTPD_SERVICE_STACK NUT_THREAD_MAINSTACK * NUT_THREAD_STACK_MULT + NUT_THREAD_STACK_ADD
#endif

/* ICCAVR Demo is limited. Try to use the bare minimum. */
#if defined(__IMAGECRAFT__)
#define EXCLUDE_ASP
#define EXCLUDE_SSI
#endif

static char *html_mt = "text/html";

/*
 * Halt on fatal errors.
 */
static void Fatal(char *msg) NUT_NORETURN_FUNC;
void Fatal(char *msg)
{
    while(1) {
        /* Print a message ... */
        puts(msg);
        /* Give Debugger a chancd to attach */
        NutSleep(1000);
    }
}

/*
 * Saves some duplicate code.
 */
static void StartPage(FILE *stream, REQUEST * req, const char *title)
{
    /* These useful API calls create a HTTP response for us. */
    NutHttpSendHeaderTop(stream, req, 200, "Ok");
    NutHttpSendHeaderBottom(stream, req, html_mt, -1);

    fprintf(stream, "<HTML><HEAD><TITLE>%s</TITLE></HEAD><BODY><H1>%s</H1>\r\n", title, title);
}

static void StartTable(FILE *stream, ...)
{
    va_list ap;
    char *cp;

    fputs("<TABLE BORDER><TR>", stream);
    va_start(ap, stream);
    while ((cp = va_arg(ap, char *)) != NULL) {
        fprintf(stream, "<TH>%s</TH>", cp);
    }
    va_end(ap);
    fputs("</TR>\r\n", stream);
}

/*
 * CGI Sample: Show request parameters.
 *
 * See httpd.h for REQUEST structure.
 *
 * This routine must have been registered by NutRegisterCgi() and is
 * automatically called by NutHttpProcessRequest() when the client
 * request the URL 'cgi-bin/test.cgi'.
 */
static int ShowQuery(FILE *stream, REQUEST * req)
{
    static const char *method_name[4] = { "?", "GET", "POST", "HEAD" };

    /* Send HTML header. */
    StartPage(stream, req, "Parameters");

    /* Send request parameters. */
    fprintf(stream, "Method: %s<BR>\r\n", method_name[req->req_method & 3]);
    fprintf(stream, "Version: HTTP/%d.%d<BR>\r\n", req->req_version / 10, req->req_version % 10);
    fprintf(stream, "Content length: %ld<BR>\r\n", req->req_length);
    if (req->req_url)
        fprintf(stream, "URL: %s<BR>\r\n", req->req_url);
    if (req->req_query)
        fprintf(stream, "Argument: %s<BR>\r\n", req->req_query);
    if (req->req_type)
        fprintf(stream, "Content type: %s<BR>\r\n", req->req_type);
    if (req->req_cookie)
        fprintf(stream, "Cookie: %s<BR>\r\n", req->req_cookie);
    if (req->req_auth)
        fprintf(stream, "Auth info: %s<BR>\r\n", req->req_auth);
    if (req->req_agent)
        fprintf(stream, "User agent: %s<BR>\r\n", req->req_agent);

    /* Send HTML footer and flush output buffer. */
    fputs("</BODY></HTML>", stream);
    fflush(stream);

    return 0;
}

/*
 * CGI Sample: Show list of threads.
 *
 * This routine must have been registered by NutRegisterCgi() and is
 * automatically called by NutHttpProcessRequest() when the client
 * request the URL 'cgi-bin/threads.cgi'.
 */
static int ShowThreads(FILE * stream, REQUEST * req)
{
    static char *thread_states[4] = {
        "TRM",
        "<FONT COLOR=#CC0000>RUN</FONT>",
        "<FONT COLOR=#339966>RDY</FONT>",
        "SLP"
    };
    NUTTHREADINFO *tdp;
    NUTTHREADINFO *tlist;
    int i;
    int n;

    /* Take a snapshot of our thread list. */
    for (tdp = nutThreadList, n = 0; tdp; tdp = tdp->td_next, n++);
    tlist = calloc(n, sizeof(NUTTHREADINFO));
    if (tlist == NULL) {
        return -1;
    }
    for (tdp = nutThreadList, i = 0; tdp && i < n; tdp = tdp->td_next, i++) {
        memcpy(&tlist[i], tdp, sizeof(NUTTHREADINFO));
    }

    /* Send HTML header. */
    StartPage(stream, req, "Threads");
    StartTable(stream,
        "Name",
        "Priority",
        "Status",
        "Event<BR>Queue",
        "Timer",
        "Stack-<BR>pointer",
        "Free<BR>Stack",
        NULL);

    /* Send table with list of threads. */
    for (i = 0; i < n; i++) {
        fprintf(stream,
            "<TR><TD>%s</TD>" /* Name */
            "<TD>%u</TD>" /* Priority */
            "<TD>%s</TD>" /* Status */
            "<TD>%08zX</TD>" /* Event queue */
            "<TD>%08zX</TD>" /* Timer */
            "<TD>%08zX</TD>" /* Stack pointer */
            "<TD>%zu %s</TD></TR>\r\n", /* Stack available */
            tlist[i].td_name,
            tlist[i].td_priority,
            thread_states[tlist[i].td_state],
            (size_t) tlist[i].td_queue,
            (size_t) tlist[i].td_timer,
            (size_t) tlist[i].td_sp,
            ((size_t) tlist[i].td_sp - (size_t) tlist[i].td_memory),
            *((uint32_t *) tlist[i].td_memory) != DEADBEEF ? "Corrupted" : "");
    }

    /* Release the thread list copy. */
    free(tlist);

    /* Send HTML footer and flush output buffer. */
    fputs("</TABLE></BODY></HTML>", stream);
    fflush(stream);

    return 0;
}

/*
 * CGI Sample: Show list of timers.
 *
 * This routine must have been registered by NutRegisterCgi() and is
 * automatically called by NutHttpProcessRequest() when the client
 * request the URL 'cgi-bin/timers.cgi'.
 */
static int ShowTimers(FILE * stream, REQUEST * req)
{
    NUTTIMERINFO *tnp;
    NUTTIMERINFO *tlist;
    uint32_t ticks_left;
    int i;
    int n;

    /* Take a snapshot of our timer list. */
    for (tnp = nutTimerList, n = 0; tnp; tnp = tnp->tn_next, n++);
    tlist = calloc(n, sizeof(NUTTIMERINFO));
    if (tlist == NULL) {
        return -1;
    }
    for (tnp = nutTimerList, i = 0; tnp && i < n; tnp = tnp->tn_next, i++) {
        memcpy(&tlist[i], tnp, sizeof(NUTTIMERINFO));
    }

    /* Send HTML header. */
    StartPage(stream, req, "Timers");
    StartTable(stream,
        "Count-<BR />down",
        "Tick<BR />Reload",
        "Callback<BR />Address",
        "Callback<BR />Argument",
        NULL);

    /* Send table with list of timers. */
    ticks_left = 0;
    for (i = 0; i < n; i++) {
        ticks_left += tnp->tn_ticks_left;
        fprintf(stream,
            "<TR><TD>%lu</TD>"
            "<TD>%lu</TD>"
            "<TD>%08zX</TD>"
            "<TD>%08zX</TD></TR>\r\n",
            ticks_left,
            tlist[i].tn_ticks,
            (size_t) tlist[i].tn_callback,
            (size_t) tlist[i].tn_arg);
    }

    /* Release the thread list copy. */
    free(tlist);

    /* Send HTML footer and flush output buffer. */
    fputs("</TABLE></BODY></HTML>", stream);
    fflush(stream);

    return 0;
}

/*
 * CGI Sample: Show list of sockets.
 *
 * This routine must have been registered by NutRegisterCgi() and is
 * automatically called by NutHttpProcessRequest() when the client
 * request the URL 'cgi-bin/sockets.cgi'.
 */
static int ShowSockets(FILE * stream, REQUEST * req)
{
    static const char *sock_states[13] = {
        "CLOSED",
        "LISTEN",
        "SYNSENT",
        "SYNRCVD",
        "<FONT COLOR=#CC0000>ESTABL</FONT>",
        "CLOSEWAIT",
        "FINWAIT1",
        "CLOSING",
        "LASTACK",
        "FINWAIT2",
        "TIMEWAIT",
        "DESTROY",
        "?"
    };
    extern TCPSOCKET *tcpSocketList;
    TCPSOCKET *ts;
    TCPSOCKET *tlist;
    int i;
    int n;

    /* Take a snapshot of our socket list. */
    for (ts = tcpSocketList, n = 0; ts; ts = ts->so_next, n++);
    tlist = calloc(n, sizeof(TCPSOCKET));
    if (tlist == NULL) {
        return -1;
    }
    for (ts = tcpSocketList, i = 0; ts && i < n; ts = ts->so_next, i++) {
        memcpy(&tlist[i], ts, sizeof(TCPSOCKET));
    }

    /* Send HTML header. */
    StartPage(stream, req, "TCP Sockets");
    StartTable(stream, "Local", "Remote", "Status", NULL);

    for (i = 0; i < n; i++) {
        fprintf(stream, "<TR><TD>%s:%u</TD>", inet_ntoa(tlist[i].so_local_addr), ntohs(tlist[i].so_local_port));
        fprintf(stream, "<TD>%s:%u</TD><TD>", inet_ntoa(tlist[i].so_remote_addr), ntohs(tlist[i].so_remote_port));
        fputs(sock_states[tlist[i].so_state < 11 ? tlist[i].so_state : 11], stream);
        fputs("</TD></TR>\r\n", stream);
        fflush(stream);
    }

    free(tlist);

    fputs("</TABLE></BODY></HTML>", stream);
    fflush(stream);

    return 0;
}

/*
 * CGI Sample: Proccessing a form.
 *
 * This routine must have been registered by NutRegisterCgi() and is
 * automatically called by NutHttpProcessRequest() when the client
 * request the URL 'cgi-bin/form.cgi'.
 *
 * Thanks to Tom Boettger, who provided this sample for ICCAVR.
 */
int ShowForm(FILE * stream, REQUEST * req)
{
    /* Send HTML header. */
    StartPage(stream, req, "Form Result");

    if (req->req_query) {
        int i;
        int count;

        /* Extract count parameters. */
        count = NutHttpGetParameterCount(req);
        /* Send the parameters back to the client. */
        for (i = 0; i < count; i++) {
            fprintf(stream, "%s: %s<BR>\r\n",
                NutHttpGetParameterName(req, i),
                NutHttpGetParameterValue(req, i));
        }
    }

    fputs("</BODY></HTML>", stream);
    fflush(stream);

    return 0;
}

#if !defined(EXCLUDE_ASP) && !defined(EXCLUDE_SSI)

static struct _tm *GetLocaltime(void) {
    struct _tm gmt;
    time_t now;

    now = 0;
    if (0 == NutRtcGetTime(&gmt)) {
        now = _mkgmtime(&gmt);
    }
    else
        now = time(NULL);
    return localtime(&now);
}

static void PrintTime(FILE *stream)
{
#if defined(HTTPD_EXCLUDE_DATE)
    fputs(__TIME__, stream);
#else
    struct _tm *lot;
    lot = GetLocaltime();
    fprintf(stream, "%02d:%02d:%02d", lot->tm_hour, lot->tm_min, lot->tm_sec);
#endif
}

static void PrintDate(FILE *stream)
{
#if defined(HTTPD_EXCLUDE_DATE)
    fputs(__DATE__, stream);
#else
    struct _tm *lot;
    lot = GetLocaltime();
    fprintf(stream, "%02d.%02d.%04d", lot->tm_mday, lot->tm_mon + 1, lot->tm_year + 1900);
#endif
}

#endif

#ifndef EXCLUDE_ASP

/*
 *  ASPCallback
 *
 * This routine must have been registered by
 * NutRegisterAspCallback() and is automatically called by
 * NutHttpProcessFileRequest() when the server process a page
 * with an asp function.
 *
 * Return 0 on success, -1 otherwise.
 */
static int ASPCallback(char *pASPFunction, FILE * stream)
{
    int rc = 0;

    if (strcmp(pASPFunction, "usr_date") == 0) {
        PrintDate(stream);
    }
    else if (strcmp(pASPFunction, "usr_time") == 0) {
        PrintTime(stream);
    }
    else {
        rc = -1;
    }
    return rc;
}

#endif

#ifndef EXCLUDE_SSI

/*
 * CGI Sample: Dynamic output cgi included by ssi.shtml file
 *
 * This routine must have been registered by NutRegisterCgi() and is
 * automatically called by NutHttpProcessRequest() when the client
 * request the URL 'cgi-bin/form.cgi'.
 *
 * Thanks to Tom Boettger, who provided this sample for ICCAVR.
 */
int SSIDemoCGI(FILE * stream, REQUEST * req)
{
    fputs("CGI ssi-demo called with", stream);
    if (req->req_query) {
        char *name;
        char *value;
        int i;
        int count;

        count = NutHttpGetParameterCount(req);

        /* Extract count parameters. */
        fputs(" these parameters:\r\n<p>", stream);
        for (i = 0; i < count; i++) {
            name = NutHttpGetParameterName(req, i);
            value = NutHttpGetParameterValue(req, i);

            /* Send the parameters back to the client. */
            fprintf(stream, "%s: %s<BR>\r\n", name, value);
        }
    } else {
        /* Called without any parameter, show the current time */
        fputs("out any parameter.<br><br>Current time is: ", stream);
        PrintDate(stream);
        fputs(" -- ", stream);
        PrintTime(stream);
        fputs("<br>\r\n", stream);

    }
    fflush(stream);

    return 0;
}

#endif


/*! \fn Service(void *arg)
 * \brief HTTP service thread.
 *
 * The endless loop in this thread waits for a client connect, processes
 * the HTTP request and disconnects. If one client has established a
 * connection, further connect attempts will be delayed in the TCP
 * stack (backlog).
 *
 * Typically browsers open more than one connection in order
 * to load images concurrently. So we run this routine by
 * several threads.
 *
 */
THREAD(Service, arg)
{
    TCPSOCKET *sock;
    FILE *stream;
    uint8_t id = (uint8_t) ((uintptr_t) arg);

    /*
     * Now loop endless for connections.
     */
    for (;;) {

        /*
         * Create a socket.
         */
        if ((sock = NutTcpCreateSocket()) == 0) {
            printf("[%u] Creating socket failed\n", id);
            NutSleep(5000);
            continue;
        }

        /*
         * Listen on port 80. This call will block until we get a connection
         * from a client.
         */
        NutTcpAccept(sock, 80);
        printf("[%u] Connected, %zu bytes free\n", id, NutHeapAvailable());

        /*
         * Wait until at least 4 kByte of free RAM is available. This will
         * keep the client connected in low memory situations.
         */
        while (NutHeapAvailable() < 4096) {
            printf("[%u] Low mem\n", id);
            NutSleep(1000);
        }

        /*
         * Associate a stream with the socket so we can use standard I/O calls.
         */
        if ((stream = _fdopen((int) ((uintptr_t) sock), "r+b")) == 0) {
            printf("[%u] Creating stream failed\n", id);
        } else {
            /*
             * This API call saves us a lot of work. It will parse the
             * client's HTTP request, send any requested file from the
             * registered file system or handle CGI requests by calling
             * our registered CGI routine.
             */
            NutHttpProcessRequest(stream);

            /*
             * Destroy the virtual stream device.
             */
            fclose(stream);
        }

        /*
         * Close our socket.
         */
        NutTcpCloseSocket(sock);
        printf("[%u] Disconnected\n", id);
    }
}

static void DhcpWait(HANDLE timer, void *arg)
{
    putchar('.');
}

/*!
 * \brief Main application routine.
 *
 * Nut/OS automatically calls this entry after initialization.
 */
int main(void)
{
    uint32_t baud = 115200;
    uint8_t i;
    HANDLE h;

    /*
     * Initialize stdio console.
     */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
    printf("\n\nNut/OS %s HTTP Daemon...", NutVersionString());

#ifdef RTC_CHIP
    if (NutRegisterRtc(&RTC_CHIP)) {
        puts("RTC failed");
    }
#endif
    /*
     * Initialize Ethernet controller.
     */
    if (NutRegisterDevice(&DEV_ETHER, 0, 0)) {
        Fatal("Registering device failed");
    }
    printf("Configure %s", DEV_ETHER_NAME);
    h = NutTimerStart(1000, DhcpWait, NULL, 0);
    if (NutDhcpIfConfig(DEV_ETHER_NAME, 0, 60000)) {
        Fatal("failed, run editconf first!");
    }
    NutTimerStop(h);
    printf("%s ready\n", inet_ntoa(confnet.cdn_ip_addr));

    /*
     * Register file system.
     */
    NutRegisterDevice(&devUrom, 0, 0);

    /*
     * Register our CGI functions.
     */
    NutRegisterCgiBinPath("cgi-bin/;user/cgi-bin/;admin/cgi-bin/");
    NutRegisterCgi("test.cgi", ShowQuery);
    NutRegisterCgi("threads.cgi", ShowThreads);
    NutRegisterCgi("timers.cgi", ShowTimers);
    NutRegisterCgi("sockets.cgi", ShowSockets);
    NutRegisterCgi("form.cgi", ShowForm);
    NutRegisterAuth("admin", "root:root");
    NutRegisterAuth("user", "user:user");

    /*
     * Register SSI and ASP handler
     */
#ifndef EXCLUDE_SSI
    /*
     * Register a cgi included by the ssi demo. This will show how dynamic
     * content is included in a ssi page and how the request parameters for
     * a site are passed down to the included cgi.
     */
    NutRegisterCgi("ssi-demo.cgi", SSIDemoCGI);
    NutRegisterSsi();
#endif
#ifndef EXCLUDE_ASP
    NutRegisterAsp();
    NutRegisterAspCallback(ASPCallback);
#endif

    /*
     * Start HTTP server threads, passing a thread number.
     */
    for (i = 1; i <= 4; i++) {
        NutThreadCreate("httpd", Service, (void *) (uintptr_t) i, HTTPD_SERVICE_STACK);
    }

    /*
     * We could do something useful here, like serving a watchdog.
     */
    for (;;) {
        NutSleep(60000);
    }
    return 0;
}
