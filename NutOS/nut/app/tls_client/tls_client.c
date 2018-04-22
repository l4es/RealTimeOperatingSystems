/*
 * Copyright (C) 2014 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * \example tls_client/tls_client.c
 *
 * Requests an URL from the Internet using a SSL/TLS connection
 * and transfers the HTML source code to the serial device.
 *
 * Your local Ethernet network must provide Internet access.
 * Connect the RS232 port of the Ethernut with a free COM
 * port of your PC and run a terminal emulator at 115200 Baud.
 *
 * If your local network does not support DHCP, it may be
 * required to modify the MY_IP, MY_MASK and MY_GATE below.
 *
 * This sample demonstrates the tls library in client mode usage.
 * It creates a TLS connection and prints out the server certificate,
 * the used cypher suite and session parameters.
 *
 * To use this example, you need to enable the TLS and crypto library
 * in your configuration.
 * You should enable the following settings in your my_board.conf
 * file and re-configure your build tree:
 * 
 * CRYPTO_BIGINT_BARRETT = ""
 * CRYPTO_BIGINT_CRT = ""
 * CRYPTO_BIGINT_SLIDING_WINDOW = ""
 * CRYPTO_BIGINT_SQUARE = ""
 * TLS_SSL_ENABLE_CLIENT = ""
 * TLS_SSL_PROT_MEDIUM = ""
 * TLS_SSL_ENABLE_V23_HANDSHAKE = ""
 */

#include <toolchain.h>
#include <cfg/arch.h>
#include <sys/thread.h>
#include <sys/heap.h>
#include <sys/time.h>
#include <sys/timer.h>
#include <sys/socket.h>
#include <sys/confnet.h>
#include <sys/heap.h>
#include <sys/version.h>
#include <sys/nutdebug.h>
#include <arpa/inet.h>
#include <net/route.h>
#include <netdb.h>
#include <pro/dhcp.h>
#include <pro/sntp.h>
#include <dev/board.h>
#include <dev/debug.h>
#include <errno.h>

#include <gorp/edline.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <io.h>

#include <tls/ssl.h>

#include "cert.h"
#include "private_key.h"

#if !defined(TLS_SSL_CERT_VERIFICATION)
int ssl_verify_cert(const SSL *ssl)
{
    printf("%s", unsupported_str);
    return -1;
}


const char * ssl_get_cert_dn(const SSL *ssl, int component)
{
    printf("%s", unsupported_str);
    return NULL;
}

const char * ssl_get_cert_subject_alt_dnsname(const SSL *ssl, int index)
{
    printf("%s", unsupported_str);
    return NULL;
}
SSL * ssl_client_new(SSL_CTX *ssl_ctx, int client_fd,
                 const uint8_t *session_id, uint8_t sess_id_size)
{
    printf("%s", unsupported_str);
    return NULL;
}
#endif  /* TLS_SSL_CERT_VERIFICATION */

#define DBG_BAUDRATE 115200

#define DNSSERVERIP     "192.168.1.254"
#define MY_MAC          {0x02,0x06,0x98,0x20,0x00,0x00}
#define MY_IP           "192.168.1.10"
#define MY_MASK         "255.255.255.0"
#define MY_GATE         "192.168.1.254"
#define MYTZ            -1  /* This is CET */
#define MYTIMED         "130.149.17.21"

typedef struct {
    char *schm_uri;
    char *schm_user;
    char *schm_pass;
    char *schm_host;
    char *schm_port;
    char *schm_path;
    uint16_t schm_portnum;
} HTTP_SCHEME;


/**
 * Display what cipher we are using
 */
static void display_cipher(SSL *ssl)
{
    printf("CIPHER is ");
    switch (ssl_get_cipher_id(ssl))
    {
        case SSL_AES128_SHA:
            printf("AES128-SHA");
            break;

        case SSL_AES256_SHA:
            printf("AES256-SHA");
            break;

        case SSL_RC4_128_SHA:
            printf("RC4-SHA");
            break;

        case SSL_RC4_128_MD5:
            printf("RC4-MD5");
            break;

        default:
            printf("Unknown - %d", ssl_get_cipher_id(ssl));
            break;
    }

    printf("\n");
}

/**
 * Display what session id we have.
 */
static void display_session_id(SSL *ssl)
{
    int i;
    const uint8_t *session_id = ssl_get_session_id(ssl);
    int sess_id_size = ssl_get_session_id_size(ssl);

    if (sess_id_size > 0)
    {
        printf("\n-----BEGIN SSL SESSION PARAMETERS-----\n");
        for (i = 0; i < sess_id_size; i++)
        {
            printf("%02x", session_id[i]);
        }

        printf("\n-----END SSL SESSION PARAMETERS-----\n\n");
    }
}

/*!
 * \brief Release a HTTP scheme structure.
 *
 * \param schm Pointer to a \ref HttpSchemeParse "previously allocated"
 *             HTTP scheme structure.
 */
void HttpSchemeRelease(HTTP_SCHEME *schm)
{
    if (schm) {
        if (schm->schm_uri) {
            free(schm->schm_uri);
        }
        free(schm);
    }
}

/*!
 * \brief Create a HTTP scheme structure from a URI.
 *
 * \param uri URI to parse. The expected format is
 * \code
 * [<user>[:<password>]@]<host>[:<port>][/<path>]
 * \endcode
 *
 * \return Pointer to an allocated \ref HTTP_SCHEME "HTTP scheme"
 *         structure. If this structure is no longer used, the caller
 *         must call HttpSchemeRelease(). In case of an error, NULL is
 *         returned.
 */
HTTP_SCHEME *HttpSchemeParse(CONST char *uri)
{
    HTTP_SCHEME *schm = NULL;
    char *cp;

    /* Create a blank scheme structure. */
    if (*uri && (schm = malloc(sizeof(HTTP_SCHEME))) != NULL) {
        memset(schm, 0, sizeof(HTTP_SCHEME));

        /* Create a local copy of the URI string. */
        if ((schm->schm_uri = strdup(uri)) != NULL) {
            /* Split the local copy. */
            schm->schm_host = schm->schm_uri;
            for (cp = schm->schm_uri; *cp; cp++) {
                if (*cp == ':') {
                    *cp = '\0';
                    schm->schm_port = cp + 1;
                }
                else if (*cp == '/') {
                    *cp = 0;
                    schm->schm_path = cp + 1;
                    break;
                }
                else if (*cp == '@') {
                    *cp = 0;
                    schm->schm_user = schm->schm_host;
                    schm->schm_pass = schm->schm_port;
                    schm->schm_host = cp + 1;
                    schm->schm_port = NULL;
                }
            }
            if (schm->schm_port) {
                schm->schm_portnum = (uint16_t)atoi(schm->schm_port);
            }
            else {
                schm->schm_portnum = 80;
            }
            return schm;
        }
    }
    HttpSchemeRelease(schm);
    return NULL;
}


/**
 * Implement the SSL/TLS client logic.
 */
static int tls_client(char *uri)
{
    SSL_CTX   *ssl_ctx;
    SSL       *ssl       = NULL;
    TCPSOCKET *sock;
    int        client_fd;
    uint32_t   remote_ip;
    uint32_t   port      = 443;
    uint32_t   options   = SSL_SERVER_VERIFY_LATER /*| SSL_DISPLAY_CERTS | SSL_DISPLAY_STATES | SSL_NO_DEFAULT_KEY */;
    uint32_t   timeout;
    int        rc = 0;
    HTTP_SCHEME *http_sch;

    http_sch = HttpSchemeParse(uri);

    remote_ip = NutDnsGetHostByName((const uint8_t*)http_sch->schm_host);
    if (http_sch->schm_port) {
        port = atoi (http_sch->schm_port);
    }

    if ((ssl_ctx = ssl_ctx_new(options, SSL_DEFAULT_CLNT_SESS)) == NULL) {
        NUTPANIC("Error: Client context is invalid\n");
    }

    ssl_obj_memory_load(ssl_ctx, SSL_OBJ_RSA_KEY, my_private_key,
            my_private_key_len, NULL);

    ssl_obj_memory_load(ssl_ctx, SSL_OBJ_X509_CERT, my_certificate,
            my_certificate_len, NULL);

/*  If you have enough RAM, you could load the CA bundle. The server
    certiciate will be validated then.

    if (ssl_obj_memory_load(ssl_ctx, SSL_OBJ_X509_CACERT, ca_bundle_crt, sizeof(ca_bundle_crt), NULL))
    {
        printf("Error loading ca-bundle\n");
    }
*/

    if ((sock = NutTcpCreateSocket()) != 0) {

        /* Define a connect timeout of 5ms */
        timeout = 1000;
        NutTcpSetSockOpt(sock, SO_SNDTIMEO, &timeout, sizeof(timeout));

        if ((rc = NutTcpConnect(sock, remote_ip, port)) == 0) {
            /* Reset the write timeout to infinite */
            timeout = 1000;
            NutTcpSetSockOpt(sock, SO_SNDTIMEO, &timeout, sizeof(timeout));
            timeout = 100;
            NutTcpSetSockOpt(sock, SO_RCVTIMEO, &timeout, sizeof(timeout));
            client_fd = (int)sock;

            ssl = ssl_client_new(ssl_ctx, client_fd, NULL, 0);
            /* check the return status */
            if ((rc = ssl_handshake_status(ssl)) != SSL_OK) {
                ssl_display_error(rc);
                ssl_free(ssl);
                ssl_ctx_free(ssl_ctx);
                NutTcpCloseSocket(sock);
                HttpSchemeRelease(http_sch);
                return -1;
            }
            printf("SSL session established: RAM: %zd\n", NutHeapAvailable()); fflush(stdout);

            printf("Server cert detailes:\n");
            const char *common_name = ssl_get_cert_dn(ssl, SSL_X509_CERT_COMMON_NAME);
            printf("Common Name (CN)           : %s\n", common_name);

            const char *organisation = ssl_get_cert_dn(ssl, SSL_X509_CERT_ORGANIZATION);
            printf("Organisation (O)           : %s\n", organisation);

            const char *organisational_unit = ssl_get_cert_dn(ssl, SSL_X509_CERT_ORGANIZATIONAL_NAME);
            printf("Organisational Unit (OU)   : %s\n", organisational_unit);

            common_name = ssl_get_cert_dn(ssl, SSL_X509_CA_CERT_COMMON_NAME);
            printf("CA-Common Name (CN)        : %s\n", common_name);

            organisation = ssl_get_cert_dn(ssl, SSL_X509_CA_CERT_ORGANIZATION);
            printf("CA-Organisation (O)        : %s\n", organisation);

            organisational_unit = ssl_get_cert_dn(ssl, SSL_X509_CA_CERT_ORGANIZATIONAL_NAME);
            printf("CA-Organisational Unit (OU): %s\n", organisational_unit);

            display_session_id(ssl);
            display_cipher(ssl);

            printf("Verify server certificate: ");
            switch(ssl_verify_cert(ssl)) {
                case SSL_X509_ERROR(X509_OK): printf("OK\n"); break;
                case SSL_X509_ERROR(X509_NOT_OK): printf("NOT OK\n"); break;
                case SSL_X509_ERROR(X509_VFY_ERROR_NO_TRUSTED_CERT): printf("Not trusted\n"); break;
                case SSL_X509_ERROR(X509_VFY_ERROR_BAD_SIGNATURE): printf("Bad signature\n"); break;
                case SSL_X509_ERROR(X509_VFY_ERROR_NOT_YET_VALID): printf("Not yet valid\n"); break;
                case SSL_X509_ERROR(X509_VFY_ERROR_EXPIRED): printf("Expired\n"); break;
                case SSL_X509_ERROR(X509_VFY_ERROR_SELF_SIGNED): printf("Self signed\n"); break;
                case SSL_X509_ERROR(X509_VFY_ERROR_UNSUPPORTED_DIGEST): printf("Unsupported digest\n"); break;
            }

            char *buf = malloc(1024);
            memset(buf, 0, 1024);

            sprintf(buf, "GET /%s HTTP/1.1\r\n"
                         "User-Agent: Ethernut [en] (NutOS)\r\n"
                         "Host: %s\r\n"
                         "\r\n", http_sch->schm_path != NULL ? http_sch->schm_path : "", http_sch->schm_host);

            rc = ssl_write(ssl, (uint8_t*)buf, strlen(buf));

            printf("Available memory after session establishment: %zd\n", NutHeapAvailable());
            printf("-----------------------------------------------------------\n");
            printf("Request Header:\n%s", buf);
            printf("-----------------------------------------------------------\n");

            free(buf);

            if (rc < 0) {
                printf("Error on ssl_write\n");
            } else
            do {
                char *inbuf;

                rc = ssl_read(ssl, (uint8_t **)&inbuf);

                if (rc == 0) {
                    /* Timeout on read... continue */
                    continue;
                } else
                if (rc > 0) {
                    puts(inbuf);
                    fflush(stdout);
                } else {
                    if ((rc == SSL_ERROR_CONN_LOST) || (rc == SSL_CLOSE_NOTIFY)) {
                        printf("-----------------------------------------------------------\n");
                        printf("Connection closed by foreign host\n");
                    } else {
                        printf("-----------------------------------------------------------\n");
                        printf("Error on SSL read: %d\n", rc);
                    }
                }
            } while (rc >= 0);
            ssl_free(ssl);
        } else {
            if (NutTcpError(sock) == ETIMEDOUT) {
                printf("Timeout\n"); fflush(stdout);
            } else {
                printf("Connection refused\n"); fflush(stdout);
            }
        }
        NutTcpCloseSocket(sock);
    }
    ssl_ctx_free(ssl_ctx);
    HttpSchemeRelease(http_sch);

    printf("\nEverything cleaned up: RAM: %zd\n", NutHeapAvailable()); fflush(stdout);
    return 0;
}


/*!
* \brief This thread handles the TLS connection and implements a basic TLS client
*
*/
THREAD(TlsClient, arg)
{
    char     input[64] = "";
    char    *url = NULL;
    EDLINE  *ed;

    for(;;) {
        puts ("-----------------------------------------------------------\n");
        puts ("Please enter URL: ");
        strcpy(input, "https://");

        ed = EdLineOpen(EDIT_MODE_ECHO);
        EdLineRead(ed, input, sizeof(input));
        EdLineClose(ed);

        printf("\n");

        if (strncasecmp(input, "https://", 8) == 0) {
            url = &input[8];
        } else
        if (strncasecmp(input, "http://", 7) == 0) {
            url = &input[7];
        } else{
            url = input;
        }

        tls_client(url);
    }
}

/*!
* \brief Main application routine.
*
*/
int main(void)
{
	FILE     *uart;
	uint32_t baud = 115200;
    uint32_t ip_addr;
    uint32_t timeserver = inet_addr(MYTIMED);
    time_t   now;
    static uint8_t my_mac[] = MY_MAC;

    NutSleep(200);
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
	uart = fopen(DEV_CONSOLE.dev_name, "r+");
	_ioctl(_fileno(uart), UART_SETSPEED, &baud);

	freopen(DEV_CONSOLE.dev_name, "w", stdout);
	freopen(DEV_CONSOLE.dev_name, "w", stderr);
	freopen(DEV_CONSOLE.dev_name, "r", stdin);

	printf ("TLS Demo - Nut/OS (%s)\n", NutVersionString());

    if (NutHeapAvailable() < 50000) {
        printf("We do not have enough RAM for the TLS demo...\n");
        printf("STOP!\n");
        while(1) NutSleep(1000);
    }

    printf("Configure %s...", DEV_ETHER_NAME);
    if (NutRegisterDevice(&DEV_ETHER, 0, 0)) {
        NUTPANIC("failed\n");
    } else {
        if (NutDhcpIfConfig(DEV_ETHER_NAME, my_mac, 20000)) {
            ip_addr = inet_addr(MY_IP);
            NutNetIfConfig("eth0", my_mac, ip_addr, inet_addr(MY_MASK));
            NutIpRouteAdd(0, 0, inet_addr(MY_GATE), &DEV_ETHER);
            NutDnsConfig2(0, 0, inet_addr(DNSSERVERIP), 0);
        }
        printf("%s ready\n\n", inet_ntoa(confnet.cdn_ip_addr));
        _timezone = 1 * 60L * 60L;
        if (NutSNTPGetTime(&timeserver, &now) == 0) {
            stime(&now);
            printf("Time: %s\n", ctime(&now));
        }
    }

    NutThreadCreate("tls", TlsClient, NULL, 3096);

    for (;;) {
        NutSleep(10000);
    }
    return 0;
}
