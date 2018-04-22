/*
 * Copyright (C) 2012-2013 by egnite GmbH
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


#include <pro/tcphost.h>
#include <pro/uri.h>
#include <pro/soap.h>

#include <stdlib.h>
#include <string.h>

/* Use for local debugging only. Do NOT include into NUTDEBUG.
#define DEBUG_SOAPC
*/
#ifdef DEBUG_SOAPC
#include <stdio.h>
#endif

static int ReadUntilChars(FILE *sp, const char *delim, const char *ignore, char *buf, int siz)
{
    int rc = 0;
    int skip = 0;
    char ch;

    /* Do not read more characters than requested. */
    while (rc < siz) {
        ch = fgetc(sp);
#ifdef DEBUG_SOAPC
        if (ch != EOF) {
            putchar(ch);
        }
#endif
        if (rc == 0 && ch == ' ') {
            /* Skip leading spaces. */
            skip++;
        } else {
            rc++;
            if (delim && strchr(delim, ch)) {
                /* Delimiter found. */
                break;
            }
            if (buf && (ignore == NULL || strchr(ignore, ch) == NULL)) {
                /* Add valid character to application buffer. */
                *buf++ = ch;
            }
        }
    }
    if (buf) {
        *buf = '\0';
    }
    return rc + skip;
}

static int TagRead(FILE *stream, SOAP_TAG *tag, int avail)
{
    int rc;
    char *cp;

    memset(tag, 0, sizeof(*tag));

    avail = avail > SOAP_MAX_TAG_SIZE ? SOAP_MAX_TAG_SIZE : avail;
    rc = ReadUntilChars(stream, ">", NULL, tag->soap_buff, avail);
    if (rc > 0) {
        cp = tag->soap_buff;
        if (*cp == '/') {
            cp++;
            tag->soap_ttf = SOAPTYPE_ETAG;
        }
        tag->soap_name._name = cp;
        while (*cp && *cp != ' ') {
            if (*cp == ':') {
                *cp++ = '\0';
                tag->soap_name._namespace = tag->soap_name._name;
                tag->soap_name._name = cp;
            } else {
                cp++;
            }

        }
        if (tag->soap_ttf == 0) {
            int ac;

            for (ac = 0; *cp && ac < SOAP_MAX_TAG_ATTRIBUTES; ac++) {
                *cp = '\0';
                while (*++cp == ' ');
                if (*cp == '/' && *(cp + 1) == '\0') {
                    tag->soap_ttf = SOAPTYPE_EOTAG;
                    break;
                }
                tag->soap_attr[ac].attr_name._name = cp;
                while (*cp && *cp != '=') {
                    if (*cp == ':') {
                        *cp++ = '\0';
                        tag->soap_attr[ac].attr_name._namespace = tag->soap_attr[ac].attr_name._name;
                        tag->soap_attr[ac].attr_name._name = cp;
                    } else {
                        cp++;
                    }
                }
                if (*cp) {
                    uint_fast8_t quoted = *++cp == '"';

                    cp += quoted;
                    tag->soap_attr[ac].attr_value = cp;
                    while (*cp) {
                        if (quoted) {
                            if (*cp == '"') {
                                break;
                            }
                        }
                        else if (*cp == ' ') {
                            break;
                        }
                        cp++;
                    }
                    if (*cp) {
                        *cp++ = '\0';
                    }
                }
            }
        }
    }
    return rc;
}

static int ReadResultBody(FILE *stream, int avail, SOAP_PROCEDURE *proc)
{
    SOAP_ARG *arg = NULL;
    SOAP_TAG *tag;
    int bufsiz;
    int got;
    int in_body = 0;

    tag = malloc(sizeof(*tag));
    if (avail < 16 || tag == NULL) {
        return -1;
    }
    while (avail) {
        /* Read all characters up to the next tag, but honor limits. */
        bufsiz = avail > SOAP_MAX_TAG_SIZE ? SOAP_MAX_TAG_SIZE : avail;
        got = ReadUntilChars(stream, "<", NULL, tag->soap_buff, bufsiz);
        avail -= got;
        if (got <= 0 || avail <= 4) {
            break;
        }
        tag->soap_buff[got] = '\0';

        /* If we are inside an argument, then this is the value. */
        if (arg) {
            free(arg->arg_val);
            arg->arg_val = strdup(tag->soap_buff);
        }
        /* Now read the tag. */
        got = TagRead(stream, tag, avail);
        avail -= got;
        if (got <= 0) {
            break;
        }
        /* Check for envelope tag. */
        if (strcmp(tag->soap_name._name, "Envelope") == 0) {
            if (tag->soap_ttf) {
                /* Closing tag, update state. */
                in_body = 0;
            }
        }
        /* Check for body tag. */
        else if (strcmp(tag->soap_name._name, "Body") == 0) {
            /* Update state according to opening or closing tags. */
            in_body = tag->soap_ttf == 0;
        }
        /* Check for other tags only if we are inside a body. */
        else if (in_body) {
            /* Following tags in a body contain arguments. */
            if (tag->soap_ttf == 0) {
                /* Opening tag. */
                for (arg = proc->proc_argo; arg; arg = arg->arg_next) {
                    if (strcasecmp(tag->soap_name._name, arg->arg_name) == 0) {
                        break;
                    }
                }
            } else {
                arg = NULL;
            }
        }
    }
    free(tag);

    return 0;
}

static int ReadResult(FILE *stream, SOAP_PROCEDURE *proc)
{
    int rc = -1;
    char *line;
    char *val;
    int avail = 0;

    line = malloc(HTTP_MAX_REQUEST_SIZE);
    if (line) {
        if (fgets(line, HTTP_MAX_REQUEST_SIZE, stream) == NULL) {
            /* Broken connection, stop parsing. */
        }
        else if (atoi(line + 9) == 200) {
#ifdef DEBUG_SOAPC
            printf("%s", line);
#endif
            for (;;) {
                if (fgets(line, HTTP_MAX_REQUEST_SIZE, stream) == NULL) {
                    /* Broken connection, stop parsing. */
                    break;
                }
#ifdef DEBUG_SOAPC
                printf("%s", line);
#endif
                val = strchr(line, ':');
                if (val == NULL) {
                    /* Hopefully we reached the end of the header. */
                    break;
                }
                *val++ = '\0';
                if (strcasecmp(line, "CONTENT-LENGTH") == 0) {
                    avail = atoi(val);
                }
            }
            rc = ReadResultBody(stream, avail, proc);
        }
        free(line);
    }
    return rc;
}

static int FillBody(char *body, int size, const char *urn, SOAP_PROCEDURE *proc)
{
    int len;
    SOAP_ARG *arg;

    strcpy(body, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n"
        "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\" "
        "s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\">\r\n"
        "<s:Body>\r\n");
    len = strlen(body);
    sprintf(body + len, "<u:%s xmlns:u=\"urn:%s\">\r\n", proc->proc_name, urn);
    len += strlen(body + len);
    for (arg = proc->proc_argi; arg; arg = arg->arg_next) {
        if (arg->arg_val) {
            sprintf(body + len, "<%s>%s</%s>\r\n", arg->arg_name, arg->arg_val, arg->arg_name);
        } else {
            sprintf(body + len, "<%s />\r\n", arg->arg_name);
        }
        len += strlen(body + len);
    }
    sprintf(body + len, "</u:%s>\r\n", proc->proc_name);
    len += strlen(body + len);
    strcpy(body + len, "</s:Body>\r\n"
                "</s:Envelope>\r\n");
    len += strlen(body + len);

    return len;
}

int SoapProcCallResource(SOAP_PROCEDURE *proc, const char *url, const char *uri, const char *urn, uint32_t tmo)
{
    int rc = -1;
    URI_SCHEME *schm;

    schm = UriSchemeSplit(url + 7);
    if (schm) {
        TCPSOCKET *sock = NutTcpCreateSocket();

        if (sock) {
            FILE *stream;

            stream = TcpHostConnectStream(sock, schm->schm_host, schm->schm_portnum, tmo);
            if (stream) {
                int len;
                char *body = malloc(2048);

                if (body) {
                    len = FillBody(body, 2048, urn, proc);

                    fprintf(stream, "POST %s HTTP/1.1\r\n", uri);
                    fprintf(stream, "HOST: %s:%s\r\n", schm->schm_host, schm->schm_port);
                    fputs("Content-Type: text/xml; charset=\"utf-8\"\r\n", stream);
                    if (urn) {
                        fprintf(stream, "SOAPACTION: \"urn:%s#%s\"\r\n", urn, proc->proc_name);
                    }
                    fprintf(stream, "Content-Length: %d\r\n\r\n", len);

                    fputs(body, stream);
#ifdef DEBUG_SOAPC
                    printf("POST %s HTTP/1.1\n", uri);
                    printf("HOST: %s:%s\n", schm->schm_host, schm->schm_port);
                    puts("Content-Type: text/xml; charset=\"utf-8\"");
                    if (urn) {
                        printf("SOAPACTION: \"urn:%s#%s\"\n", urn, proc->proc_name);
                    }
                    printf("Content-Length: %d\n\n", len);
                    puts(body);
#endif
                    free(body);
                    if (fflush(stream) == 0) {
                        rc = ReadResult(stream, proc);
                    }
                }
                fclose(stream);
            }
            NutTcpCloseSocket(sock);
        }
        UriSchemeRelease(schm);
    }
    return rc;
}

