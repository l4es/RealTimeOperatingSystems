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

#include <pro/soap.h>

#include <stdlib.h>
#include <string.h>

/* Use for local debugging only. Do NOT include into NUTDEBUG.
#define DEBUG_SOAPD
*/
#ifdef DEBUG_SOAPD
#include <stdio.h>
#endif

static int ReadTag(HTTP_STREAM *stream, SOAP_TAG *tag, int avail)
{
    int rc;
    char *cp;

    memset(tag, 0, sizeof(*tag));

    avail = avail > SOAP_MAX_TAG_SIZE ? SOAP_MAX_TAG_SIZE : avail;
    rc = StreamReadUntilChars(stream, ">", NULL, tag->soap_buff, avail);
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

SOAP_PROCEDURE *SoapParseCallRequest(HTTP_STREAM *stream, int avail, SOAP_PROCEDURE *list)
{
    SOAP_PROCEDURE *proc = NULL;
    SOAP_ARG *arg = NULL;
    SOAP_TAG *tag;
    int bufsiz;
    int got;
    int in_body = 0;

    tag = malloc(sizeof(*tag));
    if (avail < 16 || tag == NULL) {
        return NULL;
    }
    while (avail) {
        /* Read all characters up to the next tag, but honor limits. */
        bufsiz = avail > SOAP_MAX_TAG_SIZE ? SOAP_MAX_TAG_SIZE : avail;
        got = StreamReadUntilChars(stream, "<", NULL, tag->soap_buff, bufsiz);
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
        got = ReadTag(stream, tag, avail);
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
            if (proc == NULL) {
                /* First tag in a body specifies the action. */
                for (proc = list; proc; proc = proc->proc_next) {
                    if (strcasecmp(tag->soap_name._name, proc->proc_name) == 0) {
                        break;
                    }
                }
            } else {
                /* Following tags in a body contain arguments. */
                if (tag->soap_ttf == 0) {
                    /* Opening tag. */
                    for (arg = proc->proc_argi; arg; arg = arg->arg_next) {
                        if (strcasecmp(tag->soap_name._name, arg->arg_name) == 0) {
                            break;
                        }
                    }
                } else {
                    arg = NULL;
                }
            }
        }
    }
    return proc;
}

int SoapSendCallResponse(HTTP_STREAM *stream, SOAP_PROCEDURE *proc, const char *domain, const char *type)
{
    SOAP_ARG *arg;

    HttpSendStreamHeaderTop(stream, 200);
    s_puts("SERVER: NutOS/5.0 UPnP/1.0 TestUPnP/1.0\r\n", stream);
    s_puts("EXT:\r\n", stream);
    HttpSendStreamHeaderBottom(stream, "text", "xml", HTTP_CONN_CLOSE, -1);

    s_puts(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n"
        "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\" "
        "s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\">\r\n"
        "<s:Body>\r\n", stream);

    s_printf(stream, "<u:%sResponse xmlns:u=\"urn:%s:service:%s:1\">\r\n", proc->proc_name, domain, type);
    for (arg = proc->proc_argo; arg; arg = arg->arg_next) {
        if (proc->proc_argo->arg_val) {
            s_printf(stream, "<%s>%s</%s>\r\n", proc->proc_argo->arg_name, proc->proc_argo->arg_val, proc->proc_argo->arg_name);
        }
    }
    s_printf(stream, "</u:%sResponse>\r\n", proc->proc_name);

    s_puts("</s:Body>\r\n"
        "</s:Envelope>\r\n", stream);
    s_flush(stream);

    return 0;
}

