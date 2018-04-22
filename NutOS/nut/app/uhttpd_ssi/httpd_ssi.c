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
#include <sys/version.h>
#include <dev/board.h>
#include <dev/urom.h>
#include <pro/dhcp.h>
#endif

#include <isc/list.h>
#include <pro/uhttp/mediatypes.h>
#include <pro/uhttp/envvars.h>
#include <pro/uhttp/modules/mod_redir.h>
#include <pro/uhttp/modules/mod_cgi_func.h>
#include <pro/uhttp/modules/mod_ssi.h>

#include <stdio.h>
#include <string.h>

#define MAJOR_VERSION   1
#define MINOR_VERSION   0

enum appvar_t {
    APPVAR_VERSION = 1,
    APPVAR_NAME
};

typedef struct _APPVAR {
    char *var_name;
    enum appvar_t var_index;
} APPVAR;

APPVAR appvar_list[] = {
    { "APPNAME", APPVAR_NAME },
    { "APPVERSION", APPVAR_VERSION }
};

#define APPVAR_NUM (sizeof(appvar_list) / sizeof(APPVAR))

static const char *AppVarHandler(HTTPD_SESSION *hs, int item)
{
    static char value[128];

    value[0] = '\0';
    switch (item) {
    case APPVAR_NAME:
        strcpy(value, "SSI Sample");
        break;
    case APPVAR_VERSION:
        sprintf(value, "%d.%d", MAJOR_VERSION, MINOR_VERSION);
        break;
    }
    return value;
}

static void AppVarInit(APPVAR *list)
{
    int i;

    for (i = 0; i < APPVAR_NUM; i++) {
        EnvRegisterVariable(list[i].var_name, AppVarHandler, list[i].var_index);
    }
}

static int CgiVarList(HTTPD_SESSION *hs)
{
    extern ISC_LIST(HTTP_ENVVAR_ENTRY) envVarList;
    HTTP_ENVVAR_ENTRY *env;

    /* Locate the entry with the given name. */
    for (env = ISC_LIST_HEAD(envVarList); env; env = ISC_LIST_NEXT(env, env_link)) {
        s_printf(hs->s_stream, "<option>%s</option>\r\n", env->env_name);
    }
    return 0;
}

int main(void)
{
#ifdef NUT_OS
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
#endif

    puts("uHTTP server side include sample\nBuild " __DATE__ " " __TIME__);

#ifdef NUT_OS
    NutRegisterDevice(&DEV_ETHER, 0, 0);
    NutDhcpIfConfig(DEV_ETHER_NAME, NULL, 60000);
    NutRegisterDevice(&devUrom, 0, 0);
#endif

    StreamInit();
    MediaTypeInitDefaults();
    HttpRegisterRedir("", "/index.shtml", 301);

    HttpRegisterCgiFunction("varlist.cgi", CgiVarList);
    HttpRegisterMediaType("cgi", NULL, NULL, HttpCgiFunctionHandler);

    EnvInitDefaults();
    AppVarInit(appvar_list);
    HttpRegisterSsiVarHandler(EnvHandler);

    StreamClientAccept(HttpdClientHandler, NULL);

    puts("Exit");
#ifdef NUT_OS
    for (;;) ;
#endif

    return 0;
}
