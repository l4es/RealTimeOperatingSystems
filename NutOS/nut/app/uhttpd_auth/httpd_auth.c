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

#include <pro/uhttp/mediatypes.h>
#include <pro/uhttp/modules/mod_auth_basic.h>
#include <pro/uhttp/modules/mod_redir.h>

#include <stdio.h>

int main(void)
{
#ifdef NUT_OS
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
#endif

    puts("uHTTP authentication sample\nBuild " __DATE__ " " __TIME__);

#ifdef NUT_OS
    NutRegisterDevice(&DEV_ETHER, 0, 0);
    NutDhcpIfConfig(DEV_ETHER_NAME, NULL, 60000);
    NutRegisterDevice(&devUrom, 0, 0);
#endif

    StreamInit();
    MediaTypeInitDefaults();
    HttpRegisterRedir("", "/index.html", 301);
    HttpRegisterAuthBasic("user", "user:pass", "User Login");
    HttpRegisterAuthBasic("admin", "root:secret", "Admin Login");

    StreamClientAccept(HttpdClientHandler, "80");

    puts("Exit");
#ifdef NUT_OS
    for (;;) ;
#endif

    return 0;
}
