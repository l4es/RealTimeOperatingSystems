#
# Copyright (C) 2001-2007 by egnite Software GmbH. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this
#    software must display the following acknowledgement:
#
#    This product includes software developed by egnite Software GmbH
#    and its contributors.
#
# THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
# SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# For additional information see http://www.ethernut.de/
#
# $Log: Makefile,v $
# Revision 1.12  2009/01/09 17:54:10  haraldkipp
# Temporarily removed C++ sample due to problems with GCC 4.3.
#
# Revision 1.11  2007/06/03 08:48:17  haraldkipp
# New application sample demonstrates calendar functions.
#
# Revision 1.10  2007/05/02 11:19:39  haraldkipp
# Added special XNut target for excluding not working apps.
#
# Revision 1.9  2006/07/21 09:16:29  haraldkipp
# All samples must be build for all platforms. Running the targets is a
# different story.
#
# Revision 1.8  2006/07/10 14:21:15  haraldkipp
# C++ demo added to ARM build.
#
# Revision 1.7  2006/05/25 09:20:38  haraldkipp
# Excluded those applications from ARM build, which currently do not support
# this architecture.
#
# Revision 1.6  2006/01/23 19:54:50  haraldkipp
# logtime app added
#
# Revision 1.5  2005/07/23 13:53:34  haraldkipp
# Event queue sample added.
#
# Revision 1.4  2005/02/05 20:46:09  haraldkipp
# FTP server sample added
#
# Revision 1.3  2004/08/18 15:02:34  haraldkipp
# Application build is no longer fixed in top_srcdir
#
# Revision 1.2  2003/11/04 17:46:52  haraldkipp
# Adapted to Ethernut 2
#
# Revision 1.1  2003/05/15 14:30:09  haraldkipp
# Initial using 3.2.1
#
# Revision 1.3  2003/03/31 14:52:56  harald
# Prepare release 3.1
#
# Revision 1.2  2003/01/14 13:22:52  harald
# Shell no more required
#
# Revision 1.1  2002/10/31 18:13:20  harald
# Added by troth
#
#

all:
	-$(MAKE) -C caltime
	-$(MAKE) -C cantest
# Broken: C++ no longer available?	
#	-$(MAKE) -C cppdemo
	-$(MAKE) -C display
	-$(MAKE) -C editconf
	-$(MAKE) -C events
	-$(MAKE) -C flashtest
	-$(MAKE) -C ftpd
	-$(MAKE) -C httpd
	-$(MAKE) -C httpd_simple
	-$(MAKE) -C httpd_upnp
	-$(MAKE) -C icmp-udp
	-$(MAKE) -C i2ctest
	-$(MAKE) -C inetq
	-$(MAKE) -C inetq
	-$(MAKE) -C jtagtest
	-$(MAKE) -C logtime
	-$(MAKE) -C lua
	-$(MAKE) -C mdnsd_uhttp
	-$(MAKE) -C mmc_test
	-$(MAKE) -C pingnet
	-$(MAKE) -C pioled
	-$(MAKE) -C portdio
	-$(MAKE) -C printf
	-$(MAKE) -C pppc
	-$(MAKE) -C rs232d
	-$(MAKE) -C rs232d_select
	-$(MAKE) -C simple
	-$(MAKE) -C snmpd
	-$(MAKE) -C tcps
	-$(MAKE) -C threads
	-$(MAKE) -C timers
	-$(MAKE) -C twitest
	-$(MAKE) -C uart
	-$(MAKE) -C owibus
	-$(MAKE) -C watchdog
	-$(MAKE) -C tls_client

install:
	-$(MAKE) -C caltime install
#	-$(MAKE) -C cppdemo install
	-$(MAKE) -C display install
	-$(MAKE) -C editconf install
	-$(MAKE) -C events install
	-$(MAKE) -C flashtest install
	-$(MAKE) -C ftpd install
	-$(MAKE) -C httpd install
	-$(MAKE) -C httpd_simple install
	-$(MAKE) -C icmp-udp install
	-$(MAKE) -C i2ctest install
	-$(MAKE) -C inetq install
	-$(MAKE) -C jtagtest
	-$(MAKE) -C logtime install
	-$(MAKE) -C lua install
	-$(MAKE) -C mdnsd_uhttp install
	-$(MAKE) -C mmc_test install
	-$(MAKE) -C pingnet install
	-$(MAKE) -C pioled install
	-$(MAKE) -C portdio install
	-$(MAKE) -C printf install
	-$(MAKE) -C pppc install
	-$(MAKE) -C rs232d install
	-$(MAKE) -C rs232d_select install
	-$(MAKE) -C simple install
	-$(MAKE) -C snmpd install
	-$(MAKE) -C tcps install
	-$(MAKE) -C threads install
	-$(MAKE) -C timers install
	-$(MAKE) -C uart install
	-$(MAKE) -C owibus install
	-$(MAKE) -C watchdog
	-$(MAKE) -C tls_client install

clean:
	-$(MAKE) -C caltime clean
	-$(MAKE) -C cantest clean
#	-$(MAKE) -C cppdemo clean
	-$(MAKE) -C display clean
	-$(MAKE) -C editconf clean
	-$(MAKE) -C events clean
	-$(MAKE) -C flashtest clean
	-$(MAKE) -C ftpd clean
	-$(MAKE) -C httpd clean
	-$(MAKE) -C httpd_simple clean
	-$(MAKE) -C i2ctest clean
	-$(MAKE) -C icmp-udp clean
	-$(MAKE) -C inetq clean
	-$(MAKE) -C jtagtest
	-$(MAKE) -C logtime clean
	-$(MAKE) -C lua clean
	-$(MAKE) -C mdnsd_uhttp clean
	-$(MAKE) -C mmc_test clean
	-$(MAKE) -C pingnet clean
	-$(MAKE) -C pioled clean
	-$(MAKE) -C portdio clean
	-$(MAKE) -C printf clean
	-$(MAKE) -C pppc clean
	-$(MAKE) -C rs232d clean
	-$(MAKE) -C rs232d_select install
	-$(MAKE) -C simple clean
	-$(MAKE) -C snmpd clean
	-$(MAKE) -C tcps clean
	-$(MAKE) -C threads clean
	-$(MAKE) -C timers clean
	-$(MAKE) -C twitest clean
	-$(MAKE) -C uart clean
	-$(MAKE) -C owibus clean
	-$(MAKE) -C watchdog
	-$(MAKE) -C tls_client

at90can:
	-$(MAKE) -C caltime
	-$(MAKE) -C cantest
	-$(MAKE) -C canbus
	-$(MAKE) -C display
	-$(MAKE) -C events
	-$(MAKE) -C ftpd
	-$(MAKE) -C httpd
	-$(MAKE) -C httpd_simple
	-$(MAKE) -C inetq
	-$(MAKE) -C logtime
	-$(MAKE) -C mmc_test
	-$(MAKE) -C pingnet
	-$(MAKE) -C pioled
	-$(MAKE) -C portdio
	-$(MAKE) -C printf
	-$(MAKE) -C pppc
	-$(MAKE) -C rs232d
	-$(MAKE) -C rs232d_select install
	-$(MAKE) -C simple
	-$(MAKE) -C snmpd
	-$(MAKE) -C tcps
	-$(MAKE) -C threads
	-$(MAKE) -C timers
	-$(MAKE) -C uart
	-$(MAKE) -C owibus
	-$(MAKE) -C watchdog
