#
# Copyright (C) 2001-2006 by egnite Software GmbH. All rights reserved.
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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
# Revision 1.9  2009/03/08 20:17:25  haraldkipp
# Added libnutgorp to fix linker error.
#
# Revision 1.8  2006/07/21 09:13:58  haraldkipp
# Added creation of *.elf for 'make all'.
#
# Revision 1.7  2006/01/23 19:49:17  haraldkipp
# Re-arranging library list. Again!
#
# Revision 1.6  2005/07/26 15:44:38  haraldkipp
# Include new libnutarch into the build
#
# Revision 1.5  2004/11/08 09:57:37  drsung
# Added rule to clean also intermediate files (*.i)
#
# Revision 1.4  2004/08/18 15:02:35  haraldkipp
# Application build is no longer fixed in top_srcdir
#
# Revision 1.3  2004/08/04 16:28:16  haraldkipp
# Dataflash check added
#
# Revision 1.2  2003/11/03 16:12:02  haraldkipp
# Completely rewritten to support Ethernut 2
#
# Revision 1.1  2003/08/07 09:37:58  haraldkipp
# First public check in
#
# Revision 1.11  2003/03/31 14:52:57  harald
# Prepare release 3.1
#
# Revision 1.10  2003/02/04 16:24:24  harald
# Adapted to version 3
#
# Revision 1.9  2003/01/14 17:11:07  harald
# *** empty log message ***
#
# Revision 1.8  2002/10/31 16:22:55  harald
# Mods by troth for Linux
#
# Revision 1.7  2002/08/11 12:28:36  harald
# Using hex file extension now
#
# Revision 1.6  2002/08/02 13:59:15  harald
# *** empty log message ***
#
# Revision 1.5  2002/06/26 17:28:59  harald
# First pre-release with 2.4 stack
#
# Revision 1.4  2002/06/04 18:46:51  harald
# New make environment
#
#
PROJ = basemon
WEBDIR = html
WEBFILE= urom.c

include ../Makedefs

SRCS =  $(PROJ).c xmemtest.c dataflash.c realtek.c smsc.c uart.c webdemo.c utils.c $(WEBFILE)
LIBS =  -lnutnet -lnutpro -lnutfs -lnutos -lnutarch -lnutdev -lnutgorp -lnutcrt

OBJS =  $(SRCS:.c=.o)
TARG =  $(PROJ).hex

all: $(OBJS) $(TARG) $(ITARG) $(DTARG)

dep:
	$(CC) -I$(top_srcdir)/include -M $(PROJ).c


$(WEBFILE): $(WEBDIR)/enlogo.gif $(WEBDIR)/entail.gif $(WEBDIR)/index.html
	$(CRUROM) -r -o$(WEBFILE) $(WEBDIR)


include ../Makerules

studio: $(PROJ).obj

clean:
	-rm -f $(OBJS)
	-rm -f $(TARG) $(ITARG) $(DTARG)
	-rm -f $(PROJ).obj
	-rm -f $(PROJ).sym
	-rm -f $(PROJ).map
	-rm -f $(SRCS:.c=.lst)
	-rm -f $(SRCS:.c=.bak)
	-rm -f $(SRCS:.c=.i)
	-rm -f $(WEBFILE)
