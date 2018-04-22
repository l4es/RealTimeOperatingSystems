#
# Copyright (C) 2001-2006 by egnite Software GmbH. 
# Copyright (C) 2008 by egnite GmbH. 
#
# All rights reserved.
#
# For additional information see http://www.ethernut.de/
#
# $Id: Makefile,v 1.1 2008/10/05 17:01:15 haraldkipp Exp $
#

PROJ = libhxmp3

top_srcdir = ../..
top_blddir = ../..

LIBDIR = $(top_blddir)/lib
include $(top_srcdir)/UserConf.mk
include $(top_srcdir)/Makedefs

SRCS = mp3dec.c mp3tabs.c buffers.c

OBJS = $(SRCS:.c=.o)

TARG = $(PROJ).a

all: $(TARG)

include $(top_srcdir)/Makerules

$(TARG): $(OBJS)
	$(AR) $(ARFLAGS) $(TARG) $(OBJS)

install: $(TARG)
	$(CP) $(TARG) $(LIBDIR)/$(TARG)
ifeq ($(ARCH), unix)
	$(RANLIB) $(LIBDIR)/$(TARG)
endif

clean:
	@echo "  [CLEANING] $(TARG)..."
	@-rm -f $(OBJS)
	@-rm -f $(TARG)
	@-rm -f $(SRCS:.c=.lst)
	@-rm -f $(SRCS:.c=.lis)
	@-rm -f $(SRCS:.c=.s)
	@-rm -f $(SRCS:.c=.bak)
	@-rm -f $(SRCS:.c=.c~)
	@-rm -f $(SRCS:.c=.i)
