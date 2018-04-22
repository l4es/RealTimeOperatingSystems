#
# Copyright (C) 2008 by egnite GmbH
# Copyright (C) 2001-2006 by egnite Software GmbH
#
# All rights reserved.
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
# 3. Neither the name of the copyright holders nor the names of
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

PROJ = libnutcrt

top_srcdir = ..
top_blddir = ..

LIBDIR = $(top_blddir)/lib
include $(top_srcdir)/NutConf.mk
include $(top_srcdir)/Makedefs

SRCC =  close.c clrerr.c ioctl.c open.c getf.c read.c putf.c write.c fclose.c \
        fcloseall.c fdopen.c feof.c ferror.c fflush.c filelength.c fileno.c flushall.c \
        fmode.c fopen.c fpurge.c freopen.c fseek.c ftell.c seek.c tell.c fgetc.c fgets.c \
        fread.c fscanf.c getc.c getchar.c gets.c kbhit.c scanf.c ungetc.c vfscanf.c \
        fprintf.c fputc.c fputs.c fwrite.c printf.c putc.c putchar.c puts.c vfprintf.c \
        sprintf.c sscanf.c vsprintf.c vsscanf.c gmtime.c localtim.c mktime.c time.c \
        tzset.c errno.c malloc.c environ.c getenv.c putenv.c setenv.c

SRCCP = fprintf_p.c fputs_p.c fscanf_p.c fwrite_p.c printf_p.c puts_p.c \
	scanf_p.c sprintf_p.c sscanf_p.c vfprintf_p.c vfscanf_p.c \
	vsprintf_p.c vsscanf_p.c write_p.c

ifneq (, $(findstring __HARVARD_ARCH__, $(HWDEF)))
# Include _P functions for Harvard based MCUs
    SRCC += $(SRCCP)
endif

ifneq (, $(findstring h8300, $(ARCH)))
# Include errno definition for H8 based MCUs
    SRCC += errno.c
endif

SRCS =  $(SRCC) putf.c getf.c
SRCF =  $(SRCC) putff.c getff.c

OBJS = $(SRCS:.c=.o)
OBJF = $(SRCF:.c=.o)

TARG  = $(PROJ).a
TARGF = $(PROJ)f.a
TARGS = $(TARG) $(TARGF)

all: $(TARGS)

include $(top_srcdir)/Makerules

$(TARG): $(OBJS)
	$(AR) $(ARFLAGS) $(TARG) $(OBJS)

$(TARGF): $(OBJF)
	$(AR) $(ARFLAGS) $(TARGF) $(OBJF)

install: $(TARGS)
	$(CP) $(TARG) $(LIBDIR)/$(TARG)
	$(CP) $(TARGF) $(LIBDIR)/$(TARGF)
ifeq ($(ARCH), unix)
	$(RANLIB) $(LIBDIR)/$(TARG)
	$(RANLIB) $(LIBDIR)/$(TARGF)
endif

clean:
	@echo "  [CLEANING] $(TARGS)..."
	@-rm -f $(OBJS)
	@-rm -f $(OBJF)
	@-rm -f $(TARGS)
	@-rm -f $(SRCS:.c=.lst) $(SRCF:.c=.lst)
	@-rm -f $(SRCS:.c=.lis) $(SRCF:.c=.lis)
	@-rm -f $(SRCS:.c=.s) $(SRCF:.c=.s)
	@-rm -f $(SRCS:.c=.bak) $(SRCF:.c=.bak)
	@-rm -f $(SRCS:.c=.c~) $(SRCF:.c=.c~)
	@-rm -f $(SRCS:.c=.i) $(SRCF:.c=.i)
