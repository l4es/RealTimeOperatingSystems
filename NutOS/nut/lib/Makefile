#
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

top_srcdir = ..
include $(top_srcdir)/NutConf.mk

all:
ifneq (, $(findstring h8300, $(ARCH)))
	$(MAKE) -C $(top_srcdir)/arch/$(ARCH)/init
endif
	$(MAKE) -C $(top_srcdir)/arch
	$(MAKE) -C $(top_srcdir)/os
	$(MAKE) -C $(top_srcdir)/fs
	$(MAKE) -C $(top_srcdir)/dev
	$(MAKE) -C $(top_srcdir)/net
	$(MAKE) -C $(top_srcdir)/pro
	$(MAKE) -C $(top_srcdir)/crt
	$(MAKE) -C $(top_srcdir)/gorp
ifeq ($(ARCH), avr)
	$(MAKE) -C $(top_srcdir)/cpp
endif

install:
ifneq (, $(findstring h8300, $(ARCH)))
	$(MAKE) -C $(top_srcdir)/arch/$(ARCH)/init install
endif
	$(MAKE) -C $(top_srcdir)/arch install
	$(MAKE) -C $(top_srcdir)/os install
	$(MAKE) -C $(top_srcdir)/fs install
	$(MAKE) -C $(top_srcdir)/dev install
	$(MAKE) -C $(top_srcdir)/net install
	$(MAKE) -C $(top_srcdir)/pro install
	$(MAKE) -C $(top_srcdir)/crt install
	$(MAKE) -C $(top_srcdir)/gorp install
ifeq ($(ARCH), avr)
	$(MAKE) -C $(top_srcdir)/cpp install
endif

clean:
ifneq (, $(findstring h8300, $(ARCH)))
	$(MAKE) -C $(top_srcdir)/arch/$(ARCH)/init clean
endif
	$(MAKE) -C $(top_srcdir)/arch clean
	$(MAKE) -C $(top_srcdir)/os clean
	$(MAKE) -C $(top_srcdir)/fs clean
	$(MAKE) -C $(top_srcdir)/dev clean
	$(MAKE) -C $(top_srcdir)/net clean
	$(MAKE) -C $(top_srcdir)/pro clean
	$(MAKE) -C $(top_srcdir)/crt clean
	$(MAKE) -C $(top_srcdir)/cpp clean
	$(MAKE) -C $(top_srcdir)/gorp clean
apps:
	$(MAKE) -C $(top_srcdir)/app

apps-install:
	$(MAKE) -C $(top_srcdir)/app install

cleanall:
ifneq (, $(findstring h8300, $(ARCH)))
	$(MAKE) -C $(top_srcdir)/arch/$(ARCH)/init clean
endif
	$(MAKE) -C $(top_srcdir)/arch clean
	$(MAKE) -C $(top_srcdir)/os clean
	$(MAKE) -C $(top_srcdir)/fs clean
	$(MAKE) -C $(top_srcdir)/dev clean
	$(MAKE) -C $(top_srcdir)/net clean
	$(MAKE) -C $(top_srcdir)/pro clean
	$(MAKE) -C $(top_srcdir)/crt clean
	$(MAKE) -C $(top_srcdir)/gorp clean	
	$(MAKE) -C $(top_srcdir)/cpp clean
	$(MAKE) -C $(top_srcdir)/app clean

distclean:
ifneq (, $(findstring h8300, $(ARCH)))
	$(MAKE) -C $(top_srcdir)/arch/$(ARCH)/init clean
endif
	$(MAKE) -C $(top_srcdir)/arch clean
	$(MAKE) -C $(top_srcdir)/os clean
	$(MAKE) -C $(top_srcdir)/fs clean
	$(MAKE) -C $(top_srcdir)/dev clean
	$(MAKE) -C $(top_srcdir)/net clean
	$(MAKE) -C $(top_srcdir)/pro clean
	$(MAKE) -C $(top_srcdir)/crt clean
	$(MAKE) -C $(top_srcdir)/gorp clean	
	$(MAKE) -C $(top_srcdir)/cpp clean
	$(MAKE) -C $(top_srcdir)/app clean
	cd $(top_srcdir)
	rm -f UserConf.mk Makerules Makedefs app/Makerules app/Makedefs
	rm -f include/stdio_orig.h include/sys/types_orig.h include/errno_orig.h include/fcntl_orig.h
	rm -f include/sys/socket_orig.h include/netdb_orig.h include/netinet/in_orig.h
	rm -f `find . -name '*\.[ch]~'`
