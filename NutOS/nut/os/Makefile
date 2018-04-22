#
# Copyright (C) 2008 by egnite GmbH
# Copyright (C) 2001-2002 by egnite Software GmbH
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

PROJ = libnutos

top_srcdir = ..
top_blddir = ..

LIBDIR = $(top_blddir)/lib
include $(top_srcdir)/NutConf.mk
include $(top_srcdir)/Makedefs

SRC1 = 	nutinit.c
SRC2 =  devreg.c timer.c msg.c event.c thread.c heap.c osdebug.c confos.c \
	version.c semaphore.c mutex.c tracer.c condition.c fatal.c panic.c

#Add support for memory banks on AVR architecture
ifeq ($(ARCH), avr)
    SRC2 += bankmem.c
endif


OBJ1 = $(SRC1:.c=.o)
OBJ2 = $(SRC2:.c=.o)


TRG1 = $(OBJ1)
TRG2 = $(PROJ).a $(SRC1.c=.d)

all: $(TRG1) $(TRG2)

include $(top_srcdir)/Makerules
$(TRG1): $(SRC1.c=.d)

$(TRG2): $(OBJ2) $(SRC2.c=.d)
	$(AR) $(ARFLAGS) $(TRG2) $(OBJ2)

install: $(TRG1) $(TRG2)
	$(CP) $(TRG1) $(LIBDIR)/$(TRG1)
	$(CP) $(TRG2) $(LIBDIR)/$(TRG2)
ifeq ($(ARCH), unix)
	$(RANLIB) $(LIBDIR)/$(TRG2)
endif

clean:
	@echo "  [CLEANING] $(TARG)..."
	@-rm -f $(OBJ1) $(OBJ2)
	@-rm -f $(TRG1) $(TRG2)
	@-rm -f $(SRC1:.c=.lst) $(SRC2:.c=.lst)
	@-rm -f $(SRC1:.c=.lis) $(SRC2:.c=.lis)
	@-rm -f $(SRC1:.c=.s) $(SRC2:.c=.s)
	@-rm -f $(SRC1:.c=.bak) $(SRC2:.c=.bak)
	@-rm -f $(SRC1:.c=.c~) $(SRC2:.c=.c~)
	@-rm -f $(SRC1:.c=.i) $(SRC2:.c=.i)
