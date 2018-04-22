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

PROJ = libnutdev

top_srcdir = ..
top_blddir = ..

LIBDIR = $(top_blddir)/lib
include $(top_srcdir)/NutConf.mk
include $(top_srcdir)/Makedefs

SRCS = ihndlr.c can_dev.c usart.c null.c chat.c term.c netbuf.c ppp.c nvmem.c \
       twbbif.c mmcard.c x12rtc.c cy2239x.c npl.c nplmmc.c watchdog.c \
       rtc.c ds1307rtc.c at49bv.c avrtarget.c sbbif0.c pcf8563.c pca9555.c \
       pwm.c usart0sc16is752.c usartsc16is752.c spi_7seg.c led.c keys.c \
       at25df.c

ifeq ($(DEVICE), SAM3U)
	SRCS+=      usb/CDCDSerialDriver.c \
		    usb/CDCDSerialDriverDescriptors.c \
		    usb/CDCLineCoding.c \
		    usb/CDCSetControlLineStateRequest.c \
		    usb/USBConfigurationDescriptor.c \
		    usb/USBDCallbacks_Initialized.c \
		    usb/USBDCallbacks_Reset.c \
		    usb/USBDDriver.c \
		    usb/USBDDriverCb_CfgChanged.c \
		    usb/USBDDriverCb_IfSettingChanged.c \
		    usb/USBD_UDPHS.c \
		    usb/USBEndpointDescriptor.c \
		    usb/USBFeatureRequest.c \
		    usb/USBGenericDescriptor.c \
		    usb/USBGenericRequest.c \
		    usb/USBGetDescriptorRequest.c \
		    usb/USBInterfaceRequest.c \
		    usb/USBSetAddressRequest.c \
		    usb/USBSetConfigurationRequest.c

endif

ifeq ($(DEVICE), STM32F10X)
	SRCS += ber_fpga_api.c ber_fpga_upload.c #liu.c
endif

ifeq ($(DEVICE), AT91R40008)
SRCS += npluled.c sbimmc.c
endif

ifeq ($(DEVICE), AT91SAM7X256)
SRCS += at45db.c
endif

ifeq ($(DEVICE), AT91SAM9260)
SRCS += at45db.c
endif

OBJS = $(SRCS:.c=.o)

TARG = $(PROJ).a

all: $(TARG)

include $(top_srcdir)/Makerules

$(TARG): $(OBJS)
	$(AR) $(ARFLAGS) $(TARG) $(OBJS)

install: $(TARG)
	$(CP)     $(TARG) $(LIBDIR)/$(TARG)
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
