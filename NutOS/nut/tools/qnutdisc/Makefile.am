## **************************************************************************
## (c) 2012 Ole Reinhardt <ole.reinhardt@embedded-it.de>
## **************************************************************************
## Process this file with automake to produce Makefile.in

ACLOCAL_AMFLAGS = -I m4
MAINTAINERCLEANFILES = \
	configure \
	Makefile.in \
	depcomp \
	config.guess \
	config.sub \
	ltmain.sh \
	config.h.in \
	missing \
	aclocal.m4 \
	install-sh

SUBDIRS = src 

qnutdiscdocdir = ${prefix}/doc/qnutdisc
qnutdiscdoc_DATA = \
	README\
	COPYING\
	AUTHORS\
	ChangeLog\
	INSTALL\
	NEWS


EXTRA_DIST = $(qnutdiscdoc_DATA)

#dist-hook:
#	mkdir -p $(distdir)/src/images
#	cp -R $(srcdir)/src/images/* $(distdir)/src/images/

# Remove doc directory on uninstall
uninstall-local:
#	-rm -r $(qnutdiscdocdir)
