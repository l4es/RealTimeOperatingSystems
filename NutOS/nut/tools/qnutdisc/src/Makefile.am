## **************************************************************************
## (c) 2012 Ole Reinhardt <ole.reinhardt@embedded-it.de>
## **************************************************************************
## Process this file with automake to produce Makefile.in

ACLOCAL_AMFLAGS = -I m4
MAINTAINERCLEANFILES = Makefile.in

DEFINES       = -DQT_NO_DEBUG -DQT_GUI_LIB -DQT_NETWORK_LIB -DQT_CORE_LIB -DQT_SHARED

ui_%.h : %.ui
	$(UIC) -o "$@" "$<"

clean-ui-extra:
	rm -vf ui_*.h

## MOC compiler    
    
MOC=moc

moc_%.cpp : %.h
	$(MOC) $(DEFINES) -o "$@" "$<"

clean-moc-extra:
	rm -vf moc_*.cpp
    
## Ressources and ressource compiler    

IMAGES =

RCC=rcc

qrc_%.cpp : %.qrc $(IMAGES)
	$(RCC) -o "$@" "$<"

clean-qrc-extra:
	rm -vf qrc_*.cpp
    
## Extra clean targets     
    
clean-local: clean-ui-extra clean-moc-extra clean-qrc-extra

#clean-am: clean-ui-extra clean-moc-extra clean-qrc-extra

AM_CPPFLAGS = \
        -DPACKAGE_LOCALE_DIR=\""$(prefix)/$(DATADIRNAME)/locale"\" \
        -DPACKAGE_SRC_DIR=\""$(srcdir)"\" \
        -DPACKAGE_DATA_DIR=\""$(datadir)"\" $(QNUTDISC_CFLAGS) \
        $(DEFINES) -D_REENTRANT 

AM_CFLAGS =\
         -Wall\
         -g

MOC_SOURCES = \
	moc_nutenummodel.cpp             \
	moc_mainwindow.cpp

UI_HEADER = \
	ui_mainwindow.h

QRC_SOURCES = \
	qrc_mainwindow.cpp

BUILT_SOURCES =                          \
	$(MOC_SOURCES)                   \
	$(UI_HEADER)                     \
	$(QRC_SOURCES)

bin_PROGRAMS = qnutdisc
    
nodist_qnutdisc_SOURCES =   \
    $(BUILT_SOURCES)

qnutdisc_SOURCES =              \
	macaddress.cpp          \
	macaddress.h            \
	main.cpp		\
	mainwindow.cpp		\
	mainwindow.h		\
	nutenummodel.cpp	\
	nutenummodel.h
    
qnutdisc_LDFLAGS = 

qnutdisc_LDADD = $(QNUTDISC_LIBS)

EXTRA_DIST = mainwindow.ui mainwindow.qrc

