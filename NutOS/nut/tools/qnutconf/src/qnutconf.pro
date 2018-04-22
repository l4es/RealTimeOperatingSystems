
TEMPLATE = app
TARGET = qnutconf
DEPENDPATH += $$PWD
INCLUDEPATH += $$PWD ../../include/win32

CONFIG += qt thread
# CONFIG -= flat
QT += widgets

win32:DEFINES += _CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_WARNINGS
win32:INCLUDEPATH += ../../include/win32

DEFINES += NUTCONF_VERSION=3.1.0

# Input
SOURCES +=	\
			builder.cpp \
			dirtraverser.cpp \
			finddialog.cpp \
			main.cpp \
			mainwindow.cpp \
			../../common/nutcomponent.c \
			nutcomponentdelegate.cpp \
			nutcomponentmodel.cpp \
			nutcomponentmodelfilterproxy.cpp \
			nutcomponentdetailsmodel.cpp \
			nutcomponentmodel_p.cpp \
			settings.cpp \
			settingsdialog.cpp \
			systeminfo.cpp
			
HEADERS +=	\
			builder.h \
			dirtraverser.h \
			finddialog.h \
			mainwindow.h \
			../../common/nutcomponent.h \
			nutcomponentdelegate.h \
			nutcomponentmodel.h \
			nutcomponentmodelfilterproxy.h \
			nutcomponentdetailsmodel.h \
			nutcomponentmodel_p.h \
			settings.h \
			settingsdialog.h \
			systeminfo.h

FORMS +=	\
			mainwindow.ui \
			settingsdialog.ui \
			finddialog.ui

include(lua/lua.pri)

RESOURCES += qnutconf.qrc

RC_FILE = qnutconf.rc
VERSION = 3.1.0
QMAKE_TARGET_COMPANY = Ethernut Project
QMAKE_TARGET_DESCRIPTION = Ethernut Configurator
QMAKE_TARGET_COPYRIGHT = All rights reserved by authors

macx {

	# Adds the Mac icons to the application bundle. 
	ICON = images/qnutconf.icns 
}


