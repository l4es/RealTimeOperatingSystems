TEMPLATE = app
CONFIG += release
TARGET = qnutdisc

QT += network

#Include file(s)
include(qnutdisc.pri)

# This should rarely change at all, so I'm placing at the botton of the file.
QMAKE_TARGET_COMPANY = Ethernut Project
QMAKE_TARGET_PRODUCT = Discovery Tool
QMAKE_TARGET_DESCRIPTION = Ethernut device discovery tool
QMAKE_TARGET_COPYRIGHT = All rights reserved by authors
